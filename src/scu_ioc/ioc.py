"""pythonSoftIOC front end for SmarAct SCU controllers.

One worker thread owns the backend: record callbacks queue work for it,
and it polls the controller between commands (fast while anything is
moving). Loss of contact puts every readback INVALID/COMM and the worker
reconnects by itself.
"""

from __future__ import annotations

import argparse
import collections
import logging
import threading
import time

from softioc import alarm, builder, softioc
from softioc.asyncio_dispatcher import AsyncioDispatcher

from . import __version__
from . import protocol as P
from .backend import Backend, ConnectionLost, DeviceInfo

log = logging.getLogger("scu_ioc")

MOVE_SETTLE_S = 0.3     # DMOV stays 0 at least this long after a command
SYSTEM_POLL_S = 5.0     # temperature / HV
CONFIG_POLL_S = 10.0    # referenced flag on idle axes
INVALID = dict(severity=alarm.INVALID_ALARM, alarm=alarm.COMM_ALARM)


def _now() -> float:
    return time.monotonic()


class WorkQueue:
    """Deque + condition; urgent items (stops) jump the queue."""

    def __init__(self):
        self._q = collections.deque()
        self._cv = threading.Condition()

    def put(self, item, urgent=False):
        with self._cv:
            (self._q.appendleft if urgent else self._q.append)(item)
            self._cv.notify()

    def get(self, timeout):
        with self._cv:
            if not self._q:
                self._cv.wait(timeout)
            return self._q.popleft() if self._q else None

    def purge(self, owner):
        """Drop queued items belonging to owner (used by STOP)."""
        with self._cv:
            self._q = collections.deque(i for i in self._q if i[0] is not owner)

    def clear(self):
        with self._cv:
            items = list(self._q)
            self._q.clear()
            return items


class Axis:
    def __init__(self, ctl: "Controller", ch: int, name: str):
        self.ctl = ctl
        self.ch = ch
        self.name = name
        self.config = None
        self.present = True          # channel exists on the device
        self.status = P.STOPPED
        self.rbv = 0.0
        self.pending_until = 0.0
        self.sync_val_on_done = False
        self.done = True
        self.last_config_read = 0.0
        # set_field() processes the record, which fires on_update for
        # always_update records; count those so they can be ignored.
        self._ignore_val = 0
        self._lock = threading.Lock()
        p = f"{name}:"
        q = self._queue

        def button(rec_name, label, fn, urgent=False, moves=True):
            def cb(v):
                if v:
                    rec.set(0, process=False)
                    if urgent:
                        self.ctl.queue.purge(self)
                    if moves:
                        self._start_motion(fn)
                    else:
                        q(fn, urgent=urgent)
            rec = builder.boolOut(p + rec_name, ZNAM="", ONAM=label, initial_value=0,
                                  always_update=True, blocking=True, on_update=cb)
            return rec

        # -- motion ----------------------------------------------------------
        self.VAL = builder.aOut(p + "VAL", initial_value=0.0, PREC=3, EGU="um",
                                always_update=True, blocking=True, on_update=self._on_val)
        self.RBV = builder.aIn(p + "RBV", PREC=3, EGU="um")
        self.RLV = builder.aOut(p + "RLV", initial_value=0.0, PREC=3, EGU="um",
                                always_update=True, blocking=True, on_update=self._on_rlv)
        self.TWV = builder.aOut(p + "TWV", initial_value=10.0, PREC=3, EGU="um")
        button("TWF", "TWEAK+", lambda: self._tweak(+1))
        button("TWR", "TWEAK-", lambda: self._tweak(-1))
        self.HLM = builder.aOut(p + "HLM", initial_value=0.0, PREC=3, EGU="um")
        self.LLM = builder.aOut(p + "LLM", initial_value=0.0, PREC=3, EGU="um")
        self.HOLD = builder.longOut(p + "HOLD", initial_value=0, DRVL=0,
                                    DRVH=P.HOLD_FOREVER, EGU="ms")
        self.DMOV = builder.boolIn(p + "DMOV", ZNAM="Moving", ONAM="Done", initial_value=1)
        self.MOVN = builder.boolIn(p + "MOVN", ZNAM="Idle", ONAM="Moving", initial_value=0)
        self.STATUS = builder.mbbIn(p + "STATUS", *P.STATUS_NAMES, initial_value=0)
        button("STOP", "STOP", self._stop, urgent=True, moves=False)

        # -- open loop -------------------------------------------------------
        self.STEP = builder.longOut(p + "STEP", initial_value=0, always_update=True,
                                    blocking=True,
                                    DRVL=-(P.MAX_STEPS - 1), DRVH=P.MAX_STEPS - 1,
                                    on_update=self._on_step)
        self.STEP_AMP = builder.aOut(p + "STEP_AMP", initial_value=100.0, PREC=1,
                                     EGU="V", DRVL=15.0, DRVH=100.0)
        self.STEP_FREQ = builder.longOut(p + "STEP_FREQ", initial_value=1000,
                                         EGU="Hz", DRVL=1, DRVH=18500)

        # -- sensor / referencing -------------------------------------------
        button("CALIBRATE", "CALIBRATE", self._calibrate)
        button("REFERENCE", "REFERENCE", self._reference)
        button("SET_ZERO", "SET ZERO", self._set_zero, moves=False)
        self.AUTO_ZERO = builder.boolOut(p + "AUTO_ZERO", ZNAM="No", ONAM="Yes", initial_value=0)
        self.REFERENCED = builder.boolIn(p + "REFERENCED", ZNAM="No", ONAM="Yes")
        self.SENSOR_PRESENT = builder.boolIn(p + "SENSOR_PRESENT", ZNAM="No", ONAM="Yes")
        self.SENSOR_TYPE = builder.longOut(p + "SENSOR_TYPE", initial_value=0,
                                           on_update=self._on_sensor_type)
        self.SENSOR_TYPE_RBV = builder.longIn(p + "SENSOR_TYPE_RBV")
        self.SENSOR_NAME = builder.stringIn(p + "SENSOR_NAME", initial_value="")
        self.CL_MAX_FREQ = builder.longOut(p + "CL_MAX_FREQ", initial_value=5000, EGU="Hz",
                                           DRVL=P.CL_FREQUENCY_RANGE[0],
                                           DRVH=P.CL_FREQUENCY_RANGE[1],
                                           on_update=self._on_clf)
        self.CL_MAX_FREQ_RBV = builder.longIn(p + "CL_MAX_FREQ_RBV", EGU="Hz")
        self.SAFE_DIR = builder.boolOut(p + "SAFE_DIR", ZNAM="Backward", ONAM="Forward",
                                        initial_value=1, on_update=self._on_safe_dir)
        self.SAFE_DIR_RBV = builder.boolIn(p + "SAFE_DIR_RBV", ZNAM="Backward", ONAM="Forward")
        self.LAST_ERROR = builder.longStringIn(p + "LAST_ERROR", initial_value="", length=256)
        self._unit_records = [self.VAL, self.RBV, self.RLV, self.TWV, self.HLM, self.LLM]

    # -- helpers -------------------------------------------------------------
    @property
    def b(self) -> Backend:
        return self.ctl.backend

    @property
    def rotary(self) -> bool:
        return bool(self.config and self.config.rotary)

    def _queue(self, fn, urgent=False):
        self.ctl.queue.put((self, fn), urgent=urgent)

    def error(self, msg: str):
        log.warning("%s: %s", self.name, msg)
        self.LAST_ERROR.set(msg[:255])

    def _start_motion(self, fn):
        """Record thread: queue a motion command. DMOV drops here, so a
        client's put-callback completes with DMOV already 0."""
        if not self.ctl.connected:
            self.error("not connected")
            self.VAL.set(self.rbv, process=False)
            return
        self.pending_until = _now() + 10.0   # until the worker takes over
        self.done = False
        self.DMOV.set(0)
        self._queue(fn)

    def _begin(self, sync_val: bool):
        """Worker thread, after a motion command has been accepted."""
        self.pending_until = _now() + MOVE_SETTLE_S
        self.sync_val_on_done = sync_val
        self.done = False
        self.DMOV.set(0)
        self.MOVN.set(1)
        self.LAST_ERROR.set("")
        self.ctl.wake_fast()

    def _check_limits(self, target: float) -> None:
        hi, lo = self.HLM.get(), self.LLM.get()
        if hi > lo and not lo <= target <= hi:
            e = P.CommandError(17)
            e.args = (f"target {target:g} outside soft limits [{lo:g}, {hi:g}]",)
            raise e

    # -- callbacks (record thread) -> queued work (worker thread) -------------
    def _on_val(self, v):
        with self._lock:
            if self._ignore_val:
                self._ignore_val -= 1
                return
        self._start_motion(lambda: self._move_abs(float(v)))

    def _on_rlv(self, v):
        if v:
            self.RLV.set(0.0, process=False)
            self._start_motion(lambda: self._move_abs(self.VAL.get() + float(v)))

    def _tweak(self, sign):
        self._move_abs(self.VAL.get() + sign * abs(self.TWV.get()))

    def _on_step(self, v):
        if v:
            self._start_motion(lambda: self._step(int(v)))

    def _on_sensor_type(self, v):
        self._queue(lambda: (self.b.set_sensor_type(self.ch, int(v)), self.read_config()))

    def _on_clf(self, v):
        self._queue(lambda: (self.b.set_cl_max_frequency(self.ch, int(v)), self.read_config()))

    def _on_safe_dir(self, v):
        self._queue(lambda: (self.b.set_safe_direction(self.ch, bool(v)), self.read_config()))

    # -- worker-thread actions -------------------------------------------------
    def _move_abs(self, target: float):
        try:
            self._check_limits(target)
            self.b.move_abs(self.ch, target, int(self.HOLD.get()), self.rotary)
        except P.CommandError:
            self.VAL.set(self.rbv, process=False)
            raise
        self.VAL.set(target, process=False)
        self._begin(sync_val=False)

    def _step(self, n: int):
        amp = int(round(self.STEP_AMP.get() * 10))
        try:
            self.b.step(self.ch, n, amp, int(self.STEP_FREQ.get()))
        except ValueError as e:
            raise P.CommandError(17, str(e)) from None
        self._begin(sync_val=True)

    def _stop(self):
        self.pending_until = 0.0
        self.b.stop(self.ch)
        self.sync_val_on_done = True
        self.ctl.wake_fast()

    def _calibrate(self):
        self.b.calibrate(self.ch)
        self._begin(sync_val=True)

    def _reference(self):
        self.b.reference(self.ch, int(self.HOLD.get()), bool(self.AUTO_ZERO.get()))
        self._begin(sync_val=True)

    def _set_zero(self):
        self.b.set_zero(self.ch)
        self.poll(full=True)
        self.VAL.set(self.rbv, process=False)

    # -- state -----------------------------------------------------------------
    def read_config(self):
        cfg = self.b.axis_config(self.ch)
        units_changed = self.config is None or self.config.rotary != cfg.rotary
        self.config = cfg
        self.SENSOR_PRESENT.set(int(cfg.sensor_present))
        self.SENSOR_TYPE_RBV.set(cfg.sensor_type)
        self.SENSOR_TYPE.set(cfg.sensor_type, process=False)
        self.SENSOR_NAME.set(P.sensor_symbol(cfg.sensor_type) if cfg.sensor_present else "none")
        if cfg.cl_max_frequency is not None:
            self.CL_MAX_FREQ_RBV.set(cfg.cl_max_frequency)
            self.CL_MAX_FREQ.set(cfg.cl_max_frequency, process=False)
        if cfg.safe_direction is not None:
            self.SAFE_DIR_RBV.set(cfg.safe_direction)
            self.SAFE_DIR.set(cfg.safe_direction, process=False)
        else:
            self.SAFE_DIR_RBV.set(0, **INVALID)
        if units_changed:
            egu = "mdeg" if cfg.rotary else "um"
            for r in self._unit_records:
                if r is self.VAL:
                    with self._lock:
                        self._ignore_val += 1
                r.set_field("EGU", egu)

    def on_connect(self, info: DeviceInfo):
        self.present = self.ch < info.channels
        if not self.present:
            self.error(f"channel {self.ch} not on a {info.channels}-channel SCU")
            self.disconnected()
            return
        self.read_config()
        self.poll(full=True)
        self.VAL.set(self.rbv, process=False)
        self.done = self.status not in P.BUSY
        self.DMOV.set(int(self.done))

    def poll(self, full: bool):
        if not self.present:
            return
        now = _now()
        status = self.b.status(self.ch)
        self.status = status
        self.STATUS.set(status)
        if self.config and self.config.sensor_present:
            self.rbv = self.b.position(self.ch, self.rotary)
            self.RBV.set(self.rbv)
        else:
            self.RBV.set(0.0, severity=alarm.INVALID_ALARM, alarm=alarm.UDF_ALARM)
        busy = status in P.BUSY
        if busy:
            self.pending_until = 0.0
        done = not busy and now >= self.pending_until
        self.MOVN.set(int(busy))
        if done and not self.done:
            # just finished
            if self.sync_val_on_done:
                self.VAL.set(self.rbv, process=False)
            self.sync_val_on_done = False
            full = True
        self.done = done
        self.DMOV.set(int(done))
        if full or now - self.last_config_read > CONFIG_POLL_S:
            if self.config and self.config.sensor_present:
                self.REFERENCED.set(int(self.b.referenced(self.ch)))
            self.last_config_read = now

    @property
    def active(self) -> bool:
        """Worth fast polling: moving, or holding a target."""
        return self.present and (not self.done or self.status == P.HOLDING)

    def disconnected(self):
        for r in (self.RBV, self.STATUS, self.DMOV, self.MOVN, self.REFERENCED,
                  self.SENSOR_PRESENT, self.SENSOR_TYPE_RBV, self.CL_MAX_FREQ_RBV,
                  self.SAFE_DIR_RBV):
            r.set(r.get() or 0, **INVALID)
        self.done = True


class Controller:
    def __init__(self, backend: Backend, prefix: str, axis_names: dict,
                 poll: float, busy_poll: float):
        self.backend = backend
        self.prefix = prefix
        self.poll_period = poll
        self.busy_poll = busy_poll
        self.queue = WorkQueue()
        self.connected = False
        self.info: DeviceInfo | None = None
        self._fast = False
        self._last_system = 0.0

        self.CONNECTED = builder.boolIn("CONNECTED", ZNAM="Disconnected", ONAM="Connected",
                                        initial_value=0)
        self.MODEL = builder.stringIn("MODEL", initial_value="")
        self.DEVICE_ID = builder.stringIn("DEVICE_ID", initial_value="")
        self.FIRMWARE = builder.stringIn("FIRMWARE", initial_value="")
        self.BACKEND = builder.stringIn("BACKEND", initial_value=backend.name)
        self.VERSION = builder.stringIn("VERSION", initial_value=__version__)
        self.TEMPERATURE = builder.aIn("TEMPERATURE", EGU="C", PREC=0)
        self.HV = builder.aIn("HV", EGU="V", PREC=1)
        self.LAST_ERROR = builder.longStringIn("LAST_ERROR", initial_value="", length=256)

        def stop_all(v):
            if v:
                self.STOP_ALL.set(0, process=False)
                for a in self.axes:
                    self.queue.purge(a)
                    self.queue.put((a, a._stop), urgent=True)
        self.STOP_ALL = builder.boolOut("STOP_ALL", ZNAM="", ONAM="STOP",
                                        initial_value=0, always_update=True,
                                        on_update=stop_all)
        self.axes = [Axis(self, ch, name) for ch, name in sorted(axis_names.items())]

    def wake_fast(self):
        self._fast = True

    # -- worker ------------------------------------------------------------------
    def start(self):
        threading.Thread(target=self._run, daemon=True, name="scu-worker").start()

    def _connect(self):
        info = self.backend.connect()
        self.info = info
        log.info("connected: %s id %s firmware %s (%s backend)",
                 info.model, info.device_id, info.firmware, self.backend.name)
        self.MODEL.set(info.model[:39])
        self.DEVICE_ID.set(info.device_id)
        self.FIRMWARE.set(info.firmware)
        for a in self.axes:
            a.on_connect(info)
        self._poll_system()
        self.connected = True
        self.CONNECTED.set(1)
        self.LAST_ERROR.set("")

    def _disconnect(self, reason: str):
        if self.connected:
            log.error("lost contact: %s", reason)
        self.connected = False
        self.backend.close()
        self.CONNECTED.set(0)
        self.LAST_ERROR.set(reason[:255])
        self.TEMPERATURE.set(self.TEMPERATURE.get() or 0, **INVALID)
        self.HV.set(self.HV.get() or 0, **INVALID)
        for a in self.axes:
            a.disconnected()

    def _poll_system(self):
        t = self.backend.temperature()
        if t is not None:
            self.TEMPERATURE.set(t)
        hv = self.backend.hv()
        if hv is not None:
            self.HV.set(hv)
        self._last_system = _now()

    def _poll(self, full: bool):
        for a in self.axes:
            if full or a.active:
                a.poll(full=False)
        if full and _now() - self._last_system > SYSTEM_POLL_S:
            self._poll_system()

    def _run_item(self, item):
        axis, fn = item
        if not self.connected:
            axis.pending_until = 0.0
            axis.error("not connected")
            return
        try:
            fn()
        except P.CommandError as e:
            axis.pending_until = 0.0
            axis.error(str(e))
            # make sure VAL/DMOV reflect reality after a rejected command
            axis.poll(full=True)
            if not axis.active:
                axis.VAL.set(axis.rbv, process=False)

    def _run(self):
        backoff = 1.0
        next_full = 0.0
        while True:
            if not self.connected:
                for axis, _ in self.queue.clear():
                    axis.pending_until = 0.0
                    axis.error("not connected")
                try:
                    self._connect()
                    backoff = 1.0
                    next_full = 0.0
                except (ConnectionLost, P.ProtocolError, P.CommandError) as e:
                    self._disconnect(str(e))
                    log.info("connect failed (%s); retrying in %.0f s", e, backoff)
                    time.sleep(backoff)
                    backoff = min(backoff * 2, 30.0)
                continue
            try:
                now = _now()
                fast = self._fast or any(a.active for a in self.axes)
                self._fast = False
                if now >= next_full:
                    self._poll(full=True)
                    next_full = now + self.poll_period
                elif fast:
                    self._poll(full=False)
                period = self.busy_poll if (fast or any(a.active for a in self.axes)) \
                    else max(0.0, next_full - _now())
                item = self.queue.get(timeout=period)
                while item is not None:
                    self._run_item(item)
                    item = self.queue.get(timeout=0)
            except (ConnectionLost, P.ProtocolError) as e:
                self._disconnect(str(e))


def parse_axes(spec: str) -> dict:
    """'A,B,C' -> {0:'A',1:'B',2:'C'}; '-' skips a channel."""
    names = [s.strip() for s in spec.split(",")]
    if not 1 <= len(names) <= 3:
        raise ValueError("--axes takes 1 to 3 comma-separated names")
    out = {i: n for i, n in enumerate(names) if n and n != "-"}
    if len(set(out.values())) != len(out):
        raise ValueError("axis names must be unique")
    return out


def make_backend(a) -> Backend:
    if a.backend == "ascii":
        from .ascii import AsciiBackend
        if not a.port:
            raise SystemExit("--port is required for the ascii backend")
        return AsciiBackend(a.port, a.baud, a.timeout)
    from .dll import DllBackend
    return DllBackend(device_id=a.device_id, dll_path=a.dll)


def main(argv=None):
    ap = argparse.ArgumentParser(description="EPICS soft IOC for SmarAct SCU controllers")
    ap.add_argument("--prefix", required=True,
                    help="PV prefix, used verbatim (include any trailing ':')")
    ap.add_argument("--backend", choices=["dll", "ascii"], default="dll",
                    help="dll: stock USB firmware via SCU3DControl.dll (Windows); "
                         "ascii: RS-232 or ASCII-firmware USB via serial port")
    ap.add_argument("--port", help="ascii: serial device or pyserial URL (socket://host:port)")
    ap.add_argument("--baud", type=int, default=9600, help="ascii: baud rate (default 9600)")
    ap.add_argument("--device-id", type=int, help="dll: SCU device ID (default: first found)")
    ap.add_argument("--dll", help="dll: path to SCU3DControl.dll (default: $SCU_DLL or DLL search path)")
    ap.add_argument("--axes", default="A,B,C",
                    help="PV names for channels 0,1,2 ('-' skips a channel; default A,B,C)")
    ap.add_argument("--poll", type=float, default=1.0, help="idle poll period, s (default 1)")
    ap.add_argument("--busy-poll", type=float, default=0.1,
                    help="poll period while moving, s (default 0.1)")
    ap.add_argument("--timeout", type=float, default=1.0, help="ascii: reply timeout, s")
    ap.add_argument("--no-shell", action="store_true", help="no interactive Python shell")
    ap.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    a = ap.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    try:
        axes = parse_axes(a.axes)
    except ValueError as e:
        ap.error(str(e))

    import epicsdbbuilder
    epicsdbbuilder.SetSimpleRecordNames(prefix=a.prefix, separator="")

    dispatcher = AsyncioDispatcher()
    ctl = Controller(make_backend(a), a.prefix, axes, a.poll, a.busy_poll)
    builder.LoadDatabase()
    softioc.iocInit(dispatcher)
    ctl.start()

    if a.no_shell:
        threading.Event().wait()
    else:
        softioc.interactive_ioc(globals() | {"ctl": ctl})


if __name__ == "__main__":
    main()
