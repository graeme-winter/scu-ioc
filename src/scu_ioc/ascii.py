"""ASCII protocol backend (pyserial).

For RS-232 SCUs, and for USB SCUs running SmarAct's ASCII firmware (they
then enumerate as an FTDI virtual COM port). The stock USB firmware speaks
a binary protocol to SCU3DControl.dll instead; use the dll backend for that.

The port may be anything pyserial's serial_for_url() accepts: a device
path, "socket://host:port" for a serial device server, etc.

The controller is put in auto error report mode (E1) on connect, so every
command yields exactly one answer line: either the query answer or E<code>
(E0 = acknowledge). Unsolicited completion/error reports are never enabled,
but are skipped if they appear.
"""

from __future__ import annotations

import logging
import time

import serial

from . import protocol as P
from .backend import AxisConfig, Backend, ConnectionLost, DeviceInfo, check_step_args

log = logging.getLogger(__name__)


class AsciiBackend(Backend):
    name = "ascii"

    def __init__(self, port: str, baud: int = 9600, timeout: float = 1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._ser: serial.SerialBase | None = None

    # -- transport ---------------------------------------------------------
    def connect(self) -> DeviceInfo:
        self.close()
        try:
            self._ser = serial.serial_for_url(
                self.port, baudrate=self.baud, bytesize=8, parity="N",
                stopbits=1, timeout=0.05)
        except (serial.SerialException, OSError) as e:
            raise ConnectionLost(f"cannot open {self.port}: {e}") from e
        # Flush anything stale (e.g. a partial line from a previous session).
        time.sleep(0.05)
        self._ser.reset_input_buffer()
        self._ser.write(b"\n")
        self._ack("E1")
        model = self._query("I", r"I(.+)")[0].strip()
        device_id = self._query("GID", r"ID(\d+)")[0]
        firmware = self._query("V", r"V(\d+\.\d+\.\d+)")[0]
        channels = 1 if "1D" in model else 3
        return DeviceInfo(model, device_id, firmware, channels)

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _readline(self, deadline: float) -> str:
        buf = bytearray()
        while True:
            if time.monotonic() > deadline:
                raise ConnectionLost(f"timeout waiting for reply (got {bytes(buf)!r})")
            try:
                c = self._ser.read(1)
            except (serial.SerialException, OSError) as e:
                raise ConnectionLost(str(e)) from e
            if not c:
                continue
            if c == b"\n":
                line = buf.decode("ascii", "replace")
                # Anything before the last ':' is noise (manual 1.1).
                i = line.rfind(":")
                if i >= 0 and line[i + 1:]:
                    return line[i + 1:].strip()
                buf.clear()
                continue
            buf += c

    def _transact(self, command: str) -> str:
        if self._ser is None:
            raise ConnectionLost("not connected")
        log.debug("-> %s", command)
        try:
            self._ser.write(P.frame(command))
        except (serial.SerialException, OSError) as e:
            raise ConnectionLost(str(e)) from e
        deadline = time.monotonic() + self.timeout
        while True:
            answer = self._readline(deadline)
            log.debug("<- %s", answer)
            if P.is_unsolicited(answer):
                continue
            return answer

    def _ack(self, command: str) -> None:
        answer = self._transact(command)
        code = P.error_code(answer)
        if code is None:
            raise P.ProtocolError(f"{command!r}: expected E<code>, got {answer!r}")
        if code:
            raise P.CommandError(code, command)

    def _query(self, command: str, pattern: str):
        answer = self._transact(command)
        code = P.error_code(answer)
        if code is not None and command != "E":
            raise P.CommandError(code, command)
        return P.parse(pattern, answer)

    # -- reads -------------------------------------------------------------
    def axis_config(self, ch: int) -> AxisConfig:
        present = self._query(f"GSP{ch}", rf"SP{ch}([PN])")[0] == "P"
        stype = int(self._query(f"GST{ch}", rf"ST{ch}T(\d+)")[0])
        clf = int(self._query(f"GCLF{ch}", rf"CLF{ch}F(\d+)")[0])
        try:
            sd = int(self._query(f"GSD{ch}", rf"SD{ch}D(\d)")[0])
        except P.CommandError:
            sd = None          # not available for all sensor types
        return AxisConfig(present, stype, clf, sd)

    def status(self, ch: int) -> int:
        return P.parse_status(ch, self._transact(f"M{ch}"))

    def position(self, ch: int, rotary: bool) -> float:
        if rotary:
            a, _rev = self._query(f"GA{ch}", rf"A{ch}A{{N}}R{{N}}")
            return float(a)
        (p,) = self._query(f"GP{ch}", rf"P{ch}P{{N}}")
        return float(p)

    def referenced(self, ch: int) -> bool:
        return self._query(f"GPPK{ch}", rf"PPK{ch}K([01])")[0] == "1"

    def _system(self, prop: int) -> int:
        return int(self._query(f"GSPR{prop}", rf"SPR{prop}V(-?\d+)")[0])

    def temperature(self) -> float | None:
        try:
            return float(self._system(1))
        except P.CommandError:
            return None

    def hv(self) -> float | None:
        try:
            return self._system(2) / 10.0
        except P.CommandError:
            return None

    # -- commands ----------------------------------------------------------
    def move_abs(self, ch, target, hold_ms, rotary):
        if rotary:
            self._ack(f"MAA{ch}A{P.fmt_num(target)}R0H{int(hold_ms)}")
        else:
            self._ack(f"MPA{ch}P{P.fmt_num(target)}H{int(hold_ms)}")

    def move_rel(self, ch, delta, hold_ms, rotary):
        if rotary:
            self._ack(f"MAR{ch}A{P.fmt_num(delta)}R0H{int(hold_ms)}")
        else:
            self._ack(f"MPR{ch}P{P.fmt_num(delta)}H{int(hold_ms)}")

    def step(self, ch, steps, amplitude, frequency):
        check_step_args(steps, amplitude, frequency)
        verb = "U" if steps > 0 else "D"
        self._ack(f"{verb}{ch}F{int(frequency)}A{int(amplitude)}S{abs(int(steps))}")

    def stop(self, ch):
        self._ack(f"S{ch}")

    def calibrate(self, ch):
        self._ack(f"CS{ch}")

    def reference(self, ch, hold_ms, auto_zero):
        self._ack(f"MTR{ch}H{int(hold_ms)}Z{1 if auto_zero else 0}")

    def set_zero(self, ch):
        self._ack(f"SZ{ch}")

    def set_sensor_type(self, ch, code):
        self._ack(f"SST{ch}T{int(code)}")

    def set_cl_max_frequency(self, ch, hz):
        self._ack(f"SCLF{ch}F{int(hz)}")

    def set_safe_direction(self, ch, forward):
        self._ack(f"SSD{ch}D{1 if forward else 0}")
