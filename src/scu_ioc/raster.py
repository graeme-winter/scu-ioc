"""Serpentine step-and-shoot raster over two SCU axes.

For each grid point: absolute closed-loop move(s), wait until the
controller reports the target reached, optional settle time, read back
both axes, fire the trigger, dwell. Rows run in alternating directions
(serpentine), so only the fast axis moves within a row and the slow axis
moves once per row.

There is no hardware position compare on the SCU, so a trigger here means
"the stage has stopped at this point": software timing jitter (~ms)
affects when a shot happens, not where. Every point is logged with its
target, the measured positions and timestamps, which gives the
image-number-to-position map for the data.

Works with any backend (wire, sdk, ascii). Positions are um for linear
axes, mdeg for rotary ones.
"""

from __future__ import annotations

import csv
import threading
import time
from dataclasses import dataclass, field

from . import protocol as P
from .backend import Backend, ConnectionLost


# --------------------------------------------------------------------------
# Grid
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Line:
    """Points start, start+step, ... end along one axis."""
    channel: int
    start: float
    end: float
    step: float

    @property
    def points(self) -> list[float]:
        span = self.end - self.start
        if self.step <= 0:
            raise ValueError("step must be positive (direction comes from start/end)")
        if span == 0:
            return [self.start]
        n = span / self.step if span > 0 else -span / self.step
        k = round(n)
        if abs(n - k) > 1e-6 * max(1.0, n):
            raise ValueError(f"range {self.start}..{self.end} is not a whole number of "
                             f"{self.step} steps ({n:.4g})")
        sign = 1.0 if span > 0 else -1.0
        return [self.start + sign * i * self.step for i in range(k + 1)]


@dataclass(frozen=True)
class Point:
    index: int
    row: int
    col: int          # index along the fast line as defined (not travel order)
    fast: float
    slow: float


def serpentine(fast: Line, slow: Line) -> list[Point]:
    """Row-major serpentine: even rows run start->end, odd rows end->start."""
    fpts, spts = fast.points, slow.points
    out = []
    for r, s in enumerate(spts):
        cols = range(len(fpts)) if r % 2 == 0 else range(len(fpts) - 1, -1, -1)
        for c in cols:
            out.append(Point(len(out), r, c, fpts[c], s))
    return out


# --------------------------------------------------------------------------
# Triggers
# --------------------------------------------------------------------------

class Trigger:
    """Fired once per point after the stage has settled."""

    def fire(self) -> None:
        pass

    def close(self) -> None:
        pass

    def describe(self) -> str:
        return "none"


class SerialLineTrigger(Trigger):
    """Pulse a modem-control line (DTR or RTS) on a separate USB-serial
    adapter. On FTDI/CP210x TTL adapters an asserted line is electrically
    LOW, so with polarity "low" (default) the pin goes low for the pulse;
    use "high" if your adapter or wiring is non-inverting. Jitter is that of
    a USB control transfer (~1 ms); fine for step-and-shoot."""

    def __init__(self, port: str, line: str = "dtr", width_ms: float = 1.0,
                 active_low: bool = True, ser=None):
        import serial
        if line not in ("dtr", "rts"):
            raise ValueError("line must be dtr or rts")
        self.line, self.width = line, width_ms / 1000.0
        self.active_low = active_low
        self.port = port
        self.ser = ser or serial.serial_for_url(port, do_not_open=True)
        if ser is None:
            # set the idle level before opening, so there's no glitch
            setattr(self.ser, self.line, self._level(False))
            self.ser.open()
        setattr(self.ser, self.line, self._level(False))

    def _level(self, active: bool) -> bool:
        # pyserial True = asserted = TTL low on typical adapters
        return active if self.active_low else not active

    def fire(self):
        setattr(self.ser, self.line, self._level(True))
        time.sleep(self.width)
        setattr(self.ser, self.line, self._level(False))

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def describe(self):
        return (f"{self.line.upper()} on {self.port}, {self.width * 1000:g} ms, "
                f"active {'low' if self.active_low else 'high'}")


class EpicsPutTrigger(Trigger):
    """Write a value to a PV per point (e.g. a Zebra soft input, or a
    detector trigger PV), waiting for put completion. Needs caproto."""

    def __init__(self, pv: str, value="1", ctx=None):
        try:
            from caproto.threading.client import Context
        except ImportError as e:  # pragma: no cover
            raise RuntimeError("EPICS triggers need caproto (pip install caproto)") from e
        self.ctx = ctx or Context()
        (self.pv,) = self.ctx.get_pvs(pv, timeout=5)
        self.pv.wait_for_connection(timeout=5)
        self.name, self.value = pv, value

    def fire(self):
        v = self.value
        try:
            v = float(v) if "." in v else int(v)
        except ValueError:
            pass
        self.pv.write([v], wait=True, timeout=5)

    def close(self):
        try:
            self.ctx.disconnect()
        except Exception:
            pass

    def describe(self):
        return f"caput {self.name} {self.value}"


def make_trigger(spec: str | None, width_ms: float = 1.0) -> Trigger:
    """none | serial:<port>[:dtr|rts][:low|high] | ca:<pv>[=value]"""
    if not spec or spec == "none":
        return Trigger()
    kind, _, rest = spec.partition(":")
    if kind == "serial":
        parts = rest.split(":")
        line, pol = "dtr", "low"
        while parts and parts[-1].lower() in ("dtr", "rts", "low", "high"):
            p = parts.pop().lower()
            if p in ("dtr", "rts"):
                line = p
            else:
                pol = p
        return SerialLineTrigger(":".join(parts), line, width_ms, active_low=(pol == "low"))
    if kind == "ca":
        pv, _, value = rest.partition("=")
        return EpicsPutTrigger(pv, value or "1")
    raise ValueError(f"unknown trigger {spec!r} (none, serial:PORT[:dtr|rts][:low|high], ca:PV[=V])")


# --------------------------------------------------------------------------
# Execution
# --------------------------------------------------------------------------

class RasterAborted(RuntimeError):
    pass


@dataclass
class RasterResult:
    points_done: int
    elapsed: float
    max_error: dict = field(default_factory=dict)   # channel -> max |measured - target|
    aborted: str | None = None


def check_ready(b: Backend, channels, allow_unreferenced=False) -> dict:
    """Per-channel rotary flags; refuses channels without sensors or (unless
    allowed) not referenced."""
    rot = {}
    for ch in channels:
        cfg = b.axis_config(ch)
        if not cfg.sensor_present:
            raise RasterAborted(f"channel {'ABC'[ch]} has no sensor; raster needs closed loop")
        if not allow_unreferenced and not b.referenced(ch):
            raise RasterAborted(f"channel {'ABC'[ch]} is not referenced "
                                "(reference it, or pass --allow-unreferenced)")
        rot[ch] = cfg.rotary
    return rot


def run_raster(b: Backend, fast: Line, slow: Line, *, dwell: float = 0.0,
               settle: float = 0.0, hold_ms: int = 0, trigger: Trigger | None = None,
               log_path: str | None = None, tolerance: float | None = None,
               move_timeout: float = 30.0, poll: float = 0.002,
               stop_event: threading.Event | None = None,
               allow_unreferenced: bool = False, progress=None) -> RasterResult:
    if fast.channel == slow.channel:
        raise ValueError("fast and slow axes must be different channels")
    plan = serpentine(fast, slow)
    trigger = trigger or Trigger()
    stop_event = stop_event or threading.Event()
    rot = check_ready(b, (fast.channel, slow.channel), allow_unreferenced)
    chans = (fast.channel, slow.channel)
    max_err = {ch: 0.0 for ch in chans}
    t0 = time.monotonic()
    done = 0
    logf = open(log_path, "w", newline="") if log_path else None
    writer = None
    if logf:
        writer = csv.writer(logf)
        writer.writerow(["index", "row", "col", "fast_target", "slow_target",
                         "fast_pos", "slow_pos", "t_move", "t_settled", "t_trigger"])
    current = {ch: None for ch in chans}     # last commanded target per channel

    def wait_done(moving, deadline):
        while True:
            if stop_event.is_set():
                raise RasterAborted("stopped by request")
            busy = [ch for ch in moving if b.status(ch) in P.BUSY]
            if not busy:
                return
            if time.monotonic() > deadline:
                raise RasterAborted(f"move timed out on channel(s) "
                                    f"{', '.join('ABC'[c] for c in busy)}")
            time.sleep(poll)

    try:
        for pt in plan:
            if stop_event.is_set():
                raise RasterAborted("stopped by request")
            t_move = time.monotonic() - t0
            moving = []
            for ch, target in ((fast.channel, pt.fast), (slow.channel, pt.slow)):
                if current[ch] != target:
                    b.move_abs(ch, target, hold_ms, rot[ch])
                    current[ch] = target
                    moving.append(ch)
            if moving:
                time.sleep(min(poll, 0.002))   # let the controller report busy
                wait_done(moving, time.monotonic() + move_timeout)
            if settle > 0:
                time.sleep(settle)
            t_settled = time.monotonic() - t0
            pos = {ch: b.position(ch, rot[ch]) for ch in chans}
            for ch, target in ((fast.channel, pt.fast), (slow.channel, pt.slow)):
                err = abs(pos[ch] - target)
                max_err[ch] = max(max_err[ch], err)
                if tolerance is not None and err > tolerance:
                    raise RasterAborted(f"point {pt.index}: channel {'ABC'[ch]} at "
                                        f"{pos[ch]:.1f}, target {target:.1f} "
                                        f"(error {err:.1f} > tolerance {tolerance:g})")
            t_trig = time.monotonic() - t0
            trigger.fire()
            if writer:
                writer.writerow([pt.index, pt.row, pt.col, f"{pt.fast:.1f}", f"{pt.slow:.1f}",
                                 f"{pos[fast.channel]:.1f}", f"{pos[slow.channel]:.1f}",
                                 f"{t_move:.4f}", f"{t_settled:.4f}", f"{t_trig:.4f}"])
            done += 1
            if progress:
                progress(pt, done, len(plan), time.monotonic() - t0)
            if dwell > 0:
                end = time.monotonic() + dwell
                while time.monotonic() < end:
                    if stop_event.is_set():
                        raise RasterAborted("stopped by request")
                    time.sleep(min(0.01, max(0.0, end - time.monotonic())))
        return RasterResult(done, time.monotonic() - t0, max_err)
    except (RasterAborted, KeyboardInterrupt, P.CommandError, ConnectionLost) as e:
        for ch in chans:
            try:
                b.stop(ch)
            except Exception:
                pass
        reason = "interrupted" if isinstance(e, KeyboardInterrupt) else str(e)
        return RasterResult(done, time.monotonic() - t0, max_err, aborted=reason)
    finally:
        if logf:
            logf.close()


def summarize(fast: Line, slow: Line) -> str:
    nf, ns = len(fast.points), len(slow.points)
    return (f"{nf} x {ns} = {nf * ns} points; fast {'ABC'[fast.channel]} "
            f"{fast.start:g}..{fast.end:g} step {fast.step:g}, slow {'ABC'[slow.channel]} "
            f"{slow.start:g}..{slow.end:g} step {slow.step:g}")


def estimate_seconds(fast: Line, slow: Line, speed_um_s: float, dwell: float,
                     settle: float, overhead: float = 0.01) -> float:
    """Rough time: moves at constant speed plus per-point overheads."""
    nf, ns = len(fast.points), len(slow.points)
    move = (nf - 1) * ns * abs(fast.step) + (ns - 1) * abs(slow.step)
    return move / speed_um_s + nf * ns * (dwell + settle + overhead)

