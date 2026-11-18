"""Fly scanning: the fast axis moves continuously along each row.

The SCU has no velocity setting. Speed in a long closed-loop move is set by
the closed-loop maximum step frequency (speed ~ frequency x step length, and
step length depends on positioner, load and amplitude). So:

  * speed is *calibrated*: test moves at a trial frequency, measure the
    speed from fast position sampling, scale the frequency, repeat;
  * every row is *recorded*: the fast axis is polled during the move (about
    1 kHz on the wire backend with a 1 ms FTDI latency timer), the speed and
    its uniformity are fitted, and the time each grid point is crossed is
    logged.

The trigger fires at the start of each row's move (one per row). With a
detector free-running at a fixed frame rate from that trigger, frame k is
at t_trigger + k / rate, and the logs map that time to a position. Speed
errors become a known, recorded position error, not an unknown one.

Rows run serpentine. A run-up/run-down distance can be added so the grid
lies in the constant-speed part of the move. The slow axis steps between
rows, and the fast axis's closed-loop frequency is restored afterwards.
"""

from __future__ import annotations

import csv
import threading
import time
from dataclasses import dataclass, field

from . import protocol as P
from .backend import Backend, ConnectionLost
from .raster import Line, RasterAborted, Trigger, check_ready

F_MIN, F_MAX = P.CL_FREQUENCY_RANGE


@dataclass
class Fit:
    speed: float      # um/s (or mdeg/s), always positive
    rms: float        # rms deviation from the straight-line fit
    max_dev: float
    n: int


def fit_speed(samples, lo: float, hi: float) -> Fit | None:
    """Least-squares straight line through samples with lo <= pos <= hi."""
    lo, hi = min(lo, hi), max(lo, hi)
    pts = [(t, x) for t, x in samples if lo <= x <= hi]
    n = len(pts)
    if n < 3:
        return None
    mt = sum(t for t, _ in pts) / n
    mx = sum(x for _, x in pts) / n
    stt = sum((t - mt) ** 2 for t, _ in pts)
    if stt == 0:
        return None
    slope = sum((t - mt) * (x - mx) for t, x in pts) / stt
    res = [x - (mx + slope * (t - mt)) for t, x in pts]
    rms = (sum(r * r for r in res) / n) ** 0.5
    return Fit(abs(slope), rms, max(abs(r) for r in res), n)


def crossing_times(samples, targets):
    """For each target position, the interpolated time it was first crossed
    (None if never reached)."""
    out = []
    for x in targets:
        found = None
        for (t0, p0), (t1, p1) in zip(samples, samples[1:]):
            if p0 == x:
                found = t0
                break
            if (p0 - x) * (p1 - x) < 0 or p1 == x:
                found = t0 + (t1 - t0) * (x - p0) / (p1 - p0) if p1 != p0 else t1
                break
        out.append(found)
    return out


def _wait(b, chans, timeout, stop_event, poll=0.002):
    deadline = time.monotonic() + timeout
    time.sleep(0.002)
    while any(b.status(ch) in P.BUSY for ch in chans):
        if stop_event is not None and stop_event.is_set():
            raise RasterAborted("stopped by request")
        if time.monotonic() > deadline:
            raise RasterAborted("move timed out")
        time.sleep(poll)


def sample_move(b: Backend, ch: int, rot: bool, target: float, t0: float, *,
                timeout: float = 120.0, stop_event=None, status_every: int = 8,
                interval: float = 0.0):
    """Command an absolute move and sample position until it completes.
    Returns (t_command, samples[(t, pos)]) with times relative to t0. Each
    sample is stamped at the midpoint of its request/reply exchange.
    interval > 0 spaces samples at least that far apart (seconds)."""
    t_cmd = time.monotonic() - t0
    b.move_abs(ch, target, 0, rot)
    samples = []
    deadline = time.monotonic() + timeout
    i = 0
    next_t = time.monotonic()
    while True:
        if interval > 0:
            wait = next_t - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            next_t += interval
        ta = time.monotonic()
        p = b.position(ch, rot)
        tb = time.monotonic()
        samples.append(((ta + tb) / 2 - t0, p))
        i += 1
        if i % status_every == 0 and b.status(ch) not in P.BUSY:
            break
        if stop_event is not None and stop_event.is_set():
            raise RasterAborted("stopped by request")
        if time.monotonic() > deadline:
            raise RasterAborted("fly move timed out")
    ta = time.monotonic()
    p = b.position(ch, rot)
    samples.append(((ta + time.monotonic()) / 2 - t0, p))
    return t_cmd, samples


REACH_TOLERANCE = 10.0   # um/mdeg: a finished move further than this from its
                         # target did not get there (end stop, stall, overload)


def check_reached(b: Backend, ch: int, rot: bool, target: float, what: str) -> float:
    pos = b.position(ch, rot)
    if abs(pos - target) > REACH_TOLERANCE:
        raise RasterAborted(f"{what}: stopped at {pos:.1f}, target {target:.1f} "
                            "(end stop? check the range is inside the travel)")
    return pos


@dataclass
class SpeedMeasurement:
    fit: Fit | None
    samples: int
    interval_ms: float       # median time between samples


def measure_speed(b: Backend, ch: int, rot: bool, a: float, z: float, freq: int,
                  stop_event=None) -> SpeedMeasurement:
    """Set the closed-loop frequency, move to a, then sample a move to z.
    Speed is fitted over the central 60 % of the span. Raises RasterAborted
    if either move fails to reach its target."""
    b.set_cl_max_frequency(ch, int(freq))
    b.move_abs(ch, a, 0, rot)
    _wait(b, [ch], 120, stop_event)
    check_reached(b, ch, rot, a, "move to start")
    _, s = sample_move(b, ch, rot, z, time.monotonic(), stop_event=stop_event)
    check_reached(b, ch, rot, z, f"test move at {freq} Hz")
    m = 0.2 * (z - a)
    gaps = sorted(t1 - t0 for (t0, _), (t1, _) in zip(s, s[1:]))
    med = gaps[len(gaps) // 2] * 1000 if gaps else float("nan")
    return SpeedMeasurement(fit_speed(s, a + m, z - m), len(s), med)


def calibrate_frequency(b: Backend, ch: int, rot: bool, a: float, z: float,
                        target: float, f0: int, tol: float = 0.05, max_iter: int = 5,
                        log=print, stop_event=None):
    """Find the closed-loop frequency giving `target` speed on a..z,
    assuming speed is roughly proportional to frequency. Leaves the axis at
    a or z. Returns (frequency, measured Fit)."""
    f = int(min(max(f0, F_MIN), F_MAX))
    fit = None
    ends = [(a, z), (z, a)]
    for i in range(max_iter):
        s, e = ends[i % 2]
        meas = measure_speed(b, ch, rot, s, e, f, stop_event)
        fit = meas.fit
        if fit is None:
            raise RasterAborted(
                f"calibration: only {meas.samples} samples ({meas.interval_ms:.1f} ms apart) "
                "in the move; the row is too short for this speed, or sampling is slow "
                "(FTDI latency timer?)")
        log(f"  calibrate: {f} Hz -> {fit.speed:.1f}/s (target {target:.1f}/s, "
            f"rms {fit.rms:.2f})")
        if abs(fit.speed / target - 1) <= tol:
            return f, fit
        f_new = int(round(min(max(f * target / fit.speed, F_MIN), F_MAX)))
        if f_new == f:
            log(f"  calibrate: limited to {f} Hz; best speed {fit.speed:.1f}/s")
            return f, fit
        f = f_new
    log(f"  calibrate: not within {tol:.0%} after {max_iter} tries; using {f} Hz")
    return f, fit


@dataclass
class FlyResult:
    rows_done: int
    elapsed: float
    frequency: int
    target_speed: float | None
    row_fits: list = field(default_factory=list)   # Fit | None per row
    aborted: str | None = None


def run_fly(b: Backend, fast: Line, slow: Line, *, frame_rate: float | None = None,
            speed: float | None = None, cl_freq: int | None = None, run_up: float = 0.0,
            settle: float = 0.0, trigger: Trigger | None = None,
            log_path: str | None = None, trajectory_path: str | None = None,
            stop_event: threading.Event | None = None, allow_unreferenced: bool = False,
            progress=None, log=print, calibration_tolerance: float = 0.05,
            sample_interval: float = 0.0) -> FlyResult:
    fpts, spts = fast.points, slow.points
    if fast.channel == slow.channel:
        raise ValueError("fast and slow axes must be different channels")
    if len(fpts) < 2:
        raise ValueError("fly scans need at least two points on the fast axis")
    if run_up < 0:
        raise ValueError("run-up must be >= 0")
    trigger = trigger or Trigger()
    stop_event = stop_event or threading.Event()
    rot = check_ready(b, (fast.channel, slow.channel), allow_unreferenced)
    fch, sch = fast.channel, slow.channel
    orig_freq = b.axis_config(fch).cl_max_frequency
    target = speed if speed else (frame_rate * fast.step if frame_rate else None)
    d0 = 1.0 if fpts[-1] > fpts[0] else -1.0

    def row_ends(r):
        d = d0 if r % 2 == 0 else -d0
        first, last = (fpts[0], fpts[-1]) if r % 2 == 0 else (fpts[-1], fpts[0])
        return first - d * run_up, last + d * run_up, first, last

    t0 = time.monotonic()
    result = FlyResult(0, 0.0, cl_freq or orig_freq or 0, target)
    pf = open(log_path, "w", newline="") if log_path else None
    tf = open(trajectory_path, "w", newline="") if trajectory_path else None
    pw = csv.writer(pf) if pf else None
    tw = csv.writer(tf) if tf else None
    if pw:
        pw.writerow(["index", "row", "col", "fast_target", "slow_target", "t_trigger",
                     "t_cross", "dt_cross", "row_speed", "row_rms"])
    if tw:
        tw.writerow(["row", "t", "fast_pos"])
    index = 0
    try:
        if cl_freq is None and target:
            a, z, _, _ = row_ends(0)
            freq, _ = calibrate_frequency(b, fch, rot[fch], a, z, target,
                                          orig_freq or 5000, calibration_tolerance,
                                          log=log, stop_event=stop_event)
        else:
            freq = cl_freq or orig_freq
        result.frequency = int(freq)
        b.set_cl_max_frequency(fch, int(freq))

        for r, s in enumerate(spts):
            start, end, first, last = row_ends(r)
            # position: fast to run-up start, slow to the row (together)
            b.move_abs(fch, start, 0, rot[fch])
            b.move_abs(sch, s, 0, rot[sch])
            _wait(b, [fch, sch], 120, stop_event)
            check_reached(b, fch, rot[fch], start, f"row {r} start")
            check_reached(b, sch, rot[sch], s, f"row {r} slow position")
            if settle > 0:
                time.sleep(settle)
            t_trig = time.monotonic() - t0
            trigger.fire()
            _, samples = sample_move(b, fch, rot[fch], end, t0, stop_event=stop_event,
                                     interval=sample_interval)
            row_end_pos = samples[-1][1]
            fit = fit_speed(samples, first, last)
            result.row_fits.append(fit)
            row_pts = fpts if r % 2 == 0 else fpts[::-1]
            cols = range(len(fpts)) if r % 2 == 0 else range(len(fpts) - 1, -1, -1)
            times = crossing_times(samples, row_pts)
            if tw:
                for t, x in samples:
                    tw.writerow([r, f"{t:.5f}", f"{x:.1f}"])
            if pw:
                for c, x, tc in zip(cols, row_pts, times):
                    pw.writerow([index, r, c, f"{x:.1f}", f"{s:.1f}", f"{t_trig:.5f}",
                                 "" if tc is None else f"{tc:.5f}",
                                 "" if tc is None else f"{tc - t_trig:.5f}",
                                 "" if fit is None else f"{fit.speed:.2f}",
                                 "" if fit is None else f"{fit.rms:.2f}"])
                    index += 1
            result.rows_done += 1
            if progress:
                progress(r, len(spts), fit, times, t_trig, time.monotonic() - t0)
            missed = sum(t is None for t in times)
            if abs(row_end_pos - end) > REACH_TOLERANCE or missed:
                what = (f"{missed} grid points not reached" if missed
                        else "all grid points crossed but the run-down was cut short")
                raise RasterAborted(f"row {r}: stopped at {row_end_pos:.1f}, target "
                                    f"{end:.1f}: {what} (end stop? check run-up and range)")
        return result
    except (RasterAborted, KeyboardInterrupt, P.CommandError, ConnectionLost) as e:
        for ch in (fch, sch):
            try:
                b.stop(ch)
            except Exception:
                pass
        result.aborted = "interrupted" if isinstance(e, KeyboardInterrupt) else str(e)
        return result
    finally:
        result.elapsed = time.monotonic() - t0
        if orig_freq:
            try:
                b.set_cl_max_frequency(fch, int(orig_freq))
            except Exception:
                pass
        for f in (pf, tf):
            if f:
                f.close()
