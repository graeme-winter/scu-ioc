"""`scu`: small command-line tool for bench work, over either backend.

    scu status
    scu --lib /opt/smaract/lib/libscu3dcontrol.so status
    scu --backend wire --port /dev/ttyUSB0 status       # stock firmware, no SDK
    scu --backend ascii --port /dev/ttyUSB0 rel A 25
    scu ... step B -500 --amp 60 --freq 1000
    scu ... reference A --auto-zero
    scu list
    scu --backend ascii --port /dev/ttyUSB0 raw GP0

Close the IOC (or the SmarAct tools) first: only one client can own the
controller.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time

from . import protocol as P
from .backend import Backend, ConnectionLost


def parse_channel(s: str) -> int:
    s = s.strip().upper()
    if s in ("A", "B", "C"):
        return "ABC".index(s)
    if s in ("0", "1", "2"):
        return int(s)
    raise argparse.ArgumentTypeError(f"channel must be A/B/C or 0/1/2, not {s!r}")


def wait(b: Backend, ch: int, timeout: float) -> int:
    time.sleep(0.05)
    t0 = time.monotonic()
    while True:
        s = b.status(ch)
        if s not in P.BUSY:
            return s
        if time.monotonic() - t0 > timeout:
            b.stop(ch)
            raise TimeoutError(f"still {P.STATUS_NAMES[s]} after {timeout} s; stopped")
        time.sleep(0.05)


def describe(b: Backend, ch: int) -> str:
    cfg = b.axis_config(ch)
    parts = [f"{'ABC'[ch]}: {P.STATUS_NAMES[b.status(ch)]}"]
    if cfg.sensor_present:
        units = "mdeg" if cfg.rotary else "um"
        parts.append(f"sensor {P.sensor_symbol(cfg.sensor_type)}({cfg.sensor_type})")
        parts.append(f"pos {b.position(ch, cfg.rotary):.1f} {units}")
        parts.append("referenced" if b.referenced(ch) else "not referenced")
        parts.append(f"CL max {cfg.cl_max_frequency} Hz")
    else:
        parts.append("no sensor (open loop only)")
    return ", ".join(parts)


def _raster_lines(a):
    from .raster import Line
    out = []
    for spec in (a.fast, a.slow):
        ch = parse_channel(spec[0])
        start, end, step = (float(x) for x in spec[1:])
        line = Line(ch, start, end, abs(step))
        line.points                       # validates
        out.append(line)
    if out[0].channel == out[1].channel:
        raise ValueError("fast and slow axes must differ")
    return out


def _run_raster(a, b, fast, slow) -> int:
    from .raster import RasterAborted, make_trigger, run_raster
    trig = make_trigger(a.trigger, a.pulse)
    print(f"trigger: {trig.describe()}")
    nf = len(fast.points)

    def progress(pt, done, total, elapsed):
        if a.quiet or (done % nf and done != total):
            return
        eta = elapsed / done * (total - done)
        print(f"  row {pt.row + 1}/{total // nf}: {done}/{total} points, "
              f"{elapsed:.1f} s, {elapsed / done * 1000:.0f} ms/point, ETA {eta:.0f} s",
              flush=True)

    try:
        r = run_raster(b, fast, slow, dwell=a.dwell, settle=a.settle, hold_ms=a.hold,
                       trigger=trig, log_path=a.log, tolerance=a.tolerance,
                       allow_unreferenced=a.allow_unreferenced, progress=progress)
    except RasterAborted as e:
        print(f"not started: {e}", file=sys.stderr)
        return 1
    finally:
        trig.close()
    errs = ", ".join(f"{'ABC'[c]} {v:.1f}" for c, v in r.max_error.items())
    print(f"{r.points_done} points in {r.elapsed:.1f} s; max |position - target|: {errs}")
    if a.log:
        print(f"log: {a.log}")
    if r.aborted:
        print(f"ABORTED: {r.aborted} (axes stopped)", file=sys.stderr)
        return 3
    return 0


def _run_fly(a, b, fast, slow) -> int:
    from .fly import run_fly
    from .raster import RasterAborted, make_trigger
    for opt in ("dwell", "tolerance"):
        if getattr(a, opt):
            print(f"note: --{opt} is ignored in fly mode", file=sys.stderr)
    trig = make_trigger(a.trigger, a.pulse)
    print(f"trigger: {trig.describe()} (start of each row)")

    def progress(r, nrows, fit, times, t_trig, elapsed):
        if a.quiet:
            return
        if fit is None:
            print(f"  row {r + 1}/{nrows}: too few samples in the grid region to fit speed")
            return
        hit = [t for t in times if t is not None]
        first = f"{(hit[0] - t_trig) * 1000:.1f} ms" if hit else "never"
        print(f"  row {r + 1}/{nrows}: {fit.speed:.1f} um/s, rms {fit.rms:.2f} um "
              f"(max {fit.max_dev:.2f}), {fit.n} samples, first point at {first} "
              f"after trigger, {len(hit)}/{len(times)} points crossed, {elapsed:.1f} s",
              flush=True)

    try:
        r = run_fly(b, fast, slow, frame_rate=a.frame_rate, speed=a.fly_speed,
                    cl_freq=a.fly_freq, run_up=a.run_up, settle=a.settle, trigger=trig,
                    log_path=a.log, trajectory_path=a.trajectory,
                    allow_unreferenced=a.allow_unreferenced, progress=progress,
                    sample_interval=a.sample_ms / 1000.0)
    except (RasterAborted, ValueError) as e:
        print(f"not started: {e}", file=sys.stderr)
        return 1
    finally:
        trig.close()
    fits = [f for f in r.row_fits if f]
    if fits:
        sp = [f.speed for f in fits]
        print(f"{r.rows_done} rows in {r.elapsed:.1f} s at {r.frequency} Hz; speed "
              f"{min(sp):.1f}..{max(sp):.1f} um/s (mean {sum(sp) / len(sp):.1f}); "
              f"worst row rms {max(f.rms for f in fits):.2f} um")
        if r.target_speed:
            mean = sum(sp) / len(sp)
            print(f"mean speed is {100 * (mean / r.target_speed - 1):+.1f}% from target")
    for path in (a.log, a.trajectory):
        if path:
            print(f"log: {path}")
    if r.aborted:
        print(f"ABORTED: {r.aborted} (axes stopped)", file=sys.stderr)
        return 3
    return 0


def _run_speed(a, b) -> int:
    from .fly import measure_speed
    from .raster import RasterAborted
    ch = a.channel
    cfg = b.axis_config(ch)
    if not cfg.sensor_present:
        print("speed needs a sensor", file=sys.stderr)
        return 1
    orig = cfg.cl_max_frequency
    lo, hi = a.center - a.distance / 2, a.center + a.distance / 2
    print(f"{'ABC'[ch]}: test moves between {lo:.1f} and {hi:.1f} "
          f"(started at {b.position(ch, cfg.rotary):.1f})")
    print(f"{'freq Hz':>8} {'speed /s':>10} {'per Hz':>8} {'rms':>6} {'samples':>8} {'every':>8}")
    rc = 0
    try:
        for i, f in enumerate(a.freq):
            s, e = (lo, hi) if i % 2 == 0 else (hi, lo)
            m = measure_speed(b, ch, cfg.rotary, s, e, f)
            every = f"{m.interval_ms:.1f} ms"
            if m.fit is None:
                print(f"{f:8d} {'-':>10} {'-':>8} {'-':>6} {m.samples:8d} {every:>8}"
                      "  too few samples: use a longer --distance")
            else:
                print(f"{f:8d} {m.fit.speed:10.1f} {1000 * m.fit.speed / f:7.1f}n "
                      f"{m.fit.rms:6.2f} {m.samples:8d} {every:>8}")
    except RasterAborted as e:
        print(f"stopped: {e}", file=sys.stderr)
        b.stop(ch)
        rc = 3
    finally:
        b.set_cl_max_frequency(ch, orig)
    return rc


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="SmarAct SCU bench tool")
    ap.add_argument("--backend", choices=["sdk", "dll", "wire", "ascii"], default="sdk",
                    help="sdk (alias dll): SmarAct SCU library; wire: stock firmware "
                         "over its serial port, no SDK; ascii: ASCII firmware / RS-232")
    ap.add_argument("--port", help="wire/ascii: serial device or pyserial URL")
    ap.add_argument("--baud", type=int, help="default 500000 (wire), 9600 (ascii)")
    ap.add_argument("--device-id", type=int)
    ap.add_argument("--lib", "--dll", dest="dll",
                    help="sdk: path to SCU3DControl.dll / libscu3dcontrol.so")
    ap.add_argument("--timeout", type=float, default=60.0, help="motion timeout, s")
    ap.add_argument("--no-wait", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="sdk: list connected SCU device IDs")
    sub.add_parser("status")
    for name, helptext in (("move", "closed-loop absolute (um / mdeg)"),
                           ("rel", "closed-loop relative (um / mdeg)")):
        p = sub.add_parser(name, help=helptext)
        p.add_argument("channel", type=parse_channel)
        p.add_argument("value", type=float)
        p.add_argument("--hold", type=int, default=0, help="hold time, ms")
    p = sub.add_parser("step", help="open-loop steps (signed)")
    p.add_argument("channel", type=parse_channel)
    p.add_argument("steps", type=int)
    p.add_argument("--amp", type=float, default=100.0, help="amplitude, V (15-100)")
    p.add_argument("--freq", type=int, default=1000, help="frequency, Hz")
    p = sub.add_parser("calibrate", help="sensor calibration (moves several mm!)")
    p.add_argument("channel", type=parse_channel)
    p = sub.add_parser("reference", help="move to reference")
    p.add_argument("channel", type=parse_channel)
    p.add_argument("--auto-zero", action="store_true")
    p = sub.add_parser("zero", help="define current position as zero")
    p.add_argument("channel", type=parse_channel)
    p = sub.add_parser("sensor-type", help="set sensor type code (stored in NV memory)")
    p.add_argument("channel", type=parse_channel)
    p.add_argument("code", type=int)
    p = sub.add_parser("stop")
    p.add_argument("channel", type=parse_channel, nargs="?")
    p = sub.add_parser("raster", help="serpentine step-and-shoot raster (see --help)",
                       description="Serpentine step-and-shoot raster. At each point: "
                       "absolute move, wait for target reached, settle, read back, "
                       "trigger, dwell. Positions in um (mdeg for rotary axes).")
    p.add_argument("--fast", nargs=4, required=True, metavar=("AXIS", "START", "END", "STEP"),
                   help="fast axis (A/B/C) and its points, e.g. --fast A -12000 12000 50")
    p.add_argument("--slow", nargs=4, required=True, metavar=("AXIS", "START", "END", "STEP"),
                   help="slow axis, one row per point")
    p.add_argument("--dwell", type=float, default=0.0, help="time at each point after the trigger, s")
    p.add_argument("--settle", type=float, default=0.0,
                   help="extra wait after target reached, before read-back and trigger, s")
    p.add_argument("--hold", type=int, default=0,
                   help="hold time for each move, ms (keeps the controller correcting)")
    p.add_argument("--trigger", default="none",
                   help="none | serial:PORT[:dtr|rts][:low|high] | ca:PV[=VALUE]")
    p.add_argument("--pulse", type=float, default=1.0, help="serial trigger pulse width, ms")
    p.add_argument("--log", help="CSV log of every point (targets, positions, times)")
    p.add_argument("--tolerance", type=float,
                   help="abort if a settled position is further than this from target")
    p.add_argument("--allow-unreferenced", action="store_true")
    p.add_argument("--speed", type=float,
                   help="assumed closed-loop speed, um/s, for the time estimate")
    p.add_argument("--dry-run", action="store_true", help="print the plan and exit")
    p.add_argument("--quiet", action="store_true", help="no per-row progress")
    g = p.add_argument_group("fly mode (fast axis moves continuously; one trigger per row)")
    g.add_argument("--fly", action="store_true", help="fly the fast axis")
    g.add_argument("--frame-rate", type=float,
                   help="detector frame rate, Hz: target speed = fast STEP x rate")
    g.add_argument("--fly-speed", type=float, help="target fast-axis speed, um/s")
    g.add_argument("--fly-freq", type=int,
                   help="use this closed-loop frequency, Hz (no calibration)")
    g.add_argument("--run-up", type=float, default=0.0,
                   help="extra travel before/after each row, um, so the grid is at full speed")
    g.add_argument("--trajectory", help="CSV of every fast-axis position sample")
    g.add_argument("--sample-ms", type=float, default=0.0,
                   help="minimum interval between position samples, ms "
                        "(default: as fast as the link allows, ~1 kHz)")

    p = sub.add_parser("speed", help="measure closed-loop speed vs frequency")
    p.add_argument("channel", type=parse_channel)
    p.add_argument("--freq", type=int, nargs="+", required=True, help="frequencies to try, Hz")
    p.add_argument("--distance", type=float, default=2000.0,
                   help="test move length, um, centred on --center (default 2000)")
    p.add_argument("--center", type=float, default=0.0,
                   help="centre of the test moves, um (default 0, the reference position)")

    p = sub.add_parser("raw", help="ascii: send one command, print the answer")
    p.add_argument("command")

    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.WARNING)

    if a.cmd == "raster":
        try:
            fast, slow = _raster_lines(a)
        except (ValueError, argparse.ArgumentTypeError) as e:
            ap.error(str(e))
        from .raster import estimate_seconds, summarize
        print(summarize(fast, slow))
        if a.speed:
            t = estimate_seconds(fast, slow, a.speed, a.dwell, a.settle)
            print(f"estimated time at {a.speed:g} um/s: {t / 60:.1f} min")
        if a.fly:
            target = a.fly_speed or (a.frame_rate * fast.step if a.frame_rate else None)
            nf = len(fast.points)
            if target:
                print(f"fly: target speed {target:g} um/s; {nf} grid points per row, "
                      f"{fast.step / target * 1000:.2f} ms apart")
            if a.frame_rate:
                print(f"detector: >= {nf} frames per trigger at {a.frame_rate:g} Hz, "
                      f"one trigger per row ({len(slow.points)} rows)")
        if a.dry_run:
            return 0

    if a.backend in ("sdk", "dll"):
        from .dll import DllBackend, available_devices
        if a.cmd == "list":
            ids = available_devices(a.dll)
            print("\n".join(map(str, ids)) if ids else "no SCU devices found")
            return 0
        b = DllBackend(a.device_id, a.dll)
    else:
        if not a.port:
            ap.error(f"--port is required for the {a.backend} backend")
        if a.cmd == "list":
            ap.error("list is only available with the sdk backend")
        if a.backend == "wire":
            from .wire_backend import WireBackend
            b = WireBackend(a.port, a.baud or 500000, device_id=a.device_id)
        else:
            from .ascii import AsciiBackend
            b = AsciiBackend(a.port, a.baud or 9600)

    try:
        info = b.connect()
        if a.cmd == "raw":
            if a.backend != "ascii":
                ap.error("raw needs the ascii backend")
            print(b._transact(a.command))
            return 0
        if a.cmd == "status":
            print(f"{info.model}  id {info.device_id}  firmware {info.firmware}"
                  f"  T={b.temperature()} C  HV={b.hv()} V")
            for ch in range(info.channels):
                print("  " + describe(b, ch))
            return 0
        if a.cmd == "raster":
            if a.fly:
                return _run_fly(a, b, fast, slow)
            return _run_raster(a, b, fast, slow)
        if a.cmd == "speed":
            return _run_speed(a, b)
        if a.cmd == "stop":
            for ch in ([a.channel] if a.channel is not None else range(info.channels)):
                b.stop(ch)
            return 0

        ch = a.channel
        rotary = b.axis_config(ch).rotary
        if a.cmd == "move":
            b.move_abs(ch, a.value, a.hold, rotary)
        elif a.cmd == "rel":
            b.move_rel(ch, a.value, a.hold, rotary)
        elif a.cmd == "step":
            b.step(ch, a.steps, int(round(a.amp * 10)), a.freq)
        elif a.cmd == "calibrate":
            b.calibrate(ch)
        elif a.cmd == "reference":
            b.reference(ch, 0, a.auto_zero)
        elif a.cmd == "zero":
            b.set_zero(ch)
        elif a.cmd == "sensor-type":
            b.set_sensor_type(ch, a.code)
        if not a.no_wait:
            try:
                wait(b, ch, a.timeout)
            except KeyboardInterrupt:
                b.stop(ch)
                print("interrupted: stopped", file=sys.stderr)
                return 130
        print(describe(b, ch))
        return 0
    except (P.CommandError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except (ConnectionLost, P.ProtocolError) as e:
        print(f"communication error: {e}", file=sys.stderr)
        return 2
    finally:
        b.close()


if __name__ == "__main__":
    sys.exit(main())
