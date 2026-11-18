#!/usr/bin/env python3
"""Drive a real SCU through SmarAct's library to capture wire traffic.

Run under the ftrace shim so every byte is logged:

    make -C tools/ftrace
    SCU_FTRACE=scu-capture.trace LD_PRELOAD=$PWD/tools/ftrace/libftrace.so \\
        python3 tools/ftrace/capture_session.py --lib /opt/smaract/lib/libscu3dcontrol.so
    python3 tools/ftrace/decode.py scu-capture.trace

Each step is announced with a "# STEP" marker appended to the same trace
file, so the decoded log shows which call produced which bytes.

Safety on a mounted stage:
  * closed-loop moves are small and relative (default +/-20 um or mdeg)
    and return to the start position;
  * open-loop steps default to 50 steps each way;
  * settings stored in the controller's non-volatile memory (sensor type,
    safe direction) are written back with their *current* value, so the
    command is captured without changing anything;
  * calibration, referencing and end-stop moves travel several mm and only
    run with --calibrate / --reference / --end-stop.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from scu_ioc import protocol as P  # noqa: E402
from scu_ioc.dll import DllBackend  # noqa: E402

TRACE = os.environ.get("SCU_FTRACE")


def mark(text: str) -> None:
    """Annotate the trace (same file the shim appends to) and the console."""
    print(f"== {text}", flush=True)
    if TRACE:
        with open(TRACE, "a") as f:
            f.write(f"# STEP {text}\n")


def wait(b, ch, timeout=30.0):
    t0 = time.monotonic()
    while b.status(ch) in P.BUSY:
        if time.monotonic() - t0 > timeout:
            b.stop(ch)
            raise TimeoutError(f"channel {ch} still busy; stopped")
        time.sleep(0.05)


def attempt(label, fn):
    """Run fn, logging a controller rejection instead of aborting."""
    try:
        r = fn()
        print(f"   {label}: {r}")
        return r
    except P.CommandError as e:
        print(f"   {label}: rejected: {e}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--lib", required=True, help="path to libscu3dcontrol.so")
    ap.add_argument("--device-id", type=int)
    ap.add_argument("--channels", default="0,1,2", help="channels to exercise (default 0,1,2)")
    ap.add_argument("--travel", type=float, default=20.0, help="closed-loop test move, um/mdeg")
    ap.add_argument("--steps", type=int, default=50, help="open-loop test steps")
    ap.add_argument("--calibrate", action="store_true", help="also calibrate (moves several mm)")
    ap.add_argument("--reference", action="store_true", help="also move to reference (moves)")
    ap.add_argument("--end-stop", action="store_true", help="also drive to the safe-direction end stop")
    a = ap.parse_args(argv)
    if not TRACE:
        print("note: SCU_FTRACE not set; markers go to the console only", file=sys.stderr)

    b = DllBackend(device_id=a.device_id, dll_path=a.lib)
    mark("connect")
    info = b.connect()
    print(f"   {info}")
    chans = [int(c) for c in a.channels.split(",") if int(c) < info.channels]

    mark("system properties")
    attempt("temperature", b.temperature)
    attempt("hv", b.hv)

    for ch in chans:
        mark(f"ch{ch} configuration")
        cfg = b.axis_config(ch)
        print(f"   {cfg}")
        attempt("status", lambda: b.status(ch))
        attempt("referenced", lambda: b.referenced(ch))

        mark(f"ch{ch} rewrite settings with current values")
        attempt("set_cl_max_frequency", lambda: b.set_cl_max_frequency(ch, cfg.cl_max_frequency))
        attempt("set_sensor_type", lambda: b.set_sensor_type(ch, cfg.sensor_type))
        if cfg.safe_direction is not None:
            attempt("set_safe_direction", lambda: b.set_safe_direction(ch, bool(cfg.safe_direction)))

        mark(f"ch{ch} open-loop steps +{a.steps} / -{a.steps}")
        attempt("step+", lambda: b.step(ch, a.steps, 600, 1000))
        wait(b, ch)
        attempt("step-", lambda: b.step(ch, -a.steps, 600, 1000))
        wait(b, ch)

        if not cfg.sensor_present:
            mark(f"ch{ch} no sensor: expect rejections")
            attempt("position", lambda: b.position(ch, False))
            attempt("move_rel", lambda: b.move_rel(ch, a.travel, 0, False))
            continue

        rot = cfg.rotary
        mark(f"ch{ch} closed loop relative +/-{a.travel}")
        start = attempt("position", lambda: b.position(ch, rot))
        attempt("move_rel+", lambda: b.move_rel(ch, a.travel, 0, rot))
        wait(b, ch)
        attempt("position", lambda: b.position(ch, rot))
        attempt("move_rel-", lambda: b.move_rel(ch, -a.travel, 0, rot))
        wait(b, ch)

        mark(f"ch{ch} closed loop absolute with 300 ms hold")
        if start is not None:
            attempt("move_abs", lambda: b.move_abs(ch, start + a.travel / 2, 300, rot))
            wait(b, ch)
            attempt("status (holding?)", lambda: b.status(ch))
            time.sleep(0.4)
            attempt("move_abs back", lambda: b.move_abs(ch, start, 0, rot))
            wait(b, ch)

        mark(f"ch{ch} stop during a move")
        attempt("move_rel", lambda: b.move_rel(ch, a.travel, 0, rot))
        time.sleep(0.02)
        attempt("stop", lambda: b.stop(ch))
        attempt("status", lambda: b.status(ch))
        if start is not None:
            attempt("move_abs back", lambda: b.move_abs(ch, start, 0, rot))
            wait(b, ch)

        mark(f"ch{ch} wrong-kind call: expect wrong sensor type")
        attempt("position as other kind", lambda: b.position(ch, not rot))

        mark(f"ch{ch} channel property: movement type")
        attempt("movement", lambda: b._movement_type(ch))

        if a.calibrate:
            mark(f"ch{ch} calibrate")
            attempt("calibrate", lambda: b.calibrate(ch))
            wait(b, ch, 60)
        if a.reference:
            mark(f"ch{ch} reference")
            attempt("reference", lambda: b.reference(ch, 0, False))
            wait(b, ch, 120)
            attempt("referenced", lambda: b.referenced(ch))
        if a.end_stop:
            mark(f"ch{ch} move to end stop")
            attempt("end stop", lambda: b._call("SA_MoveToEndStop_S", b.index, ch,
                                                   cfg.safe_direction or 0, 0, 0))
            wait(b, ch, 120)

    mark("release")
    b.close()
    print("done; decode with tools/ftrace/decode.py")


if __name__ == "__main__":
    main()
