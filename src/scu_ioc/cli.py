"""`scu`: small command-line tool for bench work, over either backend.

    scu --backend dll status
    scu --backend ascii --port /dev/ttyUSB0 rel A 25
    scu ... step B -500 --amp 60 --freq 1000
    scu ... reference A --auto-zero
    scu --backend dll list
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


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="SmarAct SCU bench tool")
    ap.add_argument("--backend", choices=["dll", "ascii"], default="dll")
    ap.add_argument("--port", help="ascii: serial device or pyserial URL")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--device-id", type=int)
    ap.add_argument("--dll")
    ap.add_argument("--timeout", type=float, default=60.0, help="motion timeout, s")
    ap.add_argument("--no-wait", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="dll: list connected SCU device IDs")
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
    p = sub.add_parser("raw", help="ascii: send one command, print the answer")
    p.add_argument("command")

    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.WARNING)

    if a.backend == "dll":
        from .dll import DllBackend, available_devices
        if a.cmd == "list":
            ids = available_devices(a.dll)
            print("\n".join(map(str, ids)) if ids else "no SCU devices found")
            return 0
        b = DllBackend(a.device_id, a.dll)
    else:
        from .ascii import AsciiBackend
        if not a.port:
            ap.error("--port is required for the ascii backend")
        if a.cmd == "list":
            ap.error("list is only available with the dll backend")
        b = AsciiBackend(a.port, a.baud)

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
