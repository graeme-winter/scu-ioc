"""Simulator for the SCU binary protocol (wire.py), served on a pty.

`scu-wire-sim --link /tmp/scu-wire` behaves like a stock-firmware SCU on
the far side of the FTDI UART. Uses the same crude motion model as the
ASCII simulator (sim.SimAxis). Built from the static protocol map, so it
also tests that map: SmarAct's own libscu3dcontrol.so, pointed at this
simulator through tools/ftrace/libfakeftd2xx.so, must accept its replies.
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
import tty

from . import protocol as P
from . import wire as W
from .sim import SimAxis

# SA error codes the firmware reports
E_INVALID_COMMAND = 128
E_NOT_SUPPORTED = 129
E_NO_SENSOR = 130
E_WRONG_SENSOR = 131
E_NOT_PROCESSABLE = 138
E_INVALID_CHANNEL = 6     # host-side in the SDK; used here for bad channels


class _Err(Exception):
    def __init__(self, ch, code):
        self.ch, self.code = ch, code


class WireSCU:
    def __init__(self, sensor_types=(21, 21, 21), device_id=604573714,
                 firmware=(5, 1, 192), clock=time.monotonic):
        self.axes = [SimAxis(t, clock) for t in sensor_types]
        self.device_id = device_id
        self.firmware = firmware
        self.mode = 0
        self.temperature = 31
        self.lock = threading.Lock()
        self.log = []          # (request frame, reply frames) for tests

    def _axis(self, ch, need_sensor=False):
        if not 0 <= ch < len(self.axes):
            raise _Err(ch, E_INVALID_CHANNEL)
        a = self.axes[ch]
        a.update()
        if need_sensor and not a.sensor_present:
            raise _Err(ch, E_NO_SENSOR)
        return a

    def handle(self, frame: bytes) -> list[bytes]:
        with self.lock:
            try:
                out = self._dispatch(frame)
            except _Err as e:
                out = [W.encode_reply(W.R_ERROR, e.ch & 0xFF, e.code)]
            self.log.append((frame, out))
            return out

    def _ack(self, ch):
        return [W.encode_reply(W.R_ACK, ch)]

    def _dispatch(self, frame: bytes) -> list[bytes]:
        hdr = frame[0]
        if hdr == W.HW_RESET:
            # library reads one byte, then waits for a reset-done packet
            return [b"\x00", W.encode_reply(W.R_RESET_DONE)]
        name, v = W.decode_request(frame)
        if name.startswith("unknown"):
            return [W.encode_reply(W.R_ERROR, frame[1] if len(frame) > 1 else 0,
                                   E_INVALID_COMMAND)]
        R = W.encode_reply

        if hdr == W.GET:
            sel, ch = v
            if sel == W.SEL_ID:
                return [R(W.R_ID, self.device_id)]
            if sel == W.SEL_FIRMWARE:
                ma, mi, up = self.firmware
                return [R(W.R_FIRMWARE, up & 0xFF, up >> 8, 0, mi, ma)]
            a = self._axis(ch)
            if sel == W.SEL_STATUS:
                return [R(W.R_STATUS, ch, a.status)]
            if sel == W.SEL_SENSOR_PRESENT:
                return [R(W.R_SENSOR_PRESENT, ch, int(a.sensor_present))]
            if sel == W.SEL_SENSOR_TYPE:
                return [R(W.R_SENSOR_TYPE, ch, a.sensor_type)]
            if sel == W.SEL_CL_FREQ:
                return [R(W.R_CL_FREQ, ch, a.cl_freq)]
            if sel == W.SEL_AMPLITUDE:
                return [R(W.R_AMPLITUDE, ch, a.amplitude)]
            a = self._axis(ch, need_sensor=True)
            if sel == W.SEL_POSITION:
                if a.rotary:
                    raise _Err(ch, E_WRONG_SENSOR)
                return [R(W.R_POSITION, ch, int(round(a.logical * 10)))]
            if sel == W.SEL_ANGLE:
                if not a.rotary:
                    raise _Err(ch, E_WRONG_SENSOR)
                return [R(W.R_ANGLE, ch, int(round(a.logical * 10)), 0)]
            if sel == W.SEL_SAFE_DIR:
                return [R(W.R_SAFE_DIR, ch, a.safe_dir)]
            if sel == W.SEL_PPK:
                return [R(W.R_PPK, ch, int(a.known))]
            raise _Err(ch, E_NOT_SUPPORTED)

        if hdr == W.SET_MODE:
            self.mode = v[0]
            return self._ack(v[0])
        if hdr == W.GET_SYSTEM_PROP:
            key = v[0]
            vals = {W.PROP_TEMPERATURE: self.temperature, W.PROP_VOLTAGE: 1000,
                    W.PROP_HW_VERSION: 7}
            if key not in vals:
                raise _Err(0xFF, E_NOT_SUPPORTED)
            return [R(W.R_SYSTEM_PROP, key, vals[key])]

        ch = v[0]
        a = self._axis(ch)
        if hdr == W.STOP:
            a.stop()
        elif hdr == W.SET_CL_FREQ:
            a.cl_freq = v[1]
        elif hdr == W.SET_SENSOR_TYPE:
            if v[1] not in P.SENSOR_TYPES or v[1] == 0:
                raise _Err(ch, 9)
            a.stop()
            a.sensor_type = v[1]
        elif hdr == W.SET_AMPLITUDE:
            a.amplitude = v[1]
        elif hdr == W.MOVE_STEP:
            _, steps, amp, freq = v
            a.step(steps, amp, freq)
        elif hdr == W.GET_CHANNEL_PROP:
            key = v[1]
            if key == W.PROP_MOVEMENT_TYPE:
                return [R(W.R_CHANNEL_PROP, ch, key, 1 if a.rotary else 0)]
            if key == W.PROP_DEFAULT_CL_FREQ:
                return [R(W.R_CHANNEL_PROP, ch, key, 5000)]
            raise _Err(ch, E_NOT_SUPPORTED)
        else:
            a = self._axis(ch, need_sensor=True)
            if hdr in (W.MOVE_POS_ABS, W.MOVE_POS_REL):
                if a.rotary:
                    raise _Err(ch, E_WRONG_SENSOR)
                target = v[1] / 10.0 + (a.logical if hdr == W.MOVE_POS_REL else 0.0)
                a.move_to(target, v[2])
            elif hdr in (W.MOVE_ANGLE_ABS, W.MOVE_ANGLE_REL):
                if not a.rotary:
                    raise _Err(ch, E_WRONG_SENSOR)
                target = v[1] / 10.0 + (a.logical if hdr == W.MOVE_ANGLE_REL else 0.0)
                a.move_to(target, v[3])
            elif hdr == W.CALIBRATE:
                a.calibrate()
            elif hdr == W.MOVE_TO_REF:
                a.reference(v[1], bool(v[2]))
            elif hdr == W.SET_ZERO:
                if a.status in P.BUSY:
                    raise _Err(ch, E_NOT_PROCESSABLE)
                a.set_zero()
            elif hdr == W.SET_SAFE_DIR:
                a.safe_dir = v[1]
            elif hdr == W.MOVE_TO_END_STOP:
                _, direction, hold, auto_zero = v
                lo, hi = a.limits
                end = hi if direction else lo
                if end in (float("inf"), -float("inf")):
                    raise _Err(ch, E_WRONG_SENSOR)   # no end stops on this stage
                a._start(end + (1 if direction else -1), a.cl_freq * 0.1, P.TARGETING, hold)
                if auto_zero:
                    a.offset = end
            else:
                raise _Err(ch, E_NOT_SUPPORTED)
        return self._ack(ch)


def serve_pty(sim: WireSCU, link: str | None = None, verbose: bool = False):
    master, slave = os.openpty()
    tty.setraw(slave)
    path = os.ttyname(slave)
    if link:
        try:
            os.unlink(link)
        except FileNotFoundError:
            pass
        os.symlink(path, link)

    def run():
        buf = b""
        while True:
            try:
                data = os.read(master, 1024)
            except OSError:
                return
            if not data:
                return
            buf += data
            frames, buf = W.split_frames(buf)
            for f in frames:
                out = sim.handle(f)
                if verbose:
                    print(f"-> {f.hex(' '):30s} {W.describe(f, 'W')}", file=sys.stderr)
                    for o in out:
                        d = W.describe(o, "R") if len(o) == 1 + W.payload_len(o[0]) else "raw"
                        print(f"<- {o.hex(' '):30s} {d}", file=sys.stderr)
                for o in out:
                    os.write(master, o)

    t = threading.Thread(target=run, daemon=True, name="scu-wire-sim")
    t.start()
    t.slave_fd = slave
    return path, t


def main(argv=None):
    ap = argparse.ArgumentParser(description="SCU binary-protocol simulator on a pty")
    ap.add_argument("--link", default="/tmp/scu-wire")
    ap.add_argument("--sensors", default="21,21,21",
                    help="sensor type per channel, 0 = none (default 21,21,21)")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)
    sim = WireSCU(sensor_types=[int(x) for x in a.sensors.split(",")])
    path, t = serve_pty(sim, a.link, a.verbose)
    print(f"SCU wire simulator on {path} -> {a.link}", flush=True)
    try:
        t.join()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            os.unlink(a.link)
        except OSError:
            pass


if __name__ == "__main__":
    main()
