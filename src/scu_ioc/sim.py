"""Protocol-level SCU simulator speaking the ASCII interface.

`scu-sim --link /tmp/scu` serves it on a pseudo-terminal, so the IOC (or
anything else) can be run with `--backend ascii --port /tmp/scu`.

The motion model is deliberately crude: closed-loop moves run at
cl_max_frequency x 0.1 um per step, open-loop steps cover
0.2 um x amplitude/100 V each, linear stages have end stops at +/-10.5 mm
(roughly an SLC-1740), and calibration takes one second. Physical vs
logical scale follows the manual: at power-up the current position reads
0; MTR makes the physical position known; SZ and auto-zero shift the
logical scale. Scale inversion (SSC) is not modelled.
"""

from __future__ import annotations

import argparse
import os
import random
import re
import sys
import threading
import time
import tty

from . import protocol as P

STEP_UM_AT_100V = 0.2
CL_STEP_UM = 0.1
CALIBRATE_S = 1.0
LINEAR_LIMIT_UM = 10500.0


class SimAxis:
    def __init__(self, sensor_type: int | None, clock):
        self.clock = clock
        self.sensor_present = sensor_type is not None and sensor_type != 0
        self.sensor_type = sensor_type or 21
        self.cl_freq = 5000
        self.safe_dir = 1
        self.amplitude = 1000
        self.frequency = 1000
        self.steps = P.MAX_STEPS
        self.phys = random.uniform(-3000.0, 3000.0)
        self.offset = self.phys          # power-up position reads as 0
        self.nv_offset = 0.0
        self.known = False
        self.status = P.STOPPED
        self._motion = None              # (t0, x0, x1, speed, then_status, hold_ms)
        self._busy_until = None
        self._hold_until = None

    @property
    def rotary(self) -> bool:
        return P.sensor_kind(self.sensor_type) == "rotary"

    @property
    def limits(self):
        if self.rotary:
            return (-float("inf"), float("inf"))
        return (-LINEAR_LIMIT_UM, LINEAR_LIMIT_UM)

    # -- time evolution ----------------------------------------------------
    def update(self) -> None:
        now = self.clock()
        if self._motion is not None:
            t0, x0, x1, speed, then_status, hold_ms = self._motion
            lo, hi = self.limits
            dist = abs(x1 - x0)
            travelled = min(dist, speed * (now - t0))
            sign = 1 if x1 >= x0 else -1
            self.phys = min(max(x0 + sign * travelled, lo), hi)
            at_stop = self.phys in (lo, hi) and travelled < dist
            if travelled >= dist or at_stop:
                self._motion = None
                if then_status == P.REFERENCING:
                    self.known = True
                    self.offset = self.nv_offset
                    self.status = P.STOPPED
                elif hold_ms and not at_stop:
                    self.status = P.HOLDING
                    self._hold_until = (None if hold_ms >= P.HOLD_FOREVER
                                        else now + hold_ms / 1000.0)
                else:
                    self.status = P.STOPPED
        if self._busy_until is not None and now >= self._busy_until:
            self._busy_until = None
            self.status = P.STOPPED
        if self.status == P.HOLDING and self._hold_until is not None and now >= self._hold_until:
            self._hold_until = None
            self.status = P.STOPPED

    @property
    def logical(self) -> float:
        return self.phys - self.offset

    def _start(self, target_phys, speed, status, hold_ms=0, then=None):
        self.update()
        self._busy_until = None
        self._hold_until = None
        self._motion = (self.clock(), self.phys, target_phys, speed,
                        then if then is not None else status, hold_ms)
        self.status = status

    asymmetry = 0.0     # closed-loop speed x (1 + a) moving +, x (1 - a) moving -
    response = None     # optional f(cl_freq, direction) -> speed in um/s

    def move_to(self, logical_target, hold_ms):
        self.update()
        target = logical_target + self.offset
        d = 1 if target >= self.phys else -1
        if self.response is not None:
            speed = self.response(self.cl_freq, d)
        else:
            k = 1 + self.asymmetry if d > 0 else 1 - self.asymmetry
            speed = self.cl_freq * CL_STEP_UM * k
        self._start(target, speed, P.TARGETING, hold_ms)

    def step(self, n, amplitude, frequency):
        self.amplitude, self.frequency = amplitude, frequency
        size = STEP_UM_AT_100V * amplitude / 1000.0
        if abs(n) >= P.MAX_STEPS:
            n = 10 ** 9 if n > 0 else -10 ** 9
        self._start(self.phys + n * size, frequency * size, P.MOVING)

    def stop(self):
        self.update()
        self._motion = None
        self._busy_until = None
        self._hold_until = None
        self.status = P.STOPPED

    def calibrate(self):
        self.stop()
        self.status = P.CALIBRATING
        self._busy_until = self.clock() + CALIBRATE_S

    def reference(self, hold_ms, auto_zero):
        if auto_zero:
            self.nv_offset = 0.0
        self._start(0.0, self.cl_freq * CL_STEP_UM, P.REFERENCING,
                    hold_ms, then=P.REFERENCING)

    def set_zero(self):
        self.update()
        self.offset = self.phys
        if self.known:
            self.nv_offset = self.offset


class SimSCU:
    """Command interpreter. handle(line) -> list of answer strings."""

    def __init__(self, sensor_types=(21, 21, 21), device_id="604573714",
                 firmware="1.2.5", clock=time.monotonic):
        self.axes = [SimAxis(t, clock) for t in sensor_types]
        self.model = f"SmarAct CU-{len(self.axes)}D"
        self.device_id = device_id
        self.firmware = firmware
        self.auto_error = False
        self.error_register = 0
        self.temperature = 31
        self.lock = threading.Lock()

    # Replies: query -> [answer]; others -> ["E0"] in auto mode else [].
    def _ok(self):
        self.error_register = 0
        return ["E0"] if self.auto_error else []

    def _err(self, code):
        if self.auto_error:
            return [f"E{code}"]
        self.error_register = code
        return []

    def _axis(self, ch, need_sensor=False):
        if ch is None or not 0 <= ch < len(self.axes):
            raise _Err(3)
        a = self.axes[ch]
        a.update()
        if need_sensor and not a.sensor_present:
            raise _Err(19)
        return a

    def handle(self, line: str) -> list[str]:
        with self.lock:
            try:
                return self._dispatch(line)
            except _Err as e:
                return self._err(e.code)

    def _dispatch(self, line: str) -> list[str]:
        m = re.match(r"^([A-Z]+)(-?\d*)((?:[A-Z]-?\d+(?:\.\d+)?|[A-Z][UD])*)$", line)
        if not m:
            raise _Err(1)
        cmd, idx, rest = m.groups()
        ch = int(idx) if idx else None
        prm = {}
        for k, v in re.findall(r"([A-Z])(-?\d+(?:\.\d+)?|[UD])", rest):
            prm[k] = v

        def num(k, default=None, kind=float):
            if k not in prm:
                if default is None:
                    raise _Err(18)
                return default
            try:
                return kind(prm[k])
            except ValueError:
                raise _Err(17) from None

        if cmd == "I":
            return [f"I{self.model}"]
        if cmd == "GID":
            return [f"ID{self.device_id}"]
        if cmd == "V":
            return [f"V{self.firmware}"]
        if cmd == "E":
            if ch is None:
                code, self.error_register = self.error_register, 0
                return [f"E{code}"]
            if ch not in (0, 1):
                raise _Err(4)
            self.auto_error = bool(ch)
            return ["E0"]
        if cmd == "K":
            return self._ok()
        if cmd == "GSPR":
            vals = {1: self.temperature, 2: 1000, 3: 7}
            if ch not in vals:
                raise _Err(17)
            return [f"SPR{ch}V{vals[ch]}"]

        if cmd in ("S", "M") and ch == 99:
            if cmd == "S":
                for a in self.axes:
                    a.stop()
                return self._ok()
            out = []
            for i, a in enumerate(self.axes):
                a.update()
                out.append(f"M{i}{P.STATUS_LETTERS[a.status]}")
            return out

        if cmd == "M":
            return [f"M{ch}{P.STATUS_LETTERS[self._axis(ch).status]}"]
        if cmd == "S":
            self._axis(ch).stop()
            return self._ok()
        if cmd == "GSP":
            return [f"SP{ch}{'P' if self._axis(ch).sensor_present else 'N'}"]
        if cmd == "GST":
            return [f"ST{ch}T{self._axis(ch).sensor_type}"]
        if cmd == "SST":
            t = num("T", kind=int)
            if t not in P.SENSOR_TYPES or t == 0:
                raise _Err(17)
            a = self._axis(ch)
            a.stop()
            a.sensor_type = t
            return self._ok()
        if cmd == "GCLF":
            return [f"CLF{ch}F{self._axis(ch).cl_freq}"]
        if cmd == "SCLF":
            f = num("F", kind=int)
            if not 1 <= f <= 18500:
                raise _Err(17)
            self._axis(ch).cl_freq = f
            return self._ok()
        if cmd == "GSD":
            return [f"SD{ch}D{self._axis(ch, True).safe_dir}"]
        if cmd == "SSD":
            d = num("D", kind=int)
            if d not in (0, 1):
                raise _Err(17)
            self._axis(ch, True).safe_dir = d
            return self._ok()
        if cmd == "GPPK":
            return [f"PPK{ch}K{int(self._axis(ch, True).known)}"]
        if cmd == "GP":
            a = self._axis(ch, True)
            if a.rotary:
                raise _Err(20)
            return [f"P{ch}P{a.logical:.1f}"]
        if cmd == "GA":
            a = self._axis(ch, True)
            if not a.rotary:
                raise _Err(20)
            return [f"A{ch}A{a.logical:.1f}R0"]
        if cmd in ("MPA", "MPR", "MAA", "MAR"):
            a = self._axis(ch, True)
            if a.rotary != (cmd[1] == "A"):
                raise _Err(20)
            v = num("P" if cmd[1] == "P" else "A")
            hold = num("H", 0, int)
            if not 0 <= hold <= P.HOLD_FOREVER:
                raise _Err(17)
            target = v if cmd[2] == "A" else a.logical + v
            a.move_to(target, hold)
            return self._ok()
        if cmd in ("U", "D"):
            targets = self.axes if ch == 99 else [self._axis(ch)]
            for a in targets:
                a.update()
                f = num("F", a.frequency, int)
                amp = num("A", a.amplitude, int)
                s = num("S", a.steps, int)
                if not (1 <= f <= 18500 and 150 <= amp <= 1000 and 1 <= s <= P.MAX_STEPS):
                    raise _Err(17)
                a.steps = s
                a.step(s if cmd == "U" else -s, amp, f)
            return self._ok()
        if cmd == "CS":
            self._axis(ch, True).calibrate()
            return self._ok()
        if cmd == "MTR":
            hold = num("H", 0, int)
            z = num("Z", 0, int)
            self._axis(ch, True).reference(hold, bool(z))
            return self._ok()
        if cmd == "SZ":
            a = self._axis(ch, True)
            if a.status in P.BUSY:
                raise _Err(27)
            a.set_zero()
            return self._ok()
        raise _Err(2)


class _Err(Exception):
    def __init__(self, code):
        self.code = code


# --------------------------------------------------------------------------
# pty server
# --------------------------------------------------------------------------

def serve_pty(sim: SimSCU, link: str | None = None, verbose: bool = False):
    """Create a pty, optionally symlink its slave to `link`, and serve the
    simulator on it. Returns (slave_path, thread)."""
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
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", "replace")
                i = text.rfind(":")
                if i < 0 or not text[i + 1:].strip():
                    continue
                cmd = text[i + 1:].strip()
                out = sim.handle(cmd)
                if verbose:
                    print(f"{cmd!r} -> {out}", file=sys.stderr)
                for ans in out:
                    os.write(master, P.frame(ans))

    t = threading.Thread(target=run, daemon=True, name="scu-sim")
    t.start()
    # keep the slave fd open so the pty survives client reconnects
    t.slave_fd = slave
    return path, t


def main(argv=None):
    ap = argparse.ArgumentParser(description="SmarAct SCU ASCII-protocol simulator on a pty")
    ap.add_argument("--link", default="/tmp/scu", help="symlink to create for the pty (default /tmp/scu)")
    ap.add_argument("--sensors", default="21,21,21",
                    help="sensor type per channel, 0 = no sensor; 1 or 3 entries (default 21,21,21)")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)
    types = [int(x) for x in a.sensors.split(",")]
    if len(types) not in (1, 3):
        ap.error("--sensors needs 1 or 3 entries")
    sim = SimSCU(sensor_types=types)
    path, t = serve_pty(sim, a.link, a.verbose)
    print(f"SCU simulator ({sim.model}) on {path} -> {a.link}", flush=True)
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
