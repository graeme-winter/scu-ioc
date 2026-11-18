"""Binary-protocol backend: talks to stock-firmware USB SCUs directly over
the FTDI UART with pyserial, so no SmarAct SDK is needed (any CPU, any OS
with an FTDI serial driver, e.g. a Raspberry Pi with ftdi_sio).

Protocol: wire.py and docs/wire-protocol.md. Checked against SmarAct's
library (tests/test_wire_sdk.py) and against exchanges captured from a
CU-3D with firmware 5.1.192 (tests/test_wire_backend.py).

The port may be a device path or any pyserial URL (socket://, rfc2217://),
provided whatever is at the far end runs the UART at 500000 baud, 8N1.
On Linux, set the FTDI latency timer to 1 ms (the SDK does); with the
default 16 ms every exchange takes ~16 ms. See README.
"""

from __future__ import annotations

import logging
import os
import time

import serial

from . import protocol as P
from . import wire as W
from .backend import AxisConfig, Backend, ConnectionLost, DeviceInfo, check_step_args
from .dll import DllCommandError, _version

log = logging.getLogger(__name__)

FIRST_REPLY_TIMEOUT = 2.0     # first exchange after open took 238 ms on hardware
DEVICE_CHANNEL = 0xFF         # channel byte in replies to device-level commands


def _tenths(x: float) -> int:
    return int(round(float(x) * 10))


class WireBackend(Backend):
    name = "wire"

    def __init__(self, port: str, baud: int = W.BAUD, timeout: float = 1.0,
                 device_id: int | None = None, serial_factory=None):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.device_id = device_id
        self._factory = serial_factory or self._open_serial
        self._ser = None

    # -- transport ---------------------------------------------------------
    def _open_serial(self):
        return serial.serial_for_url(self.port, baudrate=self.baud, bytesize=8,
                                     parity="N", stopbits=1, timeout=self.timeout,
                                     write_timeout=self.timeout)

    def connect(self) -> DeviceInfo:
        self.close()
        try:
            self._ser = self._factory()
        except (serial.SerialException, OSError) as e:
            raise ConnectionLost(f"cannot open {self.port}: {e}") from e
        _check_latency_timer(self.port)
        try:
            self._ser.reset_input_buffer()
        except (serial.SerialException, OSError) as e:
            raise ConnectionLost(str(e)) from e

        dev_id = self._get(W.SEL_ID, 0, W.R_ID, timeout=FIRST_REPLY_TIMEOUT)[0]
        if self.device_id is not None and dev_id != self.device_id:
            raise ConnectionLost(f"device on {self.port} has ID {dev_id}, "
                                 f"expected {self.device_id}")
        fw = self._get(W.SEL_FIRMWARE, 0, W.R_FIRMWARE)
        firmware = _version(W.firmware_version(bytes(fw)))
        self._command(W.encode(W.SET_MODE, 0), ack_channel=None)   # synchronous
        try:
            self.status(2)
            channels = 3
        except P.CommandError:
            channels = 1
        return DeviceInfo(f"SmarAct SCU-{channels}D (USB)", str(dev_id), firmware, channels)

    def close(self) -> None:
        # The SDK sends nothing on release; neither do we.
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _read_exact(self, n: int, deadline: float) -> bytes:
        buf = b""
        while len(buf) < n:
            if time.monotonic() > deadline:
                raise ConnectionLost(f"timeout: wanted {n} bytes, got {buf.hex(' ') or 'none'}")
            try:
                chunk = self._ser.read(n - len(buf))
            except (serial.SerialException, OSError) as e:
                raise ConnectionLost(str(e)) from e
            buf += chunk
        return buf

    def _exchange(self, frame: bytes, timeout: float | None = None) -> bytes:
        """Send one request frame, return one reply frame."""
        if self._ser is None:
            raise ConnectionLost("not connected")
        log.debug("-> %s  %s", frame.hex(" "), W.describe(frame, "W"))
        try:
            self._ser.write(frame)
        except (serial.SerialException, OSError) as e:
            raise ConnectionLost(str(e)) from e
        deadline = time.monotonic() + (timeout or self.timeout)
        hdr = self._read_exact(1, deadline)
        reply = hdr + self._read_exact(W.payload_len(hdr[0]), deadline)
        log.debug("<- %s  %s", reply.hex(" "), W.describe(reply, "R"))
        return reply

    def _error(self, reply: bytes, what: str):
        _ch, code = reply[1], reply[2]
        return DllCommandError(f"wire:{what}", code)

    def _desync(self, reply: bytes, what: str, expected: int):
        # Anything else means we've lost frame alignment: let the caller
        # reconnect (which purges) rather than guess.
        return P.ProtocolError(f"{what}: reply {reply.hex(' ')}, expected header "
                               f"0x{expected:02x} or an error")

    def _get(self, selector: int, ch: int, reply_hdr: int, timeout=None):
        what = W.SELECTORS.get(selector, hex(selector))
        reply = self._exchange(W.encode(W.GET, selector, ch), timeout)
        if reply[0] == W.R_ERROR:
            raise self._error(reply, f"get {what}")
        if reply[0] != reply_hdr:
            raise self._desync(reply, f"get {what}", reply_hdr)
        _, vals = W.decode_reply(reply)
        if selector not in (W.SEL_ID, W.SEL_FIRMWARE) and vals[0] != ch:
            raise P.ProtocolError(f"get {what}: reply for channel {vals[0]}, asked {ch}")
        return vals if selector in (W.SEL_ID, W.SEL_FIRMWARE) else vals[1:]

    def _command(self, frame: bytes, ack_channel: int | None):
        what = W.describe(frame, "W").split(" ")[0]
        reply = self._exchange(frame)
        if reply[0] == W.R_ERROR:
            raise self._error(reply, what)
        if reply[0] != W.R_ACK:
            raise self._desync(reply, what, W.R_ACK)
        if ack_channel is not None and reply[1] != ack_channel:
            raise P.ProtocolError(f"{what}: ack for channel {reply[1]}, sent {ack_channel}")

    def _ch(self, header: int, ch: int, *values):
        self._command(W.encode(header, ch, *values), ack_channel=ch)

    # -- reads -------------------------------------------------------------
    def axis_config(self, ch):
        present = bool(self._get(W.SEL_SENSOR_PRESENT, ch, W.R_SENSOR_PRESENT)[0])
        stype = self._get(W.SEL_SENSOR_TYPE, ch, W.R_SENSOR_TYPE)[0]
        clf = self._get(W.SEL_CL_FREQ, ch, W.R_CL_FREQ)[0]
        try:
            sd = self._get(W.SEL_SAFE_DIR, ch, W.R_SAFE_DIR)[0]
        except P.CommandError:
            sd = None
        return AxisConfig(present, stype, clf, sd, self._movement_type(ch))

    def _movement_type(self, ch):
        reply = self._exchange(W.encode(W.GET_CHANNEL_PROP, ch, W.PROP_MOVEMENT_TYPE))
        if reply[0] == W.R_ERROR:      # firmware 5.1.192 answers E2 FF 09
            return None
        if reply[0] != W.R_CHANNEL_PROP:
            raise self._desync(reply, "get channel property", W.R_CHANNEL_PROP)
        _, (rch, key, value) = W.decode_reply(reply)
        return {0: "linear", 1: "rotary", 2: "rotary"}.get(value)

    def status(self, ch):
        return self._get(W.SEL_STATUS, ch, W.R_STATUS)[0]

    def position(self, ch, rotary):
        if rotary:
            return self._get(W.SEL_ANGLE, ch, W.R_ANGLE)[0] / 10.0
        return self._get(W.SEL_POSITION, ch, W.R_POSITION)[0] / 10.0

    def referenced(self, ch):
        return bool(self._get(W.SEL_PPK, ch, W.R_PPK)[0])

    def _system(self, key: int):
        reply = self._exchange(W.encode(W.GET_SYSTEM_PROP, key))
        if reply[0] == W.R_ERROR:
            return None
        if reply[0] != W.R_SYSTEM_PROP:
            raise self._desync(reply, "get system property", W.R_SYSTEM_PROP)
        _, (rkey, value) = W.decode_reply(reply)
        return value

    def temperature(self):
        v = self._system(W.PROP_TEMPERATURE)
        return None if v is None else float(v)

    def hv(self):
        v = self._system(W.PROP_VOLTAGE)
        return None if v is None else v / 10.0

    # -- commands ----------------------------------------------------------
    @staticmethod
    def _hold(hold_ms):
        h = int(hold_ms)
        if not 0 <= h <= P.HOLD_FOREVER:
            raise ValueError(f"hold must be 0..{P.HOLD_FOREVER} ms")
        return h

    def move_abs(self, ch, target, hold_ms, rotary):
        if rotary:
            self._ch(W.MOVE_ANGLE_ABS, ch, _tenths(target), 0, self._hold(hold_ms))
        else:
            self._ch(W.MOVE_POS_ABS, ch, _tenths(target), self._hold(hold_ms))

    def move_rel(self, ch, delta, hold_ms, rotary):
        if rotary:
            self._ch(W.MOVE_ANGLE_REL, ch, _tenths(delta), 0, self._hold(hold_ms))
        else:
            self._ch(W.MOVE_POS_REL, ch, _tenths(delta), self._hold(hold_ms))

    def step(self, ch, steps, amplitude, frequency):
        check_step_args(steps, amplitude, frequency)
        self._ch(W.MOVE_STEP, ch, int(steps), int(amplitude), int(frequency))

    def stop(self, ch):
        self._ch(W.STOP, ch)

    def calibrate(self, ch):
        self._ch(W.CALIBRATE, ch)

    def reference(self, ch, hold_ms, auto_zero):
        self._ch(W.MOVE_TO_REF, ch, self._hold(hold_ms), 1 if auto_zero else 0)

    def set_zero(self, ch):
        self._ch(W.SET_ZERO, ch)

    def set_sensor_type(self, ch, code):
        self._ch(W.SET_SENSOR_TYPE, ch, int(code))

    def set_cl_max_frequency(self, ch, hz):
        lo, hi = P.CL_FREQUENCY_RANGE
        if not lo <= int(hz) <= hi:
            raise P.CommandError(17, f"closed-loop frequency {hz} outside {lo}..{hi}")
        self._ch(W.SET_CL_FREQ, ch, int(hz))

    def set_safe_direction(self, ch, forward):
        self._ch(W.SET_SAFE_DIR, ch, 1 if forward else 0)


_warned = set()


def _check_latency_timer(port: str) -> None:
    """Warn (once per port) if a Linux FTDI port has the default 16 ms latency."""
    try:
        dev = os.path.basename(os.path.realpath(port))
        path = f"/sys/bus/usb-serial/devices/{dev}/latency_timer"
        if not os.path.exists(path):
            return
        ms = int(open(path).read().strip())
        if ms > 1 and port not in _warned:
            _warned.add(port)
            log.warning("%s: FTDI latency timer is %d ms; set it to 1 for faster "
                        "polling (echo 1 > %s, or the udev rule in the README)", port, ms, path)
    except (OSError, ValueError):
        pass
