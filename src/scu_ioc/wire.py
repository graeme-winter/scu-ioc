"""SCU binary ("wire") protocol, as spoken by SmarAct's SCU3DControl library
to the stock USB firmware over the FTDI UART.

Reconstructed from libscu3dcontrol.so 1.5.16 (x86-64) by static analysis;
see docs/wire-protocol.md for evidence and confidence per item. This module
is the single source of truth used by the simulator, the trace decoder and
(eventually) a pyserial backend.

Link: 500000 baud (FTDI divisor 6), 8N1, no flow control.

Framing, both directions: one header byte, then N payload bytes, where
    header = (code << 4) | N        N = 0..15
Multi-byte values are little-endian. Request and reply codes are separate
namespaces: 0x22 means one thing sent and another received.

Units are the SDK's: position 1/10 um, angle 1/10 mdeg, amplitude 1/10 V.
Device error codes are the SA_* codes >= 128 (e.g. 130 = no sensor).
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

BAUD = 500000

# --------------------------------------------------------------------------
# Requests (host -> SCU): header byte -> (name, payload struct format)
# Format strings are struct formats after the header, little-endian.
# --------------------------------------------------------------------------

GET = 0x12              # [GET, selector, channel]

# Selectors for GET; value is the reply header expected.
SEL_ID, SEL_FIRMWARE, SEL_STATUS, SEL_POSITION, SEL_CL_FREQ = 0x0, 0x1, 0x2, 0x3, 0x4
SEL_SENSOR_TYPE, SEL_ANGLE, SEL_SENSOR_PRESENT, SEL_AMPLITUDE = 0x5, 0x6, 0x7, 0x8
SEL_ALIGNMENT, SEL_SAFE_DIR, SEL_SCALE, SEL_PPK = 0x9, 0xA, 0xB, 0xC
SEL_FW_CONFIG = 0xFA

SELECTORS = {
    SEL_ID: "id", SEL_FIRMWARE: "firmware", SEL_STATUS: "status",
    SEL_POSITION: "position", SEL_CL_FREQ: "cl_max_freq",
    SEL_SENSOR_TYPE: "sensor_type", SEL_ANGLE: "angle",
    SEL_SENSOR_PRESENT: "sensor_present", SEL_AMPLITUDE: "amplitude",
    SEL_ALIGNMENT: "alignment", SEL_SAFE_DIR: "safe_direction",
    SEL_SCALE: "scale", SEL_PPK: "physical_position_known",
    SEL_FW_CONFIG: "firmware_config",
}

STOP = 0x11
SET_CL_FREQ = 0x13
SET_ALIGNMENT = 0x16
MOVE_STEP = 0x17
MOVE_POS_ABS = 0x27
MOVE_ANGLE_ABS = 0x29
SET_ZERO = 0x31
SET_REPORT_ON_COMPLETE = 0x32
SET_AMPLITUDE = 0x33
MOVE_TO_END_STOP = 0x35
SET_SCALE = 0x36
MOVE_POS_REL = 0x37
MOVE_ANGLE_REL = 0x39
CALIBRATE = 0x41
SET_SENSOR_TYPE = 0x42
MOVE_TO_REF = 0x44
GET_CHANNEL_PROP = 0x53
SET_SYSTEM_PROP = 0x56
SET_CHANNEL_PROP = 0x57
SET_SAFE_DIR = 0x72
SET_MODE = 0x81         # [mode]: 0 sync, 1 async (inferred from init)
GET_SYSTEM_PROP = 0xB2
HW_RESET = 0xC0         # only sent for SA_HARDWARE_RESET

REQUESTS = {
    GET: ("get", "<BB"),                       # selector, channel
    STOP: ("stop", "<B"),
    SET_CL_FREQ: ("set_cl_max_freq", "<BH"),
    MOVE_STEP: ("move_step", "<BhHH"),         # ch, steps, amplitude, frequency
    MOVE_POS_ABS: ("move_position_abs", "<BiH"),   # ch, pos, hold
    MOVE_POS_REL: ("move_position_rel", "<BiH"),
    MOVE_ANGLE_ABS: ("move_angle_abs", "<BihH"),   # ch, angle, revolution, hold
    MOVE_ANGLE_REL: ("move_angle_rel", "<BihH"),
    SET_ZERO: ("set_zero", "<B"),
    SET_REPORT_ON_COMPLETE: ("set_report_on_complete", "<BB"),
    SET_AMPLITUDE: ("set_amplitude", "<BH"),
    MOVE_TO_END_STOP: ("move_to_end_stop", "<BBHB"),  # ch, dir, hold, auto_zero
    SET_SCALE: ("set_scale", "<BiB"),          # ch, scale, inverted (layout unverified)
    CALIBRATE: ("calibrate", "<B"),
    SET_SENSOR_TYPE: ("set_sensor_type", "<BB"),
    MOVE_TO_REF: ("move_to_reference", "<BHB"),   # ch, hold, auto_zero
    GET_CHANNEL_PROP: ("get_channel_property", "<BH"),
    SET_SYSTEM_PROP: ("set_system_property", "<Hi"),
    SET_CHANNEL_PROP: ("set_channel_property", "<BHi"),
    SET_SAFE_DIR: ("set_safe_direction", "<BB"),
    SET_MODE: ("set_mode", "<B"),
    GET_SYSTEM_PROP: ("get_system_property", "<H"),
    HW_RESET: ("hardware_reset", ""),
    SET_ALIGNMENT: ("set_alignment", "<BBHBB"),   # layout unverified
}

# --------------------------------------------------------------------------
# Replies (SCU -> host)
# --------------------------------------------------------------------------

R_RESET_DONE = 0x20
R_ACK = 0xA1
R_ERROR = 0xE2
R_ID = 0x14
R_FIRMWARE = 0x15
R_STATUS = 0x22
R_CL_FREQ = 0x23
R_POSITION = 0x25
R_ALIGNMENT = 0x26
R_AMPLITUDE = 0x43
R_SCALE = 0x46
R_ANGLE = 0x47
R_SENSOR_TYPE = 0x52
R_SENSOR_PRESENT = 0x62
R_SYSTEM_PROP = 0x66
R_CHANNEL_PROP = 0x67
R_SAFE_DIR = 0x82
R_PPK = 0xA2

REPLIES = {
    R_RESET_DONE: ("reset_done", ""),
    R_ACK: ("ack", "<B"),                      # channel (or mode for SET_MODE)
    R_ERROR: ("error", "<BB"),                 # channel, SA error code
    R_ID: ("id", "<I"),
    R_FIRMWARE: ("firmware", "<BBBBB"),        # see firmware_version()
    R_STATUS: ("status", "<BB"),
    R_CL_FREQ: ("cl_max_freq", "<BH"),
    R_POSITION: ("position", "<Bi"),
    R_ALIGNMENT: ("alignment", "<BBHBB"),      # layout unverified
    R_AMPLITUDE: ("amplitude", "<BH"),
    R_SCALE: ("scale", "<BiB"),                # layout unverified
    R_ANGLE: ("angle", "<Bih"),
    R_SENSOR_TYPE: ("sensor_type", "<BB"),
    R_SENSOR_PRESENT: ("sensor_present", "<BB"),
    R_SYSTEM_PROP: ("system_property", "<Hi"),
    R_CHANNEL_PROP: ("channel_property", "<BHi"),
    R_SAFE_DIR: ("safe_direction", "<BB"),
    R_PPK: ("physical_position_known", "<BB"),
}

# Reply expected for each GET selector.
GET_REPLY = {
    SEL_ID: R_ID, SEL_FIRMWARE: R_FIRMWARE, SEL_STATUS: R_STATUS,
    SEL_POSITION: R_POSITION, SEL_CL_FREQ: R_CL_FREQ, SEL_SENSOR_TYPE: R_SENSOR_TYPE,
    SEL_ANGLE: R_ANGLE, SEL_SENSOR_PRESENT: R_SENSOR_PRESENT, SEL_AMPLITUDE: R_AMPLITUDE,
    SEL_ALIGNMENT: R_ALIGNMENT, SEL_SAFE_DIR: R_SAFE_DIR, SEL_SCALE: R_SCALE,
    SEL_PPK: R_PPK,
}

# Property keys (SCU3DControlConstants.h)
PROP_TEMPERATURE, PROP_VOLTAGE, PROP_HW_VERSION = 1, 2, 3
PROP_TARGET_THRESHOLD, PROP_KP, PROP_KPD = 3, 5, 6
PROP_DEFAULT_CL_FREQ, PROP_ADVANCED_STEPPING, PROP_MOVEMENT_TYPE = 15, 21, 25


class FrameError(ValueError):
    pass


def header(code: int, n: int) -> int:
    return ((code & 0xF) << 4) | (n & 0xF)


def payload_len(hdr: int) -> int:
    return hdr & 0x0F


def encode(hdr: int, *values) -> bytes:
    """Build a request (or, in the simulator, a reply) frame."""
    table = REQUESTS if hdr in REQUESTS else REPLIES
    name, fmt = table[hdr]
    body = struct.pack(fmt, *values) if fmt else b""
    if len(body) != payload_len(hdr):
        raise FrameError(f"{name}: payload {len(body)} bytes, header says {payload_len(hdr)}")
    return bytes([hdr]) + body


def encode_reply(hdr: int, *values) -> bytes:
    name, fmt = REPLIES[hdr]
    body = struct.pack(fmt, *values) if fmt else b""
    if len(body) != payload_len(hdr):
        raise FrameError(f"{name}: payload {len(body)} bytes, header says {payload_len(hdr)}")
    return bytes([hdr]) + body


def decode(frame: bytes, table: dict):
    """(name, values) for a complete frame, or raise FrameError."""
    if not frame:
        raise FrameError("empty frame")
    hdr = frame[0]
    if len(frame) != 1 + payload_len(hdr):
        raise FrameError(f"frame {frame.hex(' ')}: length mismatch")
    if hdr not in table:
        return f"unknown_0x{hdr:02x}", tuple(frame[1:])
    name, fmt = table[hdr]
    return name, struct.unpack(fmt, frame[1:]) if fmt else ()


def decode_request(frame: bytes):
    return decode(frame, REQUESTS)


def decode_reply(frame: bytes):
    return decode(frame, REPLIES)


def firmware_version(payload: bytes) -> int:
    """SDK-style packed version from an R_FIRMWARE payload. The library
    uses bytes 0, 1, 3, 4 and ignores byte 2 (meaning unknown)."""
    return payload[0] | payload[1] << 8 | payload[3] << 16 | payload[4] << 24


def split_frames(data: bytes):
    """Split a byte stream into frames using the header length nibble.
    Returns (frames, leftover)."""
    frames = []
    i = 0
    while i < len(data):
        n = 1 + payload_len(data[i])
        if i + n > len(data):
            break
        frames.append(bytes(data[i:i + n]))
        i += n
    return frames, bytes(data[i:])


def describe(frame: bytes, direction: str) -> str:
    """Human-readable one-liner for traces. direction: 'W' or 'R'."""
    name, vals = decode(frame, REQUESTS if direction == "W" else REPLIES)
    if direction == "W" and frame[0] == GET:
        sel, ch = vals
        if sel in (SEL_ID, SEL_FIRMWARE):
            # the SDK leaves this byte uninitialised for device-level gets
            return f"get {SELECTORS[sel]} (unused byte 0x{ch:02x})"
        return f"get {SELECTORS.get(sel, hex(sel))} ch={ch}"
    if direction == "R" and frame[0] == R_FIRMWARE:
        v = firmware_version(frame[1:])
        return f"firmware {v >> 24}.{(v >> 16) & 0xFF}.{v & 0xFFFF} (byte2=0x{frame[3]:02x})"
    return f"{name} {', '.join(str(v) for v in vals)}".strip()
