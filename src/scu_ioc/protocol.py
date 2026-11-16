"""SCU constants and ASCII protocol helpers.

Sources: SmarAct "SCU ASCII Programming Interface" v2.0-C (2016) and
"SCU Programmer's Guide" v2.0-C / SCU3DControl.h v1.4.6.

Everything above the backends works in these units:
    linear position   micrometres (float)
    angle             millidegrees (float)
    amplitude         1/10 V (int, 150..1000)
    frequency         Hz (int)
    hold time         ms (int, 60000 = hold until stopped)
"""

from __future__ import annotations

import re

# Movement status codes (same numbering as SA_*_STATUS in the DLL).
STOPPED, SETTING_AMPLITUDE, MOVING, TARGETING, HOLDING, CALIBRATING, REFERENCING = range(7)

STATUS_NAMES = [
    "Stopped",
    "Setting amplitude",
    "Moving",
    "Targeting",
    "Holding",
    "Calibrating",
    "Referencing",
]

# ASCII "M" answer letter <-> status code.
STATUS_LETTERS = "SAMTHCR"

# Statuses during which a commanded action is still running. HOLDING counts
# as finished: the target has been reached and is being actively held.
BUSY = frozenset({SETTING_AMPLITUDE, MOVING, TARGETING, CALIBRATING, REFERENCING})

HOLD_FOREVER = 60000
MAX_STEPS = 30000          # +/-30000 means unbounded in the controller
AMPLITUDE_RANGE = (150, 1000)
FREQUENCY_RANGE = (1, 18500)
CL_FREQUENCY_RANGE = (50, 18500)

# Sensor type code -> (symbol, kind, description). From the ASCII manual
# appendix 5.3 and SCU3DControl.h.
SENSOR_TYPES = {
    0: ("NONE", "linear", "no sensor"),
    1: ("M", "linear", "SLC/SL with microsensor, end stop ref"),
    2: ("GA", "rotary", "goniometer 43.5 mm radius"),
    3: ("GB", "rotary", "goniometer 56.0 mm radius"),
    4: ("GC", "rotary", "SR1910m, end stops"),
    5: ("GD", "rotary", "goniometer SGO60.5m"),
    6: ("GE", "rotary", "goniometer SGO77.5m"),
    7: ("RA", "rotary", "SFW absolute sensor"),
    8: ("GF", "rotary", "SR1209m, end stops"),
    9: ("RB", "rotary", "SR1910m, unlimited rotation"),
    10: ("SR36M", "rotary", "SR3610m"),
    11: ("SR36ME", "rotary", "SR3610m, end stops"),
    12: ("SR50M", "rotary", "SR5018m"),
    13: ("SR50ME", "rotary", "SR5018m, end stops"),
    14: ("MM50", "linear", "magnetic encoder, end stop ref"),
    15: ("G935M", "rotary", "goniometer SGO93.5me"),
    16: ("MD", "linear", "M with double piezo"),
    17: ("TT254", "rotary", "STT25.4m (unsupported)"),
    18: ("LC", "linear", "SLCxxxxl distance coded marks"),
    19: ("LR", "rotary", "SRxxxxl with ref mark"),
    20: ("LCD", "linear", "LC with large actuator"),
    21: ("L", "linear", "SLCxxxxl single ref mark"),
    22: ("LD", "linear", "L with large actuator"),
    23: ("LE", "linear", "SLCxxxxl end stop ref"),
    24: ("LED", "linear", "LE with large actuator"),
    # Positioner type codes from SDK 1.5.16 (SCU3DControlConstants.h).
    25: ("SL_S1I1E1", "linear", "linear, inductive sensor, 293 nm"),
    26: ("SL_D1I1E1", "linear", "SL_S1I1E1 with large actuator"),
    27: ("SL_S1I2E2", "linear", "linear, inductive sensor, 73 nm"),
    28: ("SL_D1I2E2", "linear", "SL_S1I2E2 with large actuator"),
    29: ("ST_S1I1E2", "rotary", "25.4 mm mirror tip-tilt axis"),
    30: ("SG_D1L1S", "rotary", "goniometer 60.5 mm, single ref"),
    31: ("SG_D1L1E", "rotary", "goniometer 60.5 mm, end stop ref"),
    32: ("SG_D1L2S", "rotary", "goniometer 77.5 mm, single ref"),
    33: ("SG_D1L2E", "rotary", "goniometer 77.5 mm, end stop ref"),
    34: ("SG_D1M1E", "rotary", "goniometer 60.5 mm, M sensor"),
    35: ("SG_D1M2E", "rotary", "goniometer 77.5 mm, M sensor"),
    36: ("SI_S1L1S", "rotary", "iris diaphragm 21.42 mm, single ref"),
    37: ("ST_S1I2E2", "rotary", "50.8 mm mirror tip-tilt axis"),
    38: ("SR_T5L3S", "rotary", "rotary, L sensor, single ref"),
    39: ("SI_S1L4E", "rotary", "iris diaphragm 50.42 mm, end stop ref"),
    40: ("SI_S1L1E", "rotary", "iris diaphragm 21.42 mm, end stop ref"),
    41: ("SI_S1L3E", "rotary", "iris diaphragm 16.42 mm, end stop ref"),
}


def sensor_kind(code: int) -> str:
    return SENSOR_TYPES.get(code, ("?", "linear", ""))[1]


def sensor_symbol(code: int) -> str:
    return SENSOR_TYPES.get(code, (f"T{code}", "", ""))[0]


# Controller error codes (ASCII "E<code>"); DLL codes are mapped onto these
# where they overlap, see dll.py.
ERROR_TEXT = {
    0: "No error",
    1: "Parse error",
    2: "Unknown command",
    3: "Invalid channel",
    4: "Invalid mode",
    13: "Syntax error",
    15: "Overflow",
    17: "Invalid parameter",
    18: "Missing parameter",
    19: "No sensor present",
    20: "Wrong sensor type",
    21: "End stop reached",
    22: "Targeting timeout",
    23: "HV range error (power cycle needed)",
    24: "Temperature overheat",
    25: "Calibration failed",
    26: "Referencing failed",
    27: "Not processable now",
}


class ProtocolError(RuntimeError):
    """Unexpected or unparseable reply: treat the link as suspect."""


class CommandError(RuntimeError):
    """The controller rejected a command (link is fine)."""

    def __init__(self, code: int, command: str = ""):
        self.code = code
        self.command = command
        text = ERROR_TEXT.get(code, "unknown error")
        super().__init__(f"{text} (E{code})" + (f" for {command!r}" if command else ""))


# --------------------------------------------------------------------------
# ASCII framing: ':' + command + '\n' in both directions.
# --------------------------------------------------------------------------

_ERROR_RE = re.compile(r"^E(\d+)$")
_NUM = r"(-?\d+(?:\.\d+)?)"


def frame(command: str) -> bytes:
    return b":" + command.encode("ascii") + b"\n"


def error_code(answer: str) -> int | None:
    """E<code> answers -> code, anything else -> None."""
    m = _ERROR_RE.match(answer)
    return int(m.group(1)) if m else None


def is_unsolicited(answer: str) -> bool:
    """Completion / channel-error reports (SRC / SER), which we never enable
    but tolerate: C<ch> and CE<ch>E<code>."""
    return bool(re.match(r"^C\d+$|^CE\d+E\d+$", answer))


def fmt_num(x: float) -> str:
    """Positions/angles for MPA/MPR/MAA/MAR: integer where exact, else one
    decimal (the documented resolution of GP/GA answers)."""
    r = round(float(x), 1)
    return str(int(r)) if r == int(r) else f"{r:.1f}"


def parse(pattern: str, answer: str):
    """Match a full answer against a regex with {N} for a number; returns
    the captured groups as strings or raises ProtocolError."""
    rx = "^" + pattern.replace("{N}", _NUM) + "$"
    m = re.match(rx, answer)
    if not m:
        raise ProtocolError(f"unexpected answer {answer!r} (wanted /{pattern}/)")
    return m.groups()


def parse_status(ch: int, answer: str) -> int:
    (letter,) = parse(rf"M{ch}([A-Z])", answer)
    try:
        return STATUS_LETTERS.index(letter)
    except ValueError:
        raise ProtocolError(f"unknown status letter in {answer!r}") from None
