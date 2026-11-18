"""Common interface for the ways of talking to an SCU (wire, sdk, ascii).

Backends are blocking and not thread safe; callers (the IOC, the CLI)
serialise access. Units are those listed in protocol.py.

Exceptions:
    protocol.CommandError   controller rejected a command; link is fine
    ConnectionLost          transport failure; close() and reconnect
"""

from __future__ import annotations

from dataclasses import dataclass

from . import protocol as P


class ConnectionLost(RuntimeError):
    pass


@dataclass
class DeviceInfo:
    model: str        # e.g. "SmarAct CU-3D"
    device_id: str    # serial / ID number
    firmware: str
    channels: int     # 1 or 3


@dataclass
class AxisConfig:
    sensor_present: bool
    sensor_type: int
    cl_max_frequency: int | None   # None where not readable
    safe_direction: int | None     # 0 backward, 1 forward; None if n/a
    movement: str | None = None    # "linear"/"rotary" if the controller says

    @property
    def rotary(self) -> bool:
        kind = self.movement or P.sensor_kind(self.sensor_type)
        return kind == "rotary"


class Backend:
    """Abstract SCU backend. Channel indices are 0-based."""

    name = "abstract"

    # -- connection --------------------------------------------------------
    def connect(self) -> DeviceInfo:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    # -- reads -------------------------------------------------------------
    def axis_config(self, ch: int) -> AxisConfig:
        raise NotImplementedError

    def status(self, ch: int) -> int:
        raise NotImplementedError

    def position(self, ch: int, rotary: bool) -> float:
        """um for linear, mdeg for rotary. Needs a sensor."""
        raise NotImplementedError

    def referenced(self, ch: int) -> bool:
        raise NotImplementedError

    def temperature(self) -> float | None:
        return None

    def hv(self) -> float | None:
        return None

    # -- commands ----------------------------------------------------------
    def move_abs(self, ch: int, target: float, hold_ms: int, rotary: bool) -> None:
        raise NotImplementedError

    def move_rel(self, ch: int, delta: float, hold_ms: int, rotary: bool) -> None:
        raise NotImplementedError

    def step(self, ch: int, steps: int, amplitude: int, frequency: int) -> None:
        raise NotImplementedError

    def stop(self, ch: int) -> None:
        raise NotImplementedError

    def calibrate(self, ch: int) -> None:
        raise NotImplementedError

    def reference(self, ch: int, hold_ms: int, auto_zero: bool) -> None:
        raise NotImplementedError

    def set_zero(self, ch: int) -> None:
        raise NotImplementedError

    def set_sensor_type(self, ch: int, code: int) -> None:
        raise NotImplementedError

    def set_cl_max_frequency(self, ch: int, hz: int) -> None:
        raise NotImplementedError

    def set_safe_direction(self, ch: int, forward: bool) -> None:
        raise NotImplementedError


def check_step_args(steps: int, amplitude: int, frequency: int) -> None:
    """Shared validation; unbounded moves are deliberately not allowed."""
    if not 1 <= abs(steps) < P.MAX_STEPS:
        raise ValueError(f"steps must satisfy 1 <= |steps| < {P.MAX_STEPS}")
    lo, hi = P.AMPLITUDE_RANGE
    if not lo <= amplitude <= hi:
        raise ValueError(f"amplitude must be {lo}..{hi} (1/10 V)")
    lo, hi = P.FREQUENCY_RANGE
    if not lo <= frequency <= hi:
        raise ValueError(f"frequency must be {lo}..{hi} Hz")
