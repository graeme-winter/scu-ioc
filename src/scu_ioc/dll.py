"""SCU3DControl.dll backend (Windows, stock USB firmware).

The DLL talks to the controller through FTDI's D2XX driver (FTD2XX.dll,
installed with the SmarAct/FTDI USB driver). Only synchronous (_S) calls
are used. The DLL is process-global: one backend instance per process.

DLL units are 1/10 um and 1/10 mdeg; this module converts to um / mdeg.
"""

from __future__ import annotations

import ctypes
import logging
import os
from ctypes import POINTER, byref, c_int, c_uint
from pathlib import Path

from . import protocol as P
from .backend import AxisConfig, Backend, ConnectionLost, DeviceInfo, check_step_args

log = logging.getLogger(__name__)

SA_OK = 0
SA_SYNCHRONOUS_COMMUNICATION = 0

SA_ERRORS = {
    1: "SA_INITIALIZATION_ERROR",
    2: "SA_NOT_INITIALIZED_ERROR",
    3: "SA_NO_DEVICES_FOUND_ERROR",
    4: "SA_TOO_MANY_DEVICES_ERROR",
    5: "SA_INVALID_DEVICE_INDEX_ERROR",
    6: "SA_INVALID_CHANNEL_INDEX_ERROR",
    7: "SA_TRANSMIT_ERROR",
    8: "SA_WRITE_ERROR",
    9: "SA_INVALID_PARAMETER_ERROR",
    10: "SA_READ_ERROR",
    12: "SA_INTERNAL_ERROR",
    13: "SA_WRONG_MODE_ERROR",
    14: "SA_PROTOCOL_ERROR",
    15: "SA_TIMEOUT_ERROR",
    17: "SA_ID_LIST_TOO_SMALL_ERROR",
    18: "SA_DEVICE_ALREADY_ADDED_ERROR",
    19: "SA_DEVICE_NOT_FOUND_ERROR",
    128: "SA_INVALID_COMMAND_ERROR",
    129: "SA_COMMAND_NOT_SUPPORTED_ERROR",
    130: "SA_NO_SENSOR_PRESENT_ERROR",
    131: "SA_WRONG_SENSOR_TYPE_ERROR",
    132: "SA_END_STOP_REACHED_ERROR",
    133: "SA_COMMAND_OVERRIDDEN_ERROR",
    134: "SA_HV_RANGE_ERROR",
    135: "SA_TEMP_OVERHEAT_ERROR",
    136: "SA_CALIBRATION_FAILED_ERROR",
    137: "SA_REFERENCING_FAILED_ERROR",
    138: "SA_NOT_PROCESSABLE_ERROR",
    255: "SA_OTHER_ERROR",
}

# Communication failures: drop the connection and re-initialise.
_LINK_ERRORS = {1, 2, 3, 7, 8, 10, 14, 15, 19}

# DLL device-error codes -> ASCII-protocol codes, so the IOC shows the
# same text for either backend.
_TO_ASCII = {6: 3, 9: 17, 128: 2, 130: 19, 131: 20, 132: 21, 134: 23,
             135: 24, 136: 25, 137: 26, 138: 27}


class DllCommandError(P.CommandError):
    def __init__(self, func: str, sa_code: int):
        self.sa_code = sa_code
        code = _TO_ASCII.get(sa_code, sa_code)
        super().__init__(code, func)
        if sa_code not in _TO_ASCII:
            self.args = (f"{SA_ERRORS.get(sa_code, 'unknown')} ({sa_code}) from {func}",)


U, I = c_uint, c_int
pU, pI = POINTER(c_uint), POINTER(c_int)
_SIGNATURES = {
    "SA_GetDLLVersion": [pU],
    "SA_GetAvailableDevices": [pU, pU],
    "SA_AddDeviceToInitDevicesList": [U],
    "SA_ClearInitDevicesList": [],
    "SA_InitDevices": [U],
    "SA_ReleaseDevices": [],
    "SA_GetNumberOfDevices": [pU],
    "SA_GetDeviceID": [U, pU],
    "SA_GetDeviceFirmwareVersion": [U, pU],
    "SA_SetClosedLoopMaxFrequency_S": [U, U, U],
    "SA_GetClosedLoopMaxFrequency_S": [U, U, pU],
    "SA_SetZero_S": [U, U],
    "SA_GetSensorPresent_S": [U, U, pU],
    "SA_SetSensorType_S": [U, U, U],
    "SA_GetSensorType_S": [U, U, pU],
    "SA_SetSafeDirection_S": [U, U, U],
    "SA_GetSafeDirection_S": [U, U, pU],
    "SA_GetSystemProperty_S": [U, I, pI],
    "SA_MoveStep_S": [U, U, I, U, U],
    "SA_MovePositionAbsolute_S": [U, U, I, U],
    "SA_MovePositionRelative_S": [U, U, I, U],
    "SA_MoveAngleAbsolute_S": [U, U, I, I, U],
    "SA_MoveAngleRelative_S": [U, U, I, I, U],
    "SA_CalibrateSensor_S": [U, U],
    "SA_MoveToReference_S": [U, U, U, U],
    "SA_Stop_S": [U, U],
    "SA_GetStatus_S": [U, U, pU],
    "SA_GetPosition_S": [U, U, pI],
    "SA_GetAngle_S": [U, U, pI, pI],
    "SA_GetPhysicalPositionKnown_S": [U, U, pU],
}


def default_dll_path() -> str:
    if os.environ.get("SCU_DLL"):
        return os.environ["SCU_DLL"]
    return "SCU3DControl.dll"


def load_dll(path: str | None = None):
    path = path or default_dll_path()
    if os.path.dirname(path) and hasattr(os, "add_dll_directory"):
        os.add_dll_directory(str(Path(path).resolve().parent))
    dll = ctypes.CDLL(path)   # no calling convention in the header -> cdecl
    for name, argtypes in _SIGNATURES.items():
        fn = getattr(dll, name)
        fn.argtypes = argtypes
        fn.restype = c_uint
    return dll


def _version(v: int) -> str:
    return f"{v >> 24}.{(v >> 16) & 0xFF}.{v & 0xFFFF}"


def _to_tenths(x: float) -> int:
    return int(round(float(x) * 10))


class DllBackend(Backend):
    name = "dll"

    def __init__(self, device_id: int | None = None, dll_path: str | None = None, dll=None):
        self.device_id = device_id
        self._dll_path = dll_path
        self._dll = dll
        self._open = False
        self.index = 0

    # -- plumbing ----------------------------------------------------------
    def _call(self, name: str, *args) -> None:
        code = getattr(self._dll, name)(*args)
        if code == SA_OK:
            return
        if code in _LINK_ERRORS:
            raise ConnectionLost(f"{name}: {SA_ERRORS.get(code, code)} ({code})")
        raise DllCommandError(name, code)

    def _get(self, name: str, *args, ctype=c_uint):
        v = ctype()
        self._call(name, *args, byref(v))
        return v.value

    # -- connection --------------------------------------------------------
    def connect(self) -> DeviceInfo:
        self.close()
        if self._dll is None:
            try:
                self._dll = load_dll(self._dll_path)
            except OSError as e:
                raise ConnectionLost(f"cannot load SCU3DControl.dll: {e}") from e
        self._call("SA_ClearInitDevicesList")
        if self.device_id is not None:
            self._call("SA_AddDeviceToInitDevicesList", self.device_id)
        self._call("SA_InitDevices", SA_SYNCHRONOUS_COMMUNICATION)
        self._open = True
        if self._get("SA_GetNumberOfDevices") < 1:
            raise ConnectionLost("no SCU devices acquired")
        device_id = self._get("SA_GetDeviceID", self.index)
        firmware = _version(self._get("SA_GetDeviceFirmwareVersion", self.index))
        # 1D devices reject channel indices > 0.
        try:
            self._get("SA_GetStatus_S", self.index, 2)
            channels = 3
        except DllCommandError:
            channels = 1
        return DeviceInfo(f"SmarAct SCU-{channels}D (USB)", str(device_id), firmware, channels)

    def close(self) -> None:
        if self._open:
            try:
                self._dll.SA_ReleaseDevices()
            except Exception:
                pass
            self._open = False

    # -- reads -------------------------------------------------------------
    def axis_config(self, ch):
        d = self.index
        present = bool(self._get("SA_GetSensorPresent_S", d, ch))
        stype = self._get("SA_GetSensorType_S", d, ch)
        clf = self._get("SA_GetClosedLoopMaxFrequency_S", d, ch)
        try:
            sd = self._get("SA_GetSafeDirection_S", d, ch)
        except P.CommandError:
            sd = None
        return AxisConfig(present, stype, clf, sd)

    def status(self, ch):
        return self._get("SA_GetStatus_S", self.index, ch)

    def position(self, ch, rotary):
        if rotary:
            a, r = c_int(), c_int()
            self._call("SA_GetAngle_S", self.index, ch, byref(a), byref(r))
            return a.value / 10.0
        return self._get("SA_GetPosition_S", self.index, ch, ctype=c_int) / 10.0

    def referenced(self, ch):
        return bool(self._get("SA_GetPhysicalPositionKnown_S", self.index, ch))

    def temperature(self):
        try:
            return float(self._get("SA_GetSystemProperty_S", self.index, 1, ctype=c_int))
        except P.CommandError:
            return None

    def hv(self):
        try:
            return self._get("SA_GetSystemProperty_S", self.index, 2, ctype=c_int) / 10.0
        except P.CommandError:
            return None

    # -- commands ----------------------------------------------------------
    def move_abs(self, ch, target, hold_ms, rotary):
        if rotary:
            self._call("SA_MoveAngleAbsolute_S", self.index, ch, _to_tenths(target), 0, int(hold_ms))
        else:
            self._call("SA_MovePositionAbsolute_S", self.index, ch, _to_tenths(target), int(hold_ms))

    def move_rel(self, ch, delta, hold_ms, rotary):
        if rotary:
            self._call("SA_MoveAngleRelative_S", self.index, ch, _to_tenths(delta), 0, int(hold_ms))
        else:
            self._call("SA_MovePositionRelative_S", self.index, ch, _to_tenths(delta), int(hold_ms))

    def step(self, ch, steps, amplitude, frequency):
        check_step_args(steps, amplitude, frequency)
        self._call("SA_MoveStep_S", self.index, ch, int(steps), int(amplitude), int(frequency))

    def stop(self, ch):
        self._call("SA_Stop_S", self.index, ch)

    def calibrate(self, ch):
        self._call("SA_CalibrateSensor_S", self.index, ch)

    def reference(self, ch, hold_ms, auto_zero):
        self._call("SA_MoveToReference_S", self.index, ch, int(hold_ms), 1 if auto_zero else 0)

    def set_zero(self, ch):
        self._call("SA_SetZero_S", self.index, ch)

    def set_sensor_type(self, ch, code):
        self._call("SA_SetSensorType_S", self.index, ch, int(code))

    def set_cl_max_frequency(self, ch, hz):
        self._call("SA_SetClosedLoopMaxFrequency_S", self.index, ch, int(hz))

    def set_safe_direction(self, ch, forward):
        self._call("SA_SetSafeDirection_S", self.index, ch, 1 if forward else 0)


def available_devices(dll_path: str | None = None, dll=None) -> list[int]:
    """IDs of connected SCUs not yet acquired by any process."""
    dll = dll or load_dll(dll_path)
    ids = (c_uint * 32)()
    n = c_uint(len(ids))
    code = dll.SA_GetAvailableDevices(ids, byref(n))
    if code == 3:
        return []
    if code != SA_OK:
        raise DllCommandError("SA_GetAvailableDevices", code)
    return [ids[i] for i in range(n.value)]
