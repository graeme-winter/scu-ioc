"""DllBackend against a fake DLL object (the real one is Windows-only)."""

import pytest

from scu_ioc import protocol as P
from scu_ioc.backend import ConnectionLost
from scu_ioc.dll import DllBackend, available_devices


class FakeDll:
    def __init__(self, channels=3):
        self.calls = []
        self.pos = [0, 0, 0]
        self.channels = channels
        self.fail = {}

    def __getattr__(self, name):
        def f(*args):
            self.calls.append((name, args))
            if name in self.fail:
                return self.fail[name]
            h = type(self).__dict__.get("_" + name)
            return h(self, *args) if h else 0
        return f

    @staticmethod
    def _out(ref, v):
        ref._obj.value = v

    def _SA_GetAvailableDevices(self, ids, n):
        ids[0] = 604573714
        self._out(n, 1)
        return 0

    def _SA_GetNumberOfDevices(self, n):
        self._out(n, 1); return 0

    def _SA_GetDeviceID(self, d, v):
        self._out(v, 604573714); return 0

    def _SA_GetDeviceFirmwareVersion(self, d, v):
        self._out(v, (1 << 24) | (2 << 16) | 5); return 0

    def _SA_GetStatus_S(self, d, ch, v):
        if ch >= self.channels:
            return 6
        self._out(v, 0); return 0

    def _SA_GetSensorPresent_S(self, d, ch, v):
        self._out(v, 1 if ch < 2 else 0); return 0

    def _SA_GetSensorType_S(self, d, ch, v):
        self._out(v, 21 if ch != 1 else 19); return 0

    def _SA_GetClosedLoopMaxFrequency_S(self, d, ch, v):
        self._out(v, 5000); return 0

    def _SA_GetSafeDirection_S(self, d, ch, v):
        if ch == 2:
            return 130
        self._out(v, 1); return 0

    def _SA_GetPosition_S(self, d, ch, v):
        self._out(v, self.pos[ch]); return 0

    def _SA_MovePositionAbsolute_S(self, d, ch, p, h):
        self.pos[ch] = p; return 0

    def _SA_GetSystemProperty_S(self, d, key, v):
        self._out(v, {1: 33, 2: 998}[key]); return 0


@pytest.fixture
def fake():
    return FakeDll()


@pytest.fixture
def b(fake):
    be = DllBackend(dll=fake)
    be.connect()
    return be


def test_connect(fake):
    b = DllBackend(device_id=604573714, dll=fake)
    info = b.connect()
    assert info.device_id == "604573714"
    assert info.firmware == "1.2.5"
    assert info.channels == 3
    assert ("SA_AddDeviceToInitDevicesList", (604573714,)) in fake.calls
    b.close()
    assert fake.calls[-1][0] == "SA_ReleaseDevices"


def test_one_channel(fake):
    fake.channels = 1
    assert DllBackend(dll=fake).connect().channels == 1


def test_units(b, fake):
    b.move_abs(0, 12.34, 100, False)
    assert ("SA_MovePositionAbsolute_S", (0, 0, 123, 100)) in fake.calls
    assert b.position(0, False) == pytest.approx(12.3)
    b.move_rel(1, -1.5, 0, True)
    assert ("SA_MoveAngleRelative_S", (0, 1, -15, 0, 0)) in fake.calls
    b.step(2, -200, 600, 2000)
    assert ("SA_MoveStep_S", (0, 2, -200, 600, 2000)) in fake.calls
    assert b.temperature() == 33.0 and b.hv() == pytest.approx(99.8)


def test_config(b):
    c = b.axis_config(2)
    assert not c.sensor_present and c.safe_direction is None
    assert b.axis_config(1).rotary


def test_error_mapping(b, fake):
    fake.fail["SA_GetPosition_S"] = 130
    with pytest.raises(P.CommandError) as e:
        b.position(0, False)
    assert e.value.code == 19
    fake.fail["SA_GetStatus_S"] = 15        # SA_TIMEOUT_ERROR
    with pytest.raises(ConnectionLost):
        b.status(0)


def test_available_devices(fake):
    assert available_devices(dll=fake) == [604573714]
    fake.fail["SA_GetAvailableDevices"] = 3
    assert available_devices(dll=fake) == []
