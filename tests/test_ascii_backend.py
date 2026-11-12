import pytest

from scu_ioc import protocol as P
from scu_ioc.backend import ConnectionLost


def test_connect_info(backend, sim):
    assert sim.auto_error            # backend enabled auto error report
    info = backend.connect()
    assert info.model == "SmarAct CU-3D"
    assert info.device_id == "604573714"
    assert info.channels == 3


def test_axis_config(backend):
    c0 = backend.axis_config(0)
    assert c0.sensor_present and c0.sensor_type == 21 and not c0.rotary
    assert c0.cl_max_frequency == 5000 and c0.safe_direction == 1
    c1 = backend.axis_config(1)
    assert c1.rotary
    c2 = backend.axis_config(2)
    assert not c2.sensor_present and c2.safe_direction is None


def test_moves(backend, clock):
    backend.move_abs(0, 12.3, 0, False)
    assert backend.status(0) == P.TARGETING
    clock.advance(10)
    assert backend.status(0) == P.STOPPED
    assert backend.position(0, False) == pytest.approx(12.3)
    backend.move_rel(1, -500.5, 0, True)
    clock.advance(10)
    assert backend.position(1, True) == pytest.approx(-500.5)


def test_rejections(backend):
    with pytest.raises(P.CommandError) as e:
        backend.position(2, False)
    assert e.value.code == 19
    with pytest.raises(P.CommandError) as e:
        backend.move_abs(0, 1, 0, True)
    assert e.value.code == 20
    with pytest.raises(ValueError):
        backend.step(0, 30000, 1000, 1000)     # unbounded refused locally
    with pytest.raises(ValueError):
        backend.step(0, 10, 100, 1000)


def test_step_direction(backend, clock):
    backend.step(2, -100, 1000, 1000)
    assert backend.status(2) == P.MOVING
    clock.advance(1)
    assert backend.status(2) == P.STOPPED


def test_settings(backend):
    backend.set_cl_max_frequency(0, 2000)
    backend.set_safe_direction(0, False)
    backend.set_sensor_type(1, 21)
    c = backend.axis_config(0)
    assert c.cl_max_frequency == 2000 and c.safe_direction == 0
    assert not backend.axis_config(1).rotary


def test_system(backend):
    assert backend.temperature() == 31.0
    assert backend.hv() == 100.0


def test_timeout_is_connection_lost(backend, sim):
    sim.handle = lambda line: []          # controller goes silent
    backend.timeout = 0.2
    with pytest.raises(ConnectionLost):
        backend.status(0)
