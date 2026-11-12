import pytest

from scu_ioc.ascii import AsciiBackend
from scu_ioc.sim import SimSCU, serve_pty


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def sim(clock):
    return SimSCU(sensor_types=(21, 19, 0), clock=clock)


@pytest.fixture
def backend(sim):
    path, _ = serve_pty(sim)
    b = AsciiBackend(path, timeout=1.0)
    b.connect()
    yield b
    b.close()
