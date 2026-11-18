"""WireBackend against (a) exchanges captured from real hardware and
(b) the wire simulator over a pty."""

import collections
from pathlib import Path

import pytest

from scu_ioc import protocol as P
from scu_ioc import wire as W
from scu_ioc.backend import ConnectionLost
from scu_ioc.wire_backend import WireBackend
from scu_ioc.wire_sim import WireSCU, serve_pty

CAPTURE = Path(__file__).parent / "data" / "scu3d-fw5.1.192-exchanges.txt"


def _key(req: bytes) -> bytes:
    # the SDK leaves the third byte of ID/firmware gets uninitialised
    if req[0] == W.GET and req[1] in (W.SEL_ID, W.SEL_FIRMWARE):
        return req[:2]
    return req


class ReplaySerial:
    """Serial stand-in answering each request with the reply the real
    controller gave to the identical request in the capture. A request the
    SDK never sent fails the test: our encoder must match the SDK's bytes."""

    def __init__(self, path=CAPTURE):
        self.replies = collections.defaultdict(list)
        for line in path.read_text().splitlines():
            if line.startswith("#") or not line.strip():
                continue
            req, rep = line.split()
            self.replies[_key(bytes.fromhex(req))].append(bytes.fromhex(rep))
        self.used = collections.Counter()
        self.rx = b""
        self.sent = []

    def write(self, data):
        k = _key(bytes(data))
        assert k in self.replies, f"request never sent by the SDK: {bytes(data).hex(' ')}"
        options = self.replies[k]
        self.rx += options[min(self.used[k], len(options) - 1)]
        self.used[k] += 1
        self.sent.append(bytes(data))
        return len(data)

    def read(self, n):
        out, self.rx = self.rx[:n], self.rx[n:]
        return out

    def reset_input_buffer(self):
        self.rx = b""

    def close(self):
        pass


@pytest.fixture
def replay():
    port = ReplaySerial()
    b = WireBackend("replay", serial_factory=lambda: port, timeout=0.2)
    return b, port


def test_replay_connect(replay):
    b, port = replay
    info = b.connect()
    assert info.device_id == "654905362"
    assert info.firmware == "5.1.192"
    assert info.channels == 3
    assert port.sent[2] == bytes.fromhex("81 00")      # acked with A1 FF


def test_replay_config_and_reads(replay):
    b, _ = replay
    b.connect()
    c = b.axis_config(0)
    assert (c.sensor_present, c.sensor_type, c.cl_max_frequency, c.safe_direction) == (True, 21, 10000, 0)
    assert c.movement is None          # firmware rejects the property: E2 FF 09
    assert not c.rotary
    assert b.position(0, False) == pytest.approx(0.8)
    assert b.temperature() == 24.0
    assert b.hv() == pytest.approx(98.6)
    assert b.referenced(0) is False
    assert not b.axis_config(2).sensor_present


def test_replay_commands_match_sdk_bytes(replay):
    b, port = replay
    b.connect()
    b.set_cl_max_frequency(0, 10000)
    b.set_sensor_type(0, 21)
    b.set_safe_direction(0, False)
    b.step(0, 50, 600, 1000)
    b.step(0, -50, 600, 1000)
    b.move_rel(0, 20.0, 0, False)
    b.move_rel(0, -20.0, 0, False)
    b.move_abs(1, 10.0, 300, False)
    b.stop(0)
    assert P.STATUS_NAMES[b.status(0)] in P.STATUS_NAMES


def test_replay_errors(replay):
    b, _ = replay
    b.connect()
    with pytest.raises(P.CommandError) as e:
        b.position(0, True)                 # angle on a linear axis
    assert e.value.code == 20
    with pytest.raises(P.CommandError) as e:
        b.position(2, False)                # no sensor
    assert e.value.code == 19
    with pytest.raises(P.CommandError) as e:
        b.move_rel(2, 20.0, 0, False)
    assert e.value.code == 19


class ScriptSerial(ReplaySerial):
    def __init__(self, reply):
        self.reply = reply
        self.rx = b""

    def write(self, data):
        self.rx += self.reply
        return len(data)


def test_desync_and_timeout():
    b = WireBackend("x", serial_factory=lambda: ScriptSerial(bytes.fromhex("25 00 00 00 00 00")),
                    timeout=0.1)
    with pytest.raises(P.ProtocolError):
        b._ser = ScriptSerial(bytes.fromhex("25 00 00 00 00 00"))
        b.status(0)                          # position reply to a status request
    b._ser = ScriptSerial(b"")
    with pytest.raises(ConnectionLost):
        b.status(0)
    b._ser = ScriptSerial(bytes.fromhex("22 01 00"))
    with pytest.raises(P.ProtocolError):
        b.status(0)                          # reply for the wrong channel


# -- against the simulator over a real pty ------------------------------------

@pytest.fixture
def simb(clock):
    sim = WireSCU(sensor_types=(21, 19, 0), clock=clock)
    path, _ = serve_pty(sim)
    b = WireBackend(path, timeout=1.0)
    yield b, sim, clock
    b.close()


def test_sim_connect_and_moves(simb):
    b, sim, clock = simb
    info = b.connect()
    assert info.channels == 3 and info.firmware == "5.1.192"
    assert b.axis_config(1).rotary
    b.move_abs(0, 123.4, 0, False)
    assert b.status(0) == P.TARGETING
    clock.advance(10)
    assert b.status(0) == P.STOPPED
    assert b.position(0, False) == pytest.approx(123.4)
    b.move_rel(1, -250.5, 0, True)
    clock.advance(10)
    assert b.position(1, True) == pytest.approx(-250.5)
    b.reference(0, 0, True)
    clock.advance(60)
    assert b.referenced(0)
    with pytest.raises(ValueError):
        b.step(0, 30000, 1000, 1000)


def test_device_id_check(simb):
    b, _, _ = simb
    b.device_id = 1234
    with pytest.raises(ConnectionLost):
        b.connect()
