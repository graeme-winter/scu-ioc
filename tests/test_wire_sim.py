from scu_ioc import protocol as P
from scu_ioc import wire as W
from scu_ioc.wire_sim import WireSCU


def ask(sim, hdr, *vals):
    out = sim.handle(W.encode(hdr, *vals))
    assert len(out) == 1
    return W.decode_reply(out[0])


def test_identity():
    sim = WireSCU()
    assert ask(sim, W.GET, W.SEL_ID, 0xF7) == ("id", (604573714,))
    name, v = ask(sim, W.GET, W.SEL_FIRMWARE, 0)
    assert W.firmware_version(bytes(v)) == (5 << 24) | (1 << 16) | 192
    assert ask(sim, W.SET_MODE, 0) == ("ack", (0,))


def test_move_and_status(clock):
    sim = WireSCU(sensor_types=(21, 19, 0), clock=clock)
    assert ask(sim, W.MOVE_POS_ABS, 0, 1000, 0) == ("ack", (0,))
    assert ask(sim, W.GET, W.SEL_STATUS, 0) == ("status", (0, P.TARGETING))
    clock.advance(10)
    assert ask(sim, W.GET, W.SEL_STATUS, 0) == ("status", (0, P.STOPPED))
    assert ask(sim, W.GET, W.SEL_POSITION, 0) == ("position", (0, 1000))


def test_errors():
    sim = WireSCU(sensor_types=(21, 19, 0))
    assert ask(sim, W.GET, W.SEL_POSITION, 2) == ("error", (2, 130))
    assert ask(sim, W.GET, W.SEL_POSITION, 1) == ("error", (1, 131))
    assert ask(sim, W.MOVE_ANGLE_ABS, 0, 5, 0, 0) == ("error", (0, 131))
    assert ask(sim, W.GET_SYSTEM_PROP, 99)[0] == "error"


def test_properties():
    sim = WireSCU(sensor_types=(21, 19, 0))
    assert ask(sim, W.GET_CHANNEL_PROP, 1, W.PROP_MOVEMENT_TYPE) == ("channel_property", (1, 25, 1))
    assert ask(sim, W.GET_SYSTEM_PROP, W.PROP_TEMPERATURE) == ("system_property", (1, 31))
