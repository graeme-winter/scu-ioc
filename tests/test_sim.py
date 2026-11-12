from scu_ioc import protocol as P


def test_error_modes(sim):
    assert sim.handle("S0") == []            # non-auto: silent
    assert sim.handle("XYZ0") == []
    assert sim.handle("E") == ["E2"]
    assert sim.handle("E") == ["E0"]         # register cleared on read
    assert sim.handle("E1") == ["E0"]
    assert sim.handle("S0") == ["E0"]
    assert sim.handle("XYZ0") == ["E2"]
    assert sim.handle("GP5") == ["E3"]


def test_identification(sim):
    assert sim.handle("I") == ["ISmarAct CU-3D"]
    assert sim.handle("GID") == ["ID604573714"]
    assert sim.handle("V") == ["V1.2.5"]


def test_closed_loop_move(sim, clock):
    sim.handle("E1")
    assert sim.handle("GP0") == ["P0P0.0"]   # power-up position reads 0
    assert sim.handle("MPA0P100H0") == ["E0"]
    assert sim.handle("M0") == ["M0T"]
    clock.advance(0.1)                        # 500 um/s default
    assert sim.handle("GP0") == ["P0P50.0"]
    clock.advance(1.0)
    assert sim.handle("M0") == ["M0S"]
    assert sim.handle("GP0") == ["P0P100.0"]


def test_hold(sim, clock):
    sim.handle("E1")
    sim.handle("MPR0P10H200")
    clock.advance(0.05)
    assert sim.handle("M0") == ["M0H"]
    clock.advance(0.2)
    assert sim.handle("M0") == ["M0S"]


def test_rotary_and_sensorless(sim):
    sim.handle("E1")
    assert sim.handle("GP1") == ["E20"]
    assert sim.handle("GA1") == ["A1A0.0R0"]
    assert sim.handle("MPA1P5") == ["E20"]
    assert sim.handle("GP2") == ["E19"]
    assert sim.handle("GSP2") == ["SP2N"]


def test_open_loop(sim, clock):
    sim.handle("E1")
    assert sim.handle("U2F1000A1000S100") == ["E0"]
    assert sim.handle("M2") == ["M2M"]
    clock.advance(1.0)
    assert sim.handle("M2") == ["M2S"]
    assert sim.handle("U2F1000A100S100") == ["E17"]   # amplitude < 15 V


def test_reference_and_zero(sim, clock):
    sim.handle("E1")
    assert sim.handle("GPPK0") == ["PPK0K0"]
    sim.handle("MTR0H0Z1")
    assert sim.handle("M0") == ["M0R"]
    clock.advance(30)
    assert sim.handle("M0") == ["M0S"]
    assert sim.handle("GPPK0") == ["PPK0K1"]
    assert sim.handle("GP0") == ["P0P0.0"]
    sim.handle("MPA0P250")
    clock.advance(10)
    sim.handle("SZ0")
    assert sim.handle("GP0") == ["P0P0.0"]


def test_end_stop(sim, clock):
    sim.handle("E1")
    sim.handle("MPR0P50000")
    clock.advance(1000)
    st = sim.handle("M0")
    assert st == ["M0S"]
    pos = float(sim.handle("GP0")[0][3:])
    assert pos < 50000                        # clamped by the end stop


def test_calibrate(sim, clock):
    sim.handle("E1")
    sim.handle("CS0")
    assert sim.handle("M0") == ["M0C"]
    clock.advance(1.5)
    assert sim.handle("M0") == ["M0S"]


def test_stop_all_and_status_all(sim):
    sim.handle("E1")
    sim.handle("U99F100A1000S1000")
    assert sim.handle("M99") == ["M0M", "M1M", "M2M"]
    sim.handle("S99")
    assert sim.handle("M99") == ["M0S", "M1S", "M2S"]
