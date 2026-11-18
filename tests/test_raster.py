import csv
import threading

import pytest

from scu_ioc import protocol as P
from scu_ioc.raster import (Line, SerialLineTrigger, Trigger, make_trigger, run_raster,
                            serpentine)
from scu_ioc.wire_backend import WireBackend
from scu_ioc.wire_sim import WireSCU, serve_pty


def test_line_points():
    assert Line(0, 0, 300, 100).points == [0, 100, 200, 300]
    assert Line(0, 300, 0, 100).points == [300, 200, 100, 0]
    assert Line(0, 5, 5, 10).points == [5]
    assert Line(0, -1.5, 1.5, 0.5).points == pytest.approx([-1.5, -1, -0.5, 0, 0.5, 1, 1.5])
    with pytest.raises(ValueError):
        Line(0, 0, 1000, 300).points
    with pytest.raises(ValueError):
        Line(0, 0, 100, 0).points


def test_serpentine_order():
    pts = serpentine(Line(0, 0, 2, 1), Line(1, 10, 20, 10))
    assert [(p.row, p.col, p.fast, p.slow) for p in pts] == [
        (0, 0, 0, 10), (0, 1, 1, 10), (0, 2, 2, 10),
        (1, 2, 2, 20), (1, 1, 1, 20), (1, 0, 0, 20)]
    assert [p.index for p in pts] == list(range(6))


class CountingTrigger(Trigger):
    def __init__(self):
        self.n = 0

    def fire(self):
        self.n += 1


@pytest.fixture
def stage():
    sim = WireSCU(sensor_types=(21, 21, 0))
    for a in sim.axes[:2]:
        a.cl_freq = 18500            # fast sim moves: 1850 um/s
    path, _ = serve_pty(sim)
    b = WireBackend(path)
    b.connect()
    for ch in (0, 1):
        b.reference(ch, 0, True)
    for ch in (0, 1):
        while b.status(ch) in P.BUSY:
            pass
    yield b, sim
    b.close()


def test_raster_runs_and_logs(stage, tmp_path):
    b, sim = stage
    trig = CountingTrigger()
    log = tmp_path / "r.csv"
    r = run_raster(b, Line(0, 0, 60, 20), Line(1, -20, 0, 20), trigger=trig,
                   log_path=str(log), tolerance=1.0)
    assert r.aborted is None and r.points_done == 8 and trig.n == 8
    rows = list(csv.DictReader(open(log)))
    assert [(float(x["fast_target"]), float(x["slow_target"])) for x in rows] == [
        (0, -20), (20, -20), (40, -20), (60, -20), (60, 0), (40, 0), (20, 0), (0, 0)]
    for x in rows:
        assert float(x["fast_pos"]) == pytest.approx(float(x["fast_target"]), abs=0.1)
        assert float(x["t_trigger"]) >= float(x["t_settled"]) >= float(x["t_move"])
    # only the fast axis moves within a row: one slow move per row
    slow_moves = [f for f, _ in sim.log if f[0] == 0x27 and f[1] == 1]
    assert len(slow_moves) == 2


def test_raster_refuses_unreferenced_and_sensorless():
    sim = WireSCU(sensor_types=(21, 21, 0))
    path, _ = serve_pty(sim)
    b = WireBackend(path)
    b.connect()
    from scu_ioc.raster import RasterAborted
    with pytest.raises(RasterAborted, match="not referenced"):
        run_raster(b, Line(0, 0, 10, 10), Line(1, 0, 10, 10))
    with pytest.raises(RasterAborted, match="no sensor"):
        run_raster(b, Line(0, 0, 10, 10), Line(2, 0, 10, 10), allow_unreferenced=True)


def test_raster_stop_event_stops_axes(stage):
    b, sim = stage
    stop = threading.Event()

    def progress(pt, done, total, elapsed):
        if done == 3:
            stop.set()

    r = run_raster(b, Line(0, 0, 100, 10), Line(1, 0, 10, 10), stop_event=stop,
                   progress=progress, dwell=0.05)
    assert r.aborted == "stopped by request" and r.points_done == 3
    assert any(f[0] == 0x11 for f, _ in sim.log[-3:])       # stop sent


def test_raster_tolerance_abort(stage):
    b, sim = stage
    real = b.position
    b.position = lambda ch, rot: real(ch, rot) + (5.0 if ch == 0 else 0.0)
    r = run_raster(b, Line(0, 0, 20, 10), Line(1, 0, 0, 10), tolerance=1.0)
    assert r.aborted and "tolerance" in r.aborted and r.points_done == 0


class FakeLines:
    def __init__(self):
        self.events = []

    def __setattr__(self, k, v):
        if k in ("dtr", "rts"):
            self.events.append((k, v))
        object.__setattr__(self, k, v)

    def close(self):
        pass


def test_serial_trigger_polarity():
    f = FakeLines()
    t = SerialLineTrigger("x", "rts", 0.1, active_low=True, ser=f)
    t.fire()
    assert f.events == [("rts", False), ("rts", True), ("rts", False)]
    f2 = FakeLines()
    SerialLineTrigger("x", "dtr", 0.1, active_low=False, ser=f2).fire()
    assert f2.events == [("dtr", True), ("dtr", False), ("dtr", True)]


def test_make_trigger_parsing(monkeypatch):
    assert make_trigger("none").describe() == "none"
    seen = {}

    class Stub:
        def __init__(self, port, line, width, active_low):
            seen.update(port=port, line=line, width=width, low=active_low)

    monkeypatch.setattr("scu_ioc.raster.SerialLineTrigger", Stub)
    make_trigger("serial:/dev/ttyUSB1:rts:high", 2.0)
    assert seen == dict(port="/dev/ttyUSB1", line="rts", width=2.0, low=False)
    make_trigger("serial:socket://h:4000", 1.0)
    assert seen["port"] == "socket://h:4000" and seen["line"] == "dtr" and seen["low"]
    with pytest.raises(ValueError):
        make_trigger("gpio:17")
