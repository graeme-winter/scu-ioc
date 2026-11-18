import csv

import pytest

from scu_ioc import protocol as P
from scu_ioc.fly import (calibrate_frequency, crossing_times, fit_speed, run_fly)
from scu_ioc.raster import Line, Trigger
from scu_ioc.wire_backend import WireBackend
from scu_ioc.wire_sim import WireSCU, serve_pty


def test_fit_speed():
    s = [(t / 100, 10.0 * t) for t in range(100)]           # 1000 um/s
    f = fit_speed(s, 100, 800)
    assert f.speed == pytest.approx(1000) and f.rms < 1e-9 and f.n == 71
    assert fit_speed(s[:2], 0, 10) is None
    down = [(t, -x) for t, x in s]
    assert fit_speed(down, -800, -100).speed == pytest.approx(1000)


def test_crossing_times():
    s = [(0.0, 0.0), (1.0, 10.0), (2.0, 20.0)]
    assert crossing_times(s, [5.0, 10.0, 15.0, 30.0]) == [0.5, 1.0, 1.5, None]
    back = [(0.0, 20.0), (1.0, 10.0), (2.0, 0.0)]
    assert crossing_times(back, [15.0, 0.0]) == [0.5, 2.0]


class Count(Trigger):
    n = 0

    def fire(self):
        self.n += 1


@pytest.fixture
def stage():
    sim = WireSCU(sensor_types=(21, 21, 0))
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


def test_calibration_converges_and_clamps(stage):
    b, sim = stage
    f, fit = calibrate_frequency(b, 0, False, 0, 300, 800.0, 2000, log=lambda *_: None)
    assert f == pytest.approx(8000, rel=0.01) and fit.speed == pytest.approx(800, rel=0.05)
    # 0.1 um/step in the sim, so 18500 Hz caps the speed at 1850 um/s
    f, fit = calibrate_frequency(b, 0, False, 0, 300, 5000.0, 2000, log=lambda *_: None)
    assert f == 18500 and fit.speed == pytest.approx(1850, rel=0.05)


def test_fly_scan(stage, tmp_path):
    b, sim = stage
    trig = Count()
    log, traj = tmp_path / "fly.csv", tmp_path / "traj.csv"
    r = run_fly(b, Line(0, -100, 100, 20), Line(1, 0, 20, 10), frame_rate=50,
                run_up=20, trigger=trig, log_path=str(log), trajectory_path=str(traj),
                log=lambda *_: None)
    assert r.aborted is None and r.rows_done == 3 and trig.n == 3
    assert r.frequencies[+1] == pytest.approx(10000, rel=0.01)   # 20 um x 50 Hz = 1000 um/s
    assert r.frequencies[-1] == pytest.approx(10000, rel=0.01)
    assert all(f.speed == pytest.approx(1000, rel=0.05) for f in r.row_fits)
    rows = list(csv.DictReader(open(log)))
    assert len(rows) == 33 and all(x["t_cross"] for x in rows)
    # serpentine: row 1 runs from +100 back to -100
    r1 = [float(x["fast_target"]) for x in rows if x["row"] == "1"]
    assert r1 == sorted(r1, reverse=True)
    # first grid point is crossed run_up / speed = 20 ms after the trigger
    assert float(rows[0]["dt_cross"]) == pytest.approx(0.02, abs=0.005)
    # crossings are evenly spaced at the frame period
    dts = [float(x["dt_cross"]) for x in rows if x["row"] == "0"]
    gaps = [b_ - a_ for a_, b_ in zip(dts, dts[1:])]
    assert max(gaps) == pytest.approx(0.02, abs=0.003) and min(gaps) == pytest.approx(0.02, abs=0.003)
    # closed-loop frequency restored afterwards (sim default 5000)
    assert b.axis_config(0).cl_max_frequency == 5000


def test_fly_abort_restores_frequency(stage):
    b, sim = stage

    def progress(r, n, fit, times, t_trig, elapsed):
        raise KeyboardInterrupt

    r = run_fly(b, Line(0, 0, 100, 20), Line(1, 0, 20, 10), cl_freq=12000,
                progress=progress, log=lambda *_: None)
    assert r.aborted == "interrupted" and r.rows_done == 1
    assert b.axis_config(0).cl_max_frequency == 5000


def test_fly_needs_two_points(stage):
    b, _ = stage
    with pytest.raises(ValueError):
        run_fly(b, Line(0, 5, 5, 10), Line(1, 0, 10, 10))


def test_measure_speed_detects_end_stop(stage):
    from scu_ioc.fly import measure_speed
    from scu_ioc.raster import RasterAborted
    b, _ = stage
    with pytest.raises(RasterAborted, match="end stop"):
        measure_speed(b, 0, False, 8000, 12000, 18500)     # sim end stop at 10500


def test_fly_row_into_end_stop_aborts(stage):
    b, _ = stage
    r = run_fly(b, Line(0, 9800, 10400, 100), Line(1, 0, 0, 10), cl_freq=18500,
                run_up=300, log=lambda *_: None)
    assert r.aborted and "run-down was cut short" in r.aborted


def test_speed_command_centres_on_zero(stage, capsys):
    from scu_ioc import cli
    b, sim = stage
    b.move_abs(0, 9000, 0, False)
    while b.status(0) in P.BUSY:
        pass
    port = b.port
    b.close()
    assert cli.main(["--backend", "wire", "--port", port, "speed", "A",
                     "--freq", "18500", "--distance", "2000"]) == 0
    out = capsys.readouterr().out
    assert "between -1000.0 and 1000.0" in out and "1850.0" in out and "+0.0%" in out


def test_asymmetric_slide_gets_per_direction_frequencies(tmp_path):
    sim = WireSCU(sensor_types=(21, 21, 0), asymmetry=0.1)   # +10% / -10%
    path, _ = serve_pty(sim)
    b = WireBackend(path)
    b.connect()
    for ch in (0, 1):
        b.reference(ch, 0, True)
    for ch in (0, 1):
        while b.status(ch) in P.BUSY:
            pass
    r = run_fly(b, Line(0, -100, 100, 20), Line(1, 0, 30, 10), frame_rate=50,
                run_up=20, log=lambda *_: None)
    assert r.aborted is None
    assert r.frequencies[+1] == pytest.approx(10000 / 1.1, rel=0.03)
    assert r.frequencies[-1] == pytest.approx(10000 / 0.9, rel=0.03)
    assert r.row_dirs == [1, -1, 1, -1]
    # both directions now run at the target speed
    assert all(f.speed == pytest.approx(1000, rel=0.05) for f in r.row_fits)
    b.close()
