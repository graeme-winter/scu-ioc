import csv

import pytest

from scu_ioc.flyplot import analyse, main, summary


def _write_scan(tmp_path, speeds, step=50.0, first=-200.0, last=200.0, delay=0.1):
    """Synthetic fly logs: row r moves at speeds[r % 2] (+ then -) and
    reaches its first grid point `delay` s after its trigger."""
    log, traj = tmp_path / "fly.csv", tmp_path / "traj.csv"
    n = int(round((last - first) / step)) + 1
    grid = [first + i * step for i in range(n)]
    t = 0.0
    with open(log, "w", newline="") as lf, open(traj, "w", newline="") as tf:
        lw, tw = csv.writer(lf), csv.writer(tf)
        lw.writerow(["index", "row", "col", "fast_target", "slow_target", "t_trigger",
                     "t_cross", "dt_cross", "row_speed", "row_rms"])
        tw.writerow(["row", "t", "fast_pos"])
        idx = 0
        for r in range(4):
            d = 1 if r % 2 == 0 else -1
            v = speeds[r % 2]
            pts = grid if d > 0 else grid[::-1]
            t_trig = t
            x_start = pts[0] - d * v * delay
            dur = (abs(pts[-1] - x_start) + 20) / v
            for i in range(int(dur * 1000) + 1):           # 1 kHz samples
                tt = i / 1000.0
                tw.writerow([r, f"{t_trig + tt:.5f}", f"{x_start + d * v * tt:.3f}"])
            for c, x in enumerate(pts):
                tc = t_trig + abs(x - x_start) / v
                lw.writerow([idx, r, c, x, 0, f"{t_trig:.5f}", f"{tc:.5f}",
                             f"{tc - t_trig:.5f}", v, 0])
                idx += 1
            t = t_trig + dur + 1.0
    return str(log), str(traj)


def test_exact_speed_has_no_drift(tmp_path):
    log, traj = _write_scan(tmp_path, (2500.0, 2500.0))
    rows, step = analyse(log, traj, frame_rate=50)
    assert step == 50 and len(rows) == 4
    for r in rows:
        assert r.speed == pytest.approx(2500, rel=1e-4)
        assert r.rms < 0.01
        assert r.first_delay == pytest.approx(0.1, abs=1e-3)
        assert max(abs(e) for _, e in r.frame_error) < 0.05


def test_slow_rows_drift_behind_in_both_directions(tmp_path):
    # 1 % slow: 49.5 um per 20 ms frame, so 0.5 um behind per frame
    log, traj = _write_scan(tmp_path, (2475.0, 2475.0))
    rows, _ = analyse(log, traj, frame_rate=50)
    for r in rows:
        k, e = r.frame_error[-1]
        assert e == pytest.approx(-0.5 * k, abs=0.3)          # negative = behind
    text = summary(rows, 50.0, 50)
    assert "-1.00% vs 2500/s" in text and "+ rows" in text and "- rows" in text


def test_asymmetric_rows_reported_separately(tmp_path):
    log, traj = _write_scan(tmp_path, (2550.0, 2450.0))
    rows, _ = analyse(log, traj, frame_rate=50)
    assert {r.direction: round(r.speed) for r in rows} == {1: 2550, -1: 2450}
    ups = [r.frame_error[-1][1] for r in rows if r.direction > 0]
    downs = [r.frame_error[-1][1] for r in rows if r.direction < 0]
    assert all(e > 0 for e in ups) and all(e < 0 for e in downs)


def test_cli_writes_plot(tmp_path):
    pytest.importorskip("matplotlib")
    log, traj = _write_scan(tmp_path, (2500.0, 2490.0))
    out = tmp_path / "p.png"
    assert main([log, traj, "--frame-rate", "50", "-o", str(out)]) == 0
    assert out.stat().st_size > 10000
    assert main([log, traj, "--no-plot"]) == 0
