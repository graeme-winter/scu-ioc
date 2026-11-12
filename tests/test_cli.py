from scu_ioc import cli
from scu_ioc.sim import SimSCU, serve_pty


def test_cli_against_sim(capsys):
    sim = SimSCU(sensor_types=(21, 19, 0))
    path, _ = serve_pty(sim)
    base = ["--backend", "ascii", "--port", path]
    assert cli.main(base + ["status"]) == 0
    out = capsys.readouterr().out
    assert "CU-3D" in out and "A: Stopped" in out and "no sensor" in out
    assert cli.main(base + ["rel", "A", "25.5"]) == 0
    assert "pos 25.5 um" in capsys.readouterr().out
    assert cli.main(base + ["move", "B", "-120"]) == 0
    assert "pos -120.0 mdeg" in capsys.readouterr().out
    assert cli.main(base + ["move", "C", "5"]) == 1
    assert "No sensor present" in capsys.readouterr().err
    assert cli.main(base + ["raw", "GCLF0"]) == 0
    assert capsys.readouterr().out.strip() == "CLF0F5000"
