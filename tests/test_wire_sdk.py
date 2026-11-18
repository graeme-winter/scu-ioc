"""Check the wire map against SmarAct's real libscu3dcontrol.so.

The vendor library talks, through tools/ftrace/libfakeftd2xx.so and the
capture shim, to the wire simulator. If the library accepts every reply and
the decoder finds nothing outside the map, the map agrees with the vendor's
own encoder and parser.

Skipped unless SCU_SDK_LIB points at libscu3dcontrol.so (x86-64 Linux) and
a C compiler is available. The SDK itself is not part of this repository.
"""

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SDK_LIB = os.environ.get("SCU_SDK_LIB")

pytestmark = pytest.mark.skipif(
    not SDK_LIB or not Path(SDK_LIB).exists() or not shutil.which("cc")
    or not sys.platform.startswith("linux"),
    reason="set SCU_SDK_LIB to libscu3dcontrol.so (and have cc) to run")


def test_vendor_library_against_simulator(tmp_path):
    from scu_ioc.wire_sim import WireSCU, serve_pty

    src = ROOT / "tools" / "ftrace"
    shim = tmp_path / "libftrace.so"
    subprocess.run(["cc", "-O2", "-shared", "-fPIC", "-o", str(shim), str(src / "ftrace.c"),
                    "-ldl", "-lpthread"], check=True)
    libdir = tmp_path / "lib"
    libdir.mkdir()
    subprocess.run(["cc", "-O2", "-shared", "-fPIC", "-Wl,-soname,libftd2xx.so",
                    "-o", str(libdir / "libftd2xx.so"), str(src / "fakeftd2xx.c")], check=True)
    (libdir / "libscu3dcontrol.so").symlink_to(Path(SDK_LIB).resolve())

    sim = WireSCU(sensor_types=(21, 19, 0))
    port, _ = serve_pty(sim)
    trace = tmp_path / "trace.log"
    env = dict(os.environ, SCU_FAKE_FTDI_PORT=port, SCU_FTRACE=str(trace),
               LD_PRELOAD=str(shim), PYTHONPATH=str(ROOT / "src"))
    r = subprocess.run([sys.executable, str(src / "capture_session.py"),
                        "--lib", str(libdir / "libscu3dcontrol.so"),
                        "--travel", "10", "--steps", "20", "--calibrate", "--reference"],
                       env=env, capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "rejected: No sensor present" in r.stdout          # ch2 has no sensor
    d = subprocess.run([sys.executable, str(src / "decode.py"), "--summary", str(trace)],
                       capture_output=True, text=True)
    assert d.returncode == 0, d.stdout
    assert "no problems" in d.stdout
