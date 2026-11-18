"""End to end: scu-sim on a pty, scu-ioc against it, caproto as client.
Skipped unless caproto is installed. Takes ~15 s."""

import os
import subprocess
import sys
import time
import uuid

import pytest

caproto = pytest.importorskip("caproto")
from caproto.threading.client import Context  # noqa: E402

ENV = dict(os.environ, EPICS_CA_AUTO_ADDR_LIST="NO", EPICS_CA_ADDR_LIST="127.0.0.1",
           EPICS_CAS_INTF_ADDR_LIST="127.0.0.1")


@pytest.fixture(scope="module", params=["ascii", "wire"])
def ioc(request, tmp_path_factory):
    backend = request.param
    link = str(tmp_path_factory.mktemp("scu") / "pty")
    prefix = f"T{uuid.uuid4().hex[:6].upper()}:SCU:"
    simmod = "scu_ioc.sim" if backend == "ascii" else "scu_ioc.wire_sim"
    sim = subprocess.Popen([sys.executable, "-m", simmod, "--link", link,
                            "--sensors", "21,19,0"], env=ENV,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    for _ in range(50):
        if os.path.exists(link):
            break
        time.sleep(0.1)
    ioc = subprocess.Popen([sys.executable, "-m", "scu_ioc.ioc", "--backend", backend,
                            "--port", link, "--prefix", prefix, "--no-shell"], env=ENV,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for k, v in ENV.items():
        if k.startswith("EPICS_CA_"):
            os.environ[k] = v
    ctx = Context()

    class Client:
        def __init__(self):
            self.pvs = {}

        def pv(self, n):
            if n not in self.pvs:
                self.pvs[n] = ctx.get_pvs(prefix + n, timeout=5)[0]
            return self.pvs[n]

        def get(self, n):
            r = self.pv(n).read(timeout=5)
            v = r.data
            if r.data_type in (caproto.ChannelType.CHAR,):
                return bytes(v).rstrip(b"\0").decode()
            v = v[0]
            return v.decode() if isinstance(v, bytes) else v.item()

        def put(self, n, v):
            self.pv(n).write([v], wait=True, timeout=5)

        def wait_done(self, axis, timeout=20):
            t0 = time.time()
            while time.time() - t0 < timeout:
                if self.get(axis + ":DMOV") == 1:
                    return
                time.sleep(0.05)
            raise TimeoutError(axis)

    c = Client()
    t0 = time.time()
    while True:
        try:
            if c.get("CONNECTED") == 1:
                break
        except Exception:
            pass
        if time.time() - t0 > 20:
            raise RuntimeError("IOC did not connect")
        time.sleep(0.2)
    yield c
    ioc.terminate()
    sim.terminate()
    ioc.wait(5)
    sim.wait(5)


def test_identity(ioc):
    assert ioc.get("MODEL") in ("SmarAct CU-3D", "SmarAct SCU-3D (USB)")
    assert ioc.get("DEVICE_ID") == "604573714"
    assert ioc.get("A:SENSOR_NAME") == "L"
    assert ioc.get("B:SENSOR_NAME") == "LR"
    assert ioc.get("C:SENSOR_PRESENT") == 0


def test_absolute_move(ioc):
    ioc.put("A:VAL", 250.0)
    assert ioc.get("A:DMOV") == 0          # dropped before put completes
    ioc.wait_done("A")
    assert ioc.get("A:RBV") == pytest.approx(250.0)


def test_tweak_and_relative(ioc):
    ioc.put("A:VAL", 100.0)
    ioc.wait_done("A")
    ioc.put("A:TWV", 12.5)
    ioc.put("A:TWF", 1)
    ioc.wait_done("A")
    ioc.put("A:RLV", -50.0)
    ioc.wait_done("A")
    assert ioc.get("A:VAL") == pytest.approx(62.5)
    assert ioc.get("A:RBV") == pytest.approx(62.5)


def test_soft_limits(ioc):
    ioc.put("A:HLM", 1000.0)
    ioc.put("A:LLM", -1000.0)
    try:
        before = ioc.get("A:RBV")
        ioc.put("A:VAL", 5000.0)
        time.sleep(0.5)
        assert ioc.get("A:DMOV") == 1
        assert ioc.get("A:VAL") == pytest.approx(before)
        assert "soft limits" in ioc.get("A:LAST_ERROR")
    finally:
        ioc.put("A:HLM", 0.0)
        ioc.put("A:LLM", 0.0)


def test_stop(ioc):
    ioc.put("A:VAL", 9000.0)
    time.sleep(0.3)
    ioc.put("A:STOP", 1)
    ioc.wait_done("A")
    assert ioc.get("A:RBV") < 9000
    assert ioc.get("A:VAL") == pytest.approx(ioc.get("A:RBV"))


def test_open_loop_and_rejection(ioc):
    ioc.put("C:STEP", -100)
    ioc.wait_done("C")
    ioc.put("C:VAL", 10.0)
    time.sleep(0.5)
    assert "No sensor present" in ioc.get("C:LAST_ERROR")
    assert ioc.get("C:DMOV") == 1


def test_reference(ioc):
    ioc.put("B:AUTO_ZERO", 1)
    ioc.put("B:REFERENCE", 1)
    ioc.wait_done("B", timeout=30)
    assert ioc.get("B:REFERENCED") == 1
    assert ioc.get("B:RBV") == pytest.approx(0.0)
