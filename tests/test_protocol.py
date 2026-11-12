import pytest

from scu_ioc import protocol as P


def test_frame():
    assert P.frame("GP0") == b":GP0\n"


def test_error_code():
    assert P.error_code("E0") == 0
    assert P.error_code("E17") == 17
    assert P.error_code("ST0T1") is None
    assert P.error_code("") is None


def test_unsolicited():
    assert P.is_unsolicited("C0")
    assert P.is_unsolicited("CE1E21")
    assert not P.is_unsolicited("CLF0F5000")


@pytest.mark.parametrize("x,s", [(1000, "1000"), (12.34, "12.3"), (-0.05, "-0.1"),
                                 (-13.5, "-13.5"), (2.0, "2")])
def test_fmt_num(x, s):
    assert P.fmt_num(x) == s


def test_parse():
    assert P.parse(r"P0P{N}", "P0P-13.5") == ("-13.5",)
    assert P.parse(r"A0A{N}R{N}", "A0A-187.2R0") == ("-187.2", "0")
    with pytest.raises(P.ProtocolError):
        P.parse(r"P0P{N}", "P1P3")


def test_parse_status():
    assert P.parse_status(0, "M0S") == P.STOPPED
    assert P.parse_status(2, "M2T") == P.TARGETING
    assert P.parse_status(1, "M1R") == P.REFERENCING
    with pytest.raises(P.ProtocolError):
        P.parse_status(0, "M0X")


def test_sensor_kinds():
    assert P.sensor_kind(21) == "linear"
    assert P.sensor_kind(19) == "rotary"
    assert P.sensor_symbol(21) == "L"
