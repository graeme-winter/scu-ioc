import struct

import pytest

from scu_ioc import wire as W


@pytest.mark.parametrize("table", [W.REQUESTS, W.REPLIES])
def test_formats_match_header_lengths(table):
    for hdr, (name, fmt) in table.items():
        size = struct.calcsize(fmt) if fmt else 0
        assert size == W.payload_len(hdr), f"{name} 0x{hdr:02x}"


def test_encode_matches_sdk_bytes():
    # byte sequences observed from libscu3dcontrol.so 1.5.16
    assert W.encode(W.MOVE_POS_ABS, 0, 1234, 0) == bytes.fromhex("27 00 d2 04 00 00 00 00")
    assert W.encode(W.MOVE_STEP, 2, -150, 600, 2000) == bytes.fromhex("17 02 6a ff 58 02 d0 07")
    assert W.encode(W.MOVE_ANGLE_REL, 1, 500, 0, 0) == bytes.fromhex("39 01 f4 01 00 00 00 00 00 00")
    assert W.encode(W.MOVE_TO_REF, 0, 0, 1) == bytes.fromhex("44 00 00 00 01")
    assert W.encode(W.GET, W.SEL_STATUS, 2) == bytes.fromhex("12 02 02")
    assert W.encode(W.GET_CHANNEL_PROP, 0, W.PROP_MOVEMENT_TYPE) == bytes.fromhex("53 00 19 00")
    assert W.encode(W.GET_SYSTEM_PROP, W.PROP_TEMPERATURE) == bytes.fromhex("b2 01 00")
    assert W.encode(W.SET_MODE, 0) == bytes.fromhex("81 00")


def test_decode_replies():
    assert W.decode_reply(bytes.fromhex("14 12 10 09 24")) == ("id", (604573714,))
    assert W.decode_reply(bytes.fromhex("25 00 16 ff ff ff")) == ("position", (0, -234))
    assert W.decode_reply(bytes.fromhex("47 01 37 f6 ff ff 00 00")) == ("angle", (1, -2505, 0))
    assert W.decode_reply(bytes.fromhex("e2 02 82")) == ("error", (2, 130))
    with pytest.raises(W.FrameError):
        W.decode_reply(bytes.fromhex("25 00 16"))


def test_firmware_version():
    assert W.firmware_version(bytes.fromhex("c0 00 77 01 05")) == (5 << 24) | (1 << 16) | 192


def test_split_frames():
    frames, rest = W.split_frames(bytes.fromhex("22 00 03 a1 01 25 00 01"))
    assert frames == [bytes.fromhex("22 00 03"), bytes.fromhex("a1 01")]
    assert rest == bytes.fromhex("25 00 01")


def test_describe():
    assert W.describe(bytes.fromhex("12 00 f7"), "W") == "get id (unused byte 0xf7)"
    assert W.describe(bytes.fromhex("12 03 01"), "W") == "get position ch=1"
    assert W.describe(bytes.fromhex("15 c0 00 00 01 05"), "R").startswith("firmware 5.1.192")
    assert W.describe(bytes.fromhex("f1 00"), "W").startswith("unknown_0xf1")
