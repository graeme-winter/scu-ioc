#!/usr/bin/env python3
"""Annotate an ftrace log (from libftrace.so) using the wire-protocol map.

    python3 tools/ftrace/decode.py /tmp/scu.trace            # conversation
    python3 tools/ftrace/decode.py --summary /tmp/scu.trace  # just the checks

Reassembles each handle's reads into frames by the header length nibble,
pairs requests with replies, and prints them with decoded fields. The
summary lists every request/reply header seen and flags anything the map
does not know, length mismatches, and replies that don't match what the
map expects for the request (the things to look at after a real capture).
"""

from __future__ import annotations

import argparse
import collections
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from scu_ioc import wire as W  # noqa: E402

LINE = re.compile(r"^(?P<t>[\d.]+) (?P<dir>[WR]) h=(?P<h>\S+) n=(?P<n>\d+) -> st=(?P<st>\d+) "
                  r"(?:done|got)=(?P<k>\d+) \|(?P<bytes>[0-9a-f ]*)$")
OTHER = re.compile(r"^(?P<t>[\d.]+) (?P<what>FT_\w+.*)$")

# Reply header the map expects for each request (None: ack or error).
def expected_reply(req: bytes):
    if req[0] == W.GET and len(req) >= 2:
        return W.GET_REPLY.get(req[1])
    if req[0] == W.GET_CHANNEL_PROP:
        return W.R_CHANNEL_PROP
    if req[0] == W.GET_SYSTEM_PROP:
        return W.R_SYSTEM_PROP
    if req[0] == W.HW_RESET:
        return "raw"
    return W.R_ACK


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("trace")
    ap.add_argument("--summary", action="store_true", help="only print the summary")
    a = ap.parse_args(argv)

    rx = collections.defaultdict(bytes)
    last_req = {}
    seen_req = collections.Counter()
    seen_rep = collections.Counter()
    problems = []
    out = [] if a.summary else None

    def emit(s):
        if out is None:
            print(s)

    for lineno, line in enumerate(Path(a.trace).read_text().splitlines(), 1):
        if line.startswith("#"):
            emit(line)
            continue
        m = LINE.match(line)
        if not m:
            o = OTHER.match(line)
            emit(f"{o['t']:>10}    {o['what']}" if o else line)
            continue
        h, t = m["h"], m["t"]
        data = bytes.fromhex(m["bytes"].replace(" ", ""))
        if m["st"] != "0":
            problems.append(f"line {lineno}: D2XX status {m['st']} on {m['dir']}")
        if m["dir"] == "W":
            if rx[h]:
                problems.append(f"line {lineno}: {len(rx[h])} unread reply bytes discarded: {rx[h].hex(' ')}")
                rx[h] = b""
            frames, rest = W.split_frames(data)
            if rest or not frames:
                problems.append(f"line {lineno}: write is not whole frames: {data.hex(' ')}")
            for f in frames:
                desc = W.describe(f, "W")
                seen_req[f[0]] += 1
                if desc.startswith("unknown"):
                    problems.append(f"line {lineno}: unknown request {f.hex(' ')}")
                emit(f"{t:>10} -> {f.hex(' '):32s} {desc}")
                last_req[h] = f
            continue
        # read
        if int(m["k"]) < int(m["n"]):
            emit(f"{t:>10}    (read wanted {m['n']}, got {m['k']}: timeout?)")
        rx[h] += data
        req = last_req.get(h)
        if req is not None and req[0] == W.HW_RESET and len(rx[h]) == 1 and seen_rep.get("reset_ack", 0) == 0:
            emit(f"{t:>10} <- {rx[h].hex():32s} (reset acknowledge byte)")
            seen_rep["reset_ack"] += 1
            rx[h] = b""
            continue
        frames, rest = W.split_frames(rx[h])
        rx[h] = rest
        for f in frames:
            desc = W.describe(f, "R")
            seen_rep[f[0]] += 1
            emit(f"{t:>10} <- {f.hex(' '):32s} {desc}")
            if desc.startswith("unknown"):
                problems.append(f"line {lineno}: unknown reply {f.hex(' ')}")
            if req is not None:
                exp = expected_reply(req)
                if f[0] not in (exp, W.R_ERROR) and exp != "raw":
                    problems.append(f"line {lineno}: reply {f.hex(' ')} to {req.hex(' ')}; "
                                    f"map expected header 0x{exp:02x}")
                if f[0] == W.R_ACK and req[0] not in (W.SET_MODE,) and f[1:2] != req[1:2]:
                    problems.append(f"line {lineno}: ack channel {f[1]} != request channel {req[1]}")

    print("\n== requests seen")
    for hdr, n in sorted(seen_req.items()):
        print(f"  0x{hdr:02x} {W.REQUESTS.get(hdr, ('UNKNOWN',))[0]:28s} x{n}")
    print("== replies seen")
    for hdr, n in sorted((k, v) for k, v in seen_rep.items() if isinstance(k, int)):
        print(f"  0x{hdr:02x} {W.REPLIES.get(hdr, ('UNKNOWN',))[0]:28s} x{n}")
    print("== problems" if problems else "== no problems: everything matches the map")
    for p in problems:
        print("  " + p)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
