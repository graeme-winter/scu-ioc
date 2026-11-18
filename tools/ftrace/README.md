# ftrace: capturing SCU wire traffic

Tools for confirming the binary protocol map (`docs/wire-protocol.md`)
against a real SCU, using SmarAct's own library on an x86‑64 Linux host.

| File | Purpose |
| --- | --- |
| `ftrace.c` → `libftrace.so` | LD_PRELOAD shim: logs every D2XX call and every byte read or written. Read‑only. |
| `capture_session.py` | Drives every command the IOC uses, with `# STEP` markers in the trace. |
| `decode.py` | Annotates a trace using `wire.py` and flags anything outside the map. |
| `fakeftd2xx.c` → `libfakeftd2xx.so` | Testing only: stands in for libftd2xx and forwards to a pty, so the vendor library can run against `scu-wire-sim`. |

## Build

```sh
make -C tools/ftrace          # builds libftrace.so (needs gcc)
```

## Capture against the real SCU

Set up the host as in the main README's Linux section (SDK libraries in
`/opt/smaract/lib`, the udev rule so `ftdi_sio` lets go and the device is
accessible). Check `scu list --lib /opt/smaract/lib/libscu3dcontrol.so`
shows the controller. Then, with the stage clear of obstructions:

```sh
export LIB=/opt/smaract/lib/libscu3dcontrol.so
SCU_FTRACE=scu-capture.trace LD_PRELOAD=$PWD/tools/ftrace/libftrace.so \
    python3 tools/ftrace/capture_session.py --lib $LIB
python3 tools/ftrace/decode.py scu-capture.trace | less
python3 tools/ftrace/decode.py --summary scu-capture.trace
```

The default session:

- moves each sensor axis ±20 µm (or m°) closed loop and back;
- steps 50 open‑loop steps each way;
- does an absolute move with a 300 ms hold, and a stop mid‑move;
- writes the closed‑loop frequency, sensor type and safe direction back
  with **their current values**, which captures the commands without
  changing the controller's stored settings;
- provokes the expected rejections (wrong sensor kind, no sensor).

Use `--travel` and `--steps` to change the move sizes, and `--channels 0,1`
to skip channels. These options travel several mm, so only add them when
the stage is free to move:

- `--calibrate`: sensor calibration (several mm both ways)
- `--reference`: move to the reference mark
- `--end-stop`: drive to the end stop in the safe direction

To capture the IOC itself, run it under the shim the same way. Polling makes
the trace large, so a few minutes is plenty:

```sh
SCU_FTRACE=ioc.trace LD_PRELOAD=$PWD/tools/ftrace/libftrace.so \
    scu-ioc --lib $LIB --prefix TEST:SCU: --no-shell
```

`SCU_FTRACE_STATUS=1` also logs `FT_GetStatus` calls (only used in async
mode).

## What to look for

`decode.py --summary` exits non‑zero and lists problems if it sees:

- a header the map doesn't know, in either direction;
- a reply other than the one the map expects for a request;
- an ack for the wrong channel;
- bytes left unread, a D2XX error, or a read timeout.

Its "requests seen" and "replies seen" lists also show which parts of the
map the capture covered. The open questions at the end of
`docs/wire-protocol.md` are the other things to check in the trace,
especially the status sequence and timing during real moves.

## Validating the map without hardware

`tests/test_wire_sdk.py` runs the vendor library against `scu-wire-sim`
through `libfakeftd2xx.so` and the shim, runs the capture session, and
requires a clean decode:

```sh
SCU_SDK_LIB=/opt/smaract/lib/libscu3dcontrol.so pytest tests/test_wire_sdk.py
```

To do the same by hand:

```sh
make -C tools/ftrace libftrace.so libfakeftd2xx.so
mkdir -p /tmp/fakelib
ln -sf $LIB /tmp/fakelib/libscu3dcontrol.so
ln -sf $PWD/tools/ftrace/libfakeftd2xx.so /tmp/fakelib/libftd2xx.so
scu-wire-sim --link /tmp/scu-wire -v &
SCU_FAKE_FTDI_PORT=/tmp/scu-wire SCU_FTRACE=sim.trace \
    LD_PRELOAD=$PWD/tools/ftrace/libftrace.so \
    scu --lib /tmp/fakelib/libscu3dcontrol.so status
```

## Licence note

The traces and the map are for interoperability with our own hardware.
Check SmarAct's SDK licence before sharing either outside the group.
