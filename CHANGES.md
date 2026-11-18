# Changes

## 0.2.0

**Running without SmarAct's software**
- `wire` backend: talks to stock-firmware USB SCUs directly over the FTDI
  serial port (500 kbaud), so it runs on Linux x86/ARM and macOS with no
  SDK. The protocol was reconstructed from libscu3dcontrol.so 1.5.16
  (`docs/wire-protocol.md`) and checked three ways: against SmarAct's library,
  by replaying exchanges captured from a CU-3D (firmware 5.1.192), and on that
  hardware.
- `tools/ftrace/`: an LD_PRELOAD capture shim for libftd2xx traffic, a trace
  decoder, a safe capture session, and a fake libftd2xx for running the vendor
  library against a simulator.
- `scu-wire-sim`: binary-protocol simulator, optionally with
  direction-dependent or custom speed curves.

**SDK backend**
- `sdk` backend (renamed from `dll`, which is kept as an alias) supports the
  Linux SDK (libscu3dcontrol.so) as well as the Windows DLL:
  - per-platform library names, and libftd2xx preloaded from the library's
    directory;
  - lenient symbol binding;
  - the movement-type property, plus positioner type codes 25-41.
- `--lib` (alias `--dll`) points at the library.

**Scans**
- `scu raster`: serpentine step-and-shoot raster with trigger outputs
  (DTR/RTS pulse on a USB-serial adapter, or an EPICS put) and a per-point
  CSV log.
- `scu raster --fly`: the fast axis flies, with one trigger per row.
  - Speed is calibrated per direction with secant steps (default tolerance
    1%), and the fast axis is sampled during each row.
  - The logs record grid-crossing times and the measured slow position.
  - `--run-up`, `--trajectory` and `--sample-ms` options.
- `scu speed`: closed-loop speed against frequency, in both directions.
- `scu-flyplot`: per-row smoothness, frame position error, speed and
  trigger-delay plots; a sample map; a per-frame position table
  (`--frames-out`).

**Fixes**
- `quit()`/`exit()` in the IOC shell no longer raise a traceback, and the
  controller is released on every exit path.
- Moves that stop short of their target (end stops) are detected in speed
  measurement and fly scans.

**Docs**
- README reorganised around backends, host setup (Linux, Raspberry Pi,
  macOS, Windows), the IOC, the bench tool, scans and analysis. It includes
  measured SLC-1740 behaviour and an updated list of known gaps.

## 0.1.0

- pythonSoftIOC IOC with motor-record-like PVs per axis, a DLL backend
  (Windows) and an ASCII backend, an ASCII simulator, the `scu` bench tool, and
  tests including Channel Access end to end.
