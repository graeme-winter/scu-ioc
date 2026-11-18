# scu-ioc

EPICS soft IOC for SmarAct SCU piezo controllers (CU-1D/3D and HCU-1D/3D,
with or without sensor module), written against SmarAct's *SCU ASCII
Programming Interface* v2.0-C and *SCU Programmer's Guide* v2.0-C
(SCU3DControl 1.4.x on Windows, libscu3dcontrol 1.5.16 on Linux). Pure Python:
[pythonSoftIOC](https://github.com/DiamondLightSource/pythonSoftIOC), plus
pyserial. No EPICS build, no motor module.

There are two ways to talk to an SCU, selected with `--backend`:

| Backend | Hardware | Host |
| --- | --- | --- |
| `sdk` (default; `dll` is an alias) | USB SCUs with the stock firmware (e.g. CU-3D-USB-OEM as shipped) | Windows (SCU3DControl.dll) or Linux x86/x86-64 (libscu3dcontrol.so); both use FTDI's D2XX driver |
| `wire` | the same stock-firmware USB SCUs, without the SDK: speaks the firmware's binary protocol over its FTDI serial port | anything with pyserial and an FTDI driver, e.g. a Raspberry Pi |
| `ascii` | RS-232 SCUs, or USB SCUs reflashed with SmarAct's ASCII firmware (they then appear as an FTDI virtual COM port) | anything with pyserial, including `socket://` device servers |

The stock USB firmware does not answer ASCII commands. The SCU user manual
says an ASCII firmware is available for the USB versions on request from
SmarAct, and the SCU software package includes the Firmware Uploader. That
is the route to running this on Linux or in a container.

Layout:

| Module | Role |
| --- | --- |
| `protocol.py` | Status codes, sensor types, error texts; ASCII framing and parsing |
| `backend.py` | Backend interface (blocking, um / mdeg / 1/10 V / Hz units) |
| `ascii.py` | ASCII serial backend |
| `dll.py` | SDK backend: ctypes binding of SCU3DControl.dll / libscu3dcontrol.so |
| `sim.py` | ASCII-protocol simulator; `scu-sim` serves it on a pty |
| `ioc.py` | pythonSoftIOC records and worker thread; `scu-ioc` entry point |
| `cli.py` | `scu` bench tool over either backend |
| `wire.py` | SCU binary protocol map (stock USB firmware), reconstructed; see `docs/wire-protocol.md` |
| `wire_backend.py` | `wire` backend: the binary protocol over pyserial |
| `wire_sim.py` | Binary‑protocol simulator; `scu-wire-sim` serves it on a pty |

`tools/ftrace/` has the capture shim, trace decoder and capture session
used to check the binary protocol map against real hardware.

## Install and run

```sh
python -m venv .venv && . .venv/bin/activate     # Windows: .venv\Scripts\activate
pip install .                                     # or: pip install -e '.[dev]' && pytest
```

### Windows, stock USB firmware

Install the SmarAct SCU software (it installs the FTDI USB driver and
FTD2XX.dll), close the SCU Configuration Tool, then:

```bat
set SCU_DLL=C:\path\to\SCU\SDK\lib64\SCU3DControl.dll
scu-ioc --prefix 24IDC:SCU: --device-id 604573714
```

Use `SDK\lib64` with 64-bit Python and `SDK\lib` with 32-bit. `--lib`
(alias `--dll`) does the same as `SCU_DLL`. `--device-id` is optional with one controller
attached; `scu list` shows what is connected. Allow Python through the
Windows firewall for Channel Access (UDP/TCP 5064, UDP 5065).

### Linux, stock USB firmware

The Linux SDK (1.5.16) has the same C API as the Windows DLL; only x86 and
x86-64 builds are supplied, so no ARM. Copy
`SDK/Redistributables/x86_64/` somewhere sensible, e.g. `/opt/smaract/lib`,
and point the IOC at it:

```sh
scu list --lib /opt/smaract/lib/libscu3dcontrol.so
scu-ioc --lib /opt/smaract/lib/libscu3dcontrol.so --prefix 24IDC:SCU:
```

libscu3dcontrol.so needs libftd2xx.so but carries no RPATH. The backend
preloads libftd2xx.so from the same directory, so LD_LIBRARY_PATH is not
needed when they sit together (as in Redistributables).

Two host settings matter, because D2XX talks to the device through usbfs
rather than a tty:

1. **The kernel's `ftdi_sio` driver must not hold the SCU.** If it does,
   the SCU appears as `/dev/ttyUSB*` and the SDK reports an initialisation
   error. `sudo rmmod ftdi_sio` is fine for a quick test, but it
   detaches every FTDI serial adapter on the host.
2. **The IOC user needs write access** to the device node under
   `/dev/bus/usb`.

A udev rule handles both for just this device. Find its IDs with `lsusb`
and `udevadm info -a -n /dev/ttyUSB0` (while ftdi_sio still has it). The
SDK doesn't set a custom VID/PID, so expect FTDI's 0403:6001. Then create
`/etc/udev/rules.d/99-smaract-scu.rules`:

```
# SmarAct SCU: usable via libftd2xx by group plugdev, with ftdi_sio detached
SUBSYSTEM=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="<SERIAL>", MODE="0660", GROUP="plugdev"
ACTION=="add", SUBSYSTEM=="usb", DRIVER=="ftdi_sio", ATTRS{serial}=="<SERIAL>", RUN+="/bin/sh -c 'echo %k > /sys/bus/usb/drivers/ftdi_sio/unbind'"
```

Reload with `sudo udevadm control --reload && sudo udevadm trigger`, then
replug. `scu list` should show the device ID.

**In a container:** keep the udev rule on the host. In the container, pass
`/dev/bus/usb` (or the specific bus/device node), put the SDK libraries in
the image, and give the IOC `--lib`. A device attached over USB/IP is
handled the same way on the host that imports it.

### Raspberry Pi, or any host without the SDK (`wire` backend)

The `wire` backend talks to a stock-firmware USB SCU through the kernel's
`ftdi_sio` driver, as an ordinary serial port at 500000 baud. So here,
unlike with the SDK, `ftdi_sio` should **keep** the device. Two settings
matter:

- **Latency timer.** FTDI's default is 16 ms, which makes every exchange
  take ~16 ms. The SDK sets 1 ms, at which the firmware answers in about
  0.5 ms. The IOC logs a warning if it finds 16.
- **A stable name.** `/dev/serial/by-id/usb-SmarAct_SCU-3D_<serial>-if00-port0`
  already exists; the rule below also adds `/dev/smaract-scu`.

Find the serial with `udevadm info -a -n /dev/ttyUSB0 | grep 'ATTRS{serial}'`,
then create `/etc/udev/rules.d/99-smaract-scu-wire.rules`:

```
# SmarAct SCU via ftdi_sio: stable name, dialout access, 1 ms latency
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="<SERIAL>", SYMLINK+="smaract-scu", GROUP="dialout", MODE="0660"
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTRS{serial}=="<SERIAL>", ATTR{latency_timer}="1"
```

Reload with `sudo udevadm control --reload && sudo udevadm trigger`, then
replug. Then:

```sh
scu --backend wire --port /dev/smaract-scu status
scu-ioc --backend wire --port /dev/smaract-scu --prefix 24IDC:SCU:
```

`--device-id 654905362` makes the IOC refuse any other controller on that
port. `--port` also accepts pyserial URLs, so a serial server (e.g. ser2net
on a USB/IP box) works if it runs the port raw at 500000 8N1, with the latency
timer set to 1 ms on that host. In a container, pass the tty with `--device`.

Don't use the SDK udev rule (which unbinds `ftdi_sio`) on the same host.

### Serial (RS-232 or ASCII firmware)

```sh
scu-ioc --backend ascii --port /dev/serial/by-id/usb-FTDI_...-if00-port0 --prefix 24IDC:SCU:
scu-ioc --backend ascii --port socket://moxa1:4001 --prefix 24IDC:SCU:
```

The factory baud rate is 9600 (`--baud` to change). The IOC user needs to
be in `dialout` for local serial ports.

### Without hardware

```sh
scu-sim --link /tmp/scu &                  # --sensors 21,19,0 for linear, rotary, no sensor
scu-ioc --backend ascii --port /tmp/scu --prefix TEST:SCU:
```

Other options: `--axes X,Y,-` names channels A, B, C in PVs (`-` skips a
channel); `--poll` sets the idle poll period (default 1 s) and
`--busy-poll` the period while anything moves or holds (default 0.1 s);
`--no-shell` disables the interactive Python shell (use under procServ);
`-v` logs all controller traffic.

## Raster scans

`scu raster` runs a serpentine step‑and‑shoot raster over two axes,
directly on the controller (any backend). Nothing else can use the
controller while it runs, so stop the IOC first. At each point it:

1. moves (absolute, closed loop) and waits for the controller to report
   the target reached;
2. waits `--settle` seconds;
3. reads back both axes (the readback is checked against `--tolerance` if given);
4. fires the trigger;
5. dwells for `--dwell` seconds.

Rows alternate direction, so within a row only the fast axis moves.

```sh
# 25 x 25 mm at 50 um pitch, centred on the reference position, fast = A (X), slow = B (Z)
scu --backend wire --port /dev/smaract-scu raster \
    --fast A -12500 12500 50 --slow B -12500 12500 50 \
    --settle 0.005 --dwell 0.02 --tolerance 2 \
    --trigger serial:/dev/ttyUSB1:dtr --pulse 1 --log run042.csv

# check the plan and a time estimate without moving
scu raster --fast A -12500 12500 50 --slow B -12500 12500 50 --dry-run --speed 1000
```

Each axis is given as `AXIS START END STEP` in µm (millidegrees for rotary
axes). The range must be a whole number of steps, and the direction comes
from START and END. Both axes must have sensors and be referenced
(`--allow-unreferenced` overrides). Ctrl‑C, a move timeout, a controller
error or a tolerance failure stops both axes and keeps the partial log.

There's no hardware position compare, so a trigger here means "the stage
has stopped at this point". Software jitter, of order a millisecond, affects
*when* a shot fires, not *where*. Triggers:

| `--trigger` | Output |
| --- | --- |
| `none` | no output; just dwell |
| `serial:PORT[:dtr\|rts][:low\|high]` | pulses DTR or RTS on a separate USB‑serial adapter for `--pulse` ms. On TTL adapters an asserted line is electrically low, hence the default `low`. Wire it to a Zebra or detector TTL input (check levels and termination). |
| `ca:PV[=VALUE]` | writes VALUE (default 1) to an EPICS PV and waits for completion, e.g. a Zebra soft input or a detector trigger PV. Needs `pip install '.[ca]'`. |

`--log` writes one CSV row per point: index, row, column, targets, measured
positions, and times relative to the start (move commanded, settled,
triggered). That's the shot‑to‑position map for the data.

`--hold MS` keeps the controller correcting at each target, which helps
if the stage creeps during long dwells.

### Fly mode

With `--fly`, the fast axis moves continuously along each row and the
trigger fires once, at the start of each row's move. This is meant for a
detector that free‑runs at a fixed frame rate from that trigger. The slow
axis steps between rows, and rows alternate direction.

```sh
# 50 um pitch at 100 Hz frames -> 5 mm/s target speed
scu --backend wire --port /dev/smaract-scu raster \
    --fast A -12500 12500 50 --slow B -12500 12500 50 --fly --frame-rate 100 \
    --run-up 200 --trigger serial:/dev/ttyUSB1:dtr --log run043.csv --trajectory run043-traj.csv
```

**Speed.** The SCU has no velocity setting. In a long closed‑loop move the
speed is set by the closed‑loop maximum step frequency (50–18,500 Hz) times
the positioner's step length, which depends on the slide, load and
amplitude, and it can differ between directions. So the speed is calibrated:
the target is `--fly-speed` µm/s, or the fast STEP × `--frame-rate`. Test
moves over the first row's span measure the speed. The frequency is adjusted
(secant steps, since speed against frequency can be lumpy on stick‑slip
drives) until the speed is within `--calibration-tolerance` (default 1%, up
to 8 moves). If that isn't reached, the best frequency found is used.
This is done **separately for each direction**,
because serpentine rows alternate, so + rows and − rows can use different
frequencies. `--fly-freq HZ` skips calibration and uses one frequency both
ways. The fast axis's original frequency is restored afterwards. To see how
your slides behave:

```sh
scu --backend wire --port /dev/smaract-scu speed A --freq 1000 2000 5000 10000 18500 --distance 2000
```

This prints, for each frequency, the speed in each direction, the
asymmetry between them, the mean step length per Hz, the non‑uniformity, and
the number and spacing of position samples. Test moves are centred on
`--center` (default 0), and a move that stops short (an end stop) ends the
test.

**What gets recorded.** During each row the fast axis is sampled about
every millisecond on the `wire` backend with a 1 ms latency timer. Each
sample is stamped at the midpoint of its exchange.
- The row's speed is fitted over the grid region, and the progress lines
  report it with its rms deviation from constant speed.
- `--log` gives, for every grid point, the time it was crossed and the time
  since that row's trigger (`dt_cross`).
- `--trajectory` keeps every sample. Use `--sample-ms` to thin it, since a
  full 25 mm square at full rate means millions of rows.

With a detector started by the trigger, frame *k* of a row is at
*k*/rate after the trigger. The log turns that into a position, so a speed
that's slightly off becomes a known position error, not an unknown one.

**Run‑up.** `--run-up` adds travel before and after each row, so the stage
is at constant speed over the grid. Check the run‑up still fits within
the travel. The first grid point is reached about run‑up/speed after the
trigger (also logged), so the detector should take frames for that long
plus the row.

**Plots.** `scu-flyplot` reads the two logs (`pip install '.[plot]'` for
the plot; the summary works without it):

```sh
scu-flyplot run043.csv run043-traj.csv --frame-rate 100 -o run043.png
```

It plots four panels:
- deviation from constant speed against position for every row, + and − rows
  in different colours;
- frame position error against frame number, along the direction of travel
  (negative means behind). This is where a detector free‑running from the
  row trigger actually took each frame, compared with the ideal pitch, so a
  speed error shows up as a straight‑line drift;
- fitted speed per row;
- trigger‑to‑first‑grid‑point delay per row.

It also prints a per‑direction summary including the drift at the end of the
rows.

Two more outputs show where the sample went:

```sh
scu-flyplot run043.csv run043-traj.csv --frame-rate 100 --map run043-map.png --frames-out run043-frames.csv
```

- `--map` draws two panels: the stage's actual path over the whole scan,
  and every detector frame at its actual position, coloured by its error along
  the direction of travel. The colour scale covers 99.5% of frames, so a single
  bad row saturates instead of washing out the rest. Patterns tied to the
  slide show as vertical stripes, drift over the scan as a change down the
  map, and a one‑off disturbance as a single streak.
- `--frames-out` writes one line per detector frame, assuming it free‑runs from
  each row trigger at `--frame-rate`. Each line has the row, frame number,
  time, actual fast and slow positions, and, for frames inside the grid, the
  nominal position and the error. This is the frame‑to‑position table for
  data processing. Frames before the first position sample of a row (the
  first few ms of run‑up) are omitted.

The slow‑axis position in these comes from the measured position logged at
the start of each row (older logs fall back to the target). A speed error of ε over N frames gives about ε × N × pitch of drift,
so 1% over 80 frames at 50 µm is 40 µm. Tighten `--calibration-tolerance`
if that matters.

Limits: speed and smoothness are whatever the slides deliver at that
frequency; the trajectory log shows how uniform they are. Timing is
software timing. The trigger is about 1 ms from the move command, and
samples are within about 1 ms.

## PVs

The prefix is used verbatim. Device-level PVs are `$(PREFIX)NAME`;
per-axis PVs are `$(PREFIX)$(AXIS):NAME`, with AXIS defaulting to A, B, C.

### Device

| PV | Type | Notes |
| --- | --- | --- |
| `CONNECTED` | bi | Disconnected / Connected |
| `MODEL`, `DEVICE_ID`, `FIRMWARE` | stringin | from the controller |
| `BACKEND`, `VERSION` | stringin | `sdk` or `ascii`; IOC version |
| `TEMPERATURE` | ai | °C, every 5 s |
| `HV` | ai | internal 100 V supply, V |
| `STOP_ALL` | bo | write 1 |
| `LAST_ERROR` | lsi | connection errors |

### Per axis

Names follow the motor record where there is an equivalent, so screens
and habits carry over, but these are separate records rather than a motor
record. Positions are µm for linear sensor types and millidegrees for rotary
ones; EGU switches when the sensor type is read.

| PV | Type | Notes |
| --- | --- | --- |
| `VAL` | ao | closed-loop absolute move; put-callback completes with `DMOV` already 0 |
| `RBV` | ai | position; INVALID without a sensor or connection |
| `RLV` | ao | relative move: `VAL += RLV` |
| `TWV`, `TWF`, `TWR` | ao, bo, bo | tweak by `TWV` |
| `DMOV`, `MOVN` | bi | done (stopped or holding) / moving |
| `STATUS` | mbbi | Stopped, Setting amplitude, Moving, Targeting, Holding, Calibrating, Referencing |
| `STOP` | bo | stops the axis and discards queued commands for it |
| `HLM`, `LLM` | ao | IOC-side soft limits for `VAL`/`RLV`/tweak; disabled when `HLM <= LLM` (default 0, 0) |
| `HOLD` | longout | ms to actively hold after closed-loop moves and referencing; 60000 = until stopped |
| `STEP` | longout | open-loop steps, signed, \|n\| < 30000 |
| `STEP_AMP`, `STEP_FREQ` | ao, longout | 15–100 V, 1–18500 Hz |
| `CALIBRATE` | bo | sensor calibration: moves several mm both ways; keep clear of end stops |
| `REFERENCE`, `AUTO_ZERO` | bo, bo | move to reference (in `SAFE_DIR`); auto-zero sets 0 there |
| `REFERENCED` | bi | physical position known |
| `SET_ZERO` | bo | current position becomes 0 (stored if referenced) |
| `SENSOR_PRESENT` | bi | |
| `SENSOR_TYPE`, `SENSOR_TYPE_RBV`, `SENSOR_NAME` | longout, longin, stringin | type code, stored in the controller's NV memory; see the table in `protocol.py` |
| `CL_MAX_FREQ`, `CL_MAX_FREQ_RBV` | longout, longin | closed-loop max step frequency, Hz (sets speed) |
| `SAFE_DIR`, `SAFE_DIR_RBV` | bo, bi | direction for calibrate/reference, stored in NV memory |
| `LAST_ERROR` | lsi | last rejected command, e.g. "No sensor present (E19)" |

A rejected `VAL` (controller error, limit, not connected) snaps back to the
current position and sets `LAST_ERROR`. `VAL` follows `RBV` after open-loop
moves, stops, calibration and referencing.

### ophyd

```python
from ophyd import Component as Cpt, EpicsSignal, EpicsSignalRO, PVPositioner

class SCUAxis(PVPositioner):
    setpoint = Cpt(EpicsSignal, ":VAL")
    readback = Cpt(EpicsSignalRO, ":RBV")
    done = Cpt(EpicsSignalRO, ":DMOV")
    stop_signal = Cpt(EpicsSignal, ":STOP")
    done_value = 1
    stop_value = 1

x = SCUAxis("24IDC:SCU:A", name="x")
```

## Commissioning an axis

1. **Sensor type.** Check `SENSOR_NAME` against the positioner's label or
   datasheet. SLC positioners with the "l" (improved micro) sensor come in
   three variants: single reference mark is L (21), distance-coded marks are
   LC (18), and end-stop referenced is LE (23). Wrong types are the usual
   cause of referencing failures.
2. **Calibrate** once per positioner/channel pairing, away from the end
   stops. The result is stored in the controller.
3. **Safe direction.** Set `SAFE_DIR` towards free travel.
4. **Reference** after each power-up (`REFERENCED` goes to Yes). Use
   `AUTO_ZERO` if you want 0 at the reference mark.
5. **Soft limits.** Set `HLM` and `LLM` for the mechanics around the stage.
   They live in the IOC only; use autosave or the startup script to restore
   them.
6. **Speed.** `CL_MAX_FREQ` sets the closed-loop speed (default 5000 Hz).

## Tests

```sh
pip install -e '.[dev]'
pytest
```

The protocol, simulator, ASCII backend (over a pty) and CLI tests take
about a second. `tests/test_ioc.py` runs `scu-sim` plus `scu-ioc` as
subprocesses and drives them with caproto (about 15 s; skipped without
caproto). The SDK backend is tested against a fake library object, plus a
check that missing symbols are bound as "not supported".

## Known gaps

- **SDK backend:** it has run against hardware on Windows (DLL) and on
  Linux (libscu3dcontrol.so 1.5.16, CU-3D with firmware 5.1.192).
- **Wire backend:** the protocol is reverse engineered (see
  `docs/wire-protocol.md`). It's checked three ways: against SmarAct's library,
  replayed against exchanges captured from a CU-3D with firmware 5.1.192,
  and against the simulator. It has **not yet driven real hardware**. Other
  firmware versions or models may differ. Asynchronous mode, scale,
  alignment and the EEPROM/SRAM commands are deliberately not implemented. Argument order and units follow the header
  and Programmer's Guide; the ctypes binding assumes cdecl (the header
  declares no calling convention, which is moot on x64).
- **ASCII backend has only been tested against the simulator.** In
  particular, the manual's MPA/MPR examples are integer µm. Whether the
  firmware accepts one decimal place (which the IOC sends for sub-µm
  targets) is unverified. If it doesn't, `LAST_ERROR` will show a parse,
  syntax or parameter error.
- **PV Access on Windows:** the bundled pvxs server fails to start
  (`event_base_new() fails`), so the IOC serves Channel Access only.
- **Not exposed:** move to end stop (MES), scale offset and inversion (SSC),
  vertical alignment (SPA), the target-reached threshold and gain channel
  properties (SCP), and the keep-alive timer (K).
- **No motor record**: `EpicsMotor` won't work; use `PVPositioner` as above.
- **Soft limits** are not applied to open-loop `STEP`.
- **Holding:** `DMOV` goes to 1 when a held target is reached. With
  `HOLD` = 60000 the axis holds, and is fast-polled, until `STOP`.
- **Simulator** motion is a crude constant-speed model; scale inversion is
  not modelled.
