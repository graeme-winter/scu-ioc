# scu-ioc

Python control for SmarAct SCU piezo controllers (CU‑1D/3D and HCU‑1D/3D,
with or without the sensor module):

- **`scu-ioc`**: an EPICS soft IOC with motor‑record‑like PVs per axis
  ([pythonSoftIOC](https://github.com/DiamondLightSource/pythonSoftIOC); no
  EPICS build, no motor module).
- **`scu`**: a bench tool for status, moves, referencing and speed
  measurement, plus serpentine **raster** and **fly** scans for a two‑axis
  fixed‑target stage.
- **`scu-flyplot`**: analysis, plots, a sample map and a frame‑to‑position
  table for fly scans.

It is written against SmarAct's *SCU ASCII Programming Interface* v2.0‑C and
*SCU Programmer's Guide* v2.0‑C (SCU3DControl 1.4.x on Windows,
libscu3dcontrol 1.5.16 on Linux), plus a reverse‑engineered map of the stock
USB firmware's binary protocol (`docs/wire-protocol.md`). It has been run
against a CU‑3D (USB OEM board, firmware 5.1.192) with two SLC‑1740 linear
slides.

## Backends

Choose one with `--backend` (the IOC and `scu` both take it):

| Backend | Controllers | Hosts |
| --- | --- | --- |
| `wire` | USB SCUs with the stock firmware (e.g. CU‑3D‑USB‑OEM as shipped) | anything with pyserial and an FTDI serial driver: Linux x86 or ARM (Raspberry Pi), macOS |
| `sdk` (default; alias `dll`) | the same stock‑firmware USB SCUs, through SmarAct's library | Windows (SCU3DControl.dll) or Linux x86/x86‑64 (libscu3dcontrol.so) |
| `ascii` | RS‑232 SCUs, or USB SCUs reflashed with SmarAct's ASCII firmware | anything with pyserial, including `socket://` serial servers |

**Which to use:**
- On Linux, a Raspberry Pi or a Mac, use **`wire`**. It needs no SmarAct
  software, uses the kernel's standard FTDI driver, and works in containers
  like any other serial device.
- On Windows, use **`sdk`**.
- `ascii` is for RS‑232 units, or units already running the ASCII firmware.

The `wire` protocol was reconstructed from SmarAct's library. It's
checked against that library, replayed against captured hardware exchanges,
and has driven the hardware. See [Known gaps](#known-gaps) for what that
does and doesn't cover.

## Install

Python 3.10 or newer is recommended (3.9 works, but its dependencies are
ageing out). On a Mac, use uv, Homebrew or python.org Python rather than
Apple's 3.9.

```sh
python3 -m venv .venv && . .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install --upgrade pip
pip install '.[plot]'                              # or: pip install -e '.[dev,plot]'

# or with uv, which also fetches a suitable Python:
uv venv --python 3.12 .venv && . .venv/bin/activate && uv pip install -e '.[dev,plot]'
```

Optional extras:

| Extra | Adds | For |
| --- | --- | --- |
| `plot` | matplotlib | `scu-flyplot` plots and maps |
| `ca` | caproto | `ca:` triggers in `scu raster` |
| `dev` | pytest, caproto | the test suite |

Old pip (before 21.3) cannot do editable installs of a `pyproject.toml`‑only
project. Upgrade pip first.

## Host setup

Only one program can use a controller at a time. Close the IOC, the SmarAct
tools and any `scu` command before starting another.

### Linux and Raspberry Pi (`wire`)

The SCU appears through `ftdi_sio` as `/dev/ttyUSB*`, at 500,000 baud. Two
settings matter:

- **The FTDI latency timer.** The default is 16 ms, which makes every exchange
  take about 16 ms. At 1 ms (what the SDK sets) the firmware answers in about
  0.5 ms. This matters most for fly scans. The IOC and `scu` warn if they find
  16.
- **A stable name.** `/dev/serial/by-id/usb-SmarAct_SCU-3D_<serial>-if00-port0`
  exists already; the rule below also adds `/dev/smaract-scu`.

Find the serial number with `udevadm info -a -n /dev/ttyUSB0 | grep 'ATTRS{serial}'`,
then create `/etc/udev/rules.d/99-smaract-scu-wire.rules`:

```
# SmarAct SCU via ftdi_sio: stable name, dialout access, 1 ms latency
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="<SERIAL>", SYMLINK+="smaract-scu", GROUP="dialout", MODE="0660"
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTRS{serial}=="<SERIAL>", ATTR{latency_timer}="1"
```

Reload with `sudo udevadm control --reload && sudo udevadm trigger` and
replug. For a quick test without the rule, run
`echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`. Then:

```sh
scu --backend wire --port /dev/smaract-scu status
scu-ioc --backend wire --port /dev/smaract-scu --device-id 654905362 --prefix 24IDC:SCU:
```

- `--device-id` makes the IOC refuse any other controller on that port.
- `--port` also takes pyserial URLs (`socket://`, `rfc2217://`), so a serial
  server, such as ser2net on a USB/IP box, works. It must run the port raw at
  500000 8N1, with the latency timer at 1 ms on that host.
- In a container, pass the tty with `--device`.

Don't install the SDK rule below on the same host: it detaches `ftdi_sio`.

### macOS (`wire`)

macOS's built‑in FTDI driver presents the SCU as `/dev/cu.usbserial-<serial>`.
Use the `cu.` device, not `tty.`.

```sh
scu --backend wire --port /dev/cu.usbserial-FT0CO27S status
```

The latency timer can't easily be changed there and stays at 16 ms. Control,
the IOC, raster scans and fly calibration all work. Fly‑scan trajectories are
sampled every 16 ms instead of about every 1 ms, which is coarse for judging
how smooth the motion is. Use Linux for that.

### Windows (`sdk`)

Install the SmarAct SCU software (it installs the FTDI driver and
FTD2XX.dll), close the SCU Configuration Tool, then:

```bat
set SCU_DLL=C:\path\to\SCU\SDK\lib64\SCU3DControl.dll
scu-ioc --prefix 24IDC:SCU: --device-id 604573714
```

- Use `SDK\lib64` with 64‑bit Python and `SDK\lib` with 32‑bit.
- `--lib` (alias `--dll`) does the same job as `SCU_DLL`.
- `scu list` shows the connected device IDs.
- Allow Python through the Windows firewall for Channel Access (UDP/TCP 5064,
  UDP 5065).

### Linux with SmarAct's library (`sdk`)

The Linux SDK (1.5.16) has the same C API as the Windows DLL. It's x86 and
x86‑64 only. Copy `SDK/Redistributables/x86_64/` to, say,
`/opt/smaract/lib`:

```sh
scu list --lib /opt/smaract/lib/libscu3dcontrol.so
scu-ioc --lib /opt/smaract/lib/libscu3dcontrol.so --prefix 24IDC:SCU:
```

libscu3dcontrol.so needs libftd2xx.so but carries no RPATH. The backend
preloads libftd2xx.so from the same directory, so `LD_LIBRARY_PATH` isn't
needed when the two sit together.

D2XX talks to the device through usbfs, so `ftdi_sio` must **not** hold the
SCU, and the IOC user needs access to its `/dev/bus/usb` node. The SDK uses
FTDI's standard 0403:6001. Create `/etc/udev/rules.d/99-smaract-scu.rules`:

```
# SmarAct SCU: usable via libftd2xx by group plugdev, with ftdi_sio detached
SUBSYSTEM=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="<SERIAL>", MODE="0660", GROUP="plugdev"
ACTION=="add", SUBSYSTEM=="usb", DRIVER=="ftdi_sio", ATTRS{serial}=="<SERIAL>", RUN+="/bin/sh -c 'echo %k > /sys/bus/usb/drivers/ftdi_sio/unbind'"
```

In a container, keep the rule on the host, pass `/dev/bus/usb`, and put the
SDK libraries in the image.

### RS‑232 or ASCII firmware (`ascii`)

```sh
scu-ioc --backend ascii --port /dev/serial/by-id/usb-FTDI_...-if00-port0 --prefix 24IDC:SCU:
scu-ioc --backend ascii --port socket://moxa1:4001 --prefix 24IDC:SCU:
```

The factory baud rate is 9600 (`--baud` to change). SmarAct can supply an
ASCII firmware for USB units, according to the SCU user manual; the stock
USB firmware doesn't answer ASCII.

### Without hardware

There are simulators for both protocols, served on a pseudo‑terminal:

```sh
scu-wire-sim --link /tmp/scu-wire &          # binary protocol (stock firmware)
scu-ioc --backend wire --port /tmp/scu-wire --prefix TEST:SCU:

scu-sim --link /tmp/scu &                    # ASCII protocol
scu-ioc --backend ascii --port /tmp/scu --prefix TEST:SCU:
```

`--sensors 21,19,0` gives a linear axis, a rotary axis and an axis with no
sensor. `scu-wire-sim --asymmetry 0.1` makes moves 10% faster in + than in −.

## Running the IOC

```sh
scu-ioc --backend wire --port /dev/smaract-scu --prefix 24IDC:SCU: [options]
```

| Option | Meaning |
| --- | --- |
| `--prefix` | PV prefix, used verbatim (include any trailing `:`) |
| `--axes X,Y,-` | PV names for channels A, B, C; `-` skips a channel (default `A,B,C`) |
| `--poll`, `--busy-poll` | poll period when idle (1 s) and while moving or holding (0.1 s) |
| `--no-shell` | no interactive Python shell (use under procServ) |
| `-v` | log all controller traffic |

`quit()`, `exit()`, end‑of‑file and Ctrl‑C (with `--no-shell`) all release the
controller cleanly. Motion in progress continues, as with a motor‑record IOC.
If the link drops, readbacks go INVALID and the IOC reconnects by itself. The
PVs are listed [below](#pvs).

## Bench tool: `scu`

```sh
scu --backend wire --port /dev/smaract-scu status
```

| Command | Does |
| --- | --- |
| `status` | controller, temperature, supply voltage, and each axis's state, sensor, position and referencing |
| `move AXIS POS`, `rel AXIS DELTA` | closed‑loop absolute or relative move (µm, or m° for rotary axes); `--hold MS` |
| `step AXIS N` | open‑loop steps (signed); `--amp V`, `--freq HZ` |
| `calibrate AXIS` | sensor calibration (moves several mm both ways) |
| `reference AXIS` | move to the reference mark; `--auto-zero` makes it 0 |
| `zero AXIS` | make the current position 0 |
| `sensor-type AXIS CODE` | set the sensor type (stored in the controller) |
| `stop [AXIS]` | stop one axis, or all |
| `speed AXIS --freq F ...` | measure closed‑loop speed against frequency, both directions |
| `raster ...` | serpentine raster or fly scan (below) |
| `list` | `sdk` only: connected device IDs |
| `raw CMD` | `ascii` only: send one command, print the reply |

Axes are `A`/`B`/`C` or `0`/`1`/`2`. Moves wait for completion unless
`--no-wait` is given. Ctrl‑C stops the axis.

## Raster scans

`scu raster` runs a serpentine scan over two axes directly on the controller.
Rows alternate direction, so only the fast axis moves within a row. Each axis
is given as `AXIS START END STEP` in µm (m° for rotary axes). The range must
be a whole number of steps, and START and END set the direction. Both axes
must have sensors and be referenced (`--allow-unreferenced` overrides).
`--dry-run` prints the plan (with `--speed UM_S`, a time estimate too).

Ctrl‑C, a timeout, a controller error or a tolerance failure stops both
axes and keeps the partial log. Fly scans also stop if a move ends short of
its target, e.g. at an end stop.

### Step and shoot

At each point: an absolute closed‑loop move, a wait for target reached,
`--settle` seconds, a read‑back of both axes (checked against
`--tolerance`), the trigger, then `--dwell` seconds.

```sh
# 25 x 25 mm at 50 um pitch about the reference position; fast = A (X), slow = B (Z)
scu --backend wire --port /dev/smaract-scu raster \
    --fast A -12500 12500 50 --slow B -12500 12500 50 \
    --settle 0.005 --dwell 0.02 --tolerance 2 \
    --trigger serial:/dev/ttyUSB1:dtr --pulse 1 --log run042.csv
```

There's no hardware position compare, so a trigger means "the stage has
stopped here". Software jitter, of order a millisecond, changes *when* a shot
fires, not *where*. `--log` writes, per point, the targets, the measured
positions and the times (move commanded, settled, triggered). `--hold MS`
keeps the controller correcting at each point, which helps if the stage
creeps during long dwells.

Triggers (also used by fly scans):

| `--trigger` | Output |
| --- | --- |
| `none` | no output |
| `serial:PORT[:dtr\|rts][:low\|high]` | pulses DTR or RTS on a separate USB‑serial adapter for `--pulse` ms. On TTL adapters an asserted line is electrically low, hence the default `low`. Wire it to a Zebra or detector TTL input, checking levels and termination. |
| `ca:PV[=VALUE]` | writes VALUE (default 1) to an EPICS PV and waits for completion, e.g. a Zebra soft input. Needs the `ca` extra. |

### Fly scans

With `--fly`, the fast axis moves continuously along each row, and the
trigger fires once at the start of each row's move. This suits a detector
that free‑runs at a fixed frame rate from that trigger. The slow axis steps
between rows.

```sh
# 50 um pitch at 100 Hz frames -> 5 mm/s
scu --backend wire --port /dev/smaract-scu raster \
    --fast A -12500 12500 50 --slow B -12500 12500 50 --fly --frame-rate 100 \
    --run-up 200 --trigger serial:/dev/ttyUSB1:dtr --log run043.csv --trajectory run043-traj.csv
```

**Speed.** The SCU has no velocity setting. In a long closed‑loop move the
speed is the closed‑loop maximum step frequency (50–18,500 Hz) times the
positioner's step length. Step length depends on the slide, load and
amplitude, and on SLC‑1740s it also varies with frequency and direction (see
below). So speed is calibrated:

- **Target:** `--fly-speed` in µm/s, or the fast STEP × `--frame-rate`.
- **Calibration moves:** test moves over the first row's span, **for each
  direction separately**. The frequency is adjusted by secant steps until
  the speed is within `--calibration-tolerance` (default 1%, up to 8 moves
  per direction), or the best frequency found is used.
- **Per‑direction frequencies:** + rows and − rows can end up with
  different frequencies.
- **Restore:** the fast axis's original frequency is put back afterwards,
  also after an abort.
- **No calibration:** `--fly-freq HZ` uses one fixed frequency both ways.

To see how a slide behaves:

```sh
scu --backend wire --port /dev/smaract-scu speed A --freq 1000 2000 5000 10000 18500 --distance 10000
```

This prints, for each frequency, the speed in each direction, the asymmetry,
the step length per Hz, the deviation from constant speed, and the number and
spacing of position samples. Test moves are centred on `--center` (default 0),
and one that stops short ends the test.

**Run‑up.** `--run-up` adds travel before and after each row so the grid is
crossed at full speed. The first grid point comes about run‑up/speed after the
trigger, so the detector must keep taking frames for that long plus the row.
Make sure the run‑up fits inside the travel. An SLC‑1740 has 26 mm, so a
full 25 mm square leaves ±500 µm at most.

**What is recorded.**
- **Sampling:** during each row the fast axis is sampled (about every 1 ms
  on Linux with a 1 ms latency timer, every 16 ms on macOS, never closer than
  0.5 ms), with each sample stamped at the midpoint of its exchange.
- **Per row:** the speed is fitted over the grid region and the progress
  lines report it with the deviation from constant speed.
- **`--log`:** for every grid point, the time it was crossed
  (`t_cross`, and `dt_cross` from the row trigger), the row's speed, and the
  measured slow‑axis position.
- **`--trajectory`:** every sample. `--sample-ms` thins it, since a full
  square means millions of rows.

With the detector started by the trigger, frame *k* is at *k*/rate after it,
and the logs map that time to a position. A small speed error becomes a
known position error rather than an unknown one. A speed error ε over N
frames drifts by about ε × N × pitch (1% over 80 frames at 50 µm is 40 µm),
so tighten `--calibration-tolerance` if that matters.

### Fly‑scan analysis: `scu-flyplot`

```sh
scu-flyplot run043.csv run043-traj.csv --frame-rate 100 -o run043.png \
    --map run043-map.png --frames-out run043-frames.csv
```

- **`-o`** draws four panels:
  - deviation from constant speed against position, per row (+ and − rows
    coloured separately);
  - frame position error against frame number, along the direction of travel
    (negative means behind), anchored at the first grid point, so speed error
    shows as straight‑line drift;
  - fitted speed per row;
  - trigger‑to‑first‑grid‑point delay per row.

  Without `-o` the plot opens in a window; `--no-plot` skips it.
- **`--map`** shows where the sample went: the stage's actual path, and every
  detector frame at its actual position, coloured by its error along travel.
  The scale covers 99.5% of frames, so one bad row saturates instead of washing
  out the rest. A fixed slide pattern shows as vertical stripes, drift over the
  scan as a change down the map, and a one‑off disturbance as a streak.
- **`--frames-out`** writes every detector frame's row, frame number, time,
  actual fast and slow positions, and, inside the grid, nominal position and
  error. This is the frame‑to‑position table for data processing. Frames
  before a row's first position sample (in the run‑up) are left out, and
  positions are for frame start times.
- A per‑direction summary (speed, smoothness, first‑point delay, end‑of‑row
  drift) is always printed.

### Measured behaviour: SLC‑1740 on a CU‑3D

These were measured with the stage horizontal on a Mac (16 ms sampling), so
treat them as indicative:

- **Speed against frequency:** about 1.7 µm per Hz (1.7 mm/s at 1 kHz,
  about 17 mm/s at 10 kHz, about 40 mm/s at 18.5 kHz). Both slides are nearly
  identical.
- **Direction asymmetry:** depends strongly on frequency, swinging between
  about −35% and +30% from 1 to 18.5 kHz. That's why calibration is done per
  direction.
- **Fly scan, 81 × 81 at 50 µm, 2.5 mm/s:**
  - row speed repeatable to about 0.1%;
  - both directions within 0.2% of target over the scan;
  - deviation from constant speed about 2 µm rms;
  - frames within about ±10 µm of the grid, except one row with a one‑off
    slowdown of about 36 µm;
  - trigger‑to‑first‑point jitter about ±0.5 ms;
  - about 0.2 s between rows.
- **Fixed pattern:** a repeatable ±5 µm pattern along the fast axis (near
  −500 and +200 µm), the same in every row and both directions. It comes from
  the slide or its sensor. A fixed‑pattern calibration film will show which.

## PVs

The prefix is used verbatim. Device‑level PVs are `$(PREFIX)NAME`; per‑axis
PVs are `$(PREFIX)$(AXIS):NAME`, with AXIS defaulting to A, B, C.

### Device

| PV | Type | Notes |
| --- | --- | --- |
| `CONNECTED` | bi | Disconnected / Connected |
| `MODEL`, `DEVICE_ID`, `FIRMWARE` | stringin | from the controller |
| `BACKEND`, `VERSION` | stringin | `sdk`, `wire` or `ascii`; IOC version |
| `TEMPERATURE` | ai | °C, every 5 s |
| `HV` | ai | internal 100 V supply, V |
| `STOP_ALL` | bo | write 1 |
| `LAST_ERROR` | lsi | connection errors |

### Per axis

Names follow the motor record where there's an equivalent, but these are
separate records, not a motor record. Positions are µm for linear sensor
types and m° for rotary ones; EGU switches when the sensor type is read.

| PV | Type | Notes |
| --- | --- | --- |
| `VAL` | ao | closed‑loop absolute move; put‑callback completes with `DMOV` already 0 |
| `RBV` | ai | position; INVALID without a sensor or connection |
| `RLV` | ao | relative move: `VAL += RLV` |
| `TWV`, `TWF`, `TWR` | ao, bo, bo | tweak by `TWV` |
| `DMOV`, `MOVN` | bi | done (stopped or holding) / moving |
| `STATUS` | mbbi | Stopped, Setting amplitude, Moving, Targeting, Holding, Calibrating, Referencing |
| `STOP` | bo | stops the axis and discards queued commands for it |
| `HLM`, `LLM` | ao | IOC‑side soft limits for `VAL`/`RLV`/tweak; disabled when `HLM <= LLM` (default 0, 0) |
| `HOLD` | longout | ms to hold actively after closed‑loop moves and referencing; 60000 = until stopped |
| `STEP` | longout | open‑loop steps, signed, \|n\| < 30000 |
| `STEP_AMP`, `STEP_FREQ` | ao, longout | 15–100 V, 1–18500 Hz |
| `CALIBRATE` | bo | sensor calibration: moves several mm both ways; keep clear of end stops |
| `REFERENCE`, `AUTO_ZERO` | bo, bo | move to reference (in `SAFE_DIR`); auto‑zero makes it 0 |
| `REFERENCED` | bi | physical position known |
| `SET_ZERO` | bo | current position becomes 0 (stored if referenced) |
| `SENSOR_PRESENT` | bi | |
| `SENSOR_TYPE`, `SENSOR_TYPE_RBV`, `SENSOR_NAME` | longout, longin, stringin | type code, stored in the controller; codes in `protocol.py` |
| `CL_MAX_FREQ`, `CL_MAX_FREQ_RBV` | longout, longin | closed‑loop maximum step frequency, Hz (sets speed) |
| `SAFE_DIR`, `SAFE_DIR_RBV` | bo, bi | direction for calibrate/reference, stored in the controller |
| `LAST_ERROR` | lsi | last rejected command, e.g. "No sensor present (E19)" |

A rejected `VAL` (controller error, soft limit, not connected) snaps back to
the current position and sets `LAST_ERROR`. `VAL` follows `RBV` after
open‑loop moves, stops, calibration and referencing.

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
   datasheet. SLC positioners with the "l" sensor come as single reference
   mark L (21), distance‑coded marks LC (18), or end‑stop referenced LE (23).
   A wrong type is the usual cause of referencing failures.
2. **Calibrate** once per positioner/channel pairing, away from the end
   stops. The result is stored in the controller.
3. **Safe direction.** Set `SAFE_DIR` towards free travel.
4. **Reference** after each power‑up, so that `REFERENCED` reads Yes. Use
   `AUTO_ZERO` for 0 at the reference mark. Raster scans assume this.
5. **Soft limits.** Set `HLM` and `LLM` for the mechanics. They live in the
   IOC only, so restore them with autosave or the startup script.
6. **Speed.** `CL_MAX_FREQ` sets the closed‑loop speed. The manual's default
   is 5000 Hz, but boards may have been set differently (the CU‑3D tested here
   was at 10000). For fly scans, run `scu speed` on each slide.

## Development

| Module | Role |
| --- | --- |
| `backend.py` | backend interface (blocking; µm / m° / 1/10 V / Hz) |
| `wire.py`, `wire_backend.py` | binary protocol map, and the `wire` backend |
| `dll.py` | `sdk` backend: ctypes binding of SCU3DControl.dll / libscu3dcontrol.so |
| `protocol.py`, `ascii.py` | status codes, sensor types, errors, ASCII framing; the `ascii` backend |
| `wire_sim.py`, `sim.py` | simulators (`scu-wire-sim`, `scu-sim`) |
| `ioc.py` | pythonSoftIOC records and worker thread (`scu-ioc`) |
| `cli.py` | `scu` |
| `raster.py`, `fly.py` | step‑and‑shoot and fly scans, speed calibration |
| `flyplot.py` | `scu-flyplot` |

Also in the repository:
- **`docs/wire-protocol.md`:** the binary protocol, with the evidence and
  confidence for each part.
- **`tools/ftrace/`:** the capture shim, trace decoder and capture session
  used to check the protocol against hardware (see its README).
- **`CHANGES.md`:** the history.

### Tests

```sh
pip install -e '.[dev,plot]'
pytest
```

There are about 100 tests, taking about 3 minutes; most of that is the fly
tests running real‑time moves on the simulator. They cover:

- **Protocol layer:** framing, the simulators, all three backends (`wire`
  over a pty and replaying exchanges captured from real hardware; `sdk` against
  a fake library), and the `scu` commands.
- **IOC end to end:** `tests/test_ioc.py` runs the IOC on the `wire` and
  `ascii` simulators and drives it over Channel Access with caproto.
- **Scans:** raster, fly and flyplot, including calibration on a simulated
  lumpy, asymmetric slide.
- **Vendor check:** `tests/test_wire_sdk.py` runs SmarAct's real library
  against the `wire` simulator. It needs `SCU_SDK_LIB=/path/to/libscu3dcontrol.so`
  and a C compiler, and is skipped otherwise.

## Known gaps

- **`wire` backend.** The protocol is reverse engineered (see
  `docs/wire-protocol.md`), and the SDK licence may restrict that, so check it
  before sharing. It has driven a CU‑3D (firmware 5.1.192) from Linux x86 and
  macOS, but hasn't yet run on a Raspberry Pi. Other firmware versions and
  models may differ. Asynchronous mode, scale, alignment and the EEPROM/SRAM
  commands are deliberately not implemented.
- **`sdk` backend.** It has run on Windows (DLL) and Linux (libscu3dcontrol.so
  1.5.16). The ctypes binding assumes cdecl; the header declares no calling
  convention, which is moot on x64.
- **`ascii` backend.** It has only been tested against the simulator. The
  manual's MPA/MPR examples use whole µm, so it's unverified whether the
  firmware accepts the one decimal place the IOC sends for sub‑µm targets. If
  it doesn't, `LAST_ERROR` will show a parse, syntax or parameter error.
- **Scans are command‑line only.** `scu raster` needs the controller to
  itself; the IOC has no raster PVs yet.
- **Fly‑scan timing is software timing.** The trigger is about 1 ms from the
  move command and position samples are within about 1 ms, with no hardware
  position compare or encoder output. Speed is calibrated once per scan;
  slow drift over a long scan isn't corrected row by row.
- **Not exposed in the IOC:** move to end stop, scale offset and inversion,
  vertical alignment, the target‑reached threshold and gain channel
  properties, and the keep‑alive timer.
- **No motor record.** `EpicsMotor` won't work; use `PVPositioner` as above.
- **Soft limits** are not applied to open‑loop `STEP`, nor to `scu`
  commands.
- **Holding.** `DMOV` goes to 1 when a held target is reached. With
  `HOLD` = 60000 the axis holds, and is fast‑polled, until `STOP`.
- **PV Access on Windows.** The bundled pvxs server fails to start
  (`event_base_new() fails`), so the IOC serves Channel Access only.
- **Simulators.** They use a crude constant‑speed motion model (optionally
  asymmetric or with a custom speed curve), and don't model scale inversion.
