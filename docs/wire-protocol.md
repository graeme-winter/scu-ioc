# SCU binary protocol (stock USB firmware)

Reconstructed from SmarAct's `libscu3dcontrol.so` 1.5.16 (x86‑64) by static
analysis, for interoperability with hardware we own (running the IOC on a
Raspberry Pi, for which no SDK build exists). Internal use. SmarAct's SDK
licence may restrict reverse engineering, so check it before relying on this
or sharing it.

Code: `src/scu_ioc/wire.py` holds the same tables and is the reference;
this document gives the evidence and confidence.

## Status of the map

| Level | Meaning |
| --- | --- |
| **Hardware‑checked** | Seen in captures from a real CU‑3D (firmware 5.1.192, ID 654905362) through the SDK, including calibration and referencing; `tests/data/` holds the exchanges and `tests/test_wire_backend.py` replays them. |
| **Vendor‑checked** | SmarAct's library sent these bytes, and accepted the simulator's replies built from this map (`tests/test_wire_sdk.py`). Layouts are right as far as the library's encoder and parser go. |
| **Static only** | Read from the disassembly, not exercised. |
| **Unknown** | Needs a hardware capture. |

Two hardware captures (Nov 2026) decoded cleanly, with every frame matching the
map. Every command and get that the IOC uses is hardware‑checked except
`move to end stop` and the angle commands, which need rotary or end‑stop
positioners. What the firmware showed:

- Latency: about 0.5 ms median from request to reply (p95 1.4 ms, 118
  exchanges) with the 1 ms latency timer. The first exchange after opening
  the port took 238 ms.
- Device‑level replies use channel byte `FF`: `81 00` is acknowledged with
  `A1 FF`.
- Firmware 5.1.192 rejects `53` (get channel property, key 25) with
  `E2 FF 09`, so it has no movement‑type property.
- The ignored byte in the `15` firmware reply is `00`.
- Status sequences: closed‑loop moves go 3 → 0, held moves 3 → 4 (holding),
  open‑loop steps 2 → 0, and stop gives 0 immediately.
- The errors seen were 131 (wrong sensor type, e.g. GetAngle on a linear axis) and
  130 (no sensor).

## Link

The FTDI chip is configured (FT_SetDivisor 6, FT_SetDataCharacteristics 8,0,0,
FT_SetFlowControl 0) for 500000 baud, 8N1, no flow control, with a 1 ms
latency timer. Timeouts are 1000 ms for both read and write (2000 ms while
waiting for a hardware reset). The library calls `FT_Purge` (both buffers)
after opening. **Vendor‑checked** (the shim logs these calls).

With the kernel `ftdi_sio` driver this is an ordinary `/dev/ttyUSB*` at
500000 baud. Set `/sys/bus/usb-serial/devices/ttyUSB*/latency_timer` to 1
to match the SDK.

The library only opens devices whose FTDI description is exactly
`SmarAct SCU-3D`. The string is assembled in code (no literal), so other
models (1D, HCU) may use other strings; they have not been seen.

## Framing

Both directions: one header byte, then N payload bytes, where

    header = (code << 4) | N        (N = 0..15)

Multi‑byte fields are little‑endian. Request and reply codes are separate
namespaces. The library reads a reply as: read 1 byte, then read N more.
There is no checksum, sync byte or sequence number. **Vendor‑checked.**

Units are the SDK's: position in 1/10 µm, angle in 1/10 m°, amplitude in
1/10 V, frequency in Hz, hold time in ms.

## Generic replies

| Header | Payload | Meaning |
| --- | --- | --- |
| `A1` | ch | acknowledge; ch matches the request, or `FF` for device‑level commands (hardware‑checked) |
| `E2` | ch, code | error; `code` is returned as the SA status (128+ device errors, e.g. 130 no sensor, 131 wrong sensor type); ch `FF` for device‑level errors |

Any other header where an ack or error is expected gives
`SA_PROTOCOL_ERROR` (14); a short read gives `SA_TIMEOUT_ERROR` (15).
**Vendor‑checked.**

## Get requests

`12 <selector> <ch>`, answered with the reply listed:

| Sel | Quantity | Reply | Reply payload |
| --- | --- | --- | --- |
| 00 | device ID | `14` | u32 id (no channel) |
| 01 | firmware version | `15` | b0 b1 x b3 b4: version = b0 \| b1<<8 \| b3<<16 \| b4<<24; byte x ignored by the library |
| 02 | status | `22` | ch, u8 status (0 stopped … 6 referencing) |
| 03 | position | `25` | ch, i32 |
| 04 | closed‑loop max frequency | `23` | ch, u16 |
| 05 | sensor type | `52` | ch, u8 |
| 06 | angle | `47` | ch, i32 angle, i16 revolution |
| 07 | sensor present | `62` | ch, u8 |
| 08 | amplitude | `43` | ch, u16 |
| 09 | positioner alignment | `26` | 6 bytes, layout static only |
| 0A | safe direction | `82` | ch, u8 |
| 0B | scale | `46` | ch, i32, u8 (static only) |
| 0C | physical position known | `A2` | ch, u8 |
| FA | firmware configuration | `E4` | static only, not used |

For 00 and 01 the library leaves the third byte uninitialised (observed:
00, F7, FF, 10). The firmware presumably ignores it.

Rows 00–08, 0A and 0C are **vendor‑checked**.

## Commands

All answered with `A1 ch` or `E2 ch code` unless noted.

| Header | Name | Payload | Status |
| --- | --- | --- | --- |
| `11` | stop | ch | vendor‑checked |
| `13` | set CL max frequency | ch, u16 Hz | vendor‑checked |
| `16` | set positioner alignment | ch, u8, u16, u8, u8 | static only |
| `17` | step (open loop) | ch, i16 steps, u16 amplitude, u16 frequency | vendor‑checked |
| `27` | move position absolute | ch, i32, u16 hold | vendor‑checked |
| `29` | move angle absolute | ch, i32 angle, i16 rev, u16 hold | vendor‑checked |
| `31` | set zero | ch | vendor‑checked |
| `32` | set report on complete | ch, u8 | static only (async mode) |
| `33` | set amplitude | ch, u16 | static only |
| `35` | move to end stop | ch, u8 dir, u16 hold, u8 auto‑zero | vendor‑checked |
| `36` | set scale | ch, i32, u8 | static only |
| `37` | move position relative | ch, i32, u16 hold | vendor‑checked |
| `39` | move angle relative | ch, i32, i16, u16 hold | vendor‑checked |
| `41` | calibrate sensor | ch | vendor‑checked |
| `42` | set sensor type | ch, u8 type (stored in NV memory) | vendor‑checked |
| `44` | move to reference | ch, u16 hold, u8 auto‑zero | vendor‑checked |
| `53` | get channel property | ch, u16 key → `67` ch, u16 key, i32 value | vendor‑checked (key 25) |
| `56` | set system property | u16 key, i32 value | static only |
| `57` | set channel property | ch, u16 key, i32 value | static only |
| `72` | set safe direction | ch, u8 (stored in NV memory) | vendor‑checked |
| `81` | set communication mode | u8 (0 sync; 1 async, inferred) → `A1 FF` | hardware‑checked for 0 |
| `B2` | get system property | u16 key → `66` u16 key, i32 value | vendor‑checked (keys 1, 2) |
| `C0` | hardware reset | none → one raw byte, then `20` within 2 s | static only (SA_HARDWARE_RESET) |

The library validates ranges before sending (channel ≤ 2, steps within
±30000, amplitude 150–1000, frequency 1–18500, hold ≤ 60000), so the
firmware's own handling of bad values is unknown.

### Do not send

`D3`/`D7` (SRAM read/write) and `E3`/`E7` (EEPROM read/write) are exported
but undocumented. EEPROM holds calibration and configuration. Nothing here
uses them, and nothing should send unknown codes to the hardware.

## Connect sequence (sync mode)

Observed from the library:

1. `FT_ListDevices`, open each device, check its description, configure the
   link, purge.
2. `12 00 xx` → `14 id`. When a device list was given, the ID is matched
   against it, and non‑matching devices are closed.
3. `12 01 xx` → `15 …` firmware version.
4. Only for SA_HARDWARE_RESET: `C0` → one byte, then `20`.
5. `81 00` → `A1 FF` (synchronous mode).

After the `81` acknowledgement the library performs some extra reads
(static only; possibly draining stale async packets).

`SA_ReleaseDevices` sends nothing: it just closes the FTDI handle
(**vendor‑checked**).

## Open questions

- Does the device need any delay between frames at 500 kbaud? None was
  needed at SDK pacing.
- Which other errors the firmware returns, e.g. end stop reached (132) and
  command overridden (133).
- Recovery via `FT_CyclePort` / `SA_Recover`.
- Async mode (`81 01`, `32`) and its unsolicited packets, which would
  allow event‑driven completion instead of polling. Static analysis only so
  far.
- USB description strings for other SCU models.
