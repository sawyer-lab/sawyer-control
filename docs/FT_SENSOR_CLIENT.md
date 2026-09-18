# Force/Torque Sensor: Instructions for the Client Application

Handoff notes for whoever builds the higher-level application on top of this
repo. Everything here is verified against the physical sensor unless marked
otherwise.

## Division of responsibility

This repository is **communication only**: it sends to and reads from the
sensor, and exposes the device's own capabilities. It holds no policy. It
applies no defaults, performs no writes at startup, and takes no view on which
configuration, output rate, or filter is appropriate.

**The client application owns every decision**: which slot means what, when to
load one, what rate a task needs, which filter (if any), when to zero, what
counts as contact, what to do about saturation. If you find yourself wanting
the bridge to "just pick something sensible", that belongs in your layer.

Consequence worth internalising: the bridge will happily report a
badly-configured sensor rather than fix it. Reading and acting on that state is
your job.

## The hardware

| | |
|---|---|
| Transducer | ATI **Delta**, part `9105-T-Delta` |
| Calibration | **SI-330-30**, serial `FT27462` |
| Ranges | Fx/Fy ±330 N, Fz ±990 N, Tx/Ty/Tz ±30 N·m |
| Quoted resolution | 0.0625 N (Fx/Fy), 0.125 N (Fz), 0.00375 N·m |
| Measured noise | 0.044 N unfiltered, 0.013 N with a 35 Hz filter (Fz, 2 s at 7 kHz) |
| Resonant frequency | ~950 Hz (Fx/Fy/Tz), ~960 Hz (Fz/Tx/Ty), bare transducer |
| Physical | 94.5 mm diameter, 33.3 mm tall, 0.913 kg |
| Net Box | serial `LOT2821`, firmware 2.2.59, MAC `00:16:BD:00:23:56` |
| Internal sample rate | 7000 Hz, fixed |

The transducer model is **not reported by the device** (`mfgtxdmdl` reads
`tested`, a factory placeholder). It was identified from the calibration ranges
and confirmed against the CAD mesh dimensions.

### Original CAD

Download the original models from ATI rather than reusing a converted URDF or
MJCF, which may have lost or rescaled geometry:

- 3D CAD, drawings, and datasheets: <https://www.ati-ia.com/products/ft/ft_models.aspx?id=Delta>
  (tabs: *3D CAD Models*, *Drawings*, *Documents*)
- Select the **standard Delta**, not IP60/IP65/IP68 — those are physically
  larger (117 / 126 / 204 mm diameter) and will not match the hardware.

## Reference material on this machine

| What | Path |
|---|---|
| **ATI manual (PDF)** | `~/Downloads/9610-05-1022.pdf` |
| Pre-change backup, all 16 slots | `~/ft_netbox_backup_20260911/netftapi2_index{0..15}.xml` |
| Existing sensor mesh (converted) | `~/Projects/sawyer-student-lab/_sawyer/sawyer_description/meshes/tossingbot_gripper_description/STL/ati_9105_t_delta.STL` |
| URDF referencing it | `~/Projects/sawyer-student-lab/_sawyer/sawyer_description/urdf/tossingbot_official_v3.urdf` |

The manual bundles three documents; section numbers below refer to the Net Box
part, `9620-05-NET FT-22`. Transducer specs (ranges, resonance, dimensions) are
in `9620-05-Transducer Section-32`, §5.14 for the Delta. Extract searchable
text with:

```bash
pdftotext -layout ~/Downloads/9610-05-1022.pdf ftmanual.txt
```

Sections worth knowing: §7.1 `setting.cgi`, §7.2 `moncon.cgi`, §7.3
`config.cgi`, §7.4 `comm.cgi`, §8.1 `netftapi2.xml` elements, §9 the RDT
protocol, §17.1 the status-code bit table.

## Two planes, two modules

The Net Box has two independent interfaces. Do not confuse them.

| | Data plane | Config plane |
|---|---|---|
| Transport | RDT over UDP 49152 | HTTP (CGI + XML) on port 80 |
| Module | `bridge/robot_api/hardware/ft_sensor.py` | `bridge/robot_api/hardware/ft_config.py` |
| Carries | samples, software bias, start/stop | rate, filter, units, tool transform, thresholds |
| Persistence | session only | **persists in the box across reboots** |
| Latency | microseconds | tens of milliseconds |

A config-plane write outlives your process, your container, and a power cycle.
Treat it as durable device state, not as a runtime parameter.

## Functions to expose

### Reference frame (tool transform)

The box can refer measurements to a point other than the transducer face, in
hardware. **Your sensor already has a 132.71 mm Z offset configured in the
active slot.**

```python
config = box.configuration()          # tool_transform, and its distance/angle units
box.set_tool_transform(slot, dx, dy, dz, rx, ry, rz,
                       distance_unit="mm", angle_unit="degrees")
```

Rules for the client:

- **Do not apply your own tool offset on top without checking.** Readings
  already arrive transformed. Double-counting is silent and produces a plausible
  wrong answer.
- Always pass `distance_unit`/`angle_unit` explicitly when writing a transform.
  Slot 15, for example, defaults to **inches**. Writing `dz=132.71` there
  without specifying units gives you 3.4 metres.
- Read `tool_distance_unit` before interpreting an existing transform.

### Filtering

```python
box.settings()["filter_code"], box.settings()["filter_cutoff_hz"]
box.set_filter(code)      # 0 = off, 1..12 per FILTER_CUTOFF_HZ
```

Cutoffs: `0` off, `1` 838 Hz, `2` 326, `3` 152, `4` 73, `5` 35, `6` 18, `7` 8,
`8` 5, `9` 1500, `10` 2000, `11` 2500, `12` 3000 Hz. Note the ordering is not
monotonic — codes 9–12 are *higher* cutoffs than 1–8.

The critical fact, measured on this hardware:

> The box samples at 7 kHz internally and decimates to the output rate
> **without filtering first**. Lowering the output rate removes no noise at all;
> it folds the full 3.5 kHz of broadband noise into your visible band.

Evidence: Fz noise was 0.0438 N at 7000 Hz unfiltered and 0.0439 N at 100 Hz
unfiltered — identical. With a filter, 0.0127 N. **Only the filter reduces
noise; the rate never does.**

Guidance to encode in your UI: pick a cutoff at or below half the output rate.
At full rate, no filter is defensible (Nyquist 3500 Hz sits well above the
transducer's ~950 Hz resonance) and lets you filter in software, which is
reversible. At low rates, an unfiltered stream is actively misleading.

### Output rate

```python
box.communications()          # rate_hz, buffer_records, rdt_enabled
box.set_rate_hz(hz)           # returns the rate actually adopted
box.set_buffer_records(n)     # 1..40 records per UDP packet
effective_rate_hz(hz)         # predict without touching the device
```

- Achievable rates are **integer fractions of 7000** only. The box rounds a
  non-achievable request **up**: 150 → 152, 99 → 100. Verified on hardware.
- `set_rate_hz` returns what the box took. **Display that, not what was
  requested.**
- 7000 Hz was measured at 6978 Hz sustained with zero dropped packets.
- Changing the rate disturbs a running stream; the data plane restarts it.

### Bias / zeroing

Two distinct mechanisms — expose both, and label them clearly:

1. **RDT software bias** (data plane, `sensor.zero()`) — computes a bias from
   the current load. This is the "tare" a user expects. Session-scoped.
2. **Bias vector** (config plane, `box.set_bias_vector([...])`,
   `box.clear_bias()`) — six raw per-strain-gage offsets, **persisted in the
   box**. `clear_bias()` removes any stored bias.

A user who "zeroes" and later sees an unexpected offset is usually looking at a
stored bias vector from a previous session. Surface it.

### Configuration slots

```python
box.configurations()                     # all 16
box.configuration(slot)
box.write_configuration(slot, name=..., calibration=..., force_unit="N",
                        torque_unit="Nm", distance_unit="mm",
                        angle_unit="degrees", user_field_a=..., user_field_b=...)
box.select_configuration(slot)           # activate
```

A slot holds: name, calibration selection, force/torque units, tool transform
and its units, two free-text fields. Rate, filter, bias and peak logging are
**global**, not per-slot — you cannot scope them to a configuration.

- `setcfgsel` is **zero-based**, one less than the numbers shown on the box's
  own web pages. Off-by-one here is an easy bug.
- **Slot 0 is the configured, in-use slot** (132.71 mm transform, two monitor
  conditions). Treat it as precious. Slot 15 is the conventional scratch slot.
- Slot 1 (`difer`) points at calibration 1, which is **empty** (`FT0000`, all
  ranges zero). Selecting it yields garbage. Do not offer it without a warning.
- Calibrations cannot be created in software. `cfgcalsel` only selects among
  those ATI burned in; this box has one real one. Changing to a different range
  (e.g. SI-165-15) requires factory recalibration.

### Scaling — the most dangerous detail

```python
box.scaling()   # counts_per_force, counts_per_torque, force_unit, torque_unit
```

RDT returns **raw counts**, not newtons. The conversion factor lives in the
active configuration and **changes with the units**:

| Units | `counts_per_force` |
|---|---|
| N | 1 000 000 |
| lbf | **1** |

Measured, not theoretical. A hardcoded `1_000_000` would be wrong by six orders
of magnitude after a unit change.

**Rule: re-read `scaling()` after any `select_configuration()` or unit change,
and never cache it across one.**

### Monitor conditions (hardware thresholds)

```python
box.monitor_conditions()
box.set_monitor_condition(index, axis, comparison, counts, output_code, enabled)
box.set_monitor_conditions_enabled(True)     # master switch
```

16 statements evaluated **inside the box at the full 7 kHz**, independent of
your output rate. This is the only way to catch a transient you are not fast
enough to sample. A trip sets **status bit 16** and latches until reset.

- Thresholds are in **counts**, not user units. Convert with
  `counts_per_force` / `counts_per_torque`.
- Your box already has two statements on Tx (`< 100000`, `> -10000`) with the
  **master switch off**. Show existing statements before letting a user add any.
- `comparison` is `">"` or `"<"` only.

### Peaks

```python
box.peaks()                     # min_counts, max_counts, enabled
box.set_peak_logging(bool)
```

Hardware min/max capture per axis since the last reset — the cheap way to answer
"what was the maximum force during that throw" without streaming at 7 kHz.
Values are in counts. Peak logging is currently **enabled**; largest loads ever
recorded are ~15 / 15 / 20.6 N, far below the 330/990 N range.

Note: the manual documents no CGI to reset peaks, only a button on the box's
Snapshot web page. Not currently exposed.

### Status

```python
from robot_api.hardware.ft_status import decode, faults, is_healthy, saturated, describe
```

Every sample carries a 32-bit status word. `0x00000000` is healthy;
`0x80010000` is healthy with a monitor condition latched; **anything else is a
real fault**. Bit 17 is transducer saturation, bit 20 RDT comms error, bit 22
network failure.

Surface saturation prominently — with ±330 N in Fx/Fy versus ±990 N in Fz,
lateral saturation is the realistic limit during throwing, and an unnoticed
saturated axis silently clips your data.

### Identity and backup

```python
box.identity()      # ip, mac, firmware, serial, internal rate
box.raw(index)      # every XML element verbatim, for snapshots
```

## Hard rules

1. **Never write network settings.** `comnetip`, `comnetmsk`, `comnetgw`,
   `comnetdhcp` are deliberately unreachable through this API; the transport
   rejects them. They load **only at power-up**, so a bad value fails silently
   until a reboot weeks later, and recovery is physical (open the Net Box, DIP
   switch 9, power cycle). If they ever need changing, use the box's web pages.

2. **There is no factory reset.** No CGI endpoint restores defaults. The only
   recovery from a bad configuration is writing known-good values back.
   **Snapshot before any write campaign.** A pre-change backup of all 16 slots
   is at `~/ft_netbox_backup_20260911/`.

3. **Validate then read back.** Every setter in `ft_config.py` validates against
   the manual's limits before issuing a request, then re-reads and returns what
   the box actually accepted. Display the returned value, never the requested
   one — the box quantizes and clamps silently.

4. **Config writes are durable.** They survive reboots and affect every other
   client of this sensor. Confirm destructive changes with the user.

## Current device state

As left on 2026-09-11, restored to its original values after testing:

- Active slot **0**, units N / N·m, `counts_per_force` 1 000 000
- Tool transform **0, 0, 132.71 mm**, 0°, 0°, 0°
- Output rate **100 Hz**, buffer 1 record, filter **off**
- Peak logging on, monitor conditions configured but master switch **off**
- Status `0x00000000`

Note this is the worst of the four configurations measured: 100 Hz with no
filter gives the noise of a 7 kHz stream with none of the bandwidth. Changing it
is a client decision, which is why the bridge left it alone.

## How to reach it: the gRPC surface

The config plane is exposed over gRPC as the `ForceTorqueConfig` service, kept
separate from `ForceTorque` because it is a different transport and latency
class. Use the Python client:

```python
from sawyer_control import ForceTorqueConfigClient

with ForceTorqueConfigClient.connect("127.0.0.1:50051") as ft:
    ft.identity(); ft.scaling(); ft.device_status()
    ft.configuration(); ft.configuration(15); ft.configurations()
    ft.settings(); ft.communications(); ft.monitor_conditions(); ft.peaks()

    ft.set_rate_hz(7000)                 # returns what the device adopted
    ft.set_filter(5)
    ft.set_peak_logging(True)
    ft.clear_bias(); ft.set_bias_vector([...])
    ft.write_configuration(15, name="throwing", force_unit="N", distance_unit="mm")
    ft.set_tool_transform(15, dz=132.71, distance_unit="mm", angle_unit="degrees")
    ft.select_configuration(15)
    ft.set_monitor_condition(0, "fz", ">", counts, 0x01)
    ft.set_monitor_conditions_enabled(True)
    ft.set_buffer_records(40); ft.set_rdt_enabled(True); ft.set_ethernet_ip_enabled(True)
```

Method names and semantics mirror `ft_config.py` exactly, so this document's
rules apply unchanged at either level.

Conventions:

- **Slot `-1` (or `slot=None` in the client) means the active configuration.**
  Protobuf cannot express an unset int, so this is how "current" is requested.
- **`WriteConfiguration` is a partial update.** Fields use protobuf presence, so
  only what you pass is written; everything else in the slot is untouched.
- **Units accept a menu name or a numeric code as a string** — `"N"`, `"Nm"`,
  `"mm"`, `"degrees"`, or `"2"`, `"3"`.
- **`FtSettings.filter_cutoff_hz` is 0 when no filter is active.** Check
  `filter_enabled` rather than treating 0 as a cutoff.
- **Setters return the device's post-write state.** Render that, not the request.
- **Error mapping**: invalid arguments become `INVALID_ARGUMENT` and are
  rejected *before* anything reaches the device; a missing or unreachable sensor
  becomes `UNAVAILABLE`.
- `ForceTorqueReading` now also carries `faults` (decoded names) and `healthy`,
  so the data plane no longer hands you a bare integer.

The bridge reads `FT_SENSOR_IP` for both planes; it is plumbed through
`docker/compose.yaml`. The container runs with `network_mode: host`, so it
reaches the sensor directly.

**The default is `disabled`.** The bridge keeps a freshest-sample cache and
hands it out by polling, which loses about a quarter of the device's samples
at 1 kHz (see *Not yet implemented* below). Anything that needs every sample
reads RDT directly from the host instead, and only one process can own that
stream — so the bridge does not take it unless asked. Set
`FT_SENSOR_IP=192.168.1.11` to serve force/torque through the bridge; robot
control is unaffected either way.

## Not yet implemented

The **config plane is complete**. The **data plane** is still the original
driver. Missing, and worth knowing before you design against it:

- **Buffered streaming** (RDT command `0x0003`, up to 40 records/packet) — not
  implemented. Required for lossless capture at high rates.
- **Push streaming** — `StreamReadings` currently polls a "freshest sample"
  cache on its own clock, so it duplicates and skips samples. At 7 kHz it
  discards roughly 98% of what the box sends.
- **Staleness and packet-loss reporting** — a dead sensor currently returns its
  last good reading indefinitely.
- **`export_state` / `import_state`** — proposed snapshot/restore, not built.

Do not design a high-rate capture feature against the current data plane.

## Try it

```bash
python scripts/demo/ft_parameters.py --host 192.168.1.11 --seconds 2
```

Captures at four rate/filter combinations, reports per-axis noise, plots traces
and spectra, and restores every setting on exit including after Ctrl-C.

## Known constants

Verified against this sensor on 2026-09-11. Prefer reading these from the
device; this table is for sanity-checking, not for hardcoding.

### Network and protocol

| Constant | Value |
|---|---|
| Sensor address | `192.168.1.11` (DHCP lease; static fallback is the same) |
| RDT port | UDP `49152` |
| RDT request header | `0x1234` |
| RDT request struct | `!HHI` — header, command, sample count (0 = infinite) |
| RDT record struct | `!IIIiiiiii` — rdt_seq, ft_seq, status, Fx, Fy, Fz, Tx, Ty, Tz (36 bytes) |
| RDT commands | `0x0000` stop, `0x0002` start realtime, `0x0003` start buffered, `0x0041` reset latch, `0x0042` set bias |
| Config plane | HTTP port 80: `netftapi2.xml`, `setting.cgi`, `comm.cgi`, `config.cgi`, `moncon.cgi` |

### Scaling and ranges

| Constant | Value |
|---|---|
| `counts_per_force` (N) | 1 000 000 |
| `counts_per_torque` (N·m) | 1 000 000 |
| `counts_per_force` (lbf) | **1** — changes with units, never hardcode |
| Sensing ranges | Fx/Fy ±330 N, Fz ±990 N, Tx/Ty/Tz ±30 N·m |
| Quoted resolution | 0.0625 N (Fx/Fy), 0.125 N (Fz), 0.00375 N·m |
| Active tool transform | `0, 0, 132.71` mm, `0, 0, 0` degrees |
| Tool transform factor (Delta) | 0.6 mm/N |

### Rates and filtering

| Constant | Value |
|---|---|
| Internal sample rate | 7000 Hz, fixed |
| Achievable output rates | integer fractions of 7000 only (166 distinct values) |
| Rounding | **up** to the next achievable rate: 150 → 152, 99 → 100 |
| Max buffer records | 40 |
| Filter codes | 0 off, 1 838, 2 326, 3 152, 4 73, 5 35, 6 18, 7 8, 8 5, 9 1500, 10 2000, 11 2500, 12 3000 Hz |
| Measured at 7 kHz | 6978 Hz sustained, 0 packets dropped |
| Fz noise, unfiltered | 0.0438 N @ 7000 Hz, 0.0439 N @ 100 Hz (identical — aliasing) |
| Fz noise, 35 Hz filter | 0.0127 N |

### Status word (bit → meaning)

| Bits | Meaning |
|---|---|
| `0x00000000` | healthy |
| `0x80010000` | healthy, monitor condition latched |
| 31 | error roll-up (set if any error bit is set) |
| 22 / 21 / 20 | network / CAN / RDT communication failure |
| **17** | **transducer saturation** |
| 16 | monitor condition latched (not an error) |
| 9 / 8 | excessive / insufficient strain gage excitation |

Full table: `bridge/robot_api/hardware/ft_status.py`, from manual §17.1.

### Limits and enumerations

| Constant | Value |
|---|---|
| Configuration slots | 16, zero-based (`setcfgsel` is one less than the web UI's numbering) |
| Calibration slots | 16; **one real calibration**, serial `FT27462` |
| Monitor condition statements | 16 |
| Monitor comparison codes | `1` = greater than, `-1` = less than |
| Monitor axis codes | 0 Fx, 1 Fy, 2 Fz, 3 Tx, 4 Ty, 5 Tz, `-1` disabled |
| Force unit codes | 1 lbf, 2 N, 3 klbf, 4 kN, 5 kgf, 6 gf |
| Torque unit codes | 1 lbf-in, 2 lbf-ft, 3 Nm, 4 Nmm, 5 kgf-cm, 6 kNm |
| Distance unit codes | 1 in, 2 ft, 3 mm, 4 cm, 5 m |
| Angle unit codes | 1 degrees, 2 radians |
| Bias vector | 6 values, int16 range each |
| Monitor threshold | int32 range, in counts |
| Output code | `0x00`–`0xFF` |
