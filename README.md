# ApexRide V1

Offline-first motorcycle telemetry logger. The device records lean angle,
pitch, acceleration, gyro, GPS position, speed, heading, altitude and GPS time
to internal flash while riding — with no phone, no SIM, no cloud and no
connection to the motorcycle's electrical system. A phone collects the data
later.

**Status: milestone 1 complete.** The full recording pipeline runs end to end on
simulated sensors and is verified by a host test suite. No hardware is required
to build or run it.

---

## Quick start (no hardware needed)

```bash
make -C tests test
```

This runs the real firmware — fusion, ride detection, encoding, buffering,
persistence, summaries, recovery and retention — against a simulated ride, and
checks the results against the simulator's ground truth.

```
179 checks, 0 failures
```

`make -C tests asan` reruns everything under AddressSanitizer and
UndefinedBehaviorSanitizer.

## Looking at the data

`ridedump` decodes a ride file — report, integrity check and terminal charts:

```bash
make -C tools
tools/build/ridedump tests/build/testdata/ride/rides/R000001.bin
```

```
Magic     ARD1   format v1   firmware 0x0100   header CRC OK
Records   4441 IMU · 444 GNSS · 3 events
CRC-32    0x...   matches the summary

Lean angle over the ride  (negative = left, positive = right)
     40.9 d |                                                #######
     31.8 d |          ########                              #     #
      4.5 d |###########-------#####-------###################-----###############
    -31.8 d |                       ########
```

Other modes pipe straight into whatever you already use:

```bash
tools/build/ridedump R000001.bin --csv-imu  > imu.csv     # spreadsheet, pandas
tools/build/ridedump R000001.bin --csv-gnss > gnss.csv
tools/build/ridedump R000001.bin --gpx      > ride.gpx    # any mapping tool
tools/build/ridedump R000001.bin --events
```

It exits non-zero on a bad header or a truncated stream, so it works in scripts.
It is also the reference decoder: the phone app has to do exactly what
`parseRide()` does, and keeping it a working program stops this document
drifting away from the truth.

## Talking to it like the phone will

The sync API runs on your machine, so the app can be built before any hardware
exists:

```bash
make -C tools
tools/build/syncserver --dir tests/build/testdata/ride --port 8080
```

Then run the auto-sync loop against it, exactly as the app will:

```bash
tools/build/syncclient --host localhost --port 8080 --out ~/rides
```

```
auto-sync: localhost:8080 -> ~/rides
  synced   2 ride(s), 232 KB transferred
```

Or drive it by hand:

```bash
curl localhost:8080/status
curl localhost:8080/sync/pending
curl localhost:8080/rides
curl "localhost:8080/rides/R000001/data?offset=0&length=8192" --output chunk.bin
curl -X POST "localhost:8080/rides/R000001/ack?crc=df7f6389"
```

This is the real `SyncProtocol` and `SyncService` over real sockets — the same
code the device runs, with a different socket layer under it. It sends CORS
headers so a browser-hosted Flutter build can call it during development.

## Building for the device

```bash
pio run -e esp32-s3-devkitc-1              # build
pio run -e esp32-s3-devkitc-1 -t upload    # flash
pio device monitor                          # 115200 baud
```

The sketch also opens in the Arduino IDE (`ApexRide/ApexRide.ino`);
select *ESP32S3 Dev Module*, 16 MB flash, OPI PSRAM, and a custom partition
scheme matching `partitions/apexride_16mb.csv`.

It boots into a scripted ~110 second simulated ride, so the whole pipeline can
be exercised on a bare DevKitC-1 with nothing else connected.

Serial commands: `s` start · `x` stop · `c` calibrate mounting · `g` gyro bias ·
`l` list rides · `i` info · `d` hex dump · `y` mark all synced · `p` sync API ·
`k` clear calibration · `f` format · `h` help.

`p` runs the sync API locally and prints the responses. There is no radio yet,
so during bring-up the serial monitor stands in for the phone — it calls the
same `route()` the Wi-Fi handler will.

---

## Layout

```
ApexRide/
  ApexRide.ino        sketch: wiring, serial console, status output
  config.h            pins, rates, thresholds (read only by the sketch)
  src/
    core/             clock, CRC-32, logging, math, buffers, shared types
    format/           binary record definitions, summary accumulation
    fusion/           Orientation — attitude estimation
    sensors/          sensor interfaces, managers, calibration, mocks
    sim/              RideSimulator — physically consistent fake bike
    ride/             ride detection, recording, composition root
    storage/          ride catalogue, retention, LittleFS + NVS backends
    sync/             transfer protocol: routing, JSON, sessions, ack checking
hostfs/               IRideStore backed by a real directory, shared by the below
refclient/            AutoSyncClient — the phone's auto-sync loop, testable
tests/                host build: the test suite
tools/                ridedump, syncserver, syncclient
partitions/           16 MB flash layout
```

Every module takes its settings through a `Config` struct rather than reading
`config.h`, so the tests can run the same code with different values.

### Testable by construction

Everything except `LittleFsRideStore`, `CalibrationStore` and the sketch is free
of Arduino headers, and compiles unchanged on the host. The filesystem sits
behind `IRideStore`; the tests supply a directory-backed implementation that can
also emulate a full volume and a truncated file. Time sits behind `Clock`, so a
110-second ride simulates in well under a second.

---

## Design decisions

### Lean angle is not `atan2(accelY, accelZ)`

In a steady corner the motorcycle leans until gravity and centripetal
acceleration resolve straight down through the tyres. The accelerometer then
reads `(a_long, 0, g/cos(lean))` — **the lateral component is zero regardless of
lean angle**. An accelerometer-only estimate reports upright at 45 degrees.

This is measurement geometry, not noise, so no amount of filtering fixes it. The
test suite measures it:

| method | max error through corners |
|---|---|
| accelerometer only, `atan2(ay, az)` | **42.0°** at 40° of lean |
| Mahony filter (gyro + accel) | 29.0° |
| Mahony + GNSS kinematic correction | **0.5°** |

A plain complementary filter is not enough either: the accelerometer keeps
insisting the bike is upright, so the estimate sags back toward zero over a long
corner. The fix is to remove the kinematic term before using the accelerometer
as a gravity reference:

```
gravity_body = accel_measured - (dv/dt + omega x velocity_body)
             = accel - (dv/dt, gyroZ * v, -gyroY * v)
```

With forward speed `v` from GNSS this recovers the true gravity direction
`(0, g·sin(lean), g·cos(lean))` exactly.

The same contamination hits **pitch** through the `dv/dt` term: without it the
filter reads `asin(a/g)`, so a 6 m/s² stop registers as 38° nose-down and the
peak-acceleration figures derived from pitch come out ~60% high. `GpsManager`
differentiates GNSS speed to supply it. The two terms are gated at different
speeds — `ω × v` above 3 m/s where it exists, `dv/dt` above 0.5 m/s because
pulling away from a standstill is exactly when it matters.

| | error vs ground truth |
|---|---|
| lean, cornering | 0.5° max, 0.2° RMS |
| pitch | 7.5° max (transient), 2.1° RMS |
| peak acceleration | 0.31 g vs 0.306 g true |
| peak braking | 0.59 g vs 0.612 g true |

When GNSS drops out the correction cannot run, so the filter falls back to a
much lower accelerometer gain (`kpUncorrected`) and coasts on the gyro. An
uncorrected accelerometer on a moving motorcycle is not a weak gravity
reference — it is a misleading one. See `src/fusion/Orientation.h`.

> This goes slightly beyond the original brief, which listed GNSS-aided
> estimation as a later refinement. It is included because without it the
> headline number the product exists to report is wrong by tens of degrees in
> exactly the situation that matters. `APEX_USE_KINEMATIC_CORRECTION 0` disables it.

### Two independent calibration layers

- **Sensor calibration** (`ImuManager`) — axis remapping and gyro zero-rate
  bias, captured automatically once the bike has been still for a moment.
- **Mounting calibration** (`Orientation`) — park upright on level ground and
  press `c`. Stored as a quaternion rather than a pair of scalar offsets, so a
  bracket that is rotated or tilted on more than one axis is corrected properly.

Both persist to NVS and survive reboot. The offset is captured from the
converged filter estimate, not a single accelerometer sample — one raw sample
carries a couple of degrees of vibration noise, which would otherwise be baked
into every subsequent reading.

### Ride detection without an ignition signal

`SLEEP → AWAKE → RECORDING → WAITING → close` driven by GNSS speed, with an IMU
fallback. One subtlety that the simulated GNSS dropout exposed: an accelerometer
**cannot** distinguish a steady cruise from standing still, so a tunnel makes the
IMU claim the bike has stopped. Ending the ride on that evidence would split one
ride into two, so the stop timeout is far longer when GNSS is unavailable
(`waitingEnterNoGnssMs`).

### Storage strategy

Samples are encoded into a RAM block buffer (PSRAM when available) and written
in whole 4 KB blocks. LittleFS rewrites an entire block for any change inside
it, so writing 24-byte records individually would multiply flash wear by ~170x.
An unclean shutdown therefore costs at most one block, which the recovery scan
handles as a truncated tail.

**Storage budget** — this is the binding constraint on the product:

| | |
|---|---|
| IMU record | 24 B on disk, logged at 50 Hz |
| GNSS record | 28 B on disk, logged at 5 Hz |
| total | ~1.3 kB/s ≈ **4.6 MB/hour** |
| littlefs partition | 12.9 MB |
| **capacity** | **~2.8 hours of riding** |

The IMU is sampled at 200 Hz and fused at 100 Hz as specified, but only 50 Hz is
written to flash; logging every fused sample would halve the capacity above.
`APEX_IMU_LOG_RATE_HZ` trades capacity against resolution. The partition table
deliberately omits an OTA slot for the same reason — see
`partitions/apexride_16mb.csv`.

### Auto-sync

Connecting is the trigger; there is no Sync button. The phone drives it — the
device is a server — but the device is not passive:

- **A session** (`/sync/begin` … `/sync/end`) bounds the window in which the
  radio stays up. Without it, auto-sync either leaves Wi-Fi on permanently,
  which a 1500 mAh cell will not tolerate, or pays the reconnect cost every
  time. An idle session closes itself so a phone that goes out of range does
  not pin the radio on.
- **Sync is refused while recording.** Serving bulk data competes with the
  recorder for flash and CPU, and dropping samples to make a sync faster is the
  wrong trade. The client sees a 409 and tries again when the rider stops.
- **`/sync/pending` returns what is outstanding, newest first** — a rider who
  just parked wants the ride they just did, and nothing is at risk of deletion
  in the meantime because unsynced rides are never reclaimed.

The loop itself lives in `refclient/AutoSyncClient` — the algorithm the Flutter
app implements, written once in testable C++ so that resume, retry and
verification-failure handling are settled rather than rediscovered in Dart
against hardware. The tests drive it in-process against the real device code;
`tools/syncclient` drives the same class over sockets.

Two failure modes that are easy to get backwards:

- **A dropped link** leaves the bytes already received intact, so the next
  connection resumes. Measured over HTTP: 77 KB re-sent instead of 116 KB.
- **A rejected checksum** means the local copy is wrong, so it is *discarded*
  and fetched again from zero. Resuming would preserve the corruption.

A ride that fails every attempt is skipped for the rest of the session —
otherwise `pending` keeps returning it and the loop never terminates.

### Sync: a download proves nothing

A ride is marked synced only after the phone downloads it, computes its own
CRC-32, sends that back, and the device confirms it matches. A transfer that
starts — or even completes — proves nothing: the phone may have written a
corrupt file or died before saving it. `acknowledge()` is the only path that
sets the flag, and it takes the phone's checksum as its argument.

```
GET  /status                        device, storage and GNSS state
GET  /rides                         manifest of every stored ride
GET  /rides/R000001                 one ride's summary, including its CRC
GET  /rides/R000001/data            ride bytes; ?offset= &length=
POST /rides/R000001/ack?crc=<hex>   verify, then mark synced
POST /rides/R000001/delete          delete, only if already synced
```

Chunks are addressed by absolute offset and carry no session state, so a
transfer that dies at 80% resumes at 80% — which matters over a link the rider
can walk out of range of. A ride still being recorded returns 409 rather than
serving bytes whose CRC will not match once it closes.

`SyncProtocol` and `SyncService` contain no networking code at all, so the whole
request surface is driven directly from host tests. The transport — Wi-Fi on the
device, POSIX sockets in `tools/syncserver` — is a thin adapter over `route()`.

### Retention

- an unsynced ride is **never** deleted automatically
- only synced rides are eligible, oldest first
- when nothing is eligible, reclaiming fails loudly rather than dropping data
- a ride is marked synced only after the phone acknowledges it (the console `y`
  command stands in until the sync path exists)

Ride IDs come from scanning the directory rather than a counter file, so a lost
counter can never overwrite an existing ride.

---

## Binary ride format (v1)

Two files per ride:

```
/rides/R000001.bin    append-only record stream
/rides/R000001.met    72-byte RideSummary, rewritten every 10 s
```

The summary is a separate file so the phone can list rides without reading any
samples, and so the data file is never seeked back into while recording. If a
`.met` is missing or fails its CRC — battery died mid-ride — `RideStorage`
rebuilds it by replaying the `.bin` through the same accumulator used during
recording, and flags it `recovered`. The test suite asserts that the rebuilt
summary is identical to the original, byte for byte.

```
.bin = [FileHeader 32 B] [RecordHeader 2 B][payload] [RecordHeader 2 B][payload] ...
```

The first four bytes are ASCII magic, so a file is identifiable in a hex dump:
`ARD1` for ride data, `ARS1` for a summary.

Every record is length-prefixed, so a reader meeting an unknown type can skip
it — that is the forward-compatibility hook for V2. All integers are
little-endian; all quantities are fixed-point. No floats and no text are stored.

| record | size | rate | contents |
|---|---|---|---|
| `ImuRecord` | 22 B | 50 Hz | roll/pitch/yaw (deg ×100), accel (milli-g), gyro (dps ×10) |
| `GnssRecord` | 26 B | 5 Hz | lat/lon (deg ×1e7), speed (cm/s), heading, altitude, HDOP, sats, fix |
| `EventRecord` | 10 B | as needed | ride start/end, fix acquired/lost, calibration, storage warnings |

GNSS is a separate stream, never duplicated into IMU records. Both share one
monotonic millisecond timebase.

Integrity: the file header carries its own CRC-32, and `RideSummary.dataCrc` is
a CRC-32 over the entire `.bin`. It is standard IEEE CRC-32 (the zlib/PNG
variant), so the phone can verify a download with any stock implementation —
the test suite pins it against the well-known `"123456789"` → `0xCBF43926`
check value.

Conversion helpers saturate rather than wrap, so a sensor glitch clips instead
of appearing as a violent flick in the opposite direction.

---

## What is not built yet

Deliberately out of scope for milestone 1, in the order they were planned:

- **Real sensor drivers.** `ICM20948Sensor` and `Atgm336hSensor` implement
  `IImuSensor` / `IGnssSensor`; nothing above that layer changes. Flip
  `APEX_USE_MOCK_IMU` / `APEX_USE_MOCK_GNSS` in `config.h`.
- **The sync transports.** The protocol, sessions, verification, resume and the
  auto-sync loop are done and tested. What is missing is BLE for discovery —
  which is what makes "connected" a thing that can happen automatically — and
  the ESP32 Wi-Fi AP that forwards HTTP into `SyncProtocol::route()`. Both are
  adapters over interfaces that already work.
- **Power management.** No deep sleep, no battery monitoring. `SLEEP` is
  currently a logical state only.
- **Flutter app.**

The pin assignments in `config.h` are placeholders and have not been checked
against the DevKitC-1 pinout.

### Ruled out of V1 by design

Not oversights — do not add these without a requirements change: microSD, SIM /
LTE, always-on cloud or MQTT, CAN bus or ECU integration, any connection to the
motorcycle's electrical system, a TFT display, a Raspberry Pi, and a custom PCB
before the prototype is validated.

Ride storage using internal flash instead of a microSD card is the decision that
shapes the most code: it is why the format is compact binary, why writes are
batched into blocks, and why capacity is the constraint the partition table is
tuned around.

---

## Bringing up the hardware

Follow the incremental order in the project brief — one subsystem at a time, USB
power only until the battery circuit has been measured.

### Two parts the BOM is missing

**A battery sense divider.** The BLE status spec reports a battery percentage,
but nothing in the current build can measure the cell: the TP4056 and MT3608 do
not expose its voltage, and the DevKitC-1 has no divider. Two resistors fix it —
tap the switched battery rail (between the switch and the MT3608 input, so it
draws nothing when off) through 100k/100k into an ADC pin, with 100nF to ground.
That halves 4.2 V to 2.1 V, inside the ADC range, at ~21 uA while running. Don't
use much larger resistors; the ESP32 ADC wants a low source impedance. Measure
before the boost converter — the 5 V rail reads 5 V until the cell collapses.

Until that exists, `/status` reports `"battery":{"available":false}` rather than
a fabricated number.

**Nothing else** — the calibration button the brief mentions is not needed. The
DevKitC-1's BOOT button on GPIO0 is free once the board has started.

Two rules worth repeating:

- **Measure the MT3608 output with a multimeter and set it to ~5.0 V before it
  is ever connected to the ESP32.** It powers up at an arbitrary voltage and
  will destroy the board.
- **Never connect a raw LiPo to the 3.3 V pin.**

Once the IMU is mounted, the axis map in `buildConfig()` must be set to match
its physical orientation. The body frame is X forward, Y left, Z up: a level,
stationary accelerometer must read `(0, 0, +9.81)`. Roll is positive leaning
right; pitch is positive nose up.
