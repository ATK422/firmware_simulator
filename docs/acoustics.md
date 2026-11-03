# Acoustic environments

The IVC boards are acoustic modems: they drive a transducer from a pin and sample a hydrophone with the ADC.
`sim_acoustics` gives them water to talk through. Nothing in it knows about the IVC hardware or modem: an
environment is simulated from the specs of the transducers, the water and the receive electronics it is given, so
a new transducer, another modem scheme or a different acoustic board is a configuration change.

> The model is for comparing environments and catching regressions, not for proving range. Its numbers are
> datasheet-style assumptions (the IVC transducer and filters are the plan's example numbers) until they are
> calibrated against recordings.

- [Environment files](#environment-files)
- [Transducers](#transducers)
- [One board in an environment](#one-board-in-an-environment)
- [The shared medium: boards hearing each other](#the-shared-medium-boards-hearing-each-other)
- [Moving transducers and Doppler](#moving-transducers-and-doppler)
- [IVC link tests and sweeps](#ivc-link-tests-and-sweeps)
- [C and Rust IVCBoard2 interop](#c-and-rust-ivcboard2-interop)
- [Making a new environment](#making-a-new-environment)

## Environment files

`environments/*.toml` describe a body of water and the sound sources in it. Coordinates are metres, `z` is depth
below the surface (positive down).

| File | What it is |
|---|---|
| `pool.toml` | A 25 x 12 x 3 m competition pool: concrete walls and bottom, warm fresh water, quiet (white noise at 50 dB), multipath to order 3, the competition pinger at 25 kHz |
| `open_water.toml` | Deep open sea: no boundaries, sea water, Wenz noise (moderate shipping, 5 m/s wind), a 30 kHz pinger |
| `harbour.toml` | A 6 m deep harbour over mud, brackish water, busy shipping, snapping shrimp, multipath to order 4, a 37 kHz pinger |

```toml
# environments/pool.toml
kind = "tank"
size_m = [25.0, 12.0, 3.0]
wall_reflection = 0.6
bottom_reflection = 0.5
water = { temperature_c = 26.0, salinity_ppt = 0.0 }
noise = { kind = "white", level_db_re_upa_per_hz = 50.0 }
multipath = { max_order = 3 }

[[source]]
kind = "pinger"
position_m = [20.0, 6.0, 2.0]
frequency_hz = 25_000
pulse_ms = 4
period_ms = 2000
source_level_db = 180
```

Every key (unknown keys are errors):

| Key | Default | Meaning |
|---|---|---|
| `kind` | required | `open` (free field: no surface, bottom or walls), `tank` / `pool` (a box from `(0, 0, 0)` to `size_m`, surface at z = 0), `lake` / `harbour` (horizontally unbounded, `depth_m` deep) |
| `size_m` | | Tank size `[x, y, depth]` |
| `depth_m` | | Lake depth |
| `bottom` | | Lake bottom type for its reflection: `mud` (0.2), `sand` (0.45), `gravel` (0.6), `rock` (0.8) |
| `wall_reflection`, `bottom_reflection`, `surface_reflection` | 0.5, 0.5 (or `bottom`'s), 0.95 | Pressure reflection coefficients; the surface flips the sign (pressure release) |
| `water.temperature_c`, `salinity_ppt`, `ph` | 20, 0, 8 | Water properties: sound speed (Mackenzie for sea water, Marczak for fresh, blended between) and absorption (Francois-Garrison), at the path's mid depth |
| `water.sound_speed_mps` | | Overrides the computed sound speed |
| `noise` | none | `{ kind = "white", level_db_re_upa_per_hz = L }`, `{ kind = "wenz", shipping = 0..1 (0.5), wind_mps = W (0) }` (Wenz curves, Coates approximation), or `{ kind = "none" }` |
| `shrimp` | | Snapping shrimp: `{ rate_hz, peak_db_re_upa, decay_us = 20 }` Poisson clicks |
| `multipath.max_order` | 2 | Highest reflection order of the image method |
| `seed` | 1 | Noise seed; runs with the same seed are identical |
| `[[source]]` | | Sound sources without a board behind them (below) |

Sources (`source_level_db` is the RMS level in dB re 1 µPa at 1 m):

| `kind` | Keys |
|---|---|
| `pinger` | `position_m`, `frequency_hz`, `pulse_ms`, `period_ms`, `source_level_db`, `start_ms` (0) |
| `tone` | `position_m`, `frequency_hz`, `source_level_db` |
| `chirp` | `position_m`, `f0_hz`, `f1_hz`, `duration_ms`, `period_ms`, `source_level_db` (linear sweeps) |
| `wav` | `position_m`, `file` (relative to the environment file), `source_level_db` (for a full-scale RMS of 1), `looped` (false), `nominal_hz` (the frequency used for absorption and sensitivity) |

How a path is computed: each source-receiver pair has one path per image source up to `max_order` reflections,
each a delay (range over sound speed), a gain (spherical spreading, absorption at the source's nominal frequency,
the reflection coefficients) and an arrival direction for the beam patterns. Ambient noise is white at the level
the noise model gives at the receive chain's centre frequency, and comes from the seed, so it is reproducible.

## Transducers

A board file's `[transducer]` (or a robot file's `[board.transducer]`, which replaces it) describes the
transducer by datasheet numbers, plus how it is wired:

```toml
# boards/overlays/talos/ivc_board.toml
[transducer]
tvr = { resonance_hz = 51_000, q = 8, peak_db = 140 }   # transmitting voltage response
rvs_db = -190                                           # receive sensitivity
beam = "omni"

[transducer.drive]                  # projector: driven from a pin
pin = 7
supply_v = 12.0

[transducer.receive]                # hydrophone: into an ADC input
adc = 1
gain_db = 40
select_pin = 3                      # GPIO 3 picks filter 0 (low) or 1 (high)
filters = [
    { highpass_hz = 40_000, lowpass_hz = 60_000, order = 4 },
    { lowpass_hz = 60_000, order = 4 },
]
```

| Key | Meaning |
|---|---|
| `tvr` | Transmitting voltage response, dB re 1 µPa/V at 1 m: a number, a table `[[hz, db], ...]` (interpolated over log frequency) or a resonance `{ resonance_hz, q, peak_db }` |
| `rvs` / `rvs_db` | Receive sensitivity, dB re 1 V/µPa, in the same three forms |
| `beam` | `"omni"` (default), `{ piston_radius_m = r }` (a baffled piston) or `{ pattern_db = [[deg, db], ...] }` |
| `axis` | Direction of the acoustic axis, `[0, 0, 1]` (straight down) by default |
| `max_drive_v` | Peak drive voltage before the projector is overdriven |
| `drive.pin`, `drive.supply_v` | The output pin and the driver stage's supply: the transducer sees 0 V for a low pin and `supply_v` for a high one |
| `drive.nominal_hz` | Frequency for the narrowband parts of the path (default: the TVR's resonance) |
| `receive.adc` | ADC input 0-3 the chain feeds |
| `receive.gain_db` | Preamp gain |
| `receive.bandpass_hz`, `order` | One band-pass `[low, high]` with a Butterworth order (2), shorthand for one filter |
| `receive.filters` | Alternative filters `{ highpass_hz, lowpass_hz, order, gain_db }`; with `select_pin`, filter 0 while the pin is low and filter 1 while it is high |
| `receive.bias_v`, `rails_v` | DC level the signal rides on (half the rails) and the rails the output clips at (`[0, 3.3]`) |

A transducer with only `receive` is a hydrophone, one with only `drive` a projector.

## One board in an environment

`run --environment FILE --position X,Y,Z` puts a board's hydrophone in an environment (its projector is not
used: there is nobody to hear it). IVCBoard2 hears the pool's pinger; it reports the pinger's amplitude once the
host sends `HostToDevice::Ivc2(ivc2::HostToDevice::PingerEnable(true))`, which a test does (the CLI's mock bridge
only pings):

```bash
firmware_simulator run $FW/ivc_board2 --board boards/talos/ivc_board.toml --mock-bridge 8 \
    --environment environments/pool.toml --position 2,6,1 --time 5s
```

`crates/sim_host/tests/acoustics.rs` checks that the pinger amplitude IVCBoard2 reports falls off with distance
as the environment's transmission loss predicts. The ADC reads the hydrophone voltage at the exact time of each
conversion.

In code, `sim_acoustics::Hydrophone::record` keeps every ADC sample a hydrophone produced and
`recording_wav(rate)` turns them into a WAV file (`sim_acoustics::wav`) for inspection.

## The shared medium: boards hearing each other

In a robot file with an `[environment]`, every board with a transducer is placed in one shared medium
([robots.md](robots.md)):

```toml
[environment]
file = "../environments/pool.toml"

[[board]]
name = "talos_ivc"
board = "../boards/talos/ivc_board.toml"
elf = "../external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release/ivc_board2"
bus = "talos"
position_m = [5.0, 6.0, 1.5]
```

Each board's drive pin feeds a projector: the pin's level changes are sampled onto a 1 MHz grid, scaled by the
driver supply and run through the TVR (a band-pass resonator for a resonance TVR), giving the pressure it
radiates. Every other board's hydrophone hears that over the environment's paths, plus the environment's own
sources and noise. Boards run in 100 µs slices, so a projector's waveform is known up to the time its board has
reached; any path longer than a slice (15 cm of water) reads exact values. Robots in a medium always run their
boards in turn, even with `--threads`.

`robots/ivc_pool.toml` (two first-generation IVCBoards, 17/19 kHz FSK) and `robots/ivc2_pool.toml` (two
IVCBoard2s) put one board on each of two vehicles' buses, 5 m apart in the pool:

```bash
firmware_simulator robot robots/ivc2_pool.toml --mock-bridge talos:8 --mock-bridge surface:8 --time 15s
```

`--ivc-send TIME@BOARD:BYTE` makes a board transmit through its mock bridge, and each mock bridge prints the bytes
its board receives ([robots.md](robots.md#mock-bridges)):

```bash
firmware_simulator robot robots/ivc2_pool.toml --mock-bridge talos:8 --mock-bridge surface:8 \
    --ivc-send 3.5s@talos_ivc:0x5a --time 13s
#   [12.200999s] surface_ivc: IVC received 0x5a
```

`robot --noise-db L` replaces every hydrophone's ambient noise with white noise at `L` dB re 1 µPa²/Hz.

## Moving transducers and Doppler

A board in a medium takes `position_m`, or a `trajectory`:

```toml
trajectory = { start_m = [5.0, 6.0, 1.5], velocity_mps = [0.2, 0.0, 0.0] }   # constant velocity
# trajectory = [[0.0, [5.0, 6.0, 1.5]], [10.0, [9.0, 6.0, 1.5]]]           # (seconds, position) waypoints
```

A board on a simulated vehicle takes neither: its transducer follows the hull (`[[vehicle.transducer]]`,
[vehicles.md](vehicles.md)). Moving hydrophones and projectors recompute their paths every millisecond and
interpolate the delays in between, so a closing or opening range shifts the received frequency and timing.
`RobotBench::place` / `Robot::place` move a board's transducer between runs (sweeps start every point from one
snapshot and move a board to its range).

## IVC link tests and sweeps

| Test | What it shows |
|---|---|
| `crates/sim_host/tests/ivc_link.rs` | IVCBoard to IVCBoard and IVCBoard2 to IVCBoard2 exchange bytes across `pool.toml`; the C IVCBoard2 modem and the Rust one exchange bytes both ways ([below](#c-and-rust-ivcboard2-interop)); the first-generation modem's reverberation finding and the C IVCBoard2's overwritten packet |
| `crates/sim_host/tests/acoustics.rs` | IVCBoard2's pinger amplitude against distance |
| `crates/sim_host/tests/vehicle.rs` (`ivc_boards_talk_from_a_drifting_vehicle`) | First-generation IVC boards still link with one of them on a drifting vehicle |
| `crates/sim_test/tests/ivc_sweep.rs` | Delivery against range and noise for both generations, in open water: boot once, snapshot, run every point in parallel from the snapshot; asserts that delivery degrades monotonically |

The sweeps print their tables with `--nocapture`:

```bash
cargo test --release -p sim_test --test ivc_sweep -- --nocapture
#   IVCBoard2, noise at 10 m: range, noise (dB re 1 uPa^2/Hz) -> delivered / sent (duplicates, wrong bytes, ACKs seen)
#       10 m     50 dB  ->  1/1  (0 dup, 0 wrong, 0 acked)
#   ...
```

They are the slowest tests in the suite (minutes). See [testing.md](testing.md#sweeps) for writing one.

In `robots/ivc_pool.toml` the first-generation boards lose a byte when one moves at 0.5 m/s, and also at some
fixed ranges. That is the modem, not Doppler or the moving-path model: its fixed threshold on the "other" band is
reached by the previous symbol's reverberation at the assumed 40 dB receive gain
(`firmware_finding_ivc_symbols_lost_to_pool_reverberation` in `ivc_link.rs`, [faults.md](faults.md#firmware-findings-so-far)).
The hydrophone unit tests check the moving paths against exact recomputation at every sample and the Doppler
shift of a walking-speed source.

## C and Rust IVCBoard2 interop

The C IVCBoard2 firmware (titan_firmware branch `blake/ivc`, `talos/IVCBoard2`: micro-ROS over USB through
`titan_dual_stdio_usb`) and the Rust `ivc_board2` (titan_firmware_rs, titan_link over CAN) are meant to speak the
same acoustic modem. On paper they do: the Rust modem is a port of the C one and keeps its waveform.

| | C (`ivc.h`, `tx.c`, `rx.c`, `consensus.c`, `fft/`) | Rust (`ivc2_core`, `firmware/ivc_board2`) |
|---|---|---|
| Tones | sync 50.5 kHz, `0` 51 kHz, `1` 51.5 kHz: a 50 % PWM square wave on GPIO 7, wrap 99, divider truncated to 1/16 | the same, same wrap and divider |
| Packet | one byte, LSB first, sync before and after every bit; 18 slots of 500 ms (9 s) | the same |
| Slot 0 | whatever the PWM was left at: the undivided 1.26 MHz square wave for the first packet after boot, the last sync after that | sync |
| CRC, ACK, retries | none (`calculate_crc`, `send_ack`, `process_packet` exist but are never called) | none |
| Receiver | 4096 samples at 500 kS/s, 64-tap FIR, decimate by 4, Hann, 1024-point FFT; SNR against a noise floor from 4 reference bands (EMA 0.05, settled after 50 frames); min SNR 4, dominance 1 | the same numbers (Goertzel on the band bins instead of the FFT) |
| Symbol decisions | consensus: 3 stable, 2 to switch; a symbol is emitted when the next tone starts | the same (the stable count saturates instead of wrapping at 255) |
| Observation rate | the newest capture once per main loop pass, about 40 to 50 ms (estimated), DMA free-running into two swap buffers | one capture on demand per pass, about 25 ms |
| Transmit queue | 8 bytes; **dequeues every pass even mid-packet**, so a second byte overwrites the packet in flight | 8 bytes; waits for the packet to end |
| Host side | ROS: `/talos/ivc/enqueue_data` and `/talos/ivc/rx_new`, `std_msgs/Int8` (bytes above 0x7f are negative) | titan_link: `Ivc2(EnqueueData)`, `Ivc2(RxNew)` |

Two checks in the emulator, without micro-ROS (`crates/sim_host/tests/ivc_link.rs`):

* `c_and_rust_ivc_board2_modems_exchange_bytes`: `test_programs/c_ivc_modem` is the C IVCBoard2 with its modem
  sources from `blake/ivc` unmodified and micro-ROS replaced by UART0 (a byte on the UART is enqueued, a received
  packet is printed as `ROS ivc/rx_new N`; `make -C test_programs c_ivc_modem` rebuilds it). In
  `robots/ivc2_pool.toml`, the C board sends 0xC3 to the Rust board and the Rust board sends 0x5A back, each
  arriving about 8.7 s after its packet started.
* `firmware_finding_c_ivc_board2_overwrites_the_packet_in_flight`: two bytes enqueued together on the C board
  arrive as one packet carrying the second; the first is lost.

`robots/ivc2_interop.toml` runs the real C firmware next to the Rust one, with the C board's image from your
titan_firmware build (`$TITAN_FIRMWARE_DIR`, [robots.md](robots.md#robot-files)) and its USB ports on fixed
terminals. The micro-ROS agent connects to the emulated board as to a real one:

```bash
export TITAN_FIRMWARE_DIR=~/titan_firmware     # on blake/ivc, with talos/IVCBoard2/build/IVCBoard2_with_bl.uf2 built
firmware_simulator robot robots/ivc2_interop.toml --mock-bridge talos:8 --realtime --time 600s \
    --ivc-send 30s@talos_ivc:0x5a
# in other terminals, with ROS 2 sourced:
ros2 run micro_ros_agent micro_ros_agent serial --dev /tmp/c_ivc-ros
ros2 topic echo /talos/ivc/rx_new
ros2 topic pub --once /talos/ivc/enqueue_data std_msgs/msg/Int8 "{data: 90}"
```

Rust to C: the Rust board logs `Transmitting 0x5a` at 30 s and the C board `packet read complete with: 90` (on
`/tmp/c_ivc-log` and as `c_ivc uart0:` lines) about 8.7 s later, and publishes 90 on `/talos/ivc/rx_new`. C to Rust:
after the `ros2 topic pub` the C board logs `writing out: ...` per bit, and about 9 s later the emulator prints
`[t] talos_ivc: IVC received 0x5a`. Send one byte at a time, at least 10 s apart, and not from both boards at once.
**To verify locally** (the micro-ROS library could not be built where this was written): the real C image in the
emulator, and whether the agent keeps its session while the emulator runs slower than the wall clock (the C modem
alone runs at about 0.4x real time with the Rust board beside it, so `--realtime` cannot keep up; the firmware's
own micro-ROS timeouts are on virtual time).

## Making a new environment

1. Copy the closest file in `environments/` and change `kind` and its size or depth. Put boards inside it: a
   position outside the water is an error when the robot loads.
2. Set the water (`temperature_c`, `salinity_ppt`) and the noise: `white` at a measured level, or `wenz` with
   shipping and wind for open water; add `shrimp` for warm shallow sites.
3. Choose `multipath.max_order`: 2-3 for a pool, 4 for shallow water. The cost grows with the number of image
   sources (order 3 in a tank is a few dozen paths per pair), so keep it as low as the test allows.
4. Add sources: a `pinger` for pinger tests, a `wav` recording of the real site's noise or interferers.
5. Point a robot file's `[environment] file` at it (with `seed` to vary the noise between runs), or pass it to
   `run --environment`.
6. Check it with a sweep (copy `ivc_sweep.rs`), or by recording a hydrophone to WAV.

To try another transducer or receive chain, change `[board.transducer]` in the robot file rather than the board
file; `robots/ivc_pool.toml` does that for the older 18 kHz modem.
