# Calibration: fitting the models to the robot, and replaying pool tests

The acoustic front end, the thrusters and the vehicle in the emulator are plausible guesses
([acoustics.md](acoustics.md), [vehicles.md](vehicles.md)). Once the team has recordings from the robot, these
commands turn them into fitted model files, and a CAN log from a pool test into a reproduction at a desk:

| Command | Data | Writes |
|---|---|---|
| `firmware_simulator calibrate acoustics MANIFEST` | Hydrophone recordings (WAV) with their geometry | A `[transducer]` overlay, an environment file, a report, plots, the simulated captures |
| `firmware_simulator calibrate thrusters CSV...` | Thrust test-stand runs (CSV) | `thruster_model` for a vehicle file, the speed curve (`throttle_fit` or `max_rpm`) and `tau_ms` for the ESC board's `[[dshot_esc]]`, a report, plots |
| `firmware_simulator calibrate vehicle CSV... --vehicle FILE` | Pool-test logs: thruster RPM against depth, IMU and DVL (CSV, from a ROS bag with `scripts/bag_to_calib.py`) | A fitted vehicle file, a report, plots |
| `firmware_simulator replay-pool-test LOG...` | candump logs from the Orin (`can0`, `can1`), optionally the recorded pose | A report of where the emulated boards' CAN traffic differs from the real boards' |

Every fit is deterministic (Levenberg–Marquardt with finite differences, Nelder–Mead where derivatives stall; no
random restarts) and runs the emulator's own models, so fitted values go straight back into the files the
emulator reads. Nothing is changed in the repository: copy what the outputs say into the board overlays, vehicle
and environment files when the fits look right, and rerun `python3 scripts/gen_boards.py` for board files.

> **To verify locally.** No real recording existed where this was written: every calibration was tested on data
> the emulator generated itself with known parameters (perturb, fit, recover; with noise too), the acoustic one
> also on a capture the IVCBoard2 firmware took in the emulator, and the replay on a log of one emulator run
> replayed into another. `scripts/bag_to_calib.py` has not read a real bag (no ROS 2 here); its conversion logic is
> tested. The topic names below are riptide's as far as this repository knows them: check them with
> `ros2 bag info`.

- [Data to collect on the robot](#data-to-collect-on-the-robot)
- [Acoustics](#acoustics)
- [Thrusters](#thrusters)
- [The vehicle](#the-vehicle)
- [Replaying a pool test](#replaying-a-pool-test)
- [Reading the results](#reading-the-results)
- [How the calibrations are tested](#how-the-calibrations-are-tested)

## Data to collect on the robot

**CAN logs (every pool test).** On the Orin, while titan_bridge (or the micro-ROS agents) run, one terminal per
bus, each writing `candump-DATE.log` in the current directory:

```bash
candump -l can0,0:0,#FFFFFFFF      # the internal CAN FD bus; the filter also logs error frames (bus errors)
candump -l can1,0:0,#FFFFFFFF      # the external bus (the smart battery)
```

Start them before the boards power up if you can (the replay then sees the boards boot), otherwise any time:
the replay lines the log up on the boards' reported uptimes. Note the time, what the robot was doing and anything
odd ("thruster 3 stuttered at 14:02").

**A ROS bag (pool tests with the vehicle in the water).** At the same time, record the thruster speeds and what
the robot measured. The defaults of `scripts/bag_to_calib.py` expect these topics (riptide's names; check with
`ros2 topic list` on the robot):

```bash
ros2 bag record -o pool_run1 \
    /talos/state/thrusters/rpm /talos/depth/pose /talos/vectornav/imu /talos/dvl/twist \
    /talos/odometry/filtered /talos/state/firmware /talos/state/kill
```

For the vehicle fit, drive manoeuvres that excite one axis at a time, each held a few seconds, at two strengths
(a light and a hard push separate linear from quadratic drag): dives and rises on the vertical thrusters, surges
forward and back, yaw turns, and a few seconds of zero thrust (buoyancy alone). Keep clear of the walls and the
bottom.

**Thrust test stand.** One CSV per run, with a header row. A time series (sampled at 20 Hz or more, each command
held a second or two) gives the motor's lag too; a table of steady points gives the curve only.

| Column | Meaning |
|---|---|
| `time_s` | Seconds (optional: without it every row is a steady point) |
| `dshot`, `throttle` or `pwm_us` | The command: a DShot 3D value (48-1047 forward, 1049-2047 reverse), a signed fraction -1..1, or servo PWM (1500 µs stop, ±400 µs full) |
| `rpm` | Measured mechanical RPM (the ESC's telemetry, or an optical tachometer); unsigned values take the command's sign |
| `thrust_n` or `thrust_kgf` | Measured thrust (unsigned values take the command's sign) |
| `voltage_v` | Supply voltage (optional; rows are fitted per voltage) |

Run forward and reverse, at the supply voltage the robot runs at (and others if you can), with the propeller
fitted the way it is on the robot.

**Hydrophone recordings.** Two kinds, each a WAV file:

* **The board's own ADC capture** (`kind = "board"`): what ADC 1 of the IVC board read, with -1..1 the ADC's input
  range (0-3.3 V), so a quiet input sits at its bias. The IVC firmware does not stream raw captures today; a
  calibration build that dumps a capture (4096 12-bit codes at 500 kS/s) over USB or CAN, or a scope or DAQ on the
  ADC 1 pin, gives one. Convert codes to WAV with `(code - 2047.5) / 2047.5` per sample, at the capture's rate.
* **A calibrated reference hydrophone** (`kind = "reference"`): its WAV from a DAQ, with its sensitivity (dB re
  1 V/µPa, from its calibration sheet) and the volts a full-scale sample stands for. It measures the pressure, so
  it calibrates the pool and the pinger apart from the board.

Record the geometry with a tape measure in the pool's coordinates (`environments/pool.toml`: x along the 25 m
side, y across, z down from the surface): where the source and the receiver were. Useful captures: the pinger at
three or more distances, with the board's gain setting noted; a few seconds of the quiet pool (noise floor); and
the same place with the board and the reference hydrophone, which separates the board's gain from its
sensitivity.

## Acoustics

A manifest (TOML) says what to fit to which recordings:

```toml
board = "../boards/talos/ivc_board.toml"        # its [transducer] is the starting point
environment = "../environments/pool.toml"
fit = ["gain_db", "lowpass_hz", "noise_db", "wall_reflection", "bottom_reflection"]
# method = "nelder-mead"                        # default "lm"
# adc_bits = 12                                 # board recordings are quantised like this
# adc_range_v = [0.0, 3.3]

[source]                                        # what the recordings hear (else the environment's sources)
kind = "pinger"
position_m = [20.0, 6.0, 2.0]
frequency_hz = 25_000
pulse_ms = 4
period_ms = 2000
source_level_db = 180

[start]                                         # starting values the files do not have
# highpass_hz = 20_000

[[recording]]
file = "ping_2m.wav"                            # relative to the manifest
receiver_m = [18.0, 6.0, 2.0]
kind = "board"
start_s = 0.0                                   # simulated time of the first sample used (the pinger's clock)
skip_s = 0.0                                    # seconds of the file to skip
duration_s = 0.03                               # how much to use (all by default)
# select = 1                                    # the filter select pin's level (1: both IVC firmwares drive it high)
# gain_offset_db = -20                          # a switchable gain set lower for this one

[[recording]]
file = "ref_8m.wav"
receiver_m = [12.0, 6.0, 2.0]
kind = "reference"
sensitivity_db = -211
volts_full_scale = 1.0
```

```bash
firmware_simulator calibrate acoustics calibration/pool.toml -o calibration/acoustics
```

The parameters (`fit`, or `--fit gain_db,noise_db`):

| Parameter | What it changes |
|---|---|
| `gain_db` | The receive chain's preamp gain |
| `sensitivity_offset_db` | Added to the transducer's receive sensitivity, in whichever form the file gives it |
| `highpass_hz`, `lowpass_hz` | The corners of the filter the recordings go through (all of them must use the same one) |
| `bias_v` | The DC level at the ADC pin: the ADC offset |
| `rail_low_v`, `rail_high_v` | Where the chain clips |
| `noise_db` | The environment's white noise level (dB re 1 µPa²/Hz) |
| `wall_reflection`, `bottom_reflection`, `surface_reflection` | The environment's reflection coefficients: the reverberation |
| `source_level_db` | The source's level (the manifest's `[source]`, or the environment's first) |

Each recording is simulated as a board there would hear it (the same `sim_acoustics` hydrophone and receive chain,
sampled at the recording's rate, quantised by the ADC) and compared by features: DC level, RMS, peak, the share
of clipped samples, the noise floor (the quietest 10 % of 0.5 ms blocks), a third-octave spectrum, and the level in
the 1 ms blocks after a ping (the reverberation's decay). `gain_db` and `sensitivity_offset_db` only matter as
their sum in board recordings; a reference recording at the same place tells them apart.

Outputs in the `-o` directory: `overlay.toml` (the board's `[transducer]` with the fitted values, for the board
overlay or a robot file's `[board.transducer]`), `environment.toml`, `report.md`, `spectrum_NAME.svg` and
`envelope_NAME.svg` per recording, and `sim_NAME.wav`, the fitted simulation of each recording to listen to or
hold against it.

**The emulator's own capture as WAV.** `run --hydrophone-wav FILE` (and `robot --hydrophone-wav FILE`, `NAME` in
the file name for the board) keeps every sample the firmware's ADC took of the hydrophone and writes them, in the
format a board recording has (12-bit, -1..1 is 0-3.3 V). The firmware captures in bursts (4100 samples every
25-40 ms for IVCBoard2), with the gaps at mid-scale; the run says when the first sample was taken, which is the
`start_s` of a recording of the first burst:

```bash
firmware_simulator run $FW/ivc_board2 --board boards/talos/ivc_board.toml --environment environments/pool.toml \
    --position 12,6,1.5 --time 1s --hydrophone-wav capture.wav
#   hydrophone: 36900 ADC sample(s) in capture.wav (0.342 s at 500000 Hz from 0.239912s, -1..1 is 0..3.3 V at the pin)
```

## Thrusters

```bash
firmware_simulator calibrate thrusters stand_16v.csv stand_14v.csv --vehicle vehicles/talos.toml \
    --board boards/mk2/esc_board.toml -o calibration/thrusters
#   thrust curve: 47.012 N forward, 35.988 N reverse at 3800 RPM
#   ESC model at 16.0 V:
#     forward_c0               10.1083 ± 0.215
#     ...
#     tau_ms                   45.0213 ± 0.052
```

It fits, through the emulator's own models:

* **the thrust curve** (`thrust = k rpm |rpm|`, separate forward and reverse) from the steady rows (500 ms after a
  command change, `--settle`), at the measured RPM, or at the ESC model's RPM when the data has none (then the
  curve is only as good as `max_rpm`; the report says so). The vehicle file's `at_rpm` is kept (`--at-rpm`
  changes it);
* **the ESC model** (`sim_devices::dshot::Esc`: its speed curve from throttle to RPM, the motor's first-order
  lag) from the RPM column. The speed curve is fitted in the form the board file gives it (`--board`): a
  `throttle_fit` (the throttle step at which the motor settles at a speed, `c0 + c1 rpm + c2 rpm²`, per direction:
  `forward_c0` ... `reverse_c2`; the mk2 ESC board's APD 80F3 bench fit is the start) or a `max_rpm` line;
  `--curve linear` or `--curve fit` picks the form. `tau_ms` is fitted from a time series. `--fit` names a subset
  (`--fit tau_ms`). Rows are fitted per supply voltage (the model has none); the output is for `--voltage`, or the
  voltage with the most rows.

`thrusters.toml` holds the line for the vehicle file and the keys for each `[[dshot_esc]]` of the ESC board's
overlay; `report.md` the residuals, by direction too (a large error in one direction means the model needs another
parameter, such as a different top speed in reverse); `thrust.svg`, `rpm.svg` and `rpm_time.svg` the fits. The
ESC model's fittable parameters are listed in `sim_calib::thrusters::ESC_PARAMS`: a change to the model's
equations needs nothing there (the fit runs the model), a new parameter one entry.

## The vehicle

Convert the bag (where ROS 2 is installed), then fit:

```bash
python3 scripts/bag_to_calib.py pool_run1 --out calibration/pool_run1 --vehicle vehicles/talos_nereus.toml
firmware_simulator calibrate vehicle calibration/pool_run1/vehicle.csv --vehicle vehicles/talos_nereus.toml \
    -o calibration/vehicle
```

The log format (what `bag_to_calib.py` writes, and what `robot --vehicle-csv` writes too):

| Column | Meaning |
|---|---|
| `time_s` | Seconds, increasing |
| `rpm_NAME` (or `rpm_K`, K the thruster's index) | Motor RPM of the vehicle file's thruster NAME, signed as the motor turns |
| `cmd_NAME` | Commanded RPM instead: passed through the ESC's lag (`--esc-tau-ms`, 60) |
| `NAME_n` | Thrust instead, N (as `--vehicle-csv` writes it) |
| `north_m`, `east_m`, `depth_m` | Position (world north-east-down) |
| `roll_deg`, `pitch_deg`, `yaw_deg` | Attitude |
| `u_mps`, `v_mps`, `w_mps` | Body velocity (DVL), forward-starboard-down |
| `p_dps`, `q_dps`, `r_dps` | Body rates (gyro) |

Any subset of the measured columns will do; the more, the better. The log is simulated in segments
(`--segment`, 4 s), each starting from the measured state, so an error does not grow over the whole log and
unmeasured parts of the state matter only within a segment. The parameters (`--fit`, comma separated) are a
vehicle file's: `mass_kg`, `volume_m3`, `height_m`, `munk_moment_scale`, `cob_m.x|y|z`, `inertia.roll|pitch|yaw`,
and `added_mass.AXIS`, `linear_drag.AXIS`, `quadratic_drag.AXIS` for AXIS surge, sway, heave, roll, pitch, yaw.
By default: `volume_m3` and the drag and added mass in surge, heave and yaw. Several logs are fitted together.
`--density 1025` for sea water.

Outputs: `vehicle.toml` (the vehicle file with the fitted values; the original's comments are not carried over),
`report.md` (parameters with their uncertainty, the RMS error per channel before and after), and plots of depth,
yaw, surge and heave velocity.

`bag_to_calib.py` converts REP 103 frames (world east-north-up, body forward-left-up) to the emulator's; its options
name the topics and the fields that hold the values (`--rpm-topic`, `--rpm-field`, `--depth-field`,
`--depth-up`, `--odom-topic`, ...: `python3 scripts/bag_to_calib.py --help`). It also writes `pose.csv` from the
odometry, for the replay.

## Replaying a pool test

```bash
firmware_simulator replay-pool-test candump-2026-11-04_140312.log candump-2026-11-04_140313.log \
    --robot robots/talos.toml --report replay.md --candump-log emulated.log
```

The frames the Orin sent (thruster RPM commands to clients 2 and 3, the kill switch, titan_link and micro-ROS
requests) go to the emulated boards at their recorded times; what the boards send is compared with what the real
boards sent, per message stream, as `can-diff` does ([robots.md](robots.md#comparing-can-diff)). It exits with 0
when they agree, 1 when they differ, 2 on errors.

What it does with a real log:

* **Interfaces.** `can0`, `can1`, ... go to the robot file's first, second, ... bus, or a bus of the same name;
  `--iface can1=external` says otherwise. Frames on an interface with no bus are counted and left out.
* **Clocks.** The log has the Orin's clock and starts whenever candump did. The emulated boards' real twins report
  their uptime in `FirmwareStatus`; a straight line through uptime against log time gives each board's power-on
  moment and its clock's drift against the Orin's (statuses that waited on a busy bus are left out of the line).
  How long a board takes from power-on to uptime zero is measured on a copy of the emulated robot first. The log is
  mapped onto the emulator's time by the first board's line; the report lists every board's, so a board that
  powered on later than the robot file says (`power_on_ms`) shows. Without uptimes (C firmware), the log's first
  frame is replayed at power-on, or at `--log-start`, and the comparison searches for the offset. `--drift-ppm`
  gives the drift instead of fitting it.
* **Boards that are not in the robot file.** What other CANmore clients sent is replayed as recorded (the bus is as
  busy as it was; `--no-other-boards` leaves it out) and listed. Identifiers that are not CANmore are replayed and
  shown as raw streams.
* **Bus errors and damaged lines.** Error frames (logged with the error filter above) and unreadable lines are
  counted, with when they happened.
* **Settling.** The real boards had been talking to the Orin long before a log that started late; the emulated
  ones meet the Orin at the log's first frame and may answer its first messages differently. `--compare-after 2s`
  compares from two seconds into the log.

The report: the logs (frames, error frames, damaged lines), the clocks, what was replayed, the stream table with
missing, extra and differing messages and each stream's timing (`FirmwareStatus` is compared without its uptime),
the boards' fault words over time with the first difference, the emulated boards' warnings, errors, resets and
lockups, and a verdict. `--only 'client 2'`, `--ignore-payload`, `--tolerance` and `--details` work as with
`can-diff`. `--candump-log` writes what the emulated boards sent, for `can-diff` against the hardware log. Fields that measure
time between host messages differ by a few microseconds even for a perfect twin (candump stamps to the
microsecond, so replayed frames arrive within a microsecond of when they did): replaying Talos's own log shows
`Esc::Telemetry` `packet_delta_us: 500000 -> 500002`; `--ignore-payload Esc::Telemetry` leaves such streams to
their timing.

**The vehicle.** With `--pose pose.csv` (from `bag_to_calib.py`) the robot's vehicle follows the recorded pose, as
an [external vehicle](vehicles.md#external-vehicles): its depth sensors read the recorded depth and its
transducers move with it, so the boards see what the real ones saw. `--pose pose.csv --pose-compare` lets the
emulator's physics move the vehicle on the replayed thruster commands and reports its depth and yaw error against
the recording instead.

A bug seen in the water then becomes: record the logs, replay them, read where the emulated firmware first does
something the real one did not (or the same thing, which points at the hardware), and reproduce it with
`robot --replay` and the debugger ([robots.md](robots.md#replaying)).

## Reading the results

* **Residual RMS** before and after: residuals are scaled (1 dB for levels, 1 % for clipping, 2 % of the thrust,
  1 % of the top speed, 2 cm of depth, 1-2° of attitude), so about 1 after the fit is as good as the data allow;
  much more means the model cannot follow the data: look at the features or channels with large residuals.
* **± (1 sigma)** is the standard error from the fit's Jacobian. One as large as the value means the data do not
  tell that parameter; a correlation warning means two parameters only matter together. Fit fewer parameters, or
  record what separates them.
* **Plots** show the measurement, the fitted model and the model before. Look at them before trusting a number.
* Commit fitted values only with a note in the file of where they came from (date, data, report).

## How the calibrations are tested

| Test | What it shows |
|---|---|
| `sim_calib` unit tests (`cargo test --release -p sim_calib`) | The optimisers on known problems; each calibration recovering parameters it was given from data its own models generated (clean and with noise): the receive chain's gain, corner, offset and the pool's noise and walls (with and without clipping), a source level and bottom from reference recordings, the thrust curve and ESC from a stand staircase (and per voltage), Talos's drag, added mass and buoyancy from a minute of manoeuvres |
| `crates/sim_calib/tests/pool_replay.rs` | A log of one emulator run, made to look like a hardware log (500 ppm clock drift, started late, a board not in the robot file, a non-CANmore identifier, bus errors, a damaged line), replayed into a fresh run: the report finds exactly the perturbation the first run had (the port battery changed, the cable pulled) and nothing else |
| `tests/calibrate_cli.rs` | The commands end to end; the acoustic fit on a capture the IVCBoard2 firmware took in a pool other than the assumed one |
| `sim_host` canlog tests | Lenient parsing of real logs: error frames, CAN XL, damaged lines |
