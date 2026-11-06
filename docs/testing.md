# Testing firmware with `sim_test`

`crates/sim_test` is the harness for firmware integration tests: an ordinary `#[test]` boots an unmodified image
on its board, drives its inputs, injects faults, and asserts on what the firmware logs, on its CAN frames and on
the `titan_link` messages a mock `titan_bridge` receives. Runs are deterministic, so assertions on exact times and
values are stable, and a booted board can be snapshotted to run many scenarios from one boot.

- [Running the tests](#running-the-tests)
- [A first test, step by step](#a-first-test-step-by-step)
- [The API](#the-api)
- [Inputs](#inputs), [Faults](#faults), [Snapshots](#snapshots), [Robots](#robots), [Sweeps](#sweeps)
- [How the test suites are organised](#how-the-test-suites-are-organised)
- [CI](#ci)

Tests that need no Rust (a robot or board, faults, inputs and checks on logs, statuses and frames) can be written as
YAML scenarios instead: [scenarios.md](scenarios.md).

## Running the tests

```bash
cd external/titan_firmware_rs/firmware && cargo build --workspace --release && cd -   # the board images
cargo test --release --workspace                                    # everything (several minutes)
cargo test --release -p sim_test --test smoke -- --nocapture        # one suite, with its table
cargo test --release -p sim_test --test faults can_bus_off          # tests whose name contains this
```

Always test in release mode: debug builds of the emulator are far too slow for firmware tests.

**Where the images come from.** `sim_test::titan_rs("power_board")` looks in
`external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release/`, or in `$TITAN_RS_RELEASE_DIR` when that
is set (a worktree, another checkout of titan_firmware_rs, CI); robot files' image paths are redirected the same
way. `sim_test::titan_c("basic_template/template_project.elf")` looks under `build/c/`
(`scripts/build_c_firmware.sh`), and `sim_test::titan_firmware("talos/ESCBoard/build/esc_board_with_bl.uf2")`
(`sim_host::titan_firmware_image`) in your titan_firmware checkout, `$TITAN_FIRMWARE_DIR`, for the C board apps
`scripts/build_c_boards.sh` builds. When an image is missing (or `TITAN_FIRMWARE_DIR` is unset) they print
`skipping: ... not built (...)` and return `None`, and the test returns early, **passing**. Run with `--nocapture` (or check for `skipping:` lines) to be sure
a green run actually ran the firmware.

**Which compiler built them.** titan_firmware_rs pins its compiler in `rust-toolchain.toml` (rustc 1.99.0 at the
pinned commit), and rustup uses it for the build in `external/titan_firmware_rs/firmware`. A different compiler gives
slightly different code and timing: the bench fingerprints change (they are per firmware build), and so do
timing-sensitive details such as which IVC packet in a reverberant pool loses a symbol. Tests must check behaviour
that holds for any build of the same source, not one build's exact outcome: windows instead of exact times, every
byte eventually delivered and acknowledged instead of "this packet gets through", statistics over several packets
instead of one. To check a test against another compiler, build the images into a scratch directory and point the
tests at it:

```bash
cd external/titan_firmware_rs/firmware
CARGO_TARGET_DIR=/tmp/fw-other rustup run 1.97.0 cargo build --workspace --release && cd -
TITAN_RS_RELEASE_DIR=/tmp/fw-other/thumbv6m-none-eabi/release cargo test --release -p sim_host --test ivc_link
```

## A first test, step by step

This walkthrough is `crates/sim_test/examples/walkthrough.rs`; `cargo run --release -p sim_test --example
walkthrough` runs it, and `cargo test` runs it as `tests/guide.rs`, which also checks that every snippet below
is still in that file. A test of your own looks the same inside a `#[test] fn`.

**1. Boot.** The builder takes the image, the board file (`boards/<name>.toml`, or a path) and a mock bridge for
the board's CANmore client; `record_bus` keeps every CAN frame. `wait_for_log` runs in 10 ms steps until a defmt
line contains the text, or fails with the log so far.

<!-- from crates/sim_test/examples/walkthrough.rs -->
```rust
let Some(elf) = sim_test::titan_rs("power_board") else { return };
let mut b = Bench::builder(&elf).board("mk2/power_board").bridge(1).record_bus().boot().unwrap();
let connected = b.wait_for_log("ROS connected", Time::from_secs(1)).unwrap();
```

**2. Status.** `run_until` and `run_for` advance virtual time. `statuses()` are the `FirmwareStatus` messages the
bridge received, with their times; `faults()` is the fault word of the last one; `link_errors()` lists anything
the bridge could not decode.

<!-- from crates/sim_test/examples/walkthrough.rs -->
```rust
b.run_until(Time::from_ms(1500)).unwrap();
let (_, status) = b.statuses().last().cloned().expect("a firmware status");
assert_eq!(status.board_name.as_str(), "mk2/power_board");
assert_eq!(status.client_id, 1);
assert_eq!(b.faults(), Some(0));
assert!(b.link_errors().is_empty());
```

**3. Board messages.** `messages()` is every `titan_link::DeviceToHost` the bridge received; `picked` filters and
maps them:

<!-- from crates/sim_test/examples/walkthrough.rs -->
```rust
fn electrical(b: &Bench) -> Vec<(Time, power::ElectricalReadings)> {
    b.picked(|m| match m {
        DeviceToHost::Power(power::DeviceToHost::Electrical(r)) => Some(*r),
        _ => None,
    })
}
```

**4. Inputs.** Physical inputs are named in the board file ([boards.md](boards.md#inputs)); `set` changes one while
the board runs:

<!-- from crates/sim_test/examples/walkthrough.rs -->
```rust
b.set("port_battery", 14.0);
b.run_for(Time::from_ms(1100));
let (_, r) = *electrical(&b).last().unwrap();
assert!((r.port_voltage - 14.0).abs() < 0.05, "{r:?}");
```

**5. Messages to the board.** `send` puts a `HostToDevice` on the link as titan_bridge would; the whole machine is
there to look at (`b.session.machine`), here the fan switch pin:

<!-- from crates/sim_test/examples/walkthrough.rs -->
```rust
let sent = b.now();
b.send(HostToDevice::Power(power::HostToDevice::ElectricalCommand(6)));
b.wait_for_log_after("Fans disabled", sent, Time::from_ms(500)).unwrap();
assert!(!b.session.machine.sys.gpio.input(fan_pin));
```

**6. CAN frames.** With `record_bus`, `frames()` is every frame on the bus (`BusRecord`: start and end time,
sender, the frame, whether it was acknowledged, and the error that destroyed it, if any).

**7. Snapshots and faults.** Boot once, then run each scenario from its own copy:

<!-- from crates/sim_test/examples/walkthrough.rs -->
```rust
let base = b.snapshot();
let mut lost = base.snapshot();
let t0 = lost.now();
lost.inject(Fault::Bridge(false)).unwrap();
let t = lost.wait_for_log_after("Lost connection to ROS", t0, Time::from_secs(2)).unwrap();
```

Faults can be scheduled, and parsed from the command line's spelling ([faults.md](faults.md)):

<!-- from crates/sim_test/examples/walkthrough.rs -->
```rust
cut.inject_at(at, "can-unplug".parse().unwrap());
cut.inject_at(at + Time::from_secs(1), "can-plug".parse().unwrap());
cut.run_for(Time::from_secs(2));
```

## The API

Everything is re-exported from `sim_test`: `Bench`, `RobotBench`, `sweep`, `Time`, `Fault`, `I2cFault`,
`AnalogFault`, `Trajectory`, `MockBridge`, `PuddlesBridge`, `Session`, `Robot`, `RobotSpec`, `Board`, `Inputs`,
`StopReason`, `ErrorCounters`, `ErrorState`, and the `titan_link` crate.

| `BenchBuilder` | |
|---|---|
| `Bench::builder(elf)` | Starts a bench for an image |
| `.board(name)` | Board file: `"mk2/power_board"` or a path ending in `.toml` |
| `.edit_board(\|b: &mut Board\| ...)` | Changes the board description first (a missing part, other straps) |
| `.bridge(client)` | Mock bridge pinging from time 0 with the kill switch released; on a board with Ethernet and no CAN (Puddles) a `PuddlesBridge` instead |
| `.bridge_with(client, start, kill)` | Starting at `start`, forwarding `kill` (`None`: no kill switch forwarding) |
| `.rom_boot()` | Boot through the bootrom and boot2 |
| `.record_bus()` | Keep every CAN frame |
| `.boot()` | Builds it (`Result<Bench, String>`) |

| `Bench` | |
|---|---|
| `now()`, `run_until(t)`, `run_for(d)` | Virtual time; `run_until` returns `Err(StopReason)` on a lockup or deadlock, `run_for` panics |
| `set(input, v)`, `get(input)`, `inputs` | Physical inputs (an unknown name panics) |
| `logged(text)`, `log_time(text)`, `log_times(text)` | defmt lines containing `text` and when they were read |
| `wait_for_log(text, limit)`, `wait_for_log_after(text, after, limit)` | Run until such a line appears |
| `uart(n)` | Text a UART sent |
| `messages()`, `picked(f)`, `statuses()`, `faults()`, `link_errors()` | What the bridge received |
| `send(msg)` | Sends a `HostToDevice` from the bridge |
| `frames()` | CAN frames (with `record_bus`) |
| `inject(fault)`, `inject_at(t, fault)` | Faults |
| `can_errors()`, `can_controller()` | The CAN controller's error counters, and its model |
| `inject_panic()`, `watchdog_reset()`, `power_cycle()` | Chip-level shortcuts |
| `snapshot()` | An independent copy (see below) |
| `session`, `bridge`, `puddles` | The machine and host side (`session.machine`, `session.logs`), the mock bridges |

For anything else go one level down: `bench.session` is a `sim_host::Session` (the machine plus RTT/defmt, UART,
CAN and Ethernet host sides), and `session.machine` the RP2040 itself (cores, `sys` with every peripheral and
attached device, `sys.gpio.input(pin)`, `sys.pin_device::<T>()`, `sys.i2c_device::<T>()`). The `sim_host` tests
(`crates/sim_host/tests/common.rs`) build sessions by hand like this.

## Inputs

Setting an input takes effect at the next ADC conversion or sensor read. The values a board starts with come
from its board file; change them there (or with `edit_board`) for a different starting point, since the firmware
may read them during boot. A vehicle drives pressure inputs itself ([vehicles.md](vehicles.md)).

## Faults

`inject` applies a fault now and returns an error if it does not fit the board; `inject_at` schedules it and
`run_until` applies it on the way (and panics if it does not fit). `RobotBench::inject(board, fault)` and
`inject_at(t, board, fault)` do the same on one board of a robot: CAN bus faults go to that board's bus, bridge
faults to the mock bridge on it. The faults and what they model: [faults.md](faults.md). `tests/faults.rs` is the
example to copy; its `scenarios` helper runs several fault scenarios from one snapshot on threads.

## Snapshots

`Bench::snapshot` and `RobotBench::snapshot` deep-copy everything (machines, devices, buses, acoustic medium,
vehicles, logs, bridge state, physical inputs) in a few milliseconds; the copy runs on independently and exactly
as the original would (`tests/harness.rs` checks that frame for frame, and that two copies with different battery
voltages run side by side, each reporting its own).

Host endpoints are the exception: the benches fork the bridges they made (`bridge`, `RobotBench`'s mock bridges and
its `eth_bridge`s), but an endpoint you added by hand (another `MockBridge`, a SocketCAN interface, a UDP relay)
must be attached to the copy again. One level down, `Session::snapshot` shares the inputs with the original;
`Session::snapshot_with_inputs` gives the copy its own.

## Robots

`RobotBench::load(file, bridges, edit)` loads `robots/<file>` (or a path), lets `edit` change the parsed robot file
first (positions, environment, straps), and puts one mock bridge on each named bus:

```rust,ignore
let mut r = RobotBench::load("ivc2_pool.toml", &[("talos", 8), ("surface", 8)], |spec| {
    spec.environment.as_mut().unwrap().file = "../environments/open_water.toml".into();
    spec.board[1].position_m = Some([10.0, 0.0, 5.0]);
})
.unwrap();
r.run_until(Time::from_ms(3500));
r.bridge("talos").send(HostToDevice::Ivc2(ivc2::HostToDevice::EnqueueData(0xA5)));
r.run_for(Time::from_ms(9600));
let got = r.picked("surface", |m| match m {
    DeviceToHost::Ivc2(ivc2::DeviceToHost::RxNew(x)) => Some(*x),
    _ => None,
});
```

`board(name)` is a board's `Session`, `robot` the `sim_host::robot::Robot` (buses, vehicles, `threads`),
`place(board, trajectory)` moves a board's transducer, and `eth_bridge(board)` puts a `PuddlesBridge` on an
Ethernet board's cable (Puddles) and returns it; bridge faults go to it when the board has no CAN bridge. For more than one bridge per bus, add `MockBridge`s to
`r.robot.buses[j].endpoints` yourself (`tests/robot.rs` does).

## Sweeps

`sweep(&base, &points, threads, |bench, point| ...)` runs a scenario once per point, each from its own snapshot of
`base`, on up to `threads` threads, and returns the results in the order of the points. Each point has inputs of
its own, so a scenario may set them. `tests/ivc_sweep.rs` is the
example: boot two IVC boards once, then for each point move one board to the point's range, set the noise, send
bytes and count what arrives.

## How the test suites are organised

`cargo test --release --workspace` runs all of these. Suites that boot titan_firmware_rs or C images skip (and
pass) without them.

| Where | What |
|---|---|
| `crates/cpu_armv6m` unit tests | Every Thumb instruction and flag case, exceptions, batching against single steps |
| `crates/rp2040` unit tests (`src/periph/*`) | Register-level peripheral behaviour from the datasheet |
| `crates/rp2040/tests/` | Hand-written programs (`m0_blink`, `m1_watchdog`, `m6_resets`), pioasm-assembled PIO programs (`pio_programs`), the bootrom's USB boot (`usb_boot`), and the performance work's equivalence checks (`batching`, `busy_wait`, `systick_reference`) |
| `crates/sim_devices`, `sim_acoustics`, `sim_physics` unit tests | Each device model, the acoustic channel, the vehicle physics |
| `crates/sim_host/tests/` | Firmware on boards built by hand, per milestone: `template`, `embassy_blinky`, `gdb` (the real GDB too: `$GDB`, `gdb-multiarch` or `arm-none-eabi-gdb`), `pico_sdk`, `can`, `sensors`, `pio`, `flash`, `hle` (`--hle-flash` leaves the flash as the ROM does), `acoustics`, `ivc_link` (also the C IVCBoard2 modem against the Rust one, and the real C IVCBoard2 from `$TITAN_FIRMWARE_DIR` on `blake/ivc`), `usb`, `robot_usb` (USB on robot boards, images from environment variable paths), `vehicle`, `parallel` (threaded robots run as in turn), `acoustic_threads` (boards in water on threads run as in turn) |
| `crates/sim_test/tests/smoke.rs` | Every titan_firmware_rs board image on its board: connects, reports its identity, no link errors, no panic |
| `crates/sim_test/tests/robot.rs` | All of Talos on its two buses |
| `crates/sim_test/tests/puddles.rs` | Puddles over Ethernet/UDP |
| `crates/sim_test/tests/harness.rs` | The harness itself (snapshots, board edits), the H-bridge board, the Dynamixel bench test |
| `crates/sim_test/tests/faults.rs` | Fault injection against the safety system, and the `firmware_finding_*` tests |
| `crates/sim_test/tests/can_log.rs` | Recording, replaying and comparing CAN traffic, a cannelloni tunnel |
| `crates/sim_test/tests/c_boards.rs` | titan_firmware's C board apps from `$TITAN_FIRMWARE_DIR`: `robots/talos_c.toml` boots every board through its bootloader into its app with CANmore heartbeats, and the Mercury ESC board's raw thruster command finding (skipped without the builds) |
| `crates/sim_test/tests/vehicle.rs`, `vehicle_talos.rs`, `vehicle_nereus.rs` | Thrusters in the loop on Puddles and Talos, and the controller's raw CAN thruster frames with nereus as the physics |
| `crates/sim_test/tests/ivc_sweep.rs` | IVC delivery against range and noise (prints tables with `--nocapture`; the slowest suite) |
| `crates/sim_test/tests/guide.rs` | This page's walkthrough |
| `crates/sim_test/tests/scenarios.rs` | Every YAML scenario in `scenarios/` ([scenarios.md](scenarios.md)), the runner's verdicts and evidence, determinism on any number of threads, and A/B comparisons of real runs |
| `crates/sim_test/tests/fuzz_smoke.rs` | A short fixed-seed fuzz campaign on one board: it runs, finds the same things on any number of threads, and every reproducer replays ([fuzzing.md](fuzzing.md)) |
| `tests/host_cli.rs` (root package) | The command line: robot record/replay/diff, `bridges` outputs, `scripts/vcan_up.sh` with stand-ins, `robot_in_a_box.sh` and `nereus_firmware_sim.sh` in `--dry-run`, the nereus bridge's Python tests (its ROS node too, against stand-ins for the dashboard and nereus, when ROS 2 is sourced: `test_firmware_bridge_ros.py`), the Docker files' consistency |
| `tests/scenario_cli.rs` (root package) | `scenario` and `compare` on the command line: exit statuses, JUnit, JSON, Markdown |
| `tests/docs.rs` (root package) | Every command line option the docs and the README mention exists |

`test_programs/` holds the hand-written Thumb programs, a small embassy firmware, pico-sdk programs and small C
titan_firmware programs (`c_dual_usb`, `c_ivc_modem`) used by these tests; their ELFs are committed (`make -C test_programs` rebuilds them).

When the firmware rather than the emulator looks wrong, write a test that asserts today's behaviour, name it
`firmware_finding_*`, and explain it in a comment and in [faults.md](faults.md#firmware-findings-so-far).

## CI

`.github/workflows/ci.yml` has three jobs:

* **test**: checks out the submodules, installs the firmware's pinned compiler (`rustup toolchain install` in
  `external/titan_firmware_rs`, which reads its `rust-toolchain.toml`), builds the titan_firmware_rs images
  (`cargo build --workspace --release`),
  runs `cargo fmt --all --check`, `cargo clippy --workspace --all-targets --release -- -D warnings` and
  `python3 scripts/gen_boards.py --check`, then `cargo test --release --workspace -- --nocapture`, and prints the
  smoke test and sweep tables from the log. It then runs the YAML scenarios with the command
  (`firmware_simulator scenario scenarios/ --junit scenarios.xml`) and keeps the JUnit XML as an artifact
  ([scenarios.md](scenarios.md#in-cargo-test-and-ci)).
* **c-firmware**: installs `gcc-arm-none-eabi`, builds the C programs with `scripts/build_c_firmware.sh` and runs
  the tests that boot them (`sim_host`'s `pico_sdk`, `flash`, `can`). It sets `CMAKE_POLICY_VERSION_MINIMUM=3.5`
  because the runners' CMake 4 refuses pico-sdk's bundled pioasm and elf2uf2 projects otherwise. (The same steps
  in the Nix shell, with its ARM GCC 13.3: the programs build and the 17 tests pass, none skipped.) The C board
  apps' tests (`c_boards.rs`, the real C IVCBoard2 in `ivc_link.rs`) are not in CI: they need titan_firmware's
  micro-ROS library; run them with `TITAN_FIRMWARE_DIR` set after `scripts/build_c_boards.sh`.
* **macos**: installs the firmware's pinned compiler, builds the titan_firmware_rs images and runs clippy and the whole suite natively on macOS (Apple
  silicon). SocketCAN is Linux only; on macOS `SocketCan::open` fails with a clear error and CAN reaches a Linux
  machine through `can-tunnel` instead.

Both submodules are public, so the workflow needs no secrets. Every job fetches `external/titan_firmware_rs`:
the emulator's workspace uses its host crates (`canmore`, `titan_link`), and the firmware tests use its images.

Prebuilt binaries for other repositories' CI, and the GitHub Action that installs them, are in [ci.md](ci.md).

### In titan_firmware_rs

titan_firmware_rs runs this repository's firmware tests on every push and pull request of its own, in
`.github/workflows/emulator.yml` there ("Emulator CI" in its README). The workflow builds the images with the
firmware's pinned compiler, checks out this repository at `master` (public, so no secret is needed), and puts its own checkout in place of `external/titan_firmware_rs`. This
repository's host crates and images then both come from that commit, and no submodule is fetched. It then runs:

```bash
cargo test --release -p sim_test --test smoke -- --nocapture   # every board boots and connects; the table goes to the job summary
cargo test --release -p sim_test --test faults -- --nocapture
cargo test --release -p sim_test --test robot -- --nocapture
cargo test --release -p sim_host --test ivc_link -- --nocapture
```

These take about 2.5 minutes on 4 cores once built (`ivc_link` about two minutes, `faults` about 12 s, `robot` 5 s,
`smoke` 3 s). There, a test that skips for want of an image fails. The workflow sets `SIM_REQUIRE_TITAN_RS=1`, which
makes `sim_test::titan_rs`, `sim_host::titan_rs_image` and the tests built on them panic instead of skipping. It
also fails the job on any `skipping: ...thumbv6m-none-eabi/release/...` line. Tests that need the C images still
skip there.

These tests encode what the firmware is expected to do, including the `firmware_finding_*` tests that pin down
today's bugs. So when titan_firmware_rs changes behaviour on purpose (a finding fixed, a timing changed), its
workflow fails against this repository's `master` until the tests here change to match. Make that change on a
branch here, check it against the new images (`TITAN_RS_RELEASE_DIR=.../release cargo test ...`), and run the
titan_firmware_rs workflow by hand with `emulator_ref` set to the branch. Bumping the `external/titan_firmware_rs`
pin here goes with the same change.
