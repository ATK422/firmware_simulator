# firmware_simulator guide

An RP2040 emulator that runs OSU UWRT's unmodified robot board firmware, `titan_firmware` (C, pico-sdk) and
`titan_firmware_rs` (Rust, embassy), without hardware: one board or a whole robot, on emulated CAN buses, with
their sensors, actuators, the water around them and the robot computer's side of the link.

| Page | For |
|---|---|
| [Quick start](#quick-start) | Build the firmware, run a board, read its logs, run a test |
| [running.md](running.md) | The `run` command: images, boot modes, logs, traces, GDB and VS Code, flash files, USB, Puddles Ethernet |
| [boards.md](boards.md) | Board files: every device table and its fields, overlays, the generator, inputs |
| [robots.md](robots.md) | Robot files and the `robot` command, mock bridges, CAN logs, replay and `can-diff`, cannelloni, `bridges`, robot in a box |
| [acoustics.md](acoustics.md) | Acoustic environments, transducers, the shared medium, IVC link tests and sweeps |
| [vehicles.md](vehicles.md) | Vehicle physics: vehicle files, thruster wiring, sensors following the hull, external vehicles, CSV logs |
| [calibration.md](calibration.md) | Fitting the acoustic, thruster and vehicle models to recordings from the robot (`calibrate`), the data to collect, and replaying a pool test's CAN logs into the emulated boards (`replay-pool-test`) |
| [nereus.md](nereus.md) | The firmware in nereus: the riptide stack's thruster commands through the emulated ESC boards move nereus's vehicle |
| [testing.md](testing.md) | Writing firmware tests with `sim_test`, the test suites, CI |
| [scenarios.md](scenarios.md) | Firmware tests in YAML without Rust (`scenario`: faults, inputs, checks on logs, statuses, frames and states; JUnit for CI), and firmware A/B comparisons (`compare`) |
| [fuzzing.md](fuzzing.md) | Fuzzing the firmware through the emulator: malformed CAN traffic, oracles, coverage feedback, minimised reproducers, CI |
| [tools.md](tools.md) | Developer tools: the firmware profiler, VCD waveforms, the live dashboard and its API, the MCP server for coding agents, JSON logs, robot traces |
| [nanoros.md](nanoros.md) | nano-ROS (early): what osu-uwrt/nano_ros is, the mock agent, its test board, findings, and which files to change when nano-ROS changes |
| [faults.md](faults.md) | Every fault, its CLI spelling and what it models; firmware findings so far |
| [internals.md](internals.md) | Architecture for contributors: crates, time, the run loop, peripherals, adding models, determinism, performance |

Things marked **to verify locally** need what the cloud environment this was built in lacks: the vcan kernel
module and `CAP_NET_ADMIN` (SocketCAN), ROS 2 and `titan_bridge`, a Docker daemon, the vhci-hcd module (USB/IP),
real hardware. Some were checked later in a Nix shell with ARM GCC, the micro-ROS library, ROS 2 Humble and GDB
(the C board apps, GDB, the nereus bridge node, `bag_to_calib.py`'s reading of messages); the pages say what, and
what still needs vcan, an agent, nereus or hardware.

## Quick start

**1. Get the sources and toolchain.**

```bash
git submodule update --init                 # external/titan_firmware_rs is private: ATK422/titan_firmware_rs
rustup target add thumbv6m-none-eabi
(cd external/titan_firmware_rs && rustup toolchain install)   # the firmware's pinned compiler (rust-toolchain.toml)
```

**2. Build the firmware and the emulator.**

```bash
(cd external/titan_firmware_rs/firmware && cargo build --workspace --release)
cargo build --release
alias firmware_simulator=$PWD/target/release/firmware_simulator
FW=external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release
```

The images land in `$FW` (`power_board`, `esc_board`, `ivc_board2`, ...). If they are built elsewhere (another
checkout, a worktree), point `TITAN_RS_RELEASE_DIR` there; tests and robot files use it.

**3. Run a board.** The power board image on its board file, next to a scripted `titan_bridge` for its CANmore
client (1), for two seconds of virtual time:

```bash
firmware_simulator run $FW/power_board --board boards/mk2/power_board.toml --mock-bridge 1 --time 2s
```

```
[INFO  firmware_simulator] board mk2/power_board (1 SPI, 2 I2C device(s); inputs: port_battery, rail_3v3, rail_balanced, sht41.humidity_rh, sht41.temperature_c, stbd_battery)
[INFO  firmware_simulator] direct boot: VTOR 0x10000100, entry 0x100001c0
[INFO  firmware_simulator] RTT control block at 0x20000008, decoding defmt
[INFO  firmware_simulator] mock bridge for client 1 attached
[0.132000s] 0.000024 INFO  mk2/power_board v1.0 (release type 2, git 94a107e)
[0.132000s] 0.000089 INFO  Crash counts since power on: CrashCounts { watchdog_timeouts: 0, panics: 0 }
[0.135100s] 0.003181 INFO  CAN controller online as client 1
[0.200301s] 0.068408 INFO  ROS connected
stopped at 2.000000s (TimeLimit); core0 234373475 cycles, pc 0x1000039c <...>; 200.71ms wall
```

**4. See what it does.** Each defmt line shows the virtual time it was read, the firmware's own timestamp, the
level and the message. Add `--candump` to see the CAN frames, `--input stbd_battery=15.1` to change a sensor
value, `--fault 1s@bridge-stop` to take the bridge away, `--trace gpio` for pin changes; a whole robot is
`firmware_simulator robot robots/talos.toml --mock-bridge internal:1`. Without `--board` the CAN controller is
missing and the firmware panics and reboots, as it would on a bare Pico.

**5. Run a test.**

```bash
cargo test --release -p sim_test --test guide         # the walkthrough of testing.md
cargo test --release -p sim_test --test smoke -- --nocapture   # every board image on its board
cargo test --release --workspace                      # everything
```

Tests whose firmware is not built print `skipping: ...` and pass; see [testing.md](testing.md#running-the-tests).
To write one, start from [the walkthrough](testing.md#a-first-test-step-by-step).
