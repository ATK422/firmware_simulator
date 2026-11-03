# firmware_simulator

An RP2040 emulator that runs unmodified [`titan_firmware`](external/titan_firmware) (C, pico-sdk) and
[`titan_firmware_rs`](external/titan_firmware_rs) (Rust, embassy) images for OSU UWRT's robot boards, so firmware
can be run, debugged and tested without hardware: one board or a whole robot in one process, on emulated CAN buses
that can be bridged to the real host tools, with sensors, actuators, faults, the water between acoustic modems
and the vehicle the thrusters move. Runs are deterministic, which makes firmware integration tests possible in
CI. The design and milestones are in [PLAN.md](PLAN.md).

**The guide is in [docs/](docs/README.md).**

| | |
|---|---|
| [Quick start](docs/README.md#quick-start) | Build the firmware, run a board, see its logs, run a test |
| [Running a board](docs/running.md) | `run`: images, boot modes, logs, traces, profiles, waveforms, GDB, flash, USB, Ethernet |
| [Board files](docs/boards.md) | Every device table, overlays, inputs |
| [Robots](docs/robots.md) | Robot files, mock bridges, CAN logs, replay and `can-diff`, `bridges`, vcan and titan_bridge |
| [Acoustics](docs/acoustics.md) | Environments, transducers, IVC links and sweeps |
| [Vehicles](docs/vehicles.md) | Thrusters moving the robot, sensors following it |
| [Testing](docs/testing.md) | `sim_test`, the test suites, CI |
| [Developer tools](docs/tools.md) | Firmware profiler, VCD waveforms, live dashboard, JSON logs |
| [Faults](docs/faults.md) | Fault injection and the firmware findings |
| [Internals](docs/internals.md) | Architecture, adding peripherals and devices, determinism, performance |

## Quick start

```bash
git submodule update --init                          # external/titan_firmware_rs is private (ATK422/titan_firmware_rs)
rustup target add thumbv6m-none-eabi
(cd external/titan_firmware_rs/firmware && cargo build --workspace --release)   # the board images
cargo build --release

FW=external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release
target/release/firmware_simulator run $FW/power_board --board boards/mk2/power_board.toml --mock-bridge 1 --time 2s
target/release/firmware_simulator robot robots/talos.toml --mock-bridge internal:1 --time 2s
cargo test --release --workspace
```

C firmware (the template and the Titan bootloader) needs `git -C external/titan_firmware submodule update --init
--recursive`, `gcc-arm-none-eabi`, `cmake` and `ninja`, then `scripts/build_c_firmware.sh`.

## Layout

| Crate | What it is |
|---|---|
| [`sim_core`](crates/sim_core) | Virtual time, the event scheduler, `Bus`/`Peripheral`/`Cpu` traits, wires, traces, ELF/UF2 images |
| [`cpu_armv6m`](crates/cpu_armv6m) | Cortex-M0+ interpreter: ARMv6-M Thumb, exceptions, NVIC, SysTick, SCB |
| [`rp2040`](crates/rp2040) | The SoC: memory map, peripherals, the two-core `Machine` and its run loop |
| [`sim_devices`](crates/sim_devices) | External chips: CAN bus, MCP2517FD, W5500, sensors, PIO-driven devices, USB host |
| [`sim_acoustics`](crates/sim_acoustics) | Underwater acoustic environments |
| [`sim_physics`](crates/sim_physics) | Vehicle physics |
| [`sim_host`](crates/sim_host) | Sessions, robots, board files, host bridges and tools |
| [`sim_test`](crates/sim_test) | Firmware integration test harness |
| `firmware_simulator` (root) | The command line |

Other directories: `boards/` (board files and their overlays), `robots/`, `environments/`, `vehicles/`,
`test_programs/` (programs the tests run), `scripts/` (board file generator, C firmware build, vcan, pioasm),
`docker/` (robot computer container, unverified).

## Third-party data

* `crates/rp2040/svd/rp2040.svd`: from the pico-sdk, BSD-3-Clause.
* `crates/rp2040/bootrom/b1.bin`: the official RP2040 B1 bootrom, BSD-3-Clause; see
  [its README](crates/rp2040/bootrom/README.md).
