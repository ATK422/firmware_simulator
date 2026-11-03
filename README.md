# firmware_simulator

An RP2040 emulator that runs unmodified [`titan_firmware`](external/titan_firmware) (C, pico-sdk) and
[`titan_firmware_rs`](external/titan_firmware_rs) (Rust, embassy) images for OSU UWRT's robot boards. The design and
milestones are in [PLAN.md](PLAN.md).

## Layout

| Crate | What it is |
|---|---|
| [`sim_core`](crates/sim_core) | Virtual time (picoseconds), keyed event scheduler, `Bus`/`Peripheral`/`Cpu` traits, wires, trace sinks, ELF/UF2 images |
| [`cpu_armv6m`](crates/cpu_armv6m) | Cortex-M0+ interpreter: full ARMv6-M Thumb, exceptions, NVIC, SysTick, SCB, MPU registers, disassembler |
| [`rp2040`](crates/rp2040) | The SoC: memory map, bus fabric, peripherals, the two-core `Machine` and its run loop; register tables generated from `rp2040.svd` |
| [`sim_devices`](crates/sim_devices) | External chip models (from M3) |
| [`sim_acoustics`](crates/sim_acoustics) | Underwater acoustic environments for IVC testing (from M4) |
| [`sim_host`](crates/sim_host) | Host bridges: GDB, RTT/defmt, SocketCAN, UDP, UART ptys (from M1) |
| [`sim_test`](crates/sim_test) | Firmware integration test harness (M7) |
| `firmware_simulator` (root) | The command line tool |

## Usage

```bash
cargo run --release -- run test_programs/blink.elf --time 100us --trace gpio
```

Options: `--time` (virtual time to run, e.g. `500ms`), `--trace instr,exc,gpio,unmodelled,device`, `--trace-file`,
`--quantum` (instructions per core before switching), `--unmodelled first|every|off` (logging of registers the
emulator does not model yet; each is named from the SVD), `--stop-on-bkpt`.

## Tests

```bash
cargo test --workspace
```

`test_programs/` holds hand-written Thumb programs used by the tests; the ELFs are committed, `make -C
test_programs` rebuilds them with `arm-none-eabi-as`.

## Third-party data

* `crates/rp2040/svd/rp2040.svd`: from the pico-sdk, BSD-3-Clause.
* `crates/rp2040/bootrom/b1.bin`: the official RP2040 B1 bootrom, BSD-3-Clause; see
  [its README](crates/rp2040/bootrom/README.md).
