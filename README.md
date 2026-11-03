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
cargo run --release -- run external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release/template --time 1s
cargo run --release -- run test_programs/embassy_blinky.elf --time 2s --trace gpio
```

The image boots as if the bootrom and boot2 had run ("direct boot"); the official bootrom is mapped at address 0
for its function table and soft-float routines. defmt logs are decoded from RTT (the `_SEGGER_RTT` control block
and the ELF's `.defmt` table) and printed with the virtual time they were read and the firmware's own timestamp.

Options: `--time` (virtual time to run, e.g. `500ms`), `--trace instr,exc,gpio,unmodelled,device`, `--trace-file`,
`--quantum` (instructions per core before switching), `--unmodelled first|every|off` (logging of registers the
emulator does not model yet; each is named from the SVD), `--stop-on-bkpt`, `--crystal-hz`, `--gdb ADDR`.

### Debugging

`--gdb 127.0.0.1:3333` waits for GDB before running. Each core is a thread; breakpoints, stepping, Ctrl-C, memory
and register access, and `monitor reset` / `monitor time` work. Virtual time stops while GDB has the target halted.

```bash
gdb-multiarch -ex "target remote :3333" -ex "break main" -ex continue path/to/firmware.elf
```

## Tests

```bash
cargo test --workspace
```

`test_programs/` holds hand-written Thumb programs and a small embassy firmware used by the tests; the ELFs are
committed, `make -C test_programs` rebuilds them (`arm-none-eabi-as`, and the `thumbv6m-none-eabi` Rust target).
Tests that boot titan_firmware_rs images skip themselves unless the firmware has been built
(`cd external/titan_firmware_rs/firmware && cargo build --workspace --release`).

## Third-party data

* `crates/rp2040/svd/rp2040.svd`: from the pico-sdk, BSD-3-Clause.
* `crates/rp2040/bootrom/b1.bin`: the official RP2040 B1 bootrom, BSD-3-Clause; see
  [its README](crates/rp2040/bootrom/README.md).
