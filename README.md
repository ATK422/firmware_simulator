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
| [`sim_devices`](crates/sim_devices) | External chip models: SPI/I2C/UART/pin-level device traits, CAN bus, MCP2517FD/MCP2518FD, SHT41, MCP3426/8, MS5837, SSD1306, CD74HC4051, BQ40Z80, WS2812, DShot ESC, Dynamixel and Hiwonder servos |
| [`sim_acoustics`](crates/sim_acoustics) | Underwater acoustic environments: water, transducers by spec, receive chains, multipath, noise, sources |
| [`sim_host`](crates/sim_host) | Host bridges: GDB, RTT/defmt, SocketCAN, UDP, UART ptys (from M1) |
| [`sim_test`](crates/sim_test) | Firmware integration test harness (M7) |
| `firmware_simulator` (root) | The command line tool |

## Usage

```bash
cargo run --release -- run external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release/template --time 1s
cargo run --release -- run test_programs/embassy_blinky.elf --time 2s --trace gpio
```

Core 0 boots the image as if the bootrom and boot2 had run ("direct boot"); the official bootrom is mapped at
address 0 for its function table and soft-float routines, and core 1 runs it for real, waiting for the launch
sequence on the SIO FIFO. The bootrom's flash functions are trapped and performed directly on the flash array
(with W25Q16 erase/program times) until the QSPI flash is modelled. UART output is printed per line (or attached to
a pseudo terminal with `--uart0 pty`). defmt logs are decoded from RTT (the `_SEGGER_RTT` control block
and the ELF's `.defmt` table) and printed with the virtual time they were read and the firmware's own timestamp.

Options: `--time` (virtual time to run, e.g. `500ms`), `--trace instr,exc,gpio,unmodelled,device`, `--trace-file`,
`--quantum` (instructions per core before switching), `--uart0/--uart1 stdout|pty|none`, `--unmodelled first|every|off` (logging of registers the
emulator does not model yet; each is named from the SVD), `--stop-on-bkpt`, `--crystal-hz`, `--gdb ADDR`.

### Boards and CAN

`boards/<dir>/<board>.toml` describes a board's external chips and wiring. They are generated from titan_firmware's
board headers by `python3 scripts/gen_boards.py` (`--check` fails if they are out of date); additions go in
`boards/overlays/`. With `--board`, the board's MCP2517FD is attached to its SPI bus and the board sits on an
in-process CAN bus (CAN FD 1M/5M for the internal bus, classic 250k for the external one) with arbitration, frame
timing and acknowledgement (a frame is only acknowledged if another node or endpoint is on the bus).

```bash
# Print the bus candump style, with a scripted titan_bridge stand-in (CANmore + titan_link) for client 1
cargo run --release -- run path/to/template --board boards/mk2/power_board.toml --mock-bridge 1 --candump --time 3s
# Bridge the bus to a SocketCAN interface for titan_bridge, canmore_cli, candump (host tools need --realtime)
sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up
cargo run --release -- run path/to/template --board boards/mk2/power_board.toml --can vcan0 --realtime
```

Several boards on one bus are put together in code with `sim_host::robot::Robot` (see `crates/sim_host/tests/can.rs`).
`RUST_LOG=sim_devices=trace` logs every MCP2517FD SPI transaction.

### Sensors and scripted inputs

Board files also list I2C devices (`[[i2c]]`: `sht41`, `mcp3426`/`mcp3428`, `ms5837_02ba`/`ms5837_30ba`,
`ssd1306`), what drives the ADC inputs (`[[analog]]`: a voltage behind a divider, or a CD74HC4051 mux) and bytes
programmed into flash (`[[flash]]`, e.g. the smart battery's serial number). Their physical values are named
inputs: `--input sht41.temperature_c=31.5`, `--input port_battery=14.2`; tests change them while the board runs
(`Board::apply` returns them). `--screenshot oled.png` saves what an SSD1306 shows.

```bash
cargo run --release -- run path/to/power_board --board boards/mk2/power_board.toml --mock-bridge 1 --input stbd_battery=15.1
```

### PIO and bit-banged devices

The PIO blocks are modelled cycle by cycle (the full instruction set, FIFOs and joins, side-set, autopush/pull,
IRQ flags, EXEC, DMA requests, pin overrides) and advanced lazily, skipping delays and stalled state machines.
Devices on PIO-driven pins watch the pin waveforms and answer on the pins: board files list `[[ws2812]]` strips
(frames are decoded and reported to the `device` trace), `[[dshot_esc]]` ESCs (DShot frames, bidirectional eRPM
replies, KISS telemetry, a motor model), `[[dynamixel]]` and `[[hiwonder]]` servos on half-duplex serial pins, and
`[[soft_i2c]]` devices on bit-banged I2C/SMBus pins (the smart battery's BQ40Z80 gauge).

`test_programs/pio/*.pio` (pico-examples programs and a handshake test) and the ESC board's own `.pio` files are
assembled by pioasm: `scripts/build_pioasm.sh` builds it from titan_firmware's pico-sdk and
`python3 scripts/assemble_pio.py` writes the `.prog` files the tests read.

### Acoustic environments

`environments/*.toml` describe a body of water (open water, a tank or pool, a lake or harbour: boundaries and
their reflections, water temperature and salinity, ambient noise, multipath order) and the sound sources in it
(pingers, tones, chirps, WAV recordings). A board with a `[transducer]` section (specs, and the receive chain to an
ADC input) is placed in one with `--environment` and `--position x,y,z` (metres, z is depth):

```bash
cargo run --release -- run path/to/ivc_board2 --board boards/talos/ivc_board.toml --mock-bridge 8 \
    --environment environments/pool.toml --position 2,6,1
```

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
(`cd external/titan_firmware_rs/firmware && cargo build --workspace --release`); tests that boot C titan_firmware
programs skip unless `scripts/build_c_firmware.sh` has built them into `build/c`. `test_programs/pico` holds
pico-sdk programs equivalent to pico-examples' `hello_uart`, `hello_multicore` and `flash/program`
(`make -C test_programs pico` rebuilds them against titan_firmware's pico-sdk).

## Third-party data

* `crates/rp2040/svd/rp2040.svd`: from the pico-sdk, BSD-3-Clause.
* `crates/rp2040/bootrom/b1.bin`: the official RP2040 B1 bootrom, BSD-3-Clause; see
  [its README](crates/rp2040/bootrom/README.md).
