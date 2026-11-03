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
| [`sim_devices`](crates/sim_devices) | External chip models: SPI/I2C/UART/pin-level device traits, CAN bus, MCP2517FD/MCP2518FD, W5500, SHT41, MCP3426/8, MS5837, SSD1306, CD74HC4051, BQ40Z80, DRV8243, WS2812, DShot ESC, Dynamixel and Hiwonder servos |
| [`sim_acoustics`](crates/sim_acoustics) | Underwater acoustic environments: water, transducers by spec, receive chains, multipath, noise, sources, and the medium boards share (projectors driven by pins, moving vehicles) |
| [`sim_host`](crates/sim_host) | Sessions and robots, board files, host bridges: GDB, RTT/defmt, SocketCAN, mock bridges, Ethernet/UDP, UART ptys, the C bootloader upload |
| [`sim_test`](crates/sim_test) | Firmware integration test harness: benches, robots, snapshots, sweeps |
| `firmware_simulator` (root) | The command line tool |

## Usage

```bash
cargo run --release -- run external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release/template --time 1s
cargo run --release -- run test_programs/embassy_blinky.elf --time 2s --trace gpio
```

Core 0 boots the image as if the bootrom and boot2 had run ("direct boot"); the official bootrom is mapped at
address 0 for its function table and soft-float routines, and core 1 runs it for real, waiting for the launch
sequence on the SIO FIFO. With `--rom-boot` both cores start in the bootrom as after power-on: it checks BOOTSEL
(the QSPI chip select, read through SIO), loads boot2 from flash over the SSI, and boot2 puts the W25Q16 model in
continuous quad read mode before jumping to the image. Either way the flash is a QSPI chip behind the SSI: XIP
reads fault while the SSI is not set up for them, and the bootrom's flash routines (`flash_range_erase`,
`flash_range_program`, ...) run for real against the chip's command set, with its erase and program times.
`--hle-flash` traps those routines and performs them directly instead (faster, less faithful). `--flash-file
FILE` keeps the flash in a file: loaded before the image is written over it, saved at the end, so what the firmware
writes (or what the C Titan bootloader flashes) persists between runs.
UART output is printed per line (or attached to
a pseudo terminal with `--uart0 pty`). defmt logs are decoded from RTT (the `_SEGGER_RTT` control block
and the ELF's `.defmt` table) and printed with the virtual time they were read and the firmware's own timestamp.

Options: `--time` (virtual time to run, e.g. `500ms`), `--trace instr,exc,gpio,unmodelled,device`, `--trace-file`,
`--quantum` (instructions per core before switching), `--uart0/--uart1 stdout|pty|none`, `--unmodelled first|every|off` (logging of registers the
emulator does not model yet; each is named from the SVD), `--stop-on-bkpt`, `--crystal-hz`, `--gdb ADDR`, `--rom-boot`, `--hle-flash`, `--flash-file FILE`.

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

### Robots: many boards, buses and the water between them

A robot file (`robots/*.toml`) lists boards with their board file and image, the CAN buses they share, when each is
switched on, per-board straps and, optionally, an acoustic environment with each board's position or trajectory
(and a transducer replacing the board file's). Several vehicles go in one file as several buses. All boards run in
one process in lockstep 100 µs slices; each bus can get mock bridges or a SocketCAN interface:

```bash
cargo run --release -- robot robots/talos.toml --mock-bridge internal:1 --candump
cargo run --release -- robot robots/talos.toml --can internal=vcan0 --can external=vcan1 --realtime
cargo run --release -- robot robots/ivc2_pool.toml --mock-bridge talos:8 --mock-bridge surface:8 --time 15s
```

In an environment, each board's transducer drive pin (`[transducer.drive]`) feeds a projector: the pin's level
changes, through the driver's supply and the transducer's TVR, become the pressure it radiates, and every other
board's hydrophone hears it over the environment's paths (delay, spreading, absorption, reflections, noise). A
vehicle on a trajectory moves its transducer, which shifts what the others hear (Doppler). `robots/ivc_pool.toml`
and `robots/ivc2_pool.toml` put two IVCBoards or two IVCBoard2s, one per vehicle, in the pool.

### Puddles: Ethernet

The Puddles backplane's W5500 is modelled in MACRAW mode. `--mock-bridge` on a board with Ethernet attaches a
scripted bridge on the robot computer's address (192.168.1.22) that speaks the Puddles UDP transport; `--udp
LOCAL=PEER` relays the board's datagrams through a host socket instead, for `titan_bridge`:

```bash
cargo run --release -- run path/to/puddles_backplane --board boards/mk2/puddles_backplane.toml --mock-bridge 1
cargo run --release -- run path/to/puddles_backplane --board boards/mk2/puddles_backplane.toml \
    --udp 127.0.0.1:2203=127.0.0.1:2204 --realtime
```

### Flash, the bootloader and resets

A watchdog reset (timeout or forced) restarts every block except the watchdog's scratch registers, the flash and
SRAM, and goes back through the bootrom; AIRCR.SYSRESETREQ restarts only the core that asked. The library also
has `Machine::run_pin_reset` (scratch registers cleared, SRAM kept) and `Machine::power_cycle` (SRAM comes up with
arbitrary contents), with `VREG_AND_CHIP_RESET.CHIP_RESET` reporting which one happened. `Machine::inject_call`
makes a core call a function, e.g. the firmware's panic path for fault injection.

`sim_host::upload` speaks the C Titan bootloader's protocol (CANmore register-mapped pages, CRC-32/MPEG-2) the
way `upload_tool` does, over the in-process CAN bus: it catches the bootloader in its boot delay, erases and
programs the application, verifies it and reboots into it. The tests build the C bootloader with a small
application (`test_programs/c_bl_app`, two versions) to exercise it.

### Fault injection

`sim_host::fault::Fault` lists what can go wrong around a board, each modelled as the hardware fails rather than
by poking firmware state; `--fault TIME@FAULT` (`TIME@BOARD:FAULT` for `robot`) injects one from the command line,
and `sim_test`'s `Bench::inject`/`inject_at` (`RobotBench::inject`/`inject_at` with a board name) from a test.

* **CAN bus**: bit errors at a rate or on the next frames (`can-error-rate:0.3`, `can-corrupt:5`) become error
  frames: the frame is cut off where the error struck, followed by an error flag, delimiter and intermission, and
  retransmitted. The MCP2517FD/MCP2518FD model keeps ISO 11898-1 fault confinement (TEC/REC, error warning and
  passive, bus-off above 255 with automatic recovery after 128 x 11 recessive bits) and shows it as the datasheet
  says: C1TREC, C1BDIAG0/1 (error counts by phase, error kinds, TXBOERR), CERRIF and IVMIF. A stuck-dominant bus
  (`can-stuck`, `can-release`) lets nothing start and drives every node's error counters up; an unplugged node
  (`can-unplug`, `can-plug`) sends alone without acknowledgement (error passive, never bus-off) and hears nothing;
  a flood (`can-flood:ID:LOAD`) competes in arbitration like a babbling node. The mock bridge's interface queue
  holds 10 frames (Linux's default `txqueuelen`), so pings are refused rather than piling up while the bus is
  unusable.
* **Sensors**: an I2C device that does not answer (`i2c-absent:0x76`), answers intermittently, freezes
  (`i2c-stuck`, replaying its last answer to each command), flips bits (`i2c-glitch:0x76:0.02`) or stretches the
  clock (`i2c-stretch:0x76:30ms`); `i2c-ok` repairs it. An ADC input shorted (`adc-short:0:0`), open (floating),
  noisy or spiking. A GPIO driven from outside (`gpio:1:low`: the power board's physical kill switch).
* **Power and the chip**: a brownout (`brownout:50ms`: the RP2040 and the chips on its 3.3 V rail lose power and the
  RP2040 comes back through a power-on reset), a hung task (`hang:0`, or `hang-masked:0` with interrupts masked,
  which starves the safety task until the watchdog fires), a panic, a watchdog timeout.
* **Links**: titan_bridge stopping and starting (`bridge-stop`, `bridge-start`), the `state/kill` it forwards
  (`kill:on`, `kill:off`, `kill:none` to stop forwarding), the Puddles Ethernet cable (`eth-unplug`, `eth-plug`) and
  packet loss on it (`packet-loss:0.3:0.3`).

```bash
cargo run --release -- run path/to/power_board --board boards/mk2/power_board.toml --mock-bridge 1 \
    --fault 1s@can-stuck --fault 3s@can-release --time 4s --candump
```

`crates/sim_test/tests/faults.rs` shows how the titan_firmware_rs safety system reacts: losing the bridge or the
kill switch kills the vehicle and the ESC board's thrusters go neutral, a failing depth sensor raises
`FAULT_DEPTH_ERROR`, a hung task trips the watchdog and leaves a crash record, the board rides out bit errors and
bus-off, Puddles drops ROS with its cable out and reconnects. Its `firmware_finding_*` tests pin down behaviour
that looks wrong in the firmware.

### Debugging

`--gdb 127.0.0.1:3333` waits for GDB before running. Each core is a thread; breakpoints, stepping, Ctrl-C, memory
and register access, and `monitor reset` / `monitor time` work. Virtual time stops while GDB has the target halted.

```bash
gdb-multiarch -ex "target remote :3333" -ex "break main" -ex continue path/to/firmware.elf
```

## Tests

```bash
cargo test --release --workspace
```

`crates/sim_test` is the harness for firmware tests: `Bench` boots one image on its board next to a mock bridge,
sets inputs, injects faults (see [Fault injection](#fault-injection)) and reads what the bridge received;
`RobotBench` does the same for a robot file; both snapshot in a few milliseconds, and `sweep` runs a scenario from
a snapshot over a set of parameters in parallel. Its tests are the smoke test of every board image
(`tests/smoke.rs`), the whole of Talos on its buses (`tests/robot.rs`), Puddles over UDP (`tests/puddles.rs`), the
H-bridge and harness checks (`tests/harness.rs`), the fault injection tests (`tests/faults.rs`) and the IVC range
and noise sweeps (`tests/ivc_sweep.rs`, run with `--nocapture` to see the tables). CI (`.github/workflows/ci.yml`)
builds titan_firmware_rs and runs all of them.

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
