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
`--quantum` (instructions per core before switching), `--uart0/--uart1 stdout|pty|none`, `--profile FILE`,
`--vcd FILE`, `--dashboard ADDR` (see [Profiling](#profiling), [Waveforms](#waveforms), [Dashboard](#dashboard)), `--unmodelled first|every|off` (logging of registers the
emulator does not model yet; each is named from the SVD), `--stop-on-bkpt`, `--crystal-hz`, `--gdb ADDR`, `--rom-boot`, `--hle-flash`, `--flash-file FILE`,
and for USB `--usb`, `--bootsel`, `--usb-cdc stdout|pty`, `--picoboot-load FILE`, `--load-via picoboot|drive`, `--usbip ADDR` (see [USB](#usb-bootsel-picoboot-uf2-and-cdc)).

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

### Vehicles: thrusters move the robot, sensors see it

A robot file's `[[vehicle]]` puts its boards in a simulated vehicle (`sim_physics`): a rigid body in six degrees
of freedom with its mass and inertia, added mass, linear and quadratic drag per axis, gravity, and buoyancy acting
above the centre of gravity (so it rights itself in roll and pitch). Near the surface only part of the hull is in
the water, so a positively buoyant vehicle floats instead of flying, and a thruster in the air pushes nothing; in a
pool or lake environment it comes to rest on the bottom. A uniform current can be given. It is integrated with RK4
at 1 ms on the robot's virtual clock after every slice, so runs stay deterministic.

Each thruster (position and direction on the hull, thrust `k rpm |rpm|` with separate forward and reverse curves)
is wired to a DShot ESC of a board and spins at that ESC's motor-model RPM (an `inverted` thruster, with a
counter-rotating propeller, pushes ahead when its motor turns backwards, as the Talos ESC boards' invert mask
expects), so the thrust comes from the firmware's actual DShot frames. Pressure sensor inputs (`depth.pressure_pa` of an MS5837) get the absolute pressure at their
port on the hull, and transducers carried by the vehicle follow it through the acoustic medium (what other boards
hear moves with it, Doppler included).

```toml
[[vehicle]]
name = "talos"
file = "../vehicles/talos.toml"     # body and thruster geometry
position_m = [5.0, 6.0, 0.0]        # north, east, depth of the centre of gravity
floating = true                     # start at the depth where it floats
thrusters = [{ thruster = "heave_fwd_stbd", board = "esc1", esc = "esc1" }]   # ... one per thruster

[[vehicle.sensor]]
board = "camera_cage"
input = "depth.pressure_pa"
mount_m = [0.30, 0.0, 0.10]         # body frame: forward, starboard, down

# [[vehicle.transducer]]            # in an acoustic environment
# board = "ivc"
# mount_m = [-0.2, 0.0, 0.15]
```

`vehicles/talos.toml` and `vehicles/puddles.toml` hold assumed parameters (no CAD or tank data yet: a 32 kg open
frame for Talos, BlueROV2 Heavy numbers from the literature for Puddles, T200 thrust curves); `robots/talos.toml`
and `robots/puddles.toml` wire them up. `--vehicle-csv` writes the vehicle's state every 10 ms, and the robot command
can attach a mock bridge or a UDP relay to an Ethernet board:

```bash
cargo run --release -- robot robots/puddles.toml --eth-mock-bridge backplane --time 10s --vehicle-csv puddles.csv
cargo run --release -- robot robots/puddles.toml --udp backplane:127.0.0.1:2203=127.0.0.1:2204 --realtime
```

The physics is in its own crate, [`sim_physics`](crates/sim_physics) (no board knowledge, like `sim_acoustics`), with
unit tests for each behaviour (neutral buoyancy, rising and sinking at the drag-limited speed, thrust, turning,
righting moments, the surface and the bottom). With the firmware in the loop, `sim_test/tests/vehicle.rs` dives
Puddles on UDP thruster commands and closes a depth controller through the backplane's own depth readings, and
`tests/vehicle_talos.rs` dives and turns Talos on its two ESC boards' commands while the camera cage reports the
depth.

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

### USB: BOOTSEL, PICOBOOT, UF2 and CDC

The USB controller is modelled in device mode (USBCTRL_REGS and its DPRAM: endpoint and buffer control, single
and double buffering, data toggles, STALL/NAK, setup packets, bus reset, SOF, the pull-up, interrupts; it needs
`clk_usb` at 48 MHz). `--usb` plugs the board's port into a host controller model (`sim_devices::usb::host`):
it debounces the attach, resets the bus, enumerates the device (SET_ADDRESS, descriptors, SET_CONFIGURATION) and
runs control, bulk and interrupt transfers transaction by transaction on the virtual clock, with SOFs every 1 ms
frame and full speed bus time (about 1.2 MB/s for a bulk pipe). `--trace device` shows what it does.

With `--bootsel` the board powers on with BOOTSEL held and the real bootrom starts its USB boot mode: it
enumerates as "RP2 Boot" with the PICOBOOT interface and the RPI-RP2 drive. Firmware that calls
`reset_usb_boot` (pico-sdk's reset interface, its 1200 baud trick, embassy's `reset_to_usb_boot`) goes there too,
through the watchdog and the bootrom, even when it was started with direct boot. `sim_host::usb` has the host
tools: `Picoboot` speaks PICOBOOT as `picotool` does (exclusive access, exit XIP, erase, write, read, reboot, and
`load` = `picotool load -v -x`), `MscDrive` speaks bulk-only mass storage and SCSI and copies a UF2 file to the
drive, `reboot_via_reset_interface` is `picotool reboot -f [-u]`. The flash is written by the bootrom's own code
through the SSI and the flash chip model.

```bash
# picotool load -v -x: blank flash, BOOTSEL, flash over PICOBOOT, reboot into the image and run it for 2 s
cargo run --release -- picoboot load test_programs/embassy_blinky.elf --time 2s
# The same by copying the UF2 to the RPI-RP2 drive, keeping the flash in a file afterwards
cargo run --release -- picoboot load app.uf2 --load-via drive --flash-file flash.bin
# Stay in BOOTSEL and watch the host enumerate it
cargo run --release -- run --bootsel --time 300ms --trace device
# pico-sdk stdio over USB (TinyUSB CDC ACM): the host opens the port (DTR/RTS, 115200 8N1) and prints its lines,
# or puts it on a pseudo terminal for picocom/screen
cargo run --release -- run test_programs/pico/hello_usb.elf --usb-cdc stdout --time 1s
cargo run --release -- run test_programs/pico/hello_usb.elf --usb-cdc pty --realtime --time 600s
```

`--usbip 127.0.0.1:3240` exports the device over USB/IP, so the real host's tools can use it after `sudo usbip
attach -r 127.0.0.1 -b 1-1` (`picotool`, `/dev/ttyACM*`, the RPI-RP2 drive); use it with `--realtime`, and attach
again after the device reboots (the server ends the session when the device leaves the bus).

`test_programs/pico/hello_usb` (pico-sdk stdio_usb) and `test_programs/c_dual_usb` (titan_firmware's
`titan_dual_stdio_usb`, two CDC ports) are the device-side test programs (`crates/sim_host/tests/usb.rs`). Not
modelled: host mode of the controller and isochronous endpoints. The host can suspend and resume the port
(`HostController::suspend`/`resume`), firmware can wake it (SIE_CTRL.RESUME), and `System::detach_usb_host`
pulls the cable.

### Debugging

`--gdb 127.0.0.1:3333` waits for GDB before running. Each core is a thread; breakpoints, stepping, Ctrl-C, memory
and register access, and `monitor reset` / `monitor time` work. Virtual time stops while GDB has the target halted.

```bash
gdb-multiarch -ex "target remote :3333" -ex "break main" -ex continue path/to/firmware.elf
```

### Profiling

`--profile FILE` samples both cores every `--profile-interval` of virtual time (10 µs by default, jittered by ±50 %
so periodic firmware work does not alias with the samples) and writes a table of functions by time per core to
FILE and folded stacks (`core0;Reset;main;...;leaf 123`) next to it as `FILE.folded`, for `flamegraph.pl`,
`inferno-flamegraph` or speedscope. `robot --profile-dir DIR` writes `DIR/BOARD.txt` and `DIR/BOARD.folded` for
every board.

```bash
cargo run --release -- run path/to/esc_board --board boards/mk2/esc_board.toml --mock-bridge 2 --time 3s --profile esc.txt
inferno-flamegraph < esc.folded > esc.svg     # or: flamegraph.pl esc.folded > esc.svg
cargo run --release -- robot robots/talos.toml --mock-bridge internal:1 --time 3s --profile-dir prof/
```

The table gives, per core, the share of samples running and sleeping (`WFI`/`WFE`), the time in each exception
handler, and for each function its self time (`self%`, of all samples; `of-run%`, of the samples the core was
running) and its total time with callees (`total%`). Sleeping samples are left out of the functions' times; in
the folded stacks they end in `[sleep]` under the code that went to sleep. Stacks are walked as a debug probe
would: through the ELF's `.debug_frame`, through the prologue of code without it (compiler-builtins assembly, the
bootrom) and through exception frames into the interrupted code, checking every return address. Names are
demangled (Rust's legacy and v0 schemes); bootrom code is named from the ROM's function and soft float/double
tables (`rom:fmul`, `rom:memcpy`). The profiler only splits the run at the sample times, so a program runs the
same (the two cores' interleaving can shift, as with a different `--quantum`). At 10 µs it adds a few percent to a
busy board (the ESC board's core 1); an idle board, which otherwise fast-forwards between events, takes about twice
as long.

### Waveforms

`--vcd FILE` writes the GPIO levels as a VCD file for GTKWave, Surfer or PulseView, in 1 ns steps, with whatever
drives each pin: SIO, PWM, PIO (DShot, WS2812, PIO UARTs), devices and injected faults. `--vcd-pins` picks the pins:
numbers, ranges and names from the board file's `[pins]` table (`--vcd-pins 0-7,25,status_ledr`); the wires are
named after the board's pins (`gpio6_esc1_pwm`). `--vcd-events` adds string variables with the decoded traffic:
defmt log lines, UART lines, CAN frames (candump notation) and device reports such as WS2812 frames (GTKWave and
Surfer show them; PulseView reads only the pins). For a robot each board is a scope, and each bus gets one with its
frames.

```bash
cargo run --release -- run path/to/esc_board --board boards/mk2/esc_board.toml --mock-bridge 2 --time 1s \
    --vcd esc.vcd --vcd-pins esc1_pwm,esc1_telem,status_ledg --vcd-events
gtkwave esc.vcd
cargo run --release -- robot robots/talos.toml --mock-bridge internal:1 --time 2s --vcd talos.vcd --vcd-pins 0-15
```

### Dashboard

`--dashboard 127.0.0.1:8080` (for `run` and `robot`) serves a live page at that address while the boards run: per
board the cores and where they are, the log and UART lines, the firmware status a mock bridge decoded (titan_link
`FirmwareStatus`, with the board's own fault names), the GPIO levels and functions, LEDs and status strip colours,
ESC RPMs and the physical inputs; per bus the frame rate, load, errors and the latest frames; the vehicle's pose,
velocity and thruster RPMs; virtual time and the emulation speed. Its controls inject a fault (the `--fault`
syntax), set an input and pause virtual time. It works with and without `--realtime`; `--dashboard-hold` keeps
serving the final state after the run. The page is self-contained (no external scripts) and polls
`GET /api/state`; scripts can use the same JSON API: `POST /api/fault` with `{"board": "esc0", "fault":
"can-stuck"}`, `POST /api/input` with `{"board": "camera_cage", "name": "depth.pressure_pa", "value": 120000}`,
`POST /api/pause` with `{"paused": true}`; each answers once the run loop has applied it. There is no
authentication: bind it to a loopback address.

```bash
cargo run --release -- robot robots/talos.toml --mock-bridge internal:1 --realtime --time 600s --dashboard 127.0.0.1:8080
curl -s -X POST -d '{"board": "camera_cage", "fault": "i2c-absent:0x76"}' http://127.0.0.1:8080/api/fault
```

### Robot traces and structured logs

`robot` takes `--trace` too, with `--trace-board esc0,camera_cage` to trace only some boards (all by default) and
`--trace-file`; each line starts with the board's name, and lines of different boards are in time order within
each 100 µs slice. `--log-json FILE` (`run` and `robot`) writes every defmt log line, UART and USB serial line,
CAN frame and injected fault as one JSON object per line, with its board or bus and its virtual time (`t` in
seconds, `t_ns` exact), and an `end` record with why the run stopped, for scripts and CI checks:

```bash
cargo run --release -- robot robots/talos.toml --mock-bridge internal:1 --time 5s --log-json talos.jsonl \
    --trace gpio --trace-board led --trace-file led.trace
jq -c 'select(.kind == "log" and .level == "error")' talos.jsonl
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
