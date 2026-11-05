# Running a board: `firmware_simulator run`

`run` boots one firmware image on one emulated RP2040, optionally with a board file's external chips, a CAN bus,
an Ethernet cable, a USB host or an acoustic environment around it, and runs it for a stretch of virtual time.

```bash
FW=external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release
firmware_simulator run $FW/power_board --board boards/mk2/power_board.toml --mock-bridge 1 --time 2s
```

(`firmware_simulator` is `target/release/firmware_simulator` after `cargo build --release`, or
`cargo run --release --` in its place.)

- [Images](#images)
- [What it prints](#what-it-prints)
- [Options](#options)
- [Boot modes and the flash](#boot-modes-and-the-flash)
- [Board files, inputs and CAN](#board-files-inputs-and-can)
- [Traces](#traces)
- [Profiles, waveforms, dashboard and JSON logs](#profiles-waveforms-dashboard-and-json-logs)
- [Debugging with GDB](#debugging-with-gdb)
- [Reverse debugging and rewinding](#reverse-debugging-and-rewinding)
- [Debugging in VS Code](#debugging-in-vs-code)
- [USB: BOOTSEL, PICOBOOT, UF2 and CDC](#usb-bootsel-picoboot-uf2-and-cdc)
- [Several micro-ROS boards at once](#several-micro-ros-boards-at-once)
- [Puddles: Ethernet](#puddles-ethernet)
- [Exit status](#exit-status)

## Images

Any RP2040 image works unmodified: ELF (what `cargo build` and pico-sdk's CMake produce) or UF2. An ELF gives the
emulator more to work with: its symbols name the PCs in the final report, its `.defmt` section decodes defmt logs,
and its `_SEGGER_RTT` symbol is where the host finds the RTT control block (a UF2 has no symbols, so no RTT
logs).

| Images | Where they come from |
|---|---|
| titan_firmware_rs boards (`power_board`, `esc_board`, `ivc_board2`, `puddles_backplane`, `template`, ...) | `cd external/titan_firmware_rs/firmware && cargo build --workspace --release`; they land in `external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release/` |
| C titan_firmware (`basic_template`, the Titan bootloader, a small OTA app) | `scripts/build_c_firmware.sh` (needs `gcc-arm-none-eabi`, `cmake`, `ninja` and titan_firmware's pico-sdk submodule) into `build/c/` |
| Hand-written test programs (`test_programs/*.elf`, `embassy_blinky.elf`, `pico/*.elf`, `c_dual_usb`, `c_ivc_modem`, `c_ivc_pinger`) | committed; `make -C test_programs` (Thumb and embassy), `make -C test_programs pico`, `make -C test_programs c_dual_usb`, `make -C test_programs c_ivc_modem` and `c_ivc_pinger` (the C IVCBoard2 modem and its pinger mode from titan_firmware's `blake/ivc`) rebuild them |

The C titan_firmware board applications need titan_firmware's prebuilt micro-ROS library (built in Docker), so the
script does not build them; the C programs the tests run are the template, the bootloader and its test app.

## What it prints

```
[INFO  firmware_simulator] board mk2/power_board (1 SPI, 2 I2C device(s); inputs: port_battery, rail_3v3, rail_balanced, sht41.humidity_rh, sht41.temperature_c, stbd_battery)
[INFO  firmware_simulator] direct boot: VTOR 0x10000100, entry 0x100001c0
[INFO  firmware_simulator] RTT control block at 0x20000008, decoding defmt
[INFO  firmware_simulator] mock bridge for client 1 attached
[0.132000s] 0.000024 INFO  mk2/power_board v1.0 (release type 2, git 94a107e)
[0.132000s] 0.000089 INFO  Crash counts since power on: CrashCounts { watchdog_timeouts: 0, panics: 0 }
[0.135100s] 0.003181 INFO  CAN controller online as client 1
[0.200301s] 0.068408 INFO  ROS connected
stopped at 2.000000s (TimeLimit); core0 234373475 cycles, pc 0x1000039c <..._11power_board18___cortex_m_rt_main0EB1f_+0x94>; 200.71ms wall
core1 249573727 cycles, pc 0x00000176, Some(Wfe)
```

* `[INFO  crate]` lines on stderr are the emulator's own log (`RUST_LOG` filters it, e.g.
  `RUST_LOG=sim_devices=trace` logs every MCP2517FD SPI transaction; the default is `info`).
* `[0.132000s] 0.000025 INFO  ...` is a defmt line: the virtual time the host read it from RTT (polled every
  100 µs), the firmware's own `defmt::timestamp!`, the level and the message. defmt is decoded from RTT channel 0
  with the ELF's `.defmt` table; RTT text that is not defmt is printed at the end as `[rtt0] ...`.
* `[0.006337s] uart0: AB Hello, UART!` is a complete line from a UART (pico-sdk stdio, the C logger).
* The last lines give the stop reason, both cores' cycle counts and PCs (with symbols) and the wall time taken,
  then a table of every unmodelled register the firmware touched (see `--unmodelled`), and, for a board in an
  acoustic environment whose hydrophone signal clipped at the receive chain's rails, how much of it did
  ([acoustics.md](acoustics.md#what-the-pinger-amplitude-does-with-distance-and-why)).

Virtual time starts at power-on; the firmware's own timestamps start when its timer does, so the two columns
differ by roughly the boot time.

## Options

| Option | Default | What it does |
|---|---|---|
| `IMAGE` | | ELF or UF2. Optional with `--bootsel` (the flash starts blank) |
| `-t, --time T` | `1s` | Virtual time to run: `500ms`, `2s`, `250us`, `100ns`, or a bare number of seconds |
| `--board FILE` | | Board file: attaches its chips, inputs and CAN bus ([boards.md](boards.md)) |
| `--input NAME=VALUE` | | Sets a physical input of the board, e.g. `sht41.temperature_c=31.5` (repeatable; needs `--board`) |
| `--mock-bridge CLIENT` | | Scripted `titan_bridge` for this CANmore client on the board's CAN bus, or on its Ethernet cable for Puddles ([below](#board-files-inputs-and-can)) |
| `--candump` | | Prints every frame on the CAN bus, candump style, with its virtual time |
| `--can IFACE` | | Bridges the CAN bus to a SocketCAN interface (`vcan0`); use with `--realtime` |
| `--candump-log FILE`, `--replay FILE`, `--replay-filter`, `--replay-offset`, `--replay-iface IFACE`, `--cannelloni LOCAL,PEER` | | CAN logs, replay and UDP tunnels ([robots.md](robots.md#can-logs-recording-replaying-comparing)) |
| `--udp LOCAL=PEER` | | Puddles: relays the board's UDP through a host socket ([below](#puddles-ethernet)) |
| `--fault TIME@FAULT` | | Injects a fault at a virtual time, e.g. `1s@can-stuck` (repeatable; [faults.md](faults.md)) |
| `--environment FILE` | | Puts the board's `[transducer]` in an acoustic environment ([acoustics.md](acoustics.md)) |
| `--position X,Y,Z` | `0,0,1` | Where the transducer is in the environment, metres (z is depth) |
| `--screenshot FILE` | | Writes what the board's SSD1306 shows to a PNG at the end (and prints it as text) |
| `--realtime` | off | Paces virtual time to the wall clock (needed with host tools); warns when the emulation falls behind ([below](#options)) |
| `--uart0`, `--uart1` | `stdout` | `stdout` (lines with their virtual time), `pty` (a pseudo terminal, input too) or `none` |
| `--rom-boot` | off | Boots through the bootrom and boot2 instead of directly ([below](#boot-modes-and-the-flash)) |
| `--flash-file FILE` | | Keeps the flash in a file between runs |
| `--hle-flash` | off | Traps the bootrom's flash functions instead of running them against the flash chip model |
| `--flash-size BYTES` | the board file's `flash_size`, else 2 MiB | Size of the QSPI flash chip |
| `--crystal-hz HZ` | 12 MHz | XOSC frequency (a board file's `crystal_hz` replaces it) |
| `--bootsel`, `--usb`, `--usb-cdc stdout\|pty\|pty:PATH`, `--usb-cdc2 pty\|pty:PATH`, `--picoboot-load FILE`, `--load-via picoboot\|drive`, `--usbip ADDR` | | USB ([below](#usb-bootsel-picoboot-uf2-and-cdc)) |
| `--trace KINDS` | | Comma separated: `instr`, `exc`, `gpio`, `unmodelled`, `device` ([below](#traces)) |
| `--trace-file FILE` | stderr | Where the trace goes |
| `--profile FILE`, `--profile-interval T` | `10us` | Profiles the firmware: functions by time per core to `FILE`, folded stacks next to it ([below](#profiles-waveforms-dashboard-and-json-logs)) |
| `--vcd FILE`, `--vcd-pins SPEC`, `--vcd-events` | all pins | Writes a VCD waveform of the GPIOs, optionally with decoded traffic |
| `--dashboard ADDR`, `--dashboard-hold`, `--dashboard-pause-at T` | | Serves a live dashboard with fault, input and pause controls |
| `--log-json FILE` | | Writes every log line, UART and USB line, CAN frame and fault as JSON lines |
| `--unmodelled MODE` | `summary` | Logging of registers the emulator does not model: `summary` (one note at the first access, the table at the end), `first` (a warning at the first access to each, and the table), `every` (a warning at every access) or `off` (nothing) |
| `--quantum N` | 16 | Instructions a core runs before the other gets a turn (1 for lockstep debugging) |
| `--stop-on-bkpt` | off | A `BKPT` stops the run instead of escalating to HardFault |
| `--stop-on-lockup` | off | A lockup stops the run, instead of the board running on (locked up) until the watchdog resets it ([faults.md](faults.md#what-the-models-do)) |
| `--gdb ADDR` | | Waits for GDB on this address before running ([below](#debugging-with-gdb)) |
| `--checkpoint-every T` | `10ms` with `--gdb` or `--dashboard`, else off | Records the run for time travel (reverse debugging, rewinding) with a checkpoint every `T` of virtual time; `0` turns it off ([below](#reverse-debugging-and-rewinding)) |
| `--checkpoint-memory MB` | 512 | Memory the checkpoints may take; older ones are thinned out beyond it |
| `--coverage FILE` | | Collects firmware code coverage and writes it as an lcov file at the end, with a table per crate ([tools.md](tools.md#code-coverage)) |

`firmware_simulator run --help` has the same list.

**Virtual time and the wall clock.** Without `--realtime` the emulator runs as fast as it can, which is not
necessarily faster than real time: a board that mostly sleeps runs far ahead of the wall clock (the template about
60x), a busy one can fall behind it (the ESC board runs at about 1.1x on its own, Talos as a whole at about 0.4x
on one core, a C image that is busy in its main loop may run below 1x). A host tool talking to the board through a
pty, socket or vcan (the micro-ROS agent, `titan_bridge`, `picocom`) then sees the firmware's time run faster or
slower than its own, so give its timeouts plenty of room, several times what hardware needs, or use `--realtime`.
With `--realtime` virtual time never runs ahead of the wall clock; when the emulation cannot keep up, `run` and
`robot` warn, at most every 5 s, by how much it is behind (`real-time mode: emulation is 1.23s behind the wall
clock`), and replies from host tools then reach the firmware late by about that much.

## Boot modes and the flash

The flash is always a QSPI chip (a W25Q16JV model with its command set and its erase and program times) behind
the XIP SSI. The official RP2040 B1 bootrom is mapped at address 0 in every mode; its function table, soft
float/double routines and flash functions run for real.

| Mode | How | What happens |
|---|---|---|
| Direct boot (default) | | Core 0 starts at the image's vector table as if the bootrom and boot2 had run (VTOR `0x10000100`, SP and PC from the table, XIP set up). Core 1 runs the bootrom for real and waits for the launch sequence on the SIO FIFO |
| ROM boot | `--rom-boot` | Both cores start in the bootrom as after power-on: it checks BOOTSEL (the QSPI chip select, read through SIO), loads boot2 from flash over the SSI, and boot2 puts the flash in continuous quad read mode before jumping to the image. A blank flash falls back to USB boot |
| BOOTSEL | `--bootsel` | Powers on with BOOTSEL held: the bootrom's USB boot mode (implies `--rom-boot` and `--usb`) |

XIP reads fault while the SSI is not set up for them, as on hardware. `--hle-flash` traps `flash_range_erase`,
`flash_range_program` and the other ROM flash entry points and carries them out directly (faster, less
faithful), leaving the SSI, the QSPI pins and the chip as the real functions do, so code that talks to the chip
afterwards (pico-sdk's `flash_do_cmd` behind `pico_unique_id`, boot2) keeps working
(`crates/sim_host/tests/hle.rs`).

`--flash-file FILE` loads the flash from `FILE` (if it exists) before the image is written over it and saves it
at the end (also after a lockup, and when Ctrl-C or SIGTERM ends the run: see [Exit status](#exit-status)), so what the firmware writes (a flash recorder, crash data, or an application the C Titan bootloader
flashed) persists between runs. Without an image (`--bootsel`) the flash is the file's contents or blank.
`run` needs an image otherwise, so there is no way to boot a flash file's contents alone: pass the image again
(it is written over the same place) or reflash it with `picoboot load`.

The C Titan bootloader can be reflashed over CAN the way `upload_tool` does it: `sim_host::upload` speaks its
protocol (CANmore register-mapped pages, CRC-32/MPEG-2) over the in-process bus, catching the bootloader in its
boot delay (`crates/sim_host/tests/flash.rs`, with a small application in two versions from `test_programs/c_bl_app`).

Resets behave as on the chip: a watchdog reset (timeout or forced) restarts every block except the watchdog's
scratch registers, the flash and SRAM, and goes back through the bootrom; `AIRCR.SYSRESETREQ` restarts only the
core that asked. A reboot through the bootrom's scratch vector (`reset_usb_boot`, `watchdog_reboot`) switches a
direct-booted machine to ROM boot for good.

## Board files, inputs and CAN

`--board` attaches what the board file describes ([boards.md](boards.md)): the MCP2517FD on its SPI bus, I2C
sensors, ADC inputs, PIO-driven devices, straps. The first log line lists the board's physical inputs; `--input`
sets them before the run:

```bash
firmware_simulator run $FW/power_board --board boards/mk2/power_board.toml --mock-bridge 1 \
    --input stbd_battery=15.1 --input sht41.temperature_c=45
```

A board with a `[can]` section sits on an in-process CAN bus (CAN FD 1/5 Mbit/s for `internal`, classic
250 kbit/s for `external`) with arbitration, bit timing and acknowledgement: a frame is only acknowledged if
another node or host endpoint is on the bus, so a board alone sees its frames go unacknowledged.

`--mock-bridge CLIENT` puts a scripted stand-in for `titan_bridge` on the bus, built on the real `canmore` and
`titan_link` crates: it pings the client every 100 ms with the kill switch released (vehicle enabled), and records
what the board sends. Like a Linux CAN interface it queues at most 10 frames, so its pings are refused rather than
piling up while the bus is unusable. `--candump` prints the traffic:

```
[0.000065s] 01420802##10002  (no ACK)
[0.134891s] 06F#C0
[0.200059s] 01420802##10002
[0.300652s] 01020801##100
```

(`(no ACK)` marks a frame nobody acknowledged, `(destroyed: ...)` one cut off by an injected bit error.)

`--can vcan0 --realtime` bridges the bus to a SocketCAN interface instead, for `titan_bridge`, `canmore_cli` and
`candump` on the host; see [robots.md](robots.md#robot-in-a-box-the-orins-side-on-vcan) (needs the vcan module,
not available in the cloud environments this was written in).

`--environment environments/pool.toml --position 2,6,1` puts a board with a `[transducer]` (IVCBoard2) in the
water; see [acoustics.md](acoustics.md).

## Traces

`--trace` prints structured events to stderr (or `--trace-file`):

| Kind | Shows |
|---|---|
| `instr` | Every executed instruction, disassembled (slow; turns off batched execution) |
| `exc` | Exception entry and return |
| `gpio` | Pin level changes: `[0.229408s] GPIO24 = 1` |
| `unmodelled` | Every access to a register the emulator does not model, named from the SVD |
| `device` | What external devices report: WS2812 frames, DShot checksum errors and signal loss, DRV8243 state changes, USB host activity |

```bash
firmware_simulator run test_programs/embassy_blinky.elf --time 1s --trace gpio
firmware_simulator run --bootsel --time 300ms --trace device
#   [0.004565s] usb: device attached
#   [0.127046s] usb: configured 2e8a:0003 Raspberry Pi RP2 Boot (serial E0C91200A4A1), 2 interface(s)
```

Unmodelled registers are also counted and listed at the end of the run, with one note in the log at the first
access (`--unmodelled first` warns at the first access to each register, `every` at every access, `off` says
nothing): writes are stored and reads return the stored or SVD reset value, so firmware that only configures a
block keeps running, but the table says where the emulator is guessing. A pico-sdk image touches a handful of them
(`TBMAN.PLATFORM`, the QSPI pads, `XIP_CTRL.FLUSH` when it writes the flash).

## Profiles, waveforms, dashboard and JSON logs

The developer tools ([tools.md](tools.md) has their output formats and how they work):

| Option | Default | What it does |
|---|---|---|
| `--profile FILE` | | Samples both cores and writes a table of functions by time per core (running and sleeping apart, time per exception handler) to `FILE`, and folded stacks for flame graph tools to `FILE` with the extension `.folded` |
| `--profile-interval T` | `10us` | Mean virtual time between samples (jittered by ±50 %) |
| `--vcd FILE` | | Writes the GPIO levels as a VCD file (1 ns steps) for GTKWave, Surfer or PulseView, whatever drives the pins: SIO, PWM, PIO, devices, faults |
| `--vcd-pins SPEC` | `all` | Pins to record: numbers, ranges and the board file's `[pins]` names, e.g. `0-7,25,status_ledr` |
| `--vcd-events` | off | Adds string variables with defmt logs, UART lines, CAN frames and device reports (GTKWave and Surfer show them) |
| `--dashboard ADDR` | | Serves a live page on `ADDR` (`127.0.0.1:8080`): cores, logs, firmware status, CAN traffic, pins, LEDs, inputs; controls inject faults, set inputs and pause. No authentication: bind it to loopback |
| `--dashboard-hold` | off | Keeps serving after the run ends |
| `--dashboard-pause-at T` | | Pauses virtual time when it reaches `T` (`0`: from the start); resume from the page |
| `--log-json FILE` | | One JSON object per line for every defmt log line, UART and USB serial line, CAN frame, injected fault and the end of the run, with virtual times |

```bash
firmware_simulator run $FW/esc_board --board boards/mk2/esc_board.toml --mock-bridge 2 --time 3s --profile esc.txt
inferno-flamegraph < esc.folded > esc.svg
firmware_simulator run $FW/esc_board --board boards/mk2/esc_board.toml --mock-bridge 2 --time 1s \
    --vcd esc.vcd --vcd-pins esc1_pwm,esc1_telem,status_ledg --vcd-events
firmware_simulator run $FW/power_board --board boards/mk2/power_board.toml --mock-bridge 1 --realtime --time 600s \
    --dashboard 127.0.0.1:8080 --log-json power.jsonl
```

Coding agents get the same through `firmware_simulator mcp`, an MCP server whose tools start runs, read their
state, logs and CAN frames, and inject faults ([tools.md](tools.md#mcp-server)).

**Code coverage.** `--coverage FILE` writes which source lines and functions of the firmware ran as an lcov
tracefile (for `genhtml` or VS Code's Coverage Gutters) and prints functions and lines hit per crate
([tools.md](tools.md#code-coverage)):

```bash
firmware_simulator run $FW/power_board --board boards/mk2/power_board.toml --mock-bridge 1 --time 2s \
    --coverage power.info
genhtml power.info -o power-coverage
```

`--log-json`, `--trace-file` and `--candump-log` are flushed every 100 ms of wall time and stdout line by line, so
`tail -f` (or a script waiting for a line) can follow a run while it goes on. None of them changes what a
single-core program does; with both cores running, the profiler's extra cuts can
shift how their instructions interleave, as a different `--quantum` would. They combine with each other and with
`--trace`. With `picoboot load`, the profile names the functions of the image that was flashed.

## Debugging with GDB

`--gdb 127.0.0.1:3333` waits for GDB before running. Each core is a thread (thread 1 is core 0); registers,
memory (writes reach flash, so `load` works), software and hardware breakpoints, stepping, Ctrl-C, `target remote`
and `target extended-remote`, data watchpoints (`watch`, `rwatch`, `awatch`: any number, any length; a watchpoint stops
after the access, before the core's next instruction), running backwards ([below](#reverse-debugging-and-rewinding)),
and these monitor commands work:

| Command | |
|---|---|
| `monitor reset` | Resets the chip from the watchdog (as `watchdog_reboot` does) and stays halted at the reset vector until GDB resumes. `reset halt`, `reset init` and `reset run` (OpenOCD's spellings, which Cortex-Debug and many launch scripts send) do the same. GDB caches registers, so run `maintenance flush register-cache` afterwards to see the new ones |
| `monitor halt` | Nothing: the target is halted whenever GDB can send a command |
| `monitor time` | Prints the virtual time, the position (the stopped core and the time on its clock) and whether the run is live or in its recorded history |
| `monitor rewind T` | Goes to virtual time `T` (`1.5s`, `200ms`, `350us`; the last 100 µs slice boundary at or before it), back or forward within what has run; `maintenance flush register-cache` shows the registers there |
| `monitor checkpoints` | The time-travel checkpoints: how many, their memory, the history they cover, what taking and replaying cost |
| `monitor help` | Lists them |

Virtual time stops while GDB has the target halted; RTT logs keep flowing while it runs. Stopping never changes the
run: a stop in the middle of a quantum or a slice (a breakpoint, a watchpoint, a step) goes on from exactly there,
and with time travel on (the default with `--gdb`) the run goes on in pieces of 500 µs ending on multiples of
500 µs, so a run with any number of stops, steps and interrupts is the same as one without. Stepping (`stepi`)
runs the selected core one step (an instruction, or an exception entry) while the other core runs as scheduled.
So, unlike OpenOCD (whose default `cortex_m maskisr auto` masks interrupts while stepping), a step can take an
interrupt: `next` (VS Code's Step Over) where an interrupt comes due, at a `wfe` say, stops at the start of its
handler, and `finish` (Step Out) goes back to where it struck. Peripheral registers read
without side effects (no FIFO pops or spinlock claims), so GDB and an SVD view can read whole register blocks.

```bash
firmware_simulator run $FW/template --board boards/mk2/power_board.toml --gdb 127.0.0.1:3333 --time 60s &
gdb-multiarch -ex "target remote :3333" -ex "break main" -ex continue $FW/template
#   Thread 1 hit Breakpoint 1, template::__cortex_m_rt_main_trampoline () at template/src/main.rs:23
#   (gdb) info threads
#   * 1    Thread 1 (core 0) template::__cortex_m_rt_main_trampoline () at template/src/main.rs:23
#     2    Thread 2 (core 1) 0x00000176 in ?? ()
#   (gdb) monitor time
#   virtual time 0.003147s
```

A connection that closes without sending anything (a port check) is not taken for GDB; the emulator keeps waiting.
Killing the session from GDB, or GDB going away while the target runs, ends the run. Detaching lets it run on to
`--time`, which counts from power-on: with `--time 0s` the run lasts exactly as long as GDB drives it. SIGINT or
SIGTERM while GDB runs the target ends the run too, and GDB hears that the program exited.

A board of a robot is debugged the same way with `robot --gdb BOARD=ADDR` (going backwards takes the whole robot
back: every board, bus, the vehicles and the water). Continuing runs the whole robot, with
its boards in turn on one thread (with or without `--threads`; a recorded history made with the boards running
ahead of each other is the same history), and the board's breakpoints stop all of it. `--fault`s that fall due while GDB
is attached are applied once it detaches:

```bash
firmware_simulator robot robots/talos.toml --mock-bridge internal:2 --gdb esc0=127.0.0.1:3333 --time 0s &
gdb-multiarch -ex "target extended-remote :3333" -ex "break main" -ex continue $FW/esc_board
```

## Reverse debugging and rewinding

Runs are deterministic, so the past can be made again. With `--gdb` (or `--dashboard`) the run is recorded: a
checkpoint (a copy of the whole board or robot: cores, memories, peripherals, devices, buses, vehicles, the water)
every `--checkpoint-every` of virtual time, and a log of what came from outside since (the runs GDB asked for and
where they stopped, what GDB wrote, inputs set, what bridges, SocketCAN, cannelloni, UDP relays and ptys sent in).
Going back restores the checkpoint before the moment and replays the log up to it; bridges are not asked again,
what they sent is given again.

```text
(gdb) break ros_task          # anything; or watch a variable
(gdb) continue                # ... the firmware panics, the board reboots ...
(gdb) reverse-continue        # back to the most recent hit before here
(gdb) reverse-stepi           # one instruction (or exception entry) back
(gdb) reverse-next            # GDB builds reverse-step, -next, -finish on these
(gdb) watch crash_counts.panics
(gdb) reverse-continue        # who wrote it: stops before the writing instruction
(gdb) monitor rewind 1.5s     # to a time
(gdb) continue                # forward again: the recorded history, then live
```

| Command | Packet | What it does |
|---|---|---|
| `reverse-continue` | `bc` | Back to the most recent breakpoint or watchpoint hit before the current position, on either core; for a watchpoint, to just before the instruction that made the access (as GDB expects backwards). With none, to the start of the history (`No more reverse-execution history`) |
| `reverse-stepi` | `bs` | Back one step of the selected core: to just before its previous instruction or exception entry (stepping back out of an interrupt handler's first instruction lands before the exception entry) |
| `reverse-step`, `reverse-next`, `reverse-finish` | `bs`, `bc` | GDB's, on top of the two |
| `monitor rewind T` | | To a virtual time |
| `continue`, `stepi` in the past | | Re-execute the recorded history, stopping at breakpoints and watchpoints on the way, then go on live from where the run had got |

**Positions.** A position is a slice boundary, or a step of one core: the core and the time on its clock when
the step starts (each step takes at least a cycle, so the time names it). Going to a position gives the state a
run that never stopped had there, both cores, every peripheral and device, the buses and, for a robot, every board,
the vehicle and the water (`crates/sim_host/tests/timetravel.rs` and `crates/sim_test/tests/timetravel.rs` check
this against fresh runs, on a board and across Talos).

**Changing the past.** Writing a register or memory, `monitor reset`, setting an input or injecting a fault while
in the past drops the recorded future: the run goes on live from there (bridges carry on from where they were at
the end). Writes are logged, so going back before one and forward again makes it again. Faults applied by
`--fault` or the dashboard take a checkpoint right after them, which replays never go through.

**What it costs.** Checkpoints share the flash, the ROM and unchanged 4 KB pieces of SRAM with the one before, so
one is about 30-140 µs and a few tens of KB for a board, under 1 ms for all of Talos (the bench's
`--checkpoint-every 10ms`, CPU time of 2 s of virtual time: template +0.01 s, ESC board +2 %, IVCBoard2 +1 %,
Talos +4 %, the IVC2 pool +15 %, its acoustic medium being the expensive part; the fingerprints do not change;
Talos on three threads, its boards running ahead of each other, +3 % CPU and about +10 % on the wall clock, the
checkpoints being taken while the boards wait).
`--checkpoint-memory` bounds them (512 MB by default: about 80 s of a single board, 7 s of Talos, before older
checkpoints are thinned out, their spacing growing with age; the first is always kept, so going back to any time
stays possible, it only replays more). Going back costs a restore (a few hundred µs) and a replay of up to
`--checkpoint-every` of virtual time, at the emulator's speed, single stepped where breakpoints or watchpoints are
set: a reverse step is a few ms, a reverse-continue a replay per checkpoint interval searched. Plain `run` and
`robot` record nothing unless given `--checkpoint-every`; then their runs cost only the checkpoints.

Not recorded: USB/IP traffic (a warning says so) and the profiler, which is off with time travel unless
`--checkpoint-every` is given (it cuts runs at its sample times, which replays do not).

## Debugging in VS Code

The repository's [`.vscode/launch.json`](../.vscode/launch.json) and [`.vscode/tasks.json`](../.vscode/tasks.json)
let you step through firmware in VS Code with the [Cortex-Debug](https://github.com/Marus/cortex-debug) extension
(`marus25.cortex-debug`, recommended in `.vscode/extensions.json`). You also need `gdb-multiarch` (Debian, Ubuntu)
or `arm-none-eabi-gdb`, and the images (see [Images](#images)). Pick a configuration in the Run and Debug view:

| Configuration | Asks for | Runs |
|---|---|---|
| Emulator: titan_firmware_rs image | An image (the [smoke test](../crates/sim_test/tests/smoke.rs)'s list) | The image on its board file, with a mock `titan_bridge` for its client |
| Emulator: Talos robot board | A board of `robots/talos.toml` | The whole robot, with GDB on that board and a mock bridge for it |
| Emulator: C pico-sdk ELF | An ELF path (default `build/c/basic_template/template_project.elf`) | The ELF on its own |
| Emulator: titan_firmware_rs image, reverse debugging (Native Debug) | An image | As the first, debugged with [Native Debug](https://github.com/WebFreak001/code-debug) (`webfreak.debug`), which has Step Back and Reverse Continue buttons |

Each configuration's pre-launch task runs [`scripts/vscode_gdb.sh`](../scripts/vscode_gdb.sh). The script builds
the emulator (`cargo run --release`), links the chosen ELF to `target/vscode/debug.elf` (every configuration's
`executable`, so you are asked only once) and starts the emulator with `--gdb 127.0.0.1:3333 --time 0s`. Cortex-Debug
connects as soon as the emulator logs "waiting for GDB", runs to `main`, and from there you can step, set
breakpoints, watch variables, see both cores in the call stack, and read peripheral registers in the Cortex
Peripherals view (from the SVD, [`crates/rp2040/svd/rp2040.svd`](../crates/rp2040/svd/rp2040.svd)). Pause, Restart
(`monitor reset halt`) and Stop work. The emulator ends with the debug session.

Launch does not reset the chip: the emulator already waits at the reset vector, and a reset would be a watchdog
reset, which titan firmware counts. If your GDB is not `gdb-multiarch`, change `gdbPath` or set
`cortex-debug.gdbPath` in your user settings. To debug another robot, or with other options, copy a task and change
its arguments; arguments after the pick in a task's `args` go to the emulator, e.g. `--realtime`.

**Reverse debugging in VS Code.** The emulator advertises `ReverseStep` and `ReverseContinue`, but Cortex-Debug
has no Step Back or Reverse Continue buttons (it does not offer them to VS Code, whatever the server supports): in
its Debug Console, type `-exec reverse-continue`, `-exec reverse-stepi`, `-exec reverse-next` or
`-exec monitor rewind 1.5s`. The Native Debug configuration shows both buttons; they send
`-exec-continue --reverse` and `-exec-step --reverse`, which `crates/sim_host/tests/gdb.rs` drives through the
real GDB.

The configurations are checked by driving the real GDB over GDB/MI with the commands Cortex-Debug and Native Debug
send (`real_gdb_mi_session_as_cortex_debug_runs_it` in `crates/sim_host/tests/gdb.rs`: connect, `monitor halt`,
run to main, both cores' threads, a peripheral block, pause, Step Back, Reverse Continue, Restart, detach), and
`gdb.rs` also has the packet-level tests. `scripts/vscode_gdb.sh` was run in its three modes (`run`, `elf` with the
C template, `robot` on Talos's `esc0`) with GDB connecting as the configurations do. VS Code itself has not been
run against it (to verify locally).

## USB: BOOTSEL, PICOBOOT, UF2 and CDC

The USB controller is modelled in device mode (endpoint and buffer control, single and double buffering, data
toggles, STALL/NAK, setup packets, bus reset, SOF, suspend and resume; it needs `clk_usb` at 48 MHz). `--usb`
plugs the board's port into a host controller model that debounces the attach, resets the bus, enumerates the
device and runs control, bulk and interrupt transfers transaction by transaction on the virtual clock (full speed
bus time: about 1.2 MB/s per bulk pipe). `--trace device` shows what it does.

| Command | What it does |
|---|---|
| `firmware_simulator run --bootsel --time 300ms --trace device` | Powers on in BOOTSEL: the real bootrom enumerates as `2e8a:0003` "RP2 Boot" with the PICOBOOT interface and the RPI-RP2 drive |
| `firmware_simulator picoboot load IMAGE --time 2s` | `picotool load -v -x`: blank flash, BOOTSEL, flash over PICOBOOT through the bootrom's own flash code, verify, reboot into the image and run it for `--time`. Further arguments are `run` options (`--board`, `--usb-cdc stdout`, `--flash-file`, ...) |
| `... picoboot load app.uf2 --load-via drive --flash-file flash.bin --time 1s` | The same by copying the UF2 to the RPI-RP2 drive (bulk-only mass storage and SCSI), keeping the flash afterwards. The bootrom reboots about 500 ms after the copy, so give it a longer `--time` than with PICOBOOT |
| `run --bootsel --picoboot-load FILE` | What `picoboot load FILE` is short for: waits (up to 5 s of virtual time) for the board to be in BOOTSEL mode, flashes `FILE`, reboots into it. With an image instead of `--bootsel`, the image has to reboot into BOOTSEL itself |
| `run test_programs/pico/hello_usb.elf --usb-cdc stdout --time 1s` | pico-sdk stdio over USB (TinyUSB CDC ACM): the host opens the port (115200 8N1, DTR/RTS) and prints its lines as `[0.136602s] usb: Hello, USB!` |
| `run ... --usb-cdc pty --realtime --time 600s` | Puts the CDC port on a pseudo terminal for `picocom`/`screen` (input reaches the device) |
| `run ... --usb-cdc pty:/tmp/board --usb-cdc2 pty:/tmp/board-ros --time 600s` | `pty:PATH` also links `PATH` to the terminal, so tools can use a fixed name. `--usb-cdc2` opens the device's second CDC port: titan_dual_stdio_usb's secondary port, which titan_firmware's micro-ROS USB transport uses (`ros2 run micro_ros_agent micro_ros_agent serial --dev /tmp/board-ros`). The first port keeps the text log |
| `run ... --usbip 127.0.0.1:3240 --realtime` | Exports the device over USB/IP for the real host's tools after `sudo usbip attach -r 127.0.0.1 -b 1-1` (`picotool`, `/dev/ttyACM*`, the drive); attach again after the device reboots. **To verify locally**: tested only with an in-process USB/IP client, not the kernel's vhci-hcd |

Firmware that asks for BOOTSEL (pico-sdk's `reset_usb_boot`, its reset interface or 1200 baud trick, embassy's
`reset_to_usb_boot`) really goes there, through the watchdog and the bootrom, even after a direct boot.
`test_programs/pico/hello_usb` (stdio_usb; `b` reboots to BOOTSEL) and `test_programs/c_dual_usb` (titan_firmware's
`titan_dual_stdio_usb`, two CDC ports) are the device-side test programs (`crates/sim_host/tests/usb.rs`). In code,
`sim_host::usb` has `Picoboot` (what `picotool` does), `MscDrive` (copy a UF2) and `reboot_via_reset_interface`
(`picotool reboot -f [-u]`).

Not modelled: the controller's host mode, isochronous endpoints, low speed devices, bit-level errors. The
RP2040-E5 erratum is not modelled (pico-sdk's workaround runs and works).

## Several micro-ROS boards at once

The C boards' micro-ROS clients choose their DDS domain themselves: titan_firmware calls
`rclc_support_init(&support, 0, NULL, &allocator)` (e.g. `talos/IVCBoard2/src/ros.c` on `blake/ivc`) with the
default init options, so each client asks its agent for domain 0, and `ROS_DOMAIN_ID` on the agent's side changes
nothing. Two emulator runs, each with its own agent, therefore publish the same topics (`/talos/ivc/rx_new`, ...)
on the same domain, and a test that listens on one sees both. Separate ports or pty paths per run only keep the
serial links apart; the DDS traffic still meets. Ways to keep runs apart:

* **One Fast DDS discovery server per run** (what the test campaign did): every participant of a run (its agent,
  its `ros2` tools, the test) is pointed at that run's server and discovers only the others pointed at it. Fast DDS
  reads `ROS_DISCOVERY_SERVER` itself, so this works for the agent too.
* **A network namespace or container per run** (`docker run --network none` with the pty paths mounted, or
  `unshare --net`): DDS discovery cannot cross it. Mind that Fast DDS's shared memory transport is per host, so
  keep `/dev/shm` separate as well (a container does).
* **Patching the firmware** to set a domain id in its init options, which is a firmware change, not an emulator one.

A run with its own discovery server, for each run with another port, server id and pty paths (checked with ROS 2
Humble's Fast DDS 2.6.12, without a micro-ROS agent: a `ros2 topic pub` and `ros2 topic echo` pointed at one
server see each other, and tools pointed at another server, or at none, do not; the server's options in 2.x are
`-i`, `-l` (`--udp-address`) and `-p` (`--udp-port`), and the Nix shell's `fastdds` wrapper does not find its
`fast-discovery-server`, which takes the same options; **to verify locally**: the agent and the board in it):

```bash
PORT=11811                     # 11812, 11813, ... for the other runs
fastdds discovery -i 0 -l 127.0.0.1 -p $PORT &   # or fast-discovery-server -i 0 -l 127.0.0.1 -p $PORT
export ROS_DISCOVERY_SERVER=127.0.0.1:$PORT
firmware_simulator run $TITAN_FIRMWARE_DIR/talos/IVCBoard2/build/IVCBoard2_with_bl.uf2 \
    --board boards/talos/ivc_board.toml --rom-boot --usb-cdc pty:/tmp/run1-log --usb-cdc2 pty:/tmp/run1-ros \
    --log-json run1.jsonl --time 600s &
ros2 run micro_ros_agent micro_ros_agent serial --dev /tmp/run1-ros &
ros2 topic echo /talos/ivc/rx_new             # this run's tools, with the same ROS_DISCOVERY_SERVER
```

A discovery client only learns about the topics it uses, so `ros2 topic echo` of a known topic works, but
`ros2 topic list` and the `ros2` daemon need to be a super client: `ROS_SUPER_CLIENT=TRUE` (Humble's Fast DDS
2.6 takes it: `ROS_SUPER_CLIENT=TRUE ros2 topic list --no-daemon` lists the run's topics, a plain client only
`/rosout` and `/parameter_events`), or a `FASTRTPS_DEFAULT_PROFILES_FILE` with a `SUPER_CLIENT` participant profile
(the ROS 2 "Using Fast DDS Discovery Server" tutorial has one). Restart the daemon (`ros2 daemon stop`) after changing
`ROS_DISCOVERY_SERVER`. Give the agent and the tools generous timeouts: a
run that is busy may be slower than real time ([Options](#options)). Several runs in parallel are otherwise
independent: eight IVCBoard2 runs on a 20-core machine kept up fine in the test campaign.

## Puddles: Ethernet

The Puddles backplane talks to the robot computer over a W5500 in MACRAW mode instead of CAN. On a board with a
`[w5500]` and no `[can]`, `--mock-bridge` (any client number) attaches a scripted bridge on the robot computer's
address (`192.168.1.22`) that answers ARP and speaks the Puddles UDP transport (titan_link datagrams on port 2203
both ways, CANmore heartbeats on 2201) with the same pings and kill switch forwarding as the CAN mock bridge. The
board is `192.168.1.42`.

```bash
firmware_simulator run $FW/puddles_backplane --board boards/mk2/puddles_backplane.toml --mock-bridge 1 --time 1s
#   [0.235001s] 0.101746 INFO  Ethernet up as 192.168.1.42 (MAC [2a, cd, 83, 2b, 51, 29])
#   [0.300601s] 0.167395 INFO  ROS connected
```

`--udp LOCAL=PEER` relays the board's datagrams through a host UDP socket instead, for the real `titan_bridge`
(listening on `PEER`, sending to `LOCAL`; **to verify locally**, no ROS 2 here):

```bash
firmware_simulator run $FW/puddles_backplane --board boards/mk2/puddles_backplane.toml \
    --udp 127.0.0.1:2203=127.0.0.1:2204 --realtime --time 600s
```

The relay only passes on datagrams from `PEER`'s IP address (from any port), as the board itself only accepts the
robot computer's IP and the cannelloni tunnel only its peer's: nobody else on the network can send `titan_link`
commands (the kill switch among them) to the emulated board. A `PEER` of `0.0.0.0:PORT` accepts every sender.

Puddles in its simulated vehicle is `robots/puddles.toml` ([robots.md](robots.md), [vehicles.md](vehicles.md)).

## Exit status

`run` exits with 0 when it reached `--time` (or stopped at a breakpoint, a `BKPT` with `--stop-on-bkpt`, or a
deadlock: every core asleep with nothing scheduled, reported on stderr), also when a core locked up on the way and
the watchdog reset the chip (the end says how often: `2 lockup(s), 1 chip reset(s)`); with an error when a core
locked up and nothing can end it (the watchdog not running; any lockup with `--stop-on-lockup`,
[faults.md](faults.md#what-the-models-do)), when an option does not fit the board (`--input` of an input the board does not have, `--mock-bridge` without a bus) or a
fault could not be applied.

SIGINT (Ctrl-C) or SIGTERM (`kill`, `timeout`) ends a `run` or `robot` cleanly: the emulator stops at the next
slice boundary (within 100 µs of virtual time) and does everything the end of `--time` does: it saves
`--flash-file`, writes the JSON log's `end` record (`"reason":"Interrupted"`), the waveform, the profile and the
candump log, and prints the stop line, `stopped at 12.345678s (Interrupted); ...`. It then exits with 128 plus the
signal's number (130 for SIGINT, 143 for SIGTERM), as a shell expects of a process a signal ended. A second signal
quits at once without any of that, and so does a signal while `--dashboard-hold` keeps the dashboard up after the
run.
