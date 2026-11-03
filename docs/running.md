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
- [Debugging with GDB](#debugging-with-gdb)
- [USB: BOOTSEL, PICOBOOT, UF2 and CDC](#usb-bootsel-picoboot-uf2-and-cdc)
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
| Hand-written test programs (`test_programs/*.elf`, `embassy_blinky.elf`, `pico/*.elf`, `c_dual_usb`) | committed; `make -C test_programs` (Thumb and embassy), `make -C test_programs pico`, `make -C test_programs c_dual_usb` rebuild them |

The C titan_firmware board applications need titan_firmware's prebuilt micro-ROS library (built in Docker), so the
script does not build them; the C programs the tests run are the template, the bootloader and its test app.

## What it prints

```
[INFO  firmware_simulator] board mk2/power_board (1 SPI, 2 I2C device(s); inputs: port_battery, rail_3v3, rail_balanced, sht41.humidity_rh, sht41.temperature_c, stbd_battery)
[INFO  firmware_simulator] direct boot: VTOR 0x10000100, entry 0x100001c0
[INFO  firmware_simulator] RTT control block at 0x20000008, decoding defmt
[INFO  firmware_simulator] mock bridge for client 1 attached
[0.132000s] 0.000025 INFO  mk2/power_board v1.0 (release type 2, git 342ee0d)
[0.132000s] 0.000123 INFO  Crash counts since power on: CrashCounts { watchdog_timeouts: 0, panics: 0 }
[0.135200s] 0.003300 INFO  CAN controller online as client 1
[0.200400s] 0.068450 INFO  ROS connected
stopped at 2.000000s (TimeLimit); core0 234375135 cycles, pc 0x1000039c <..._11power_board18___cortex_m_rt_main0EB1f_+0x94>; 200.71ms wall
core1 249575205 cycles, pc 0x00000176, Some(Wfe)
```

* `[INFO  crate]` lines on stderr are the emulator's own log (`RUST_LOG` filters it, e.g.
  `RUST_LOG=sim_devices=trace` logs every MCP2517FD SPI transaction; the default is `info`).
* `[0.132000s] 0.000025 INFO  ...` is a defmt line: the virtual time the host read it from RTT (polled every
  100 µs), the firmware's own `defmt::timestamp!`, the level and the message. defmt is decoded from RTT channel 0
  with the ELF's `.defmt` table; RTT text that is not defmt is printed at the end as `[rtt0] ...`.
* `[0.006337s] uart0: AB Hello, UART!` is a complete line from a UART (pico-sdk stdio, the C logger).
* The last lines give the stop reason, both cores' cycle counts and PCs (with symbols) and the wall time taken,
  then a table of every unmodelled register the firmware touched (see `--unmodelled`).

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
| `--realtime` | off | Paces virtual time to the wall clock (needed with host tools); warns when the emulation falls behind |
| `--uart0`, `--uart1` | `stdout` | `stdout` (lines with their virtual time), `pty` (a pseudo terminal, input too) or `none` |
| `--rom-boot` | off | Boots through the bootrom and boot2 instead of directly ([below](#boot-modes-and-the-flash)) |
| `--flash-file FILE` | | Keeps the flash in a file between runs |
| `--hle-flash` | off | Traps the bootrom's flash functions instead of running them against the flash chip model |
| `--flash-size BYTES` | the board file's `flash_size`, else 2 MiB | Size of the QSPI flash chip |
| `--crystal-hz HZ` | 12 MHz | XOSC frequency (a board file's `crystal_hz` replaces it) |
| `--bootsel`, `--usb`, `--usb-cdc stdout\|pty`, `--picoboot-load FILE`, `--load-via picoboot\|drive`, `--usbip ADDR` | | USB ([below](#usb-bootsel-picoboot-uf2-and-cdc)) |
| `--trace KINDS` | | Comma separated: `instr`, `exc`, `gpio`, `unmodelled`, `device` ([below](#traces)) |
| `--trace-file FILE` | stderr | Where the trace goes |
| `--unmodelled MODE` | `first` | Logging of registers the emulator does not model: `first` access to each, `every` access, or `off` |
| `--quantum N` | 16 | Instructions a core runs before the other gets a turn (1 for lockstep debugging) |
| `--stop-on-bkpt` | off | A `BKPT` stops the run instead of escalating to HardFault |
| `--gdb ADDR` | | Waits for GDB on this address before running ([below](#debugging-with-gdb)) |

`firmware_simulator run --help` has the same list.

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
at the end, so what the firmware writes (a flash recorder, crash data, or an application the C Titan bootloader
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
`candump` on the host; see [robots.md](robots.md#robot-in-a-box-vcan-titan_bridge-docker) (needs the vcan module,
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

Unmodelled registers are also logged as warnings on first access (`--unmodelled`), and summarised at the end:
writes are stored and reads return the stored or SVD reset value, so firmware that only configures a block keeps
running, but the warning says the emulator is guessing.

## Debugging with GDB

`--gdb 127.0.0.1:3333` waits for GDB before running. Each core is a thread (thread 1 is core 0); registers,
memory (writes reach flash, so `load` works), software and hardware breakpoints, stepping, Ctrl-C and the monitor
commands `monitor reset` and `monitor time` work. Virtual time stops while GDB has the target halted; RTT logs keep
flowing while it runs.

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

Killing the session from GDB ends the run.

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
| `run ... --usbip 127.0.0.1:3240 --realtime` | Exports the device over USB/IP for the real host's tools after `sudo usbip attach -r 127.0.0.1 -b 1-1` (`picotool`, `/dev/ttyACM*`, the drive); attach again after the device reboots. **To verify locally**: tested only with an in-process USB/IP client, not the kernel's vhci-hcd |

Firmware that asks for BOOTSEL (pico-sdk's `reset_usb_boot`, its reset interface or 1200 baud trick, embassy's
`reset_to_usb_boot`) really goes there, through the watchdog and the bootrom, even after a direct boot.
`test_programs/pico/hello_usb` (stdio_usb; `b` reboots to BOOTSEL) and `test_programs/c_dual_usb` (titan_firmware's
`titan_dual_stdio_usb`, two CDC ports) are the device-side test programs (`crates/sim_host/tests/usb.rs`). In code,
`sim_host::usb` has `Picoboot` (what `picotool` does), `MscDrive` (copy a UF2) and `reboot_via_reset_interface`
(`picotool reboot -f [-u]`).

Not modelled: the controller's host mode, isochronous endpoints, low speed devices, bit-level errors. The
RP2040-E5 erratum is not modelled (pico-sdk's workaround runs and works).

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

Puddles in its simulated vehicle is `robots/puddles.toml` ([robots.md](robots.md), [vehicles.md](vehicles.md)).

## Exit status

`run` exits with 0 when it reached `--time` (or stopped at a breakpoint, a `BKPT` with `--stop-on-bkpt`, or a
deadlock: every core asleep with nothing scheduled, reported on stderr), and with an error when a core locked up,
an option does not fit the board (`--input` of an input the board does not have, `--mock-bridge` without a bus) or a
fault could not be applied.
