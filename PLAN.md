# firmware_simulator: plan

An emulator that runs unmodified RP2040 firmware images (ELF/UF2) on a development machine, built first to test
`titan_firmware` (C, pico-sdk) and `titan_firmware_rs` (Rust, embassy) without hardware. NVIDIA Orin support comes
later; the crate layout leaves room for it (see [Orin](#orin-later)).

## Goals

1. **Run the real binaries.** No firmware changes, no HAL shims, no `cfg(sim)`. The same ELF that `probe-rs` would
   flash boots in the emulator.
2. **Talk to the real host stack.** Emulated boards appear on a Linux `vcan` CAN FD interface (and a UDP socket for
   Puddles), so `titan_bridge`, `canmore_cli`, `upload_tool` and `candump` work against them unchanged.
3. **Deterministic when wanted.** A fixed seed and a virtual clock reproduce a run exactly, which makes firmware
   integration tests possible in CI. A real-time mode paces the virtual clock to the wall clock for use with the
   host tools.
4. **Debuggable.** GDB remote stub, defmt/RTT and UART log capture, instruction and peripheral traces, and a log line
   for every access to a register that isn't modelled yet (named from the SVD).

### Non-goals (for now)

* Analog and electrical behaviour (ADC noise, signal integrity, power latch timing in microseconds).
* Bit-exact bus timing for external chips beyond what firmware can observe.
* Cycle-exact Cortex-M0+ pipeline. Cycle-*approximate* is enough; PIO is the exception (below).

## What the firmware needs

Taken from the peripherals `titan_firmware_rs/firmware` and `titan_firmware/lib` actually touch:

| Area | Used by | Priority |
|---|---|---|
| 2x Cortex-M0+, NVIC, SysTick, SCB/VTOR, WFI/WFE/SEV | everything; `embassy_rp::multicore` on ESC (core 1) | M1 |
| Bootrom (function table, float/double helpers, flash ops) | pico-sdk (`__aeabi_*` via ROM), `embassy_rp::flash` | M2 |
| RESETS, CLOCKS, XOSC, PLL, ROSC, VREG, PSM | `embassy_rp::init`, pico-sdk `runtime_init` | M1 |
| SIO: CPUID, GPIO, FIFOs, spinlocks, divider, interpolators | embassy executor, multicore, pico-sdk | M1/M2 |
| TIMER (alarms), WATCHDOG (+ scratch 0-7, reset reasons) | embassy-time, Titan Safety crash records | M1 |
| IO_BANK0 / PADS (function select, edge/level IRQs) | MCP2517FD INT pin, every board | M1 |
| SPI0 + DMA (ch 1, 2) | `titan_rp2040::can_block!` → MCP2517FD | M3 |
| I2C (DW_apb_i2c) | SHT41, MCP3426, depth, gauge, power boards | M4 |
| ADC (+ temp sensor, round robin, FIFO, DMA) | IVC hydrophones (ADC1/GPIO 27), power monitoring | M4 |
| PWM | `can_block!` (slices 6, 7), LEDs, H-bridge | M4 |
| UART (PL011) | C stdio on UART0, Hiwonder bus | M2/M4 |
| PIO + DMA | status strip (WS2812), DShot, Dynamixel UART, torpedo, LEDBoard2, IVC2/IVCDAC, H-bridge | M5 |
| XIP flash + SSI + QSPI flash chip | flash recorder (IVC2), C bootloader, `upload_tool` over CAN | M6 |
| USB device | PICOBOOT/BOOTSEL upload, C `titan_dual_stdio_usb` | later |

External chips that need behavioural models: **MCP2517FD/MCP2518FD** (SPI ↔ CAN FD), **W5500** (SPI ↔ UDP),
SHT41, MCP3426, depth sensor, CD74HC4051 mux, smart battery gauge, SSD1306, DRV8243, DShot ESC, Dynamixel servo,
WS2812 strip (decode only).

## Architecture

```
                 ┌──────────────────────────── sim process ────────────────────────────┐
 board.toml ───► │ Machine (rp2040)                                                    │
 firmware.elf    │  ┌────────┐ ┌────────┐   ┌──────────── Bus ─────────────────────┐   │
                 │  │ core 0 │ │ core 1 │──►│ ROM │ SRAM │ XIP │ APB/AHB periph… │   │   │
                 │  └────────┘ └────────┘   └───────────┬──────────────────────────┘   │
                 │        ▲ IRQ lines                   │ pins / bus wires              │
                 │        └──────────── Scheduler ──────┤ (virtual time, event queue)   │
                 │                                      ▼                               │
                 │                          Device models (MCP2517FD, SHT41, …)          │
                 └────────────┬─────────────────────┬─────────────────────┬────────────┘
                              │ vcan0 (SocketCAN FD)│ UDP                 │ GDB / RTT / trace
                       titan_bridge, canmore_cli, upload_tool, candump    gdb-multiarch, defmt-print
```

### Crates (one cargo workspace)

| Crate | Contents |
|---|---|
| `sim_core` | Virtual time, event scheduler, `Bus`/`Peripheral` traits, IRQ and pin "wires", trace sink, snapshot traits. No SoC knowledge. |
| `cpu_armv6m` | Cortex-M0+ interpreter: ARMv6-M Thumb (all 16-bit plus `BL`, `MRS`, `MSR`, `DMB`, `DSB`, `ISB`), exception entry/return, NVIC, SysTick, SCB, cycle counts. Implements a `Cpu` trait from `sim_core`. |
| `rp2040` | The SoC: memory map, bus fabric, every RP2040 peripheral, `Machine` builder. Register offsets generated at build time from `rp2040.svd` (from pico-sdk). |
| `sim_devices` | External chip models, each attached to SPI/I2C/UART/GPIO wires. |
| `sim_acoustics` | Underwater acoustic environments for IVC testing: transducers described by their specs, a water channel (spreading, absorption, multipath, noise, Doppler) and receive chains, connected to board output pins and ADC inputs. No board or chip knowledge. See [Acoustic environments](#acoustic-environments-ivc). |
| `sim_host` | Host bridges: SocketCAN, UDP, GDB remote protocol, RTT/defmt reader, UART pty. |
| `firmware_simulator` (bin) | CLI: load ELF/UF2 + board/robot description, run, attach debugger, record traces. |
| `sim_test` | Library for firmware integration tests (`#[test]` that boots an ELF, drives pins/sensors, asserts on CAN traffic). |

The CPU core and the SoC are separate crates so a second machine (Orin SPE, another MCU) can bring its own CPU and
peripherals while reusing `sim_core`, `sim_host` and the device models.

### Execution and timing

* **Single-threaded, deterministic main loop.** Both cores are stepped in round-robin quanta (default 16
  instructions, configurable down to 1 for lockstep debugging). SIO FIFO and spinlock accesses are atomic per bus
  access, so larger quanta stay correct; they only shift interleaving. Host I/O enters through channels drained at
  quantum boundaries, stamped with virtual time.
* **Virtual clock = `clk_sys` cycles.** Instruction cycle costs follow the M0+ TRM (1 cycle most ops, 2 for
  loads/stores and taken branches, 3 for `BL`, 4 for exception entry overhead in addition to stacking); XIP adds a
  configurable cache-miss penalty. Peripherals never poll: they schedule events (`timer alarm at t`, `SPI byte done at
  t`) on the scheduler.
* **Idle fast-forward.** When both cores sit in `WFI`/`WFE` with nothing pending, jump straight to the next scheduled
  event. Embassy firmware spends most of its time there, which is what makes many boards in one process affordable.
* **Clock tree is real enough.** CLOCKS/PLL configuration determines `clk_sys`, `clk_peri`, `clk_adc` frequencies;
  the TIMER ticks from `clk_ref`/watchdog tick as on hardware, so wrong clock setup shows up as wrong timing.
* **Real-time mode.** Sleeps the host thread so virtual time tracks wall time; reports when the emulator falls behind
  (`titan_bridge` kills the link after 700 ms without pings, so this matters).
* **PIO is cycle-exact.** State machines step once per `clk_sys` (with the clock divider) while any SM is enabled.
  DShot, WS2812 and the Dynamixel UART depend on it. PIO is the most likely performance hot spot; optimise only after
  measuring (batch-step SMs stalled on FIFOs, skip disabled blocks).

### Boot

* **M1 "direct boot":** XIP reads always return flash contents regardless of SSI state; load the image, set VTOR to
  `0x1000_0100`, SP/PC from the vector table, as if bootrom + boot2 had run. Gets firmware running quickly.
* **M2 real bootrom:** map the official bootrom image (`raspberrypi/pico-bootrom`, BSD-3, version B2) at `0x0`. The
  ROM function table, soft float/double and memcpy work by just executing it. Flash operations are high-level
  emulated at first by trapping the ROM entry points (`flash_range_erase`, `flash_range_program`,
  `connect_internal_flash`, `flash_exit_xip`, `flash_flush_cache`, `flash_enter_cmd_xip`).
* **M6 full boot:** SSI + QSPI flash chip model (W25Q-style command set) so boot2 and the ROM flash code run for real,
  flash writes persist to a backing file, and a watchdog reset reboots through the bootrom with scratch registers and
  RAM preserved (Camera cage `.uninit` depth surface pressure, Titan Safety crash records).

### Board and robot descriptions

```toml
# boards/talos/power_board.toml
mcu = "rp2040"
flash = "W25Q16"            # size + command set
crystal_hz = 12_000_000

[[spi]] bus = "spi0"; cs = 17; device = { mcp2517fd = { int_pin = 20, can = "internal", osc_hz = 40_000_000 } }
[[i2c]] bus = "i2c1"; addr = 0x44; device = "sht41"
[[gpio]] pin = 29; pull = "up"   # strap / board ID inputs
```

A robot file lists boards with their ELF and the CAN buses they share; `firmware_simulator robot talos.toml` starts
them all in one process on an in-process bus that is also bridged to `vcan0`/`vcan1`. Board pin maps come from
`titan_firmware/lib/titan_boards/include/boards/*.h`; generate the TOML from those headers rather than retyping them.

### Acoustic environments (IVC)

The IVC boards are acoustic modems: IVCBoard drives its transducer with a PWM square wave on GPIO 7 (17/19 kHz
tones) and samples a hydrophone with the ADC at 500 kS/s; IVCBoard2/IVCDAC do the same at 50.5-51.5 kHz, with a
GPIO-selected analog filter, and listen for 20-40 kHz pingers. Testing them needs the water between boards, and the
team wants to build different environments to test in. `sim_acoustics` provides that, and it is deliberately not tied
to the current IVC hardware or modem: an environment is simulated from the specs of whatever transducers, water and
receive electronics it is given, so a new transducer, a different modem scheme or a non-IVC acoustic board is a
config change, not emulator code.

* **Transducers by spec.** Each transducer is a projector, a hydrophone or both, described by numbers from its
  datasheet: transmitting voltage response (TVR, dB re 1 µPa/V at 1 m) and receive sensitivity (dB re 1 V/µPa),
  either as a frequency table or as a resonance (centre frequency, Q, peak level); directivity (omni, piston of a
  given radius, or a beam-pattern table); maximum drive voltage; position and orientation on its vehicle.
* **Electrical coupling, also by spec.** A projector is attached to any board output (a GPIO/PWM/PIO pin, through a
  driver stage given as supply voltage and output impedance, or a DAC-like voltage) and a hydrophone to any ADC
  channel through a receive chain given as preamp gain, band-pass corners and order (optionally several filters
  switched by a GPIO, like IVC's filter select on GPIO 3), DC bias and clipping at the rails. Nothing in the chain
  knows about IVC symbols; it sees pin waveforms in and ADC voltages out.
* **The water.** An environment file describes the body of water: open water, a pool or tank (rectangular, with wall
  reflection coefficients) or a lake/harbour (depth, bottom type); sound speed (constant or from temperature,
  salinity and depth); absorption (Francois-Garrison); spherical/cylindrical spreading; multipath by the image
  method (surface, bottom and walls, configurable reflection order); ambient noise (white, or a Wenz-style spectrum
  by sea state and shipping, plus optional snapping-shrimp impulses); and Doppler from vehicle motion. Vehicles
  have positions or scripted trajectories.
* **Other sound sources.** Pingers (frequency, pulse length, period, source level), tones and chirps, WAV playback
  (including real recordings) and interferers can be placed in the environment without any board behind them.
* **How it runs.** Projector pin waveforms are sampled into the acoustic model on the virtual clock; each
  source-receiver path is a delay, gain and absorption filter per image source; hydrophone voltages are produced
  lazily, so the ADC reads exactly the value at its conversion time. Noise comes from the machine's seed, so runs
  are reproducible. Hydrophone signals can be recorded to WAV for inspection.
* **Using it.** Environments live in `environments/*.toml` (e.g. `pool.toml`, `open_water.toml`, `harbour.toml`)
  and a robot file (or several robots, for Talos and the surface vehicle) names one and places its boards' transducers
  in it. `sim_test` can sweep range, noise level and multipath to measure byte error rates and pinger amplitude
  against distance.

```toml
# environments/pool.toml
kind = "tank"
size_m = [25.0, 12.0, 3.0]
wall_reflection = 0.6
bottom_reflection = 0.5
water = { temperature_c = 26.0, salinity_ppt = 0.0 }
noise = { kind = "white", level_db_re_upa_per_hz = 50.0 }
multipath = { max_order = 3 }

[[source]]
kind = "pinger"
position_m = [20.0, 6.0, 2.0]
frequency_hz = 25_000
pulse_ms = 4
period_ms = 2000
source_level_db = 180
```

```toml
# in a robot file
[environment]
file = "environments/pool.toml"

[[board]]
name = "ivc2"
elf = "…/ivc_board2"
position_m = [2.0, 6.0, 1.0]

[board.transducer]
tvr = { resonance_hz = 51_000, q = 8, peak_db = 140 }
rvs_db = -190
beam = "omni"
drive = { pin = 7, supply_v = 12.0 }
receive = { adc = 1, gain_db = 40, bandpass_hz = [40_000, 60_000], select_pin = 3 }
```

## Milestones

Each milestone ends with something you can run.

**M0 – Skeleton.** Workspace and crates above; ELF and UF2 loader; RP2040 memory map with SRAM/ROM/XIP; Cortex-M0+
interpreter with per-instruction unit tests; unknown-register logging from the SVD. *Done when* a hand-written Thumb
test program toggles a SIO GPIO and the trace shows it.

**M1 – Embassy boots.** RESETS, CLOCKS/XOSC/PLL/ROSC (lock/stable bits), TIMER + alarms, WATCHDOG, SIO (single
core), IO_BANK0/PADS, NVIC/SysTick, WFI/WFE fast-forward; RTT control block discovery + defmt decoding (using the
ELF's `.defmt` table); GDB stub (registers, memory, breakpoints, step). *Done when* `firmware/template` prints its
defmt logs and its LED pin toggles at the right virtual rate, and `gdb` can break in `main`.

**M2 – pico-sdk and dual core.** Real bootrom with HLE flash traps; core 1 launch protocol over the SIO FIFO,
spinlocks, divider, interpolators; UART0 to a pty/stdout. *Done when* pico-examples `hello_uart`, `multicore` and
`flash_program` pass, and a C Titan board (e.g. `talos/PowerBoard`) prints its UART boot log.

**M3 – On the CAN bus (the big one).** SPI (PL022), DMA (channels, chaining, DREQs, IRQs), GPIO edge interrupts,
MCP2517FD model (SPI instruction set, RAM, FIFOs, TEF, interrupt flags, nominal/data bit timing checks), SocketCAN FD
bridge. *Done when*:
* a Rust board's CANmore heartbeat shows in `candump vcan0`;
* `titan_bridge --board power --interface vcan0` connects, publishes heartbeat and firmware status;
* stopping the bridge kills the vehicle after 700 ms, and the safety watchdog behaves as in the README;
* a C board and a Rust board share the bus (the "coexistence" item in titan_firmware_rs's hardware checklist).

**M4 – Sensors and actuators.** I2C (master, with NAK/clock stretching), ADC (+ FIFO/DMA, scripted waveforms per
channel), PWM (counter compare readout, PWM-as-input), UART; models for SHT41, MCP3426, depth sensor, CD74HC4051,
gauge, SSD1306 (render to PNG/terminal); `sim_acoustics` core (transducer, receive chain and water models,
environment files, pingers and WAV sources, ADC coupling). *Done when* power, poac, camera cage, SBC and smart
battery boards run with scripted sensor inputs and their ROS topics carry the expected values through
`titan_bridge`, and IVCBoard2 in pinger mode reports a pinger amplitude that falls off with distance as the
environment predicts.

**M5 – PIO.** Full PIO instruction set, FIFOs/joins, side-set, autopush/pull, IRQ flags, `EXEC`, DMA DREQs, GPIO
override; WS2812 decoder, DShot ESC model (including bidirectional eRPM reply), Dynamixel servo model on a PIO UART.
*Done when* ESC board thruster commands produce correct DShot frames with eRPM feedback, Actuator Dynamixel traffic
round-trips, and status strip colours are visible in the trace. Validate PIO against pioasm-assembled test programs
and against rp2040js on the same programs.

**M6 – Flash, bootloader, resets.** SSI + QSPI flash model, persistent flash, real boot2, full watchdog/`SYSRESETREQ`
reboot path with RAM and scratch retention, PSM/power-on-reset distinctions. *Done when* `upload_tool` flashes a
board over vcan through the C Titan bootloader, IVC2's flash recorder survives the watchdog, and crash records appear
after a forced panic.

**M7 – Robot and CI.** Robot description, many boards per process, W5500 model + UDP for Puddles, `sim_test`
harness (boot ELF, inject sensor values/faults, assert on CAN frames or `titan_link` messages decoded with the
`titan_link` crate), snapshot/restore for fast test setup, GitHub Actions job that builds `titan_firmware_rs` and runs
every board's smoke test; boards of one or more robots placed in a shared acoustic environment. *Done when* the smoke
tests run in CI, and two IVC boards (IVCBoard to IVCBoard, and IVCBoard2 to IVCBoard2) exchange bytes through a
`pool.toml` environment, with a `sim_test` sweep showing delivery degrading as range and noise grow.

**Later – USB** (PICOBOOT for `upload_tool -u`, CDC for C stdio), RTC, sleep/dormant modes.

## Testing the emulator itself

* **CPU:** table-driven tests per instruction and flag case; exception stacking/tail-chaining tests; a cross-check
  against a reference model on random instruction streams (rp2040js's core, or QEMU `-M mps2-an385` restricted to
  ARMv6-M instructions with traces compared).
* **Peripherals:** register-level tests written from the datasheet (reset values, W1C/W1S/XOR aliases at
  `+0x1000/+0x2000/+0x3000`, side effects).
* **Programs:** pico-examples as a conformance suite; then every Titan board ELF as smoke tests.
* **Against hardware, once available:** record GPIO/CAN traces from a real board and diff against the emulator's.

## Risks

| Risk | Mitigation |
|---|---|
| Interpreter too slow for 12+ boards in real time | Idle fast-forward first; then decoded-instruction cache per XIP/SRAM page; JIT only if measurements demand it. Deterministic mode with an in-process `titan_link` bridge doesn't need real time at all. |
| PIO cost | Step PIO only while enabled; batch stalled SMs; profile before optimising. |
| MCP2517FD model fidelity (big register set, subtle FIFO semantics) | Model what `titan_rp2040::mcp251xfd` and the C `driver_canbus` use; log every other register access; test both drivers against it. |
| Firmware relies on timing the emulator gets wrong | Expose cycle-cost tables; flag busy-wait loops on the timer that run "too fast"; keep the hardware checklist for what simulation cannot prove. |
| Acoustic model too slow at 500 kS/s per hydrophone | Sample projector waveforms only while a projector is driven; precompute per-path impulse responses; cap the multipath order per environment. |
| Acoustic model gives false confidence | It is for comparing environments and catching regressions, not for proving range: calibrate against IVCDAC recordings once available, and keep in-water tests on the hardware checklist. |
| Silent divergence from hardware | Unknown-register log is on by default; datasheet-derived peripheral tests; later, hardware trace diffs. |

## Orin (later)

`titan_firmware` docs make the Orin the robot computer that runs the micro-ROS agent / `titan_bridge`,
`upload_tool` and `canmore_cli`. Two possible meanings, very different in size:

1. **Simulate the robot computer side (recommended).** Run the real aarch64 Jetson userspace (L4T/Ubuntu + ROS 2
   Humble + riptide stack) in a container under `qemu-user`/binfmt (or natively on an arm64 machine), attached to the
   same `vcan` buses as the emulated boards. No instruction-level SoC emulation; most of the work is a
   `docker compose` setup and a robot description that names the container. This gives a full "robot in a box".
2. **Emulate Orin firmware itself** (e.g. the Cortex-R5F Sensor Processing Engine running FreeRTOS, with its MTTCAN
   controllers). That needs an ARMv7-R CPU crate (Thumb-2, VFP) and Orin SPE peripheral models (TKE timers, HSP
   mailboxes, GPIO, MTTCAN) as a second `Machine`, reusing `sim_core`, `sim_host` and the CAN bus. Full emulation of
   the Cortex-A78AE complex running Linux is QEMU's territory and not worth rebuilding.

Until this is decided, the only thing M0-M7 must do for Orin is keep the `Cpu`/`Machine`/`Bus` boundaries clean and
keep the SoC out of `sim_core`.

## Open questions

* Which Orin meaning is intended (above)?
* Should C firmware support (M2) come before the CAN milestone, or is Rust-first enough? The plan assumes both, since
  coexistence testing needs C boards.
* Is `rp2040js` (TypeScript, MIT) acceptable as a reference oracle in tests (needs Node in CI)?
* How detailed should the acoustic channel be? The plan's recommendation is image-method multipath with
  frequency-dependent absorption, which handles pools, tanks and shallow open water; ray tracing (e.g. a
  Bellhop-style model for sound-speed profiles) only if tests in deep or stratified water need it.
* RP2350 later? The Cortex-M33/Hazard3 cores and new peripherals would be a third machine; nothing here blocks it.
