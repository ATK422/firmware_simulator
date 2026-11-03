# Internals: how the emulator is built

For contributors. The design and its reasons are in [PLAN.md](../PLAN.md); this page describes what exists.

- [Crates](#crates)
- [Virtual time and the scheduler](#virtual-time-and-the-scheduler)
- [The run loop](#the-run-loop)
- [Sessions and robots](#sessions-and-robots)
- [Peripherals and their modelling status](#peripherals-and-their-modelling-status)
- [External device models](#external-device-models)
- [Adding a peripheral](#adding-a-peripheral)
- [Adding a device model](#adding-a-device-model)
- [Determinism rules](#determinism-rules)
- [Snapshots](#snapshots)
- [Performance](#performance)

## Crates

| Crate | What it is |
|---|---|
| [`sim_core`](../crates/sim_core) | No SoC knowledge: virtual time (picoseconds), the keyed event scheduler, the `Bus`/`Peripheral`/`Cpu` traits, IRQ lines and pin levels (`wire`), scripted signals and analog sources, trace events and sinks, ELF/UF2 images, snapshot support (`clone_box!`) |
| [`cpu_armv6m`](../crates/cpu_armv6m) | The Cortex-M0+ interpreter: the whole ARMv6-M Thumb set (all 16-bit encodings plus `BL`, `MRS`, `MSR`, `DMB`, `DSB`, `ISB`, `UDF.W`), exception entry/return/tail-chaining, NVIC, SysTick, SCB, the MPU registers (stored, not enforced), `WFI`/`WFE`/`SEV`, M0+ TRM cycle counts, a disassembler, batched execution |
| [`rp2040`](../crates/rp2040) | The SoC: memory map, bus fabric and peripheral dispatch (`system.rs`), the peripherals (`periph/`), the unmodelled-register fallback named from `rp2040.svd` (register tables generated at build time), the bootrom HLE traps, and the two-core `Machine` with its run loop |
| [`sim_devices`](../crates/sim_devices) | External chips, SoC independent: the SPI/I2C/UART/pin device traits, the CAN bus, MCP2517FD/MCP2518FD, W5500, the sensors, the PIO-driven devices, the USB host controller, fault injectors |
| [`sim_acoustics`](../crates/sim_acoustics) | Water, transducers, receive chains, sources, the image method, the shared medium. No board knowledge |
| [`sim_physics`](../crates/sim_physics) | The 6-DOF vehicle model. No board knowledge |
| [`sim_host`](../crates/sim_host) | Everything that connects a machine to the world: `Session` (a board plus RTT/defmt, UARTs, CAN, Ethernet, USB host side), `Robot`, board and robot files, mock bridges, SocketCAN, Ethernet/UDP, GDB, ptys, USB tools and USB/IP, CAN logs, `can-diff`, cannelloni, `bridges`, the bootloader upload, fault application, vehicles |
| [`sim_test`](../crates/sim_test) | The test harness: `Bench`, `RobotBench`, `sweep` ([testing.md](testing.md)) |
| `firmware_simulator` (root) | The command line (`src/main.rs`, `src/host_tools.rs`) |

The dependency direction is `sim_core` ← `cpu_armv6m`, `sim_devices`, `sim_acoustics`, `sim_physics` ← `rp2040`
← `sim_host` ← `sim_test`, CLI. The CPU and the SoC are separate crates so that another machine (an Orin SPE, an
RP2350) can bring its own CPU and peripherals and reuse the rest; keep SoC knowledge out of `sim_core` and board
knowledge out of `sim_acoustics` and `sim_physics`.

## Virtual time and the scheduler

`sim_core::time::Time` counts picoseconds from power-on (a `u64` lasts about 213 days). Each clock domain converts
its cycles with a `ClockPeriod`, so firmware that reprograms the PLL changes how long a cycle takes, as on
hardware; the clock tree (`periph/clocks.rs`) derives `clk_sys`, `clk_peri`, `clk_usb`, `clk_adc`, `clk_ref` from
the XOSC/ROSC/PLL registers, and the timer ticks from `clk_ref` through the watchdog's tick generator.

**Peripherals never poll.** Each schedules the next moment something happens to it (a timer alarm, an SPI frame
finishing, a UART byte, a PIO step, an ADC conversion, a USB transaction) on the `Scheduler` under a key it owns
(`system::Event`). Scheduling a key that is pending moves it; events at the same time fire in the order they were
scheduled. Counters (timer, PWM, watchdog) are computed lazily from virtual time when read.

## The run loop

`Machine::run_until(end)` (`crates/rp2040/src/machine.rs`):

1. Each core keeps its own time. The loop runs the runnable core that is furthest behind (ties go to the core that
   did not run last) for up to a **quantum** of instructions (16 by default, `--quantum`), never past the next
   event; then fires every event both cores have reached. Peripheral accesses are stamped with the accessing
   core's time. SIO FIFO and spinlock accesses are atomic per access, so larger quanta only shift interleaving.
2. **Batches.** Within a quantum a core runs instructions back to back against a `BatchBus` that reads and writes
   SRAM, ROM and flash directly (generic over the bus so it inlines). A batch ends only where the machine has
   something to do: an access that changed the core's interrupt lines, `clk_sys`, a reset or PSM request; a PPB
   access, `SEV`, sleep, `BKPT`, a fault, a pending exception, the quantum or the time limit. Batching is off
   while anything needs to see each instruction (breakpoints, `instr`/`exc` tracing), which is also how tests get
   a reference run.
3. **Busy-wait skipping** (`Machine::skip_spin`). A quantum in which a core only read memory and the timer (seeing
   the same tick each time), wrote nothing and ended in the state it started in would repeat exactly until the
   timer ticks; such quanta are skipped up to the next event, tick or the end of the run, while the other core
   cannot run. Skipped instructions still count as executed. The ESC board's control loop spends 62 % of its
   instructions in such a loop.
4. **Idle fast-forward.** When both cores sleep (`WFI`/`WFE` with nothing pending) time jumps to the next event,
   bounded by SysTick wake-ups. Embassy firmware spends most of its time there. With nothing scheduled at all the
   run stops with `StopReason::Deadlock`.
5. **PIO** state machines are cycle exact (one instruction, delay or stall cycle per tick of their fractional
   divider) and advanced lazily, skipping delays and machines stalled on something that has not changed. A machine
   polling its own registers, FIFO levels or pins in a loop (the ESC's DShot reply wait) is not ticked through it:
   whole iterations are skipped and the rest replayed when something reads or changes it.

`Machine::skip_spins` and `Pio::skip_loops` switch the two skips off; `crates/rp2040/tests/batching.rs`,
`busy_wait.rs` and the PIO tests compare runs with and without them.

Resets go through the same loop: a watchdog reset or `SYSRESETREQ` is a `ResetRequest` the machine carries out
(`watchdog_reset`, `run_pin_reset`, `power_cycle`, `power_on_at`), with the reset kind reported in
`VREG_AND_CHIP_RESET.CHIP_RESET` and the watchdog's scratch registers kept as on the chip.

## Sessions and robots

`sim_host::Session` wraps a `Machine` with its host side and runs it in 100 µs slices (`Session::slice`): after
each slice it steps the CAN bus and its host endpoints, the Ethernet cable, the RTT reader (draining the up
channels as a debug probe would, decoding defmt with the ELF's table), the UARTs and the USB host; with
`realtime` it sleeps to keep virtual time behind the wall clock.

`sim_host::robot::Robot` runs several sessions to the same time in 100 µs slices; after each slice every bus
arbitrates and delivers what its nodes and endpoints sent, vehicles step, and the acoustic medium learns how far
each board got. Boards do not see each other within a slice, so they can run on worker threads
(`Robot::threads`, `sim_host::parallel`) with identical results, except in an acoustic medium where a hydrophone
can hear another board within the slice.

The CAN bus (`sim_devices::can::CanBus`) arbitrates by identifier, times each frame at the nominal and data rates
(stuff bits estimated), needs an acknowledging node, and models error frames, stuck-dominant buses,
unplugged nodes and floods. Nodes are the CAN controllers of the boards on it (`CanNode`); host endpoints
(`CanEndpoint`: mock bridges, SocketCAN, replay, cannelloni) take frames out and put frames in, with a bounded
transmit queue.

## Peripherals and their modelling status

| Block | Status |
|---|---|
| Cortex-M0+ x2, NVIC, SysTick (core clock or the 1 µs reference tick), SCB, MPU | Modelled (MPU stored, not enforced) |
| Bootrom | The official B1 image runs for real (function table, soft float, flash routines, USB boot); flash routines can be trapped (`--hle-flash`) |
| SRAM, XIP (cache as plain mapping, XIP SRAM), SSI + W25Q16JV flash chip | Modelled; XIP reads only work while the SSI and chip are set up for them |
| RESETS, PSM, VREG_AND_CHIP_RESET, SYSINFO | Modelled |
| CLOCKS, XOSC, ROSC, PLL_SYS, PLL_USB | Modelled (ROSC at a fixed nominal frequency) |
| SIO: CPUID, GPIO, FIFOs, spinlocks, divider, interpolators | Modelled (the divider is immediate) |
| IO_BANK0, PADS_BANK0, IO_QSPI (BOOTSEL) | Modelled: function select, overrides, pulls, edge/level interrupts |
| TIMER, WATCHDOG (tick generator, scratch, reasons) | Modelled |
| DMA | 12 channels, triggers, chaining, rings, byte swap, sniffer, pacing timers, DREQs |
| SPI (PL022) | Master mode, DMA, interrupts; no slave mode |
| I2C (DW_apb_i2c) | Master mode with NAKs, aborts, clock stretching, DMA; no slave mode, 10-bit addressing or general call |
| UART (PL011) | FIFOs, baud timing, interrupts, DMA, loopback, break; modem lines read inactive |
| ADC | Round robin, FIFO, DMA, threshold interrupt, the temperature input, conversion timing |
| PWM | All slices, both counting modes, input modes, wrap interrupts |
| PIO x2 | Full instruction set, FIFOs and joins, side-set, autopush/pull, IRQ flags, EXEC, DMA, pin overrides; cycle exact |
| USB | Device mode with its DPRAM; no host mode, isochronous endpoints or bit-level errors |
| SYSCFG, BUSCTRL, RTC, TBMAN, PADS_QSPI, XIP_CTRL, XIP_AUX | Not modelled: accesses go to the unmodelled fallback (stored, logged; reads return the stored or SVD reset value) |
| Sleep/dormant modes | Not modelled beyond `WFI`/`WFE` |

The unmodelled fallback is on by default and logs the first access to each register (`--unmodelled`), so silent
divergence from hardware shows up.

## External device models

`sim_devices`: MCP2517FD/MCP2518FD (SPI instruction set, RAM, FIFOs, TEF, interrupts, bit timing checks, fault
confinement), W5500 (MACRAW socket 0, PHY link), SHT41, MCP3426/7/8, MS5837-02BA/30BA, SSD1306 (render to PNG and
text), BQ40Z80 (SMBus with PEC, ManufacturerBlockAccess), CD74HC4051, DRV8243 (wake-up handshake), WS2812 (decode),
DShot ESC (bidirectional eRPM, KISS telemetry, motor lag), Dynamixel X-series (protocol 2.0), Hiwonder bus servos,
a USB host controller with CDC ACM, and the fault wrappers `FaultyI2c` and `FaultyAnalog`. The board files'
tables ([boards.md](boards.md)) attach them.

## Adding a peripheral

1. Write the model in `crates/rp2040/src/periph/<name>.rs` from the datasheet chapter: register reads and writes
   with the atomic aliases (`WriteMode`: normal, XOR, SET, CLR), reset values, side effects. Compute counters
   lazily from virtual time; schedule what happens next with a new `system::Event` variant rather than polling.
2. Hook it into `crates/rp2040/src/system.rs`: a field in `System`, its base in the read and write dispatch, its
   `RESETS` bit, its interrupt lines in the IRQ collection, its `Event` in the event handler, `peek`/`poke` for
   debugger access if it holds memory. Registers it does not handle should still fall through to the unmodelled
   fallback so they are logged.
3. Test it at register level in the module (`#[cfg(test)]`), from the datasheet: reset values, aliases, each
   behaviour; then with a program in `test_programs/` or a firmware test in `crates/sim_host/tests/`.
4. Anything a `System` holds must be `Clone` (snapshots) and must not depend on the wall clock or unseeded
   randomness.

## Adding a device model

1. Implement the interface it sits on, in `crates/sim_devices/src/<chip>.rs`:

   | Trait | For | Key methods |
   |---|---|---|
   | `SpiDevice` | A chip on an SPI chip select | `select`, `transfer(mosi, bits) -> miso`, `update`, `can_node` for a CAN controller |
   | `I2cDevice` | A chip on an I2C bus (hardware or bit-banged) | `responds_to(addr)`, `start`, `write`, `read`, `stop`, clock stretching |
   | `PinDevice` | Anything that watches and drives pins (PIO protocols, servos, LEDs, H-bridges) | `watches(pin)`, `pin_changed`, `update` (at times asked for with `ctx.wake_at`) |
   | `AnalogSource` | What drives an ADC input | `voltage(t, pins)` |
   | `UartDevice` | A chip on a UART | `receive(byte) -> reply` |

   Each gets a `DeviceCtx` (current time, set or read SoC pins, `drive_at` a future pin change, `wake_at`, `report`
   to the `device` trace) and provides `clone_box` (and `as_any` where tests need to find it again). Physical
   quantities are `sim_core::analog::Signal`s, so tests and vehicles can drive them; a model that reads inputs
   implements `relink_inputs` (call `Signal::relink` on each), or snapshots would share its inputs with the
   original (`board::tests::a_copy_gets_inputs_of_its_own` checks every kind).
2. Unit-test it against `sim_devices::TestCtx` with the datasheet's transactions.
3. Give it a board file table: a `Deserialize` struct and a field in `sim_host::board::Board`, attached in
   `Board::apply` (`m.sys.attach_spi`, `attach_i2c`, `attach_pin_device`, `attach_analog`), returning its inputs
   through `Inputs::add`. Use `#[serde(deny_unknown_fields)]` for new tables.
4. Wire it on the boards that have it in `boards/overlays/` and rerun `python3 scripts/gen_boards.py`.
5. Add a firmware test that shows the firmware using it, and document the table in [boards.md](boards.md).

## Determinism rules

The same inputs must give the same run, instruction for instruction; tests, `can-diff` of two runs, replay and the
benchmark fingerprints rely on it.

* Never read the wall clock in a model; only `--realtime` pacing in `Session`/`Robot` looks at it, and only to
  sleep.
* Randomness comes from a seed: the acoustic noise is a hash of the environment seed and the sample index, the
  fault injectors carry a seeded `Rng` that is copied with snapshots.
* Iterate in a defined order (`BTreeMap`, vectors), not over `HashMap`s, wherever the order can reach the
  machine.
* Events at the same time fire in scheduling order; keep it that way when changing the scheduler.
* Threads must not change results: robot boards only run in parallel within a slice where they cannot see each
  other, and sweeps run independent snapshots.
* Optimisations must leave behaviour and timing exactly as they were: run the benchmark before and after and
  compare fingerprints.

## Snapshots

Machines are plain data: a snapshot is a deep copy (`Machine::snapshot` and `restore`; `Session::snapshot` and
`Robot::snapshot` return a new session or robot to run). Boxed device models implement `clone_box` (the `clone_box!` macro gives `Clone` for
the boxed trait). What is not copied: the trace sink and host endpoints (bridges, sockets: give the copy its own;
`MockBridge::fork` and `PuddlesBridge::fork` copy a bridge's state). Physical inputs are shared
`Arc<Mutex<f64>>` handles that the device models read through `Signal::Shared`; `Inputs::deep_copy` makes new ones
and a `Relink` map from old to new, and `sim_host::board::relink_inputs` walks every device of the copied machine
(`relink_inputs` on each device trait) to point it at them. `Bench`, `RobotBench` and `Robot` snapshots do this
(`Session::snapshot_with_inputs`); a plain `Session::snapshot` keeps sharing. A robot snapshot also re-links the
copy's hydrophones and projectors to the copy's acoustic medium and its vehicles' paths.

## Performance

```bash
cargo run --release -p sim_test --example bench -- --time 2s --repeat 3 --only esc_board,talos
```

runs the template, ESC and IVCBoard2 images on their boards and the Talos (also on four threads, `talos_mt`) and
IVC2 pool robots for 2 s of virtual time, and prints the best wall time, virtual/wall (above 1 keeps up with
`--realtime`), CPU time, instructions executed, the share skipped in busy-waits, emulated MIPS, and a fingerprint
of everything the run produced (log lines and their times, bridge messages, UART/RTT text, CPU registers, cycle
counts, SRAM). The fingerprint must not change when the emulator is made faster. Wall time is noisy on a shared
machine: compare the best of several repeats, or the instruction counts under `valgrind --tool=callgrind` with
`--repeat 1`.

Measured on a shared 4-core machine (CPU time for 2 s of virtual time): template 0.04 s, ESC board 2.0 s,
IVCBoard2 1.2 s, Talos 5.8 s, IVC2 pool 3.6 s. What is left is mostly the interpreter itself (about 95 host
instructions per emulated one), PIO and peripheral polling on the ESC board, and the acoustic model in the pool;
going further would take a decoded-instruction cache or block-at-a-time execution.
