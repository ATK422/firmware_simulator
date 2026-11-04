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
   cannot run. A loop whose length does not divide the quantum comes back to a state only after a few quanta:
   consecutive quiet quanta are compared with the state at the start of a window that doubles up to 16 quanta
   (`SpinWindow`, Brent's cycle detection), and whole windows are skipped. XOSC reads also keep a quantum quiet
   (`System::xosc_fast_reads`); the skip then ends before STATUS turns stable, so embassy's wait for the crystal
   is skipped too. Skipped instructions still count as executed.
   **Timer waits** (`Machine::skip_timer_wait`, `Machine::skip_timer_waits`). A loop that polls the timer comes back
   to the same state only within a tick (its registers hold what it read), so the skip above stops at every tick,
   1 µs. When such a window closes, `CortexM0Plus::timer_wait` executes the loop abstractly from the core's state
   (`crates/cpu_armv6m/src/timer_wait.rs`): registers are known, the low word of a timer read, or something that
   depends on one; flags are known or predicates on the low word (`Lo >= k`, `Lo == k`); the high word reads as now
   while the low word does not wrap. A conditional branch on a predicate keeps its way while every read stays below
   the value at which it flips (the deadline), and once the abstract state at an address comes back the same, every
   further time round takes the same path. The machine skips whole times round whose reads come before the
   deadline, and lands the core at the loop's start at least one time round before the next event, the end of the
   run or an XOSC change: registers and flags that depend on the timer there must be written before they are read
   and before the first branch the timer decides, so that this time round gives them the values running every one
   would have before anything can see them (the known ones are set). The next quantum is cut to end where the
   quanta of a run without the skip would have (`Machine::quantum_left`): an event an access schedules within a
   quantum fires at its end, and the core takes an interrupt it raises there. Only moves, additions and
   subtractions (with carry), comparisons, a few logic operations and shifts, loads and branches are understood;
   anything else, a store, or a timer value used any other way, and the loop is left to the plain skip. The ESC
   board waits out each 400 µs DShot frame time in such a loop (`while Instant::now() < min_frame_time {}`), which
   was 45 % of its quanta.
4. **Quantum fusing** (`Machine::fuse_quanta`). While the other core cannot run, no SysTick counts and the last
   quantum wrote memory and touched nothing else (the core is computing), the next quantum's first batch may run on
   through up to 64 quanta: the boundaries in between would change nothing, since events are not due before the
   limit and nothing that could wake the other core happens in a batch that touches only memory. A fused batch ends
   after any other access (it may schedule an event or wake the other core) and after a timer read (busy-waits
   poll the timer, and are only found at quantum boundaries); the rest of that quantum runs as usual.
5. **Idle fast-forward.** When both cores sleep (`WFI`/`WFE` with nothing pending) time jumps to the next event,
   bounded by SysTick wake-ups. Embassy firmware spends most of its time there. With nothing scheduled at all the
   run stops with `StopReason::Deadlock`.
6. **PIO** state machines are cycle exact (one instruction, delay or stall cycle per tick of their fractional
   divider) and advanced lazily, skipping delays and machines stalled on something that has not changed. A machine
   polling its own registers, FIFO levels or pins in a loop (the ESC's DShot reply wait) is not ticked through it:
   whole iterations are skipped and the rest replayed when something reads or changes it. A counted loop, which
   comes back with X or Y one less, decremented by its only `JMP X--`/`JMP Y--` and read by nothing else (the
   DShot receivers' clock recovery, up to 12 times round a bit), is skipped up to the iteration that starts with
   the counter at zero (`Pio::skip_counted`). Loops, and machines being watched for one (from 1, 2, 4... up to 64
   ticks, Brent), go on through input changes on pins they do not read (`Pio::loop_inputs`). A block remembers
   each machine's next tick between runs and works out again only those of machines that ticked and stalled ones;
   a run with nothing to tick before its end only moves the cycle (`Pio::fast_run`). A CPU read of a PIO register
   that changes nothing (FSTAT, FLEVEL, IRQ, INTR...) advances the blocks once rather than twice
   (`System::pio_fast_reads`).

`Machine::skip_spins`, `Machine::skip_timer_waits`, `Machine::fuse_quanta`, `Pio::skip_loops`,
`Pio::skip_counted`, `Pio::loop_inputs` and `Pio::fast_run` switch the skips and the fusing off;
`crates/rp2040/tests/batching.rs`, `busy_wait.rs`, `timer_waits.rs`, `fused_quanta.rs`,
`crates/sim_test/tests/fast_paths.rs` and the PIO tests (random programs, with each switch on and off) compare
runs with and without them (`fast_paths.rs` also for `System::pio_fast_reads` and `System::sio_fast_irqs`, which
spares the interrupt-line recomputation after SIO accesses other than to the FIFOs, `System::xosc_fast_reads` and
`Machine::skip_spins`).

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
each board got. Boards do not see each other within a slice, so they can run on several threads
(`Robot::threads`, `sim_host::parallel`) with identical results, except in an acoustic medium where a hydrophone
can hear another board within the slice.

**Threads** (`sim_host::parallel`). For each `Robot::run_until` of at least 8 slices, scoped threads start once
and each runs a fixed group of boards in place: each board sits behind a `Mutex` that only one thread wants at a
time, so nothing moves and there is no `unsafe`. Per slice the calling thread publishes the slice's end and wakes
the workers that have boards (a per-worker slice number), runs its own group, waits for the others (spinning
50 µs, then yielding, then parking until the last worker unparks it), and then does the end of the slice alone,
in the same order as in turn (`robot::end_slice`: buses, Ethernet, devices, RTT and UARTs, vehicles). This costs
about 0.4-0.7 µs per slice. Every board's host time is measured, and every 64 slices `Balance::regroup` spreads
the boards again: for each thread count, costliest board first onto the least loaded thread, counting 3 µs per
slice for each extra thread, taking the best only if it beats the current grouping by 5 %, and keeping boards on
the thread they were on where it can (warm caches). If the slices took more than 90 % of the boards' added-up
cost on the wall clock (the threads did not pay, e.g. the cores are busy with other programs), every board runs on
the calling thread for 8 regroupings, twice as many each time in a row, up to 512. The grouping is kept between
calls. A board stopping for another reason than the slice's end is reported after the slice, lowest index first,
with the others run to the end of the slice; a panic on a worker goes on on the calling thread.

**Running ahead** (`sim_host::runahead`, on by default with threads; `Robot::run_ahead`, `robot --no-run-ahead`).
Slice by slice, every slice waits for its slowest board, and the two ESC boards' heavy slices (their DShot frames)
fall in different slices, so on threads Talos was bound by the sum of the slowest board of each slice. But boards
see each other only through the end of a slice, and through it only in three ways: a bus step that carries a frame
(a step that carries none changes no controller that counts no dominant bits and is not bus-off,
`CanNode::idle_changes_nothing`), a vehicle (it reads the ESCs' motors and writes the pressure inputs of the boards
it carries sensors for), and the order in which host hooks see lines. A frame finishes no earlier than when it
became ready plus its time on the wire (`CanBus::earliest_end`), and endpoints say when they will next hand one in
(`CanEndpoint::idle_until`: the mock bridge and `Monitor` can, SocketCAN and cannelloni cannot). So the boards run
in windows: each runs on alone on its thread, doing the machine side of its end of each slice (devices, and taking
out what RTT and the UARTs hold, `Session::collect`) and noting what its controllers show the bus and its ESCs'
motors, up to the first boundary where any board's frame could finish (an atomic minimum it lowers as soon as it
has a frame ready), an endpoint's next frame, or 64 slices. A board that went past the window's end before it was
told is rolled back to a checkpoint (`Session::checkpoint`: the machine without its flash, which is versioned so
that it is copied only when it changed, plus what the run loop carries, `Machine::run_carry`; one at the window's
first boundary and one after every 1.5 ms of host time) and run again; runs are deterministic, so it gets there
exactly as it would have. Meanwhile the calling thread commits every boundary all boards have gone past: each bus
steps against stand-ins that answer as the controllers did there (one that the step would change panics: it would
mean the bounds are wrong), the vehicles step with the noted motors, and boards whose inputs a vehicle drives (the
camera cage) run there slice by slice rather than ahead; a frame of theirs cuts the window and rolls the others
back. At the window's end the host sides take in what was collected, boundary by boundary in board order, and the
end of the slice is done as usual. A board that stops ends the window at the end of that slice, as on threads.
Boards with a pty, USB/IP, an Ethernet cable, a profiler or a trace sink (VCD, `--trace`), endpoints that cannot
tell when they send, and acoustic media keep the slice-by-slice threads. `crates/sim_host/tests/runahead.rs`
compares Talos frame by frame, line by line (the hooks' order across boards included) and state by state against
taking turns: on 2, 3 and any threads, in uneven pieces, with a vehicle that moves the camera cage's input, with
stops, and with a stress mode (`Robot::run_ahead_stress`) that makes every board ignore where its window ends
until rolled back and cuts every window at a random boundary (thousands of rollbacks);
`crates/sim_test/tests/run_ahead_thrusters.rs` does it with the thrusters spinning and the vehicle moving.

Both run their machines through `Session::run_machine`, which goes through the session's sampling profiler when
it has one (the profiler cuts the run at its sample times; a robot's worker thread takes the board's profiler with
its machine). The waveform writer, the dashboard and the JSON log only observe: trace events (shared through
`sim_core::trace::MultiSink`), the sessions' line and frame callbacks, and state read between slices. See
[tools.md](tools.md).

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

The unmodelled fallback is on by default and counts every access per register, so silent divergence from hardware
shows up: by default (`--unmodelled summary`) the log gets one note at the first unmodelled access and `run` ends
with a table of them all (`robot` with a line per board); `first` warns at each register's first access, `every`
at every access (`LogMode` in code).

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
* Threads must not change results: robot boards only run in parallel where they cannot see each other (within a
  slice, or ahead of each other up to where a frame could reach them, rolled back if they went too far), and
  sweeps run independent snapshots. A `CanEndpoint::idle_until` or `CanNode::idle_changes_nothing` that promises
  too much breaks this; the commit of a window panics when a bus step would change a node it was told would not
  change.
* Observers must not steer the machine: a trace sink only receives events, and only instruction and exception
  tracing turn batched execution off (GPIO and device traces, and so waveforms, leave it on). Cutting a run at
  any time, as the profiler does, must give the same single-core run (`crates/rp2040/tests/busy_wait.rs`).
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

runs the template, ESC and IVCBoard2 images on their boards and the Talos (also on up to two, three, four and six
threads: `talos_t2`, `talos_t3`, `talos_mt`, `talos_t6`, and on three slice by slice without running ahead:
`talos_t3_ls`; `--threads N` sets `talos_mt`'s and prints the grouping)
and IVC2 pool robots for 2 s of virtual time, and prints the best wall time, virtual/wall (above 1 keeps up with
`--realtime`), CPU time, instructions executed, the share skipped in busy-waits, emulated MIPS, and a fingerprint
of everything the run produced (log lines and their times, bridge messages, UART/RTT text, CPU registers, cycle
counts, SRAM). The fingerprint must not change when the emulator is made faster. Wall time is noisy on a shared
machine: compare the best of several repeats, or the instruction counts under `valgrind --tool=callgrind` with
`--repeat 1`.

Fingerprints are per firmware build: new images, or the same source built by another compiler, give new ones
(so do instruction counts). titan_firmware_rs pins its compiler in its `rust-toolchain.toml`, so images built from
the same commit with rustup agree. With the pinned titan_firmware_rs (94a107e, rustc 1.99.0):

| Case | Fingerprint |
|---|---|
| `template` | `cf8e88c45976e6e8` |
| `esc_board` | `7092fc6b0d773de0` |
| `ivc_board2` | `9316e6e8f76e2f2c` |
| `talos`, `talos_mt`, `talos_t2`, `talos_t3`, `talos_t6`, `talos_t3_ls` | `a02da435f15bdb96` |
| `ivc2_pool` | `d2a2e6a73362cf3e` |

Before comparing a change against these, check the images are that build (the bench prints the same fingerprints
for the same images on any machine); when the pin moves, they change and this table with it.

Measured on a shared 4-core machine (CPU time for 2 s of virtual time): template 0.03 s, ESC board 1.35 s,
IVCBoard2 0.55 s, Talos 3.7 s, IVC2 pool 2.15 s. Talos on threads (wall time): 1.6-1.7 s on three or four
(1.2-1.25x real time), 2.4 s on two (both ESC boards share a thread with others there); slice by slice on three it
was 2.5 s.
The ESC board waits out each 400 µs DShot frame time on the timer (skipped across ticks up to its deadline, see
the run loop), so what is left of its time is the frame itself: the control loop polling FSTAT while the PIO
sends the frame and receives the replies. Per 100 µs slice an ESC board takes about 70 µs on average but its frames
fall in different slices from the other ESC board's, so slice by slice the slowest board of each slice adds up to
about 2.4 s however many threads there are: about 0.8x real time. Running ahead takes that floor away: Talos is
then bound by its costliest board, an ESC board at about 1.4 s, plus about 0.2-0.3 s for the windows (430 per 2 s:
the one that is ahead waits for the other at each window's end; reruns after 100-150 rollbacks per ESC board; about
36 µs per checkpoint). Under
callgrind (400 ms, `--repeat 1`) the ESC board executes 2,510 M host instructions, against 3,692 M before these
skips: about 45 % in the interpreter and the run loop's per-quantum work, 20 % in advancing the PIO (half of it
ticking the receivers between their counted loops) and 5 % resolving pins. IVCBoard2 is all interpretation
(1,660 M); there the dispatch's indirect branch, rather than the instructions executed, sets the pace (an
interpreter with fewer instructions per op was slower on the wall clock). At boot, embassy's clock setup polls the
XOSC status register for its 128 ms start-up delay (on the ring oscillator); that wait is skipped as a busy-wait
(a three-instruction loop, so over three quanta).
