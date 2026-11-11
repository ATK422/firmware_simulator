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
| [`sim_host`](../crates/sim_host) | Everything that connects a machine to the world: `Session` (a board plus RTT/defmt, UARTs, CAN, Ethernet, USB host side), `Robot`, board and robot files, mock bridges and the protocol-neutral mock agents (`host_protocol`, the nano-ROS agent in `nanoros`: [nanoros.md](nanoros.md)), SocketCAN, Ethernet/UDP, GDB, ptys, USB tools and USB/IP, CAN logs, `can-diff`, cannelloni, `bridges`, the bootloader upload, fault application, vehicles |
| [`sim_test`](../crates/sim_test) | The test harness: `Bench`, `RobotBench`, `sweep` ([testing.md](testing.md)) |
| [`sim_calib`](../crates/sim_calib) | Calibration: fitting the acoustic, thruster and vehicle models to recordings, replaying pool tests ([calibration.md](calibration.md)) |
| `firmware_simulator` (root) | The command line (`src/main.rs`, `src/host_tools.rs`, `src/calib_cli.rs`) |

The dependency direction is `sim_core` ← `cpu_armv6m`, `sim_devices`, `sim_acoustics`, `sim_physics` ← `rp2040`
← `sim_host` ← `sim_test`, `sim_calib`, CLI. The CPU and the SoC are separate crates so that another machine (an Orin SPE, an
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
   SRAM, XIP SRAM, ROM and flash directly (generic over the bus so it inlines; titan_firmware's C bootloader runs
   from XIP SRAM). A batch ends only where the machine has
   something to do: an access that changed the core's interrupt lines, `clk_sys`, a reset or PSM request; a PPB
   access, `SEV`, sleep, `BKPT`, a fault, a pending exception, the quantum or the time limit. Batching is off
   while anything needs to see each instruction (breakpoints, `instr`/`exc` tracing), which is also how tests get
   a reference run.
   **Decoded blocks and the JIT.** A batch runs instructions from a per-core cache of decoded blocks
   (`crates/cpu_armv6m/src/block.rs`, `CortexM0Plus::decode_blocks`): runs of up to 16 instructions decoded once
   from the memory the bus serves code from, and used while memory still holds the bytes they were decoded from
   (checked at each lookup; a store into the running block ends it). With `CortexM0Plus::jit` (`--jit`, `SIM_JIT=1`;
   off by default) a block that has run whole `jit_threshold` times (300) is compiled to native code with Cranelift
   (`crates/cpu_armv6m/src/jit`; the `jit` cargo feature, on by default, on x86-64 and aarch64; otherwise blocks
   stay interpreted), as a **region**: the block and up to one block it branches or returns to. Compiled code does
   what the block executor does op for op: register and flag operations inline (registers written through to the
   core, flags kept lazily as the comparison that set them); SRAM, flash, ROM and XIP SRAM accesses inline through
   the bus's direct memory windows (an access that went elsewhere when the region was compiled goes through a thunk;
   one that went into a window and no longer does leaves the region at its instruction) (`Bus::direct_memory`, with the marks the bus keeps for them: SRAM writes,
   flash reads through the XIP cache), every other access, multi-word transfer, interworking branch and the
   instructions the block executor leaves to the interpreter through thunks that run the same Rust code; the same
   checks after each access and the same exits. Leaving compiled code at any instruction boundary is invisible:
   the batch goes on exactly there. A region goes on to the next one directly (a tail call through a per-site
   cache) after the checks `run_blocks` makes before running a block whole (steps, budget, trapped addresses) and
   a comparison of the code in memory with what the next region was compiled from. Regions are compiled on
   worker threads (up to three, half the host's cores) while the core goes on interpreting, and kept for the life of
   the process, shared by every core (`jit/service.rs`), keyed by the code they were compiled from and the shape of the bus's windows:
   copies of a core (snapshots, a robot's boards rolled back) and boards running the same firmware compile a region
   once. With the edge map on, blocks run interpreted. The differential tests (`src/difftest`, random programs
   against single steps and decoded blocks, compiled on the first run or after three) and
   `crates/sim_test/tests/engines.rs` (IVCBoard2 in lockstep) compare every way of running.
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
4. **Quantum fusing** (`Machine::fuse_quanta`). While the other core cannot run, no SysTick counts the core clock
   and (where busy-waits are skipped, below) the last quantum was not quiet (it wrote memory or reached a
   peripheral: the core is computing or polling), the next quantum's first batch may run on through up to 64 quanta: the boundaries in between would change
   nothing, since no event is due before the limit and nothing wakes the other core. A fused batch ends after an
   access that would make a boundary do something (`BatchBus::fuse`): one that moves the next event (or, when the
   end of the run is the limit, brings one before it), changes either core's interrupt lines, or needs the machine
   (a clock, reset or PSM change); accesses that leave all that alone, such as polling a UART or SPI status
   register or feeding the watchdog, it runs through. Where busy-waits are skipped (the core's SysTick counts
   nothing), it also ends at a timer or XOSC read once it has only read memory for two quanta's worth of cycles
   (busy-waits poll those, and are only found at quantum boundaries), and the quanta after that are not fused; the
   rest of that quantum runs as usual. Fused batches used to end at every access other than to memory, so the C
   IVCBoard2's main loop, which polls peripherals and the timer and feeds the watchdog between its FFTs, ran in
   16-instruction quanta (3 M per 2 s of virtual time), each costing about 600 host instructions of run-loop work.
   **SysTick on the reference clock** (CLKSOURCE clear: the watchdog's 1 µs tick, as pico-sdk and so the C firmware
   set it up) is fed lazily: when a batch starts, when the core is asked whether it wakes, and within a batch just
   before an instruction accesses a SysTick register (`Bus::take_reference_ticks`), so that every access sees the
   counter as if fed tick by tick. A batch ends at the first instruction boundary from a picosecond before the
   tick at which the counter pends its exception (a fused one also at the other core's), where the exception is
   then taken as before, and after an access that changes the tick generator. Batches used to end at every tick
   and fusing was off, which made the C IVCBoard2 twice as costly per cycle as the Rust one.
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

7. **Debugger stops** (`StopReason::is_debug_stop`: a breakpoint, `BKPT`, a watchpoint, a position reached with
   `Machine::stop_at`) return in the middle of a quantum with what is left of it noted (`Resume`: the core, the
   instructions left, the quantum's time limit); the next `run_until` goes on with that quantum before anything
   else, so a run that stopped is the same as one that did not (`crates/rp2040/tests/debug_stops.rs`, on both
   cores). `Session::run_until` likewise finishes the slice a stop interrupted before doing the slice's end. A
   **position** is a core and the time on its clock when one of its steps (an instruction or an exception entry)
   starts; `stop_at[core]` stops before the first step at or after a time, with that core's batches cut there and
   its busy-wait skipping and fusing off. Watchpoints (`Machine::watchpoints`) run the cores one instruction at a
   time through a bus that checks every data access, and stop the core before its next step. Code coverage
   (`Machine::coverage`) does too, marking each executed instruction.

`CortexM0Plus::decode_blocks` and `CortexM0Plus::jit` (`--jit`/`--no-jit`, `SIM_JIT`, the bench's `--jit`/`--no-jit`)
choose how a core's batches run; `Machine::skip_spins`, `Machine::skip_timer_waits`, `Machine::fuse_quanta`, `Pio::skip_loops`,
`Pio::skip_counted`, `Pio::loop_inputs` and `Pio::fast_run` switch the skips and the fusing off (and in the water
`Hydrophone::skip_quiet` and `Hydrophone::lanes` theirs, see *Acoustic media* below: the hydrophone unit tests and
`crates/sim_host/tests/acoustic_threads.rs` compare runs with them on and off);
`crates/rp2040/tests/batching.rs`, `busy_wait.rs`, `timer_waits.rs`, `fused_quanta.rs`,
`crates/sim_test/tests/fast_paths.rs` and the PIO tests (random programs, with each switch on and off) compare
runs with and without them (`fast_paths.rs` also for `System::pio_fast_reads` and `System::sio_fast_irqs`, which
spares the interrupt-line recomputation after SIO accesses other than to the FIFOs, `System::xosc_fast_reads` and
`Machine::skip_spins`, and for `Machine::fuse_quanta` on the C IVCBoard2 too when `TITAN_FIRMWARE_DIR` is set).
`fused_quanta.rs` has a two-core program whose fused batches poll peripherals, feed the watchdog, move the next
event and wake the other core; it fails if a fused batch runs through any of the last two.

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
(`Robot::threads`, `sim_host::parallel`) with identical results; in an acoustic medium only as far apart as the sound
between them takes to arrive (see *Acoustic media* below).

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
(`CanEndpoint::idle_until`: the mock bridge, `Monitor` and replays can), or are *live* (`CanEndpoint::live`:
SocketCAN, cannelloni, the upload link: they cannot tell, but every frame they hand in is ready at the step that
reads it, as it arrived from outside since the last one). So the boards run
in windows: each runs on alone on its thread, doing the machine side of its end of each slice (devices, and taking
out what RTT and the UARTs hold, `Session::collect`) and noting what its controllers show the bus and its ESCs'
motors, up to the first boundary where any board's frame could finish (an atomic minimum it lowers as soon as it
has a frame ready), an endpoint's next frame, or 64 slices. A board that went past the window's end before it was
told is rolled back to a rollback point (`Session::rollback_point`: the board's machine state, see
[Checkpoints](#checkpoints-one-machine-state-for-both-kinds), with the debugger's positions and the code coverage;
one at the window's first boundary and one after every 1.5 ms of host time) and run again; runs are deterministic,
so it gets there exactly as it would have. Meanwhile the calling thread commits every boundary all boards have gone past: each bus
steps against stand-ins that answer as the controllers did there (one that the step would change panics: it would
mean the bounds are wrong), the vehicles step with the noted motors, and boards whose inputs a vehicle drives (the
camera cage) run there slice by slice rather than ahead; a frame of theirs cuts the window and rolls the others
back. At the window's end the host sides take in what was collected, boundary by boundary in board order, and the
end of the slice is done as usual. A board that stops ends the window at the end of that slice, as on threads.

*Live inputs.* A live endpoint is read where taking turns reads it, at every boundary's bus step, which running
ahead does where it commits the boundary; after the step, the window is cut at the first boundary where what it
handed in could finish (`CanBus::earliest_queued_end`, at least the next one), and the boards that ran past it are
rolled back, so they see the frame where taking turns would have. A host frame therefore enters the bus at a
well-defined virtual time, the boundary whose step read it, and never in the past of a board that keeps what it
did. What makes a run differ from another is only when frames arrive (which step reads them); given the same
arrivals, taking turns, slice by slice on threads and running ahead give the same run. `canlog::Arrivals`
(`robot --arrivals-log`) logs the arrivals to the nanosecond and `canlog::arrivals_replay` (`--replay-arrivals`)
hands them in again at the same steps, so a run with real host tools can be repeated exactly. Pseudo terminals
(UART, USB CDC ports 0 and 1) are read the same way, by the board itself: `Session::collect` reads them at each
boundary where polling would, injects what it got, and keeps it until it is delivered; a rollback gives it back
(`Session::unread_from`) to be read first, so nothing is lost or reordered, and output reaches the terminal once
final. In real time with a live input (`runahead::LIVE_LEAD`), a window ends at most 2 ms of virtual time ahead
of the wall clock (where taking turns lets virtual time get before it sleeps), sleeping first while the robot is
more than 1 ms ahead, so frames are read, and the boards' frames go out, about when they are due; behind the wall
clock the windows are as long as without. Real-time pacing and its lag warnings run at each window's end. The
sensor boards of an external vehicle (moved only by `POST /api/vehicle`, between runs) need not follow it and run
ahead too.

*Observers.* Trace sinks (VCD, `--trace`, except instruction tracing, which runs slice by slice) get a board's
events through a buffer (`HeldTrace`) that a rollback truncates and each window's end passes on; profilers hold
their samples (`Profiler::hold`, `mark`, `back_to`, `commit`) the same way: both see exactly the committed run.
Boards with USB/IP or an Ethernet cable (their host sides answer the board at once: ARP, the mock bridge; rolling
them back would need snapshots of the endpoints, and Puddles is a single board anyway), endpoints that are
neither live nor can tell when they send keep the slice-by-slice threads. `crates/sim_host/tests/runahead.rs`
compares Talos frame by frame, line by line (the hooks' order across boards included) and state by state against
taking turns: on 2, 3 and any threads, in uneven pieces, with a vehicle that moves the camera cage's input, with
stops, and with a stress mode (`Robot::run_ahead_stress`) that makes every board ignore where its window ends
until rolled back and cuts every window at a random boundary (thousands of rollbacks); with live endpoints (a
mock bridge behind `socketcan::Live`, frames at chosen times, frames from a host thread on the wall clock logged
and replayed), a host frame landing in boards' windows, in real time (no read more than the lead ahead of the
wall clock), the nereus setup (`robots/talos_nereus.toml`: live host sides, an external vehicle posted between
runs), and with traces and profiles; `crates/sim_host/tests/robot_usb.rs` with a board fed from a terminal;
`crates/sim_test/tests/run_ahead_thrusters.rs` with the thrusters spinning and the vehicle moving.

*Acoustic media.* Boards in water see each other through it too: a hydrophone reads another board's projector at
the time the sound left it, the time it arrives less each path's delay. A projector's waveform is computed only
where its board has run past (`sim_acoustics::medium`), so boards that stay less than the shortest delay ahead of
where all of them were read exactly what taking turns gives them. `acoustic::Lookahead` bounds that delay over a
span of time: the shortest path (direct or reflected) between each hydrophone and every other board's projector at
the span's start, less 2.5 ms before it (the hydrophone interpolates its paths over 1 ms blocks and rolls back 1 ms
after a gap) and what both ends' greatest speeds can close until 2.5 ms after it (Doppler and moving vehicles; an
image of a moving projector moves as fast), less 50 µs. `Robot::run_until` runs a robot in water in pieces of up
to a second (`MEDIUM_PIECE`); a piece runs on threads where the bound is at least a slice through it, and running
ahead cuts each window to it (the pool's 5 m: 33 slices). Each board tells the medium how far it got after each of
its slices, as taking turns does; a rollback point notes where its projectors' drives were (`Medium::mark`), and a
rollback takes them back (`Medium::back_to`: the level changes since are forgotten; none can have been folded into
samples). A hydrophone leaves out of its sum, over the steps it computes at once, every origin that gives exactly
zero at every time its paths read (`Source::silent`: a pinger between pulses; `WaveReader::silent`: a projector
whose samples there are all zero, not driven or its ringing below the smallest `f32`): the sum starts at +0, is
never -0, and adding ±0 leaves it as it is, so `Hydrophone::skip_quiet` changes no sample (tested bit for bit, and
on the pool while a packet crosses). Between transmissions this takes away what hearing the other board's 63 paths
costs, which was about as much as the board itself.

*The path sum in lanes* (`sim_acoustics::lanes`, `Hydrophone::raw_lanes`; `Hydrophone::lanes`, on by default).
While a projector transmits, each step a hydrophone computes sums the 63 paths of the pool for it: under callgrind
(2 s of `ivc2_pool_busy`, a packet crossing from 0.37 s on) the hydrophones took 19.7 % of the pool's host
instructions summed one path at a time (5.27 G of 26.8 G, against 1.80 G in the quiet pool: 3.5 G, 13 %, was the
other board's paths). Floating-point addition is not associative, so a step's paths must be added in the order
`Hydrophone::raw` adds them; what is independent is each path's term, its gain times the origin's pressure at the
step's time less its delay (with the delay and the gain interpolated across the block, the sample before that time
and the fraction past it, and the two samples interpolated). So the taps are kept per origin in lanes of eight
(`WideTaps`: delays and gains at the block's start and their change over it, as `raw` computes them; an origin's
last lanes padded with copies of its last tap, computed and never added), each step computes eight terms at a
time with the operations `raw` does, in the same order, in loops over fixed-size arrays that LLVM vectorizes on the
baseline targets (SSE2 on x86-64, NEON on aarch64: two `f64`s per instruction; the kernels in `lanes` take
references, which tells LLVM their output does not overlap their input), and adds them to the step's sum one after
the other. Nothing asks for a fused multiply-add (Rust never contracts `a * b + c`; the x86-64 code has none even
built for `x86-64-v3`, the aarch64 code none) and nothing sets flush-to-zero, so subnormals are computed as IEEE says
on both. `f64::floor` has no SSE2 instruction (each was a call into the runtime, the costliest part of the sum one
path at a time): `lanes::floor_small` computes it exactly as `(x + 2^52) - 2^52`, less one if that rounded up, for
the `0 <= x < 2^52` the times are, and `lanes::whole` gets the sample index from the same addition's bits. A
projector's samples are read eight times at once (`WaveReader::pressure_lanes`) from the slices of its sample deque
where every one read is computed and none is before the first kept; otherwise (a time before zero, beyond what is
known, not a number) one time after the other as `pressure` reads them. Computing up to the latest sample first
changes nothing, as a sample never changes once computed and this computes nothing further than reading in turn
does; only the forgetting of samples older than the projector's second of history could tell the order of reads
apart, so the lanes are used only while a projector's paths in a block are less than 0.99 s apart in delay
(`Hydrophone::lanes_ok`; otherwise one path at a time). The hydrophone alone, hearing a projector transmit over the
pool's 63 paths at 500 kS/s (`cargo run --release -p sim_acoustics --example hydrophone_bench`), runs 2.2-2.3x as
fast (0.28 s for a second of sound, from 0.65 s; 63 ns of each 1 µs step are the receive chain and the noise); the
pool while a packet crosses executes 10 % fewer host instructions (24.0 G) and takes 1.41 s rather than 1.56 s on
one thread, the quiet pool 3.6 % fewer (its hydrophones check eight paths at a time for silence) and 1.38 s rather
than 1.42 s. AVX2 lanes (four `f64`s) were tried by building for `x86-64-v3`: 10 % more on the hydrophone alone, not
worth runtime detection and `unsafe`. The unit tests compare `raw_lanes` with `raw` bit for bit (`f64::to_bits`)
over random taps (every count from none to more than eight lanes, delays and gains changing over the block, zeros,
subnormal and huge gains), random projector samples (zeros, subnormals, reads before the first sample, after the
last known one, before time zero, not a number, across the deque's wrap), sources of each kind, quiet origins and
block edges; all-subnormal and all-zero terms; and every ADC sample of hydrophones hearing a projector driven in
bursts, a pinger, noise and shrimp, with fixed and moving ends (1.5 m/s), uneven sampling, gaps and prerolls, with
the silent origins left out or not, lanes on and off. Adding a chunk's terms in reverse, a `mul_add` in the delays
or a gain multiplied out each make them fail. `bench --no-lanes` runs the robots' hydrophones one path at a time:
every fingerprint is the same.

The board driving a projector and the hydrophones reading it hold different locks
(`medium::ProjectorState`), and a hydrophone holds the medium only to refresh its paths. Boards closer than a
slice's sound (15 cm), paths longer than 0.4 s and transducers on a simulated vehicle (`Trajectory::Live`, known
only as the vehicle moves) take turns. `crates/sim_host/tests/acoustic_threads.rs` compares the IVC pool on threads,
running ahead, slice by slice and with forced rollbacks against taking turns (CAN frames, lines in order across
boards, what the bridges received, every ADC sample of the listening hydrophone, machine state) while a packet
crosses the water, with both vehicles closing at 1.5 m/s each, the whole byte exchange, and the C IVCBoard2 pool.

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

**Checkpoints for time travel** (`sim_host::timetravel`) restore in place instead, so that whoever holds the live
session, robot, inputs and bridges keeps holding them: `Session::checkpoint` and `Session::rewind_to` (the machine
via `Machine::snapshot` and `restore`, the host side's RTT position, a defmt frame in progress, the CAN bus, the
Ethernet cable; the logs and text collected are cut back, and given back from a kept copy when going forward), a
robot's buses, vehicles (with their transducers' paths) and the medium's contents, and the inputs' values. The
flash is shared between checkpoints while unchanged (see below), the ROM, the SRAM (in 4 KB pieces) and the code
coverage map with the checkpoint before where they are equal. `TimeTravel` keeps them in a thinned ring within a memory budget, with a log of
everything from outside (runs asked for and where they stopped, debugger writes, inputs, and per call what each host
endpoint and pty gave, through `TappedCan`, `TappedEth` and the ptys' taps), and replays from the nearest one: the
taps give the recorded frames again, hooks, trace sinks and the profiler are put aside. Host changes it does not
log (faults, poses) are marked with `TimeTravel::changed`, a checkpoint replays never go through.

### Checkpoints: one machine state for both kinds

A robot running ahead and time travel both keep a board's machine as it was (`MachineState` in
`sim_host::session`): `Machine::snapshot` without the flash, what the run loop carries (`Machine::run_carry`, so
that the machine goes on exactly as it did, its statistics included), the lockup count, and the flash apart. The
flash is a `sim_core::versioned::Versioned` array: every mutable borrow of it (an SSI or IO_QSPI access, a trapped
flash function, `poke`, a host writing it) gives it a new, globally unique version, so the session keeps one copy
(`Session::keep_flash`) that every checkpoint of either kind shares while the version is the same (or, taking that
copy's version, while the contents are), and a restore copies it back only when the live flash's version differs.
The two kinds differ in what else they keep and how long they live:

| | Rollback point (`Session::rollback_point`, `roll_back`) | Time-travel checkpoint (`Session::checkpoint`, `rewind_to`) |
|---|---|---|
| Taken | in the hot loop of a window, every 1.5 ms of host time per board, dropped at the window's end | every `--checkpoint-every` of virtual time, kept in a thinned ring within a budget |
| SRAM | in the snapshot (one 264 KB copy: cheapest to take and to put back) | apart, in 4 KB pieces shared with the checkpoint before |
| Host side | nothing: the host side takes in what was collected only once the window is committed | RTT position, defmt frame in progress, text collected, CAN bus, Ethernet cable |
| Debugger positions (`stop_at`) | kept: a rolled-back board stops where it would have | not kept: replays arm their own |
| Code coverage | kept: what was rolled back counts as not run | kept (shared while unchanged): coverage follows the timeline, as the logs do |

**Time travel and running ahead together.** The committed run is the reference: everything is as taking turns
would have done it, whatever ran ahead and was rolled back. So:
* *Checkpoints are taken at committed boundaries.* `TimeTravel` cuts live runs into pieces that end where a
  checkpoint is due (`Rewindable::run_until_after`); a robot running ahead goes on to the end of the window that
  is running then (`Robot::run_until_after`: where the boards meet anyway, at most 64 slices on), so recording
  does not cut windows short, and checkpoints come every `--checkpoint-every` or up to a window later. When it
  returns every board is committed and every host side has taken in what it collected. A recorded robot runs
  ahead with the same results as recording slice by slice (`crates/sim_host/tests/runahead.rs`,
  `recording_for_time_travel_runs_ahead_the_same`).
* *External inputs are recorded per bus step.* The taps (`TappedCan`) record what an endpoint gave on each call;
  only the commit steps the buses, once per boundary in order as taking turns does, and rolled-back boards never
  call an endpoint, so the calls and what they gave are the same either way. A tapped endpoint can tell when it
  will next send (`CanEndpoint::idle_until`) if the endpoint itself can: live, the endpoint's own answer; in a
  replay, the time its next recorded frames were asked for (or, once the replay reaches the frontier, the
  endpoint's own answer, whichever is first). So replays run ahead too. A tapped live endpoint (SocketCAN,
  cannelloni) is live as its endpoint is (`CanEndpoint::live`): what it gave was ready at the step that read it,
  and so it is replayed; it tells that a frame is waiting (`CanEndpoint::waiting`) as the endpoint does, and in a
  replay from the recording. The nereus setup (`--can`, `--dashboard`, which records by default) thus runs ahead
  while recorded and in replays (`the_nereus_setup_runs_ahead_as_it_runs_in_turns`,
  `recorded_live_endpoints_run_ahead_and_replay_the_same`).
* *Debugger stops run in turns.* While GDB is attached to one of its boards (it stops at a `BKPT`), or a
  breakpoint, watchpoint or position is armed on any board, a recorded robot runs its boards in turns on the
  calling thread, as GDB without time travel always did, so stops are reported exactly as before (on threads the
  other boards would run on to the end of the slice, and a replay taking turns to the same stop would not have them
  there). The rest of the time it runs ahead. A history recorded running ahead is the same history, so
  reverse-continue and reverse steps on it give the states fresh robots taking turns have
  (`crates/sim_test/tests/timetravel.rs`, `a_robot_that_ran_ahead_goes_back_whole`).
* *Coverage counts the committed run.* Rollback points keep the coverage map and a rollback puts it back, so the
  coverage of a robot running ahead equals that of taking turns, rollbacks forced or not
  (`coverage_is_the_same_running_ahead`; `runahead::tests::rolled_back_execution_is_not_covered`). Time travel puts
  the map back with the rest of the board, so after going back it is that of a fresh run to there, and going forward
  again gives the rest back (`crates/sim_host/tests/coverage.rs`, `coverage_goes_back_with_time_travel`).

## Performance

```bash
cargo run --release -p sim_test --example bench -- --time 2s --repeat 3 --only esc_board,talos
```

runs the template, ESC and IVCBoard2 images on their boards and the Talos (also on up to two, three, four and six
threads: `talos_t2`, `talos_t3`, `talos_mt`, `talos_t6`, and on three slice by slice without running ahead:
`talos_t3_ls`; `--threads N` sets `talos_mt`'s and prints the grouping; `talos_live_t3` and `talos_live_t3_ls`
with the bridge behind a live endpoint, as on vcan; `nereus`, `nereus_t3` and `nereus_t3_ls`: the nereus setup)
and IVC2 pool robots (`ivc2_pool`, and on two threads `ivc2_pool_t2`; `ivc2_pool_busy` and `ivc2_pool_busy_t2`
the same while a packet crosses the water the whole run; with `TITAN_FIRMWARE_DIR` set, the C
IVCBoard2 alone, `ivc_board2_c`, and two of them in the pool, `ivc2_pool_c` and `ivc2_pool_c_t2`; skipped
otherwise) for 2 s of virtual time, and prints the best wall time, virtual/wall (above 1 keeps up with
`--realtime`), CPU time, instructions executed, the share skipped in busy-waits, emulated MIPS, and a fingerprint
of everything the run produced (log lines and their times, bridge messages, UART/RTT text, CPU registers, cycle
counts, SRAM). The fingerprint must not change when the emulator is made faster. Wall time is noisy on a shared
machine: compare the best of several repeats, or the instruction counts under `valgrind --tool=callgrind` with
`--repeat 1`. `--stats` prints what running ahead did in each robot case; `--no-lanes` has the hydrophones sum
their paths one at a time (the fingerprints must not change).

Fingerprints are per firmware build: new images, or the same source built by another compiler, give new ones
(so do instruction counts). titan_firmware_rs pins its compiler in its `rust-toolchain.toml`, so images built from
the same commit with rustup agree. With the pinned titan_firmware_rs (94a107e, rustc 1.99.0):

| Case | Fingerprint |
|---|---|
| `template` | `cf8e88c45976e6e8` |
| `esc_board` | `7092fc6b0d773de0` |
| `ivc_board2` | `9316e6e8f76e2f2c` |
| `talos`, `talos_mt`, `talos_t2`, `talos_t3`, `talos_t6`, `talos_t3_ls`, `talos_live_t3`, `talos_live_t3_ls` | `a02da435f15bdb96` |
| `nereus`, `nereus_t3`, `nereus_t3_ls` | `081433a1925d555f` |
| `ivc2_pool`, `ivc2_pool_t2` | `d2a2e6a73362cf3e` |
| `ivc2_pool_busy`, `ivc2_pool_busy_t2` | `38cd81918232caa8` |

The C cases' fingerprints are those of your titan_firmware build; for the `blake/ivc` IVCBoard2 these were measured
with (built in the Nix shell), `ivc_board2_c` is `ccb6914471f62e02` and `ivc2_pool_c`, `ivc2_pool_c_t2`
`cd8b837e56f153aa`.

Before comparing a change against these, check the images are that build (the bench prints the same fingerprints
for the same images on any machine); when the pin moves, they change and this table with it.

Measured on a 4-core machine (best wall time of three for 2 s of virtual time, release profile; CPU time is the
same where one thread runs): template 0.02 s, ESC board 1.29 s (1.55x real time), IVCBoard2 0.50 s (4x), the C
IVCBoard2 0.51 s (3.9x; 0.665 s while fused batches ended at every peripheral access, see the run loop, and 1.4 s,
1.4x, while SysTick's reference ticks ended every batch and its bootloader ran from XIP SRAM through the general bus
path), Talos 3.5 s (0.57x; 3.9 s before), the IVC2 pool 1.41 s in turn (1.4x) and 0.94-0.96 s on two threads
(2.1x; 2.17 s, 0.92x, when pools took turns and heard every silent path), the pool of C IVCBoard2s 1.34 s in turn
and 0.99-1.0 s on two threads (2x; it was 3.82 s, 0.52x, then 1.66 s and 1.18 s). Talos on threads (wall time):
1.6 s on three or four (1.2-1.25x real time), 2.3 s on two (both ESC boards share a thread with others there);
slice by slice on three 2.35-2.4 s. nereus_t3 1.7 s (1.17x).
The ESC board waits out each 400 µs DShot frame time on the timer (skipped across ticks up to its deadline, see
the run loop), so what is left of its time is the frame itself: the control loop polling FSTAT while the PIO
sends the frame and receives the replies. Per 100 µs slice an ESC board takes about 70 µs on average but its frames
fall in different slices from the other ESC board's, so slice by slice the slowest board of each slice adds up to
about 2.4 s however many threads there are: about 0.8x real time. Running ahead takes that floor away: Talos is
then bound by its costliest board, an ESC board at about 1.4 s, plus about 0.2-0.3 s for the windows (430 per 2 s:
the one that is ahead waits for the other at each window's end; reruns after 100-150 rollbacks per ESC board; about
36 µs per checkpoint). Under
callgrind (400 ms, `--repeat 1`) the ESC board executes 2,408 M host instructions, against 3,692 M before these
skips (2,532 M before fused batches ran through peripheral accesses): about 45 % in the interpreter and the run
loop's per-quantum work, 20 % in advancing the PIO (half of it ticking the receivers between their counted loops)
and 5 % resolving pins. IVCBoard2 is all interpretation (1,605 M); there the dispatch's indirect branch, rather
than the instructions executed, sets the pace (an interpreter with fewer instructions per op was slower on the wall
clock). Its hydrophone ADC (500 kS/s into SRAM through DMA while it captures) costs little: over 2 s, 200 k
events of which ADC conversions are most, 2.9 % of the host instructions in firing them (the conversion, the DMA
transfer, the interrupt lines), against 93 % in batches, so the ADC and DMA are not made lazy as SysTick is. The C
IVCBoard2 was the same 2.9 % for its ADC and DMA (it samples at about 93 kS/s), but 3.1 M batches for 78 M
instructions: its main loop polls the UART, SPI and timer and feeds the watchdog, which ended every fused batch,
so it ran in 16-instruction quanta with about 600 host instructions of run-loop work each (18 % of its time in the
run loop itself, 4.6 % in recomputing interrupt lines after its peripheral reads). Fused batches now run through
such accesses (see the run loop): 12.1 G host instructions for 2 s became 9.8 G, 3,836 M for its first 400 ms
(boot, mostly) against 5,178 M. At boot, embassy's clock setup polls the
XOSC status register for its 128 ms start-up delay (on the ring oscillator); that wait is skipped as a busy-wait
(a three-instruction loop, so over three quanta).

### Build profiles

`release` (what the tests, CI and the commands above use) keeps cargo's 16 codegen units and no LTO, so that a
change to one crate relinks quickly. Two optional builds are faster, and run exactly the same (same fingerprints):

* `cargo build --profile fast` (binaries in `target/fast/`): whole-program (fat) LTO in one codegen unit. The bench
  runs 3-7 % faster than release (IVCBoard2 0.55 to 0.51 s, the C IVCBoard2 0.665 to 0.635 s, the ESC board 1.42 to
  1.37 s, Talos 3.87 to 3.75 s, the IVC2 pool 1.50 to 1.43 s); a clean build takes twice as long (`firmware_simulator`
  about 125 s against 62 s on 4 cores, the bench another 66 s against 35 s) and every link redoes the LTO, which is
  why the test suite's release profile does not use it. Thin LTO in one codegen unit gained 1-2 %,
  `panic = "abort"` nothing on top of fat LTO (and the parallel runs and the scenario runner catch panics, so it is
  not an option), `-C target-cpu=native` nothing (and is not portable: measure it on your machine if you like, with
  `RUSTFLAGS="-C target-cpu=native" cargo build --profile fast`).
* `scripts/build_pgo.sh` (needs `rustup component add llvm-tools` and the titan_firmware_rs images): the `fast`
  profile with profile-guided optimisation, trained by running the bench for 1 s of virtual time per case. Against
  release: IVCBoard2 0.46 s (16 % faster), the C IVCBoard2 0.525 s (21 %), the ESC board 1.25 s (12 %), Talos
  3.39 s (12 %), the IVC2 pool 1.33 s (11 %). The script takes about 6 minutes (an instrumented build, the
  training, the final build). The profile is only as good as the code it was trained on: rerun it after changing
  the emulator.
