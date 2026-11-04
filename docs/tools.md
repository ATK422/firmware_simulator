# Developer tools: profiler, coverage, waveforms, dashboard, MCP server, JSON logs

Tools for looking inside a run, on `run` and `robot` alike: a sampling profiler of the firmware, code coverage,
VCD waveforms of the pins, a live dashboard in the browser (with rewinding), an MCP server that gives coding agents
the same as structured tools, and a JSON lines log for scripts. None of them changes what a
single-core program does, and none costs anything per instruction: they cut runs at chosen times, or read trace
events and state between slices. How to switch them on is in [running.md](running.md#profiles-waveforms-dashboard-and-json-logs)
and [robots.md](robots.md#traces-profiles-waveforms-and-the-dashboard); this page covers what they produce and how
they work.

- [At a glance](#at-a-glance)
- [Profiler](#profiler)
- [Code coverage](#code-coverage)
- [Waveforms (VCD)](#waveforms-vcd)
- [Dashboard](#dashboard)
- [MCP server](#mcp-server)
- [JSON log](#json-log)
- [Robot traces](#robot-traces)
- [To verify locally](#to-verify-locally)

## At a glance

| Tool | `run` | `robot` | Output | Code |
|---|---|---|---|---|
| Profiler | `--profile FILE` | `--profile-dir DIR` | Functions by time per core, and folded stacks for flame graphs | `sim_host::profile`, `unwind`, `demangle` |
| Coverage | `--coverage FILE` | `--coverage FILE` | lcov tracefile of the lines and functions that ran, a table per crate; `scenario --coverage DIR` | `rp2040::coverage`, `sim_host::coverage` |
| Waveforms | `--vcd FILE` | `--vcd FILE` | VCD of the GPIOs, optionally with decoded traffic | `sim_host::vcd` |
| Dashboard | `--dashboard ADDR` | `--dashboard ADDR` | A live page and JSON API, with fault, input, pause and rewind controls | `sim_host::dashboard`, `dashboard.html`, `sim_host::timetravel` |
| MCP server | `firmware_simulator mcp` | | Tools for coding agents: run the emulator, read state, logs and CAN frames, inject faults, set inputs, pause, rewind | `src/mcp/` |
| JSON log | `--log-json FILE` | `--log-json FILE` | One JSON object per log line, UART line, CAN frame, fault | `sim_host::jsonlog` |
| Traces | `--trace KINDS` | `--trace KINDS`, `--trace-board` | Text lines per event ([running.md](running.md#traces)) | `sim_core::trace` |

They combine: `sim_core::trace::MultiSink` passes each trace event to every sink that wants its category, so
`--trace`, `--vcd` and the dashboard can share a machine.

## Profiler

```bash
firmware_simulator run $FW/esc_board --board boards/mk2/esc_board.toml --mock-bridge 2 --time 3s --profile esc.txt
inferno-flamegraph < esc.folded > esc.svg          # or: flamegraph.pl esc.folded > esc.svg, or load it in speedscope
firmware_simulator robot robots/talos.toml --mock-bridge internal:1 --time 3s --profile-dir prof/
```

| Option | Default | What it does |
|---|---|---|
| `--profile FILE` (`run`) | | Writes the table to `FILE` and the folded stacks next to it (`FILE` with the extension `.folded`) |
| `--profile-dir DIR` (`robot`) | | Writes `DIR/BOARD.txt` and `DIR/BOARD.folded` for every board |
| `--profile-interval T` | `10us` | Mean virtual time between samples; each gap is jittered uniformly by ±50 %, so periodic firmware work does not alias with the samples |

### Reading the table

```
# esc_board: 3.000 s of virtual time, a sample every 10.0 µs (jittered ±50%)

## core 0: 300111 samples: running 8.2%, sleeping (WFI/WFE) 91.8%
   in exception handlers: TIMER_IRQ_0 0.5%, PIO1_IRQ_0 0.1%, DMA_IRQ_0 0.1%, IO_IRQ_BANK0 0.0%, ADC_IRQ_FIFO 0.2%

  self%  total%  of-run%  samples  function
   4.28    4.29    52.30    12842  <embassy_executor::raw::TaskStorage<esc_board::____embassy_main_task::...>>::poll
   1.08    1.14    13.17     3233  <embassy_time_queue_utils::Queue>::next_expiration
```

| Column or line | Meaning |
|---|---|
| `running`, `sleeping (WFI/WFE)` | Shares of the core's samples; `locked up` and `held off` (core 1 before launch without a bootrom, a reset) appear when there are any |
| `in exception handlers` | Running samples by active exception (IPSR), named (`SysTick`, `TIMER_IRQ_0`, ...) |
| `... of the stacks could not be walked to the root` | Stacks rooted at `[unknown]` ([below](#stack-walks)) |
| `self%` | Samples with the function at the top of the stack, of all the core's samples |
| `total%` | Samples with the function anywhere on the stack (callees included), of all samples |
| `of-run%` | Self samples of the samples the core was running |

Sleeping samples are left out of the functions' times. In the folded stacks (`core0;Reset;main;...;leaf 123`, one
line per distinct stack, both cores in one file) they end in `[sleep]` on top of the code that went to sleep.

Percentages are of emulated time, so they follow the emulator's cycle model (the M0+ TRM costs, the XIP and flash
timing), not a hardware measurement.

### How sampling works

`Session::run_machine` runs the machine through the session's `Profiler` when it has one: the profiler cuts
`Machine::run_until` at each sample time and records both cores' stacks there. There is no per-instruction hook;
without a profiler nothing changes. Robots run their boards through the same call, and with `--threads` each
board's profiler runs on the thread that runs its board (`sim_host::parallel`), so the samples are the same on any
number of threads (`crates/sim_host/tests/parallel.rs`).

Cutting a run is what a session's 100 µs slices do anyway, so on one core a profiled program runs exactly as an
unprofiled one: batched execution stays on and timer busy-waits are still skipped whole quanta at a time, only less
often (`profiling_does_not_change_what_a_single_core_program_does`, and `cutting_runs_at_sample_times_changes_nothing`
in `crates/rp2040/tests/busy_wait.rs`). With both cores running, the extra cuts can shift how their instruction
streams interleave, as a different `--quantum` would; the run is still deterministic for a given interval.

Cost at the default 10 µs, measured on a shared 4-core machine for 2 s of virtual time: the ESC board 2.2 s of CPU
without, 2.4 s with; Talos 6.2 s and 7.7 s. An idle board, which otherwise fast-forwards between events, takes
longer in proportion (the power board 0.06 s and 0.16 s).

### Stack walks

Stacks are walked the way a debug probe does it, from the core's registers and memory:

* **Call frame information.** The ELF's `.debug_frame` (read with `gimli`) gives each function's CFA and saved
  registers. FDEs of functions the linker discarded (relocated to address 0) are ignored.
* **Prologue decoding** for code without CFI (compiler-builtins assembly, the bootrom): `push`, `sub sp`, the
  large-frame `ldr` + `add sp` form and the final `pop` of an epilogue.
* **Exception frames.** An `EXC_RETURN` value in LR continues the walk through the stacked frame (MSP or PSP) into
  the interrupted code.
* **Checks.** Every return address must have the Thumb bit, point into code and follow a `BL`/`BLX`. Noreturn
  functions that call others without saving LR (`Reset`, embassy's `Executor::run`) end the walk instead of
  following a stale LR.

A stack that cannot be finished is rooted at `[unknown]` and counted in the table; it was under 1 % on every
titan_firmware_rs image tested and 0 on `test_programs/profile.S`. Inlined functions are attributed to the function
they were inlined into (there is no `.debug_info` inline lookup).

### Symbols

Function names come from the ELF's symbol table, demangled for both Rust schemes (legacy and v0; titan_firmware_rs
builds with v0, and all 561 v0 symbols of five firmware images match binutils' `c++filt`). Assembly labels of size
0 run up to the next symbol. Bootrom code is named from the ROM's function table, its soft float and double tables
and its header: `rom:fadd`, `rom:memcpy`, `rom:rom_table_lookup`. Addresses with no name (code copied to SRAM, say)
appear as hex. After `picoboot load` the names come from the image that was flashed. `run`'s stop message demangles
the names it prints too.

### What profiling the firmware found

From `robot robots/talos.toml --mock-bridge internal:1 --time 3s --profile-dir prof/` (10 µs samples):

* **ESC boards, core 1: running 95.6 %.** 88 % of all samples are self time in `esc_board::core1::core1_main`: the
  control loop busy-polls instead of waiting. 64-bit division takes about 3.5 % (`u64_div_rem`, `__udivmoddi4`,
  `__aeabi_uldivmod`), and `pio_core::InstructionOperands::encode` 0.4 %: the loop seems to encode PIO
  instructions at run time.
* **IVCBoard2, core 0: running 86 %.** Almost all of it is under the modem's receive task, in
  `ivc2_core::spectrum::Analyzer::prepare` and soft float (`rom:fadd`, `rom:fmul`, embassy-rp's float wrappers).
  **13 % of core 0 is in the bootrom's `rom_table_lookup`**, called from those wrappers on every float add and
  multiply: the images enable embassy-rp's `intrinsics` feature but not `rom-func-cache`. Enabling it should give
  most of that back (**to verify locally**: it needs a firmware rebuild).
* **Idle boards** (camera cage, PoAC, actuator, LED, SBC, smart battery) run 4.7 to 6.5 % of the time. About 128 ms
  of that is at boot, waiting in embassy-rp's `start_xosc` for XOSC STABLE: embassy-rp's default
  `delay_multiplier: 128` programs a 128 ms startup delay where pico-sdk waits about 1 ms. A boot-time and
  watchdog-recovery cost, not a bug.

## Code coverage

```bash
firmware_simulator run $FW/power_board --board boards/mk2/power_board.toml --mock-bridge 1 --time 2s \
    --coverage power.info
firmware_simulator robot robots/talos.toml --mock-bridge internal:1 --time 2s --coverage talos.info
firmware_simulator scenario scenarios/ --coverage coverage/      # lcov.info, IMAGE.info, summary.txt
genhtml coverage/lcov.info -o coverage/html
```

```text
crate                                   functions              lines
core                                  13/39     33%     127/335    38%
embassy_executor                      26/28     93%      37/45     82%
embassy_rp                            12/14     86%     219/287    76%
power_board                            4/6      67%      62/89     70%
titan_link                             1/5      20%       5/104     5%
titan_rp2040                          13/18     72%     352/449    78%
...
total                                118/188    63%    1164/1872   62%
```

(The power board for 2 s with a mock bridge: titan_link's encoders run, its decoders mostly do not, as the mock
bridge sends little; the firmware's panic handler and fault paths do not run.)

**What it counts.** While coverage is on, each core marks the halfword of every instruction it executes in a
bitmap (one bit per halfword of ROM, flash through any XIP alias, and SRAM: `rp2040::coverage::CoverageMap`). An
instruction counts once it has executed; a step that only takes an exception does not. The cores then run one
instruction at a time, as with a breakpoint set, so busy-waits that a batched run skips over (and timer waits, and
fused quanta) are executed and counted: they ran in the reference semantics. The run is otherwise the same: the
bench's fingerprints do not change with `--coverage` (`cargo run --release -p sim_test --example bench --
--coverage`), and `crates/sim_host/tests/coverage.rs` compares a two-core run with and without it. It costs about 3x
to 5x the CPU time (ESC board 4.5 s for 2 s of virtual time against 1.4 s; Talos 12.8 s against 3.8 s).

**From bits to lines.** `sim_host::coverage` reads the ELF's DWARF line table (`.debug_line`, with `gimli`): each
row's address range belongs to a file and line, a line is instrumented when it has a range and covered when an
instruction in one ran. A function is a `FUNC` symbol with a size, covered when any instruction in it ran, named
(demangled) at the line of its first instruction. lcov hit counts are 1 or 0: the bitmap says whether an
instruction ran, not how often. Boards running the same image (by file name) are merged, as are the scenarios of
`scenario --coverage`; files shared by several images (embassy, core) are merged line by line. The summary groups
functions by the crate their name starts with (the type's, for `<T as Trait>::f`) and lines by the crate their
file is in (the directory above `src/`). Images without DWARF (stripped, UF2) give functions only, as "unplaced".

**From Rust.** `machine.enable_coverage()`, run, then `sim_host::coverage::Coverage`: `add_machine(name,
&machine)`, `report()` (files, lines, functions; `lcov(name)`, `summary()`, `function(name)`). The tests in
`crates/sim_host/tests/coverage.rs` check exact lcov output for `test_programs/coverage.S` (a program with a
branch taken and one not, a function called and one not) and that a titan_firmware_rs image's panic handler is
unrun until a panic is injected.

**Viewing it.** `genhtml FILE -o DIR` (from lcov) renders HTML; VS Code's Coverage Gutters extension shows the
lines in the editor from an `lcov.info` (point its `coverage-gutters.coverageFileNames` at the file). The paths
are those the ELF was built with.

## Waveforms (VCD)

```bash
firmware_simulator run $FW/esc_board --board boards/mk2/esc_board.toml --mock-bridge 2 --time 1s \
    --vcd esc.vcd --vcd-pins esc1_pwm,esc1_telem,status_ledg --vcd-events
gtkwave esc.vcd
firmware_simulator robot robots/talos.toml --mock-bridge internal:1 --time 2s --vcd talos.vcd --vcd-pins 0-15
```

| Option | Default | What it does |
|---|---|---|
| `--vcd FILE` | | Writes the waveform to `FILE` |
| `--vcd-pins SPEC` | `all` | GPIOs to record: numbers, ranges and names from the board file's `[pins]` table, with or without their `_pin` suffix (`0-7,25,status_ledr`) |
| `--vcd-events` | off | Adds string variables with decoded traffic: defmt log lines, UART lines, CAN frames (candump notation), device reports (WS2812 frames, DShot commands, ...), and for a robot one per bus with its frames |

The file:

* **Scopes and names.** One scope per board (for `run` its board file's name, or `board` without one; for `robot`
  its name in the robot file),
  with a 1-bit wire per recorded GPIO named after the board's pins (`gpio6_esc1_pwm`, plain `gpio25` without a
  name); `--vcd-events` adds `log`, `uart0`, `uart1`, `can` and `device` string variables to it, and a scope per
  robot bus with a `frames` variable. String values have whitespace written as `_`.
* **Timescale 1 ns**, which resolves a 125 MHz `clk_sys` (8 ns) and PIO edges. The levels are the resolved pin
  levels from the GPIO trace, whatever drives them: SIO, PWM, PIO (DShot, WS2812, PIO UARTs), external devices and
  injected faults.
* **Only changes are written**, through a 50 ms reordering window: robot boards run one after another in 100 µs
  slices, and a stalled PIO block reports its edges at past cycles when it is next advanced. Changes at the same
  time are written by variable, so the file is the same with any `--threads`. A change later than the window is
  clamped to the last written time and counted, with a warning at the end (none seen on the boards tested).
* **String variables are a GTKWave extension**, which Surfer also reads; PulseView imports only the wires, which is
  why `--vcd-events` is opt-in.

Recording costs little: the GPIO trace does not turn off batched execution (only instruction and exception traces
do), and only the recorded pins' PWM slices have their edges observed. Talos for 2 s with
`--vcd-events --vcd-pins 0-15` took 6.8 s of CPU against 6.2 s without. The tests (`crates/sim_host/tests/vcd.rs`)
read the files back: the blink program toggling every 3005 cycles, the ESC board's DShot600 bits (625 and 1250 ns
low in 1667 ns bits) and its ESC model's eRPM reply (1333 ns cells), the camera cage's WS2812 bits (1.25 µs).

## Dashboard

```bash
firmware_simulator robot robots/talos.toml --mock-bridge internal:1 --realtime --time 600s --dashboard 127.0.0.1:8080
curl -s -X POST -d '{"board": "camera_cage", "fault": "i2c-absent:0x76"}' http://127.0.0.1:8080/api/fault
```

| Option | Default | What it does |
|---|---|---|
| `--dashboard ADDR` | | Serves the page and its API on `ADDR` (`127.0.0.1:8080`) while the run goes |
| `--dashboard-hold` | off | Keeps serving the final state after the run ends, until interrupted |
| `--dashboard-pause-at T` | | Pauses virtual time when it reaches `T` (`0`: from the start, after the first 10 ms piece), to be resumed from the page or the API |

The page (self-contained, no external scripts or fonts) shows, per board: the cores' states and the functions they
are in, log and UART lines, the titan_link `FirmwareStatus` a mock bridge decoded (CAN or Puddles Ethernet, fault
bits named from the board's own `FAULT_NAMES` table in flash, e.g. `FAULT_DEPTH_INIT_ERROR`), GPIO levels,
functions and directions, LEDs, WS2812 strip colours, ESC RPMs and the physical inputs. Per bus: frame count, rate,
load, errors, missing ACKs and recent frames. Per vehicle: position, depth, roll/pitch/yaw, velocity and thruster
RPMs. The header has virtual time, wall time and the emulation speed. Its controls inject a fault, set an input
and pause virtual time.

The layout holds still while values update: one card per board in a grid (cards in a row share a height), each
with the same sections in the same order (cores, bridge status, LEDs, the 30 GPIOs as a 15 × 2 grid, inputs and ESCs,
log). Long symbols are cut to one line, with paths, closures and macro noise dropped
(`<embassy_executor::platform::thread::Executor>::run::<sbc_board::__cortex_m_rt_main::{closure#0}>+0xba` shows as
`Executor::run<main>` and `+0xba`); hover for the full symbol, PC and exception. Numbers use fixed-width digits in
fixed-width cells. The log and frame panes have a fixed height and scroll on their own; a log follows new lines
unless scrolled up, when a button offers to jump back to the latest. The page follows the system's light or dark
theme.

**There is no authentication**: bind it to a loopback address.

### API

| Request | Answer |
|---|---|
| `GET /` | The page |
| `GET /api/state` | The state as JSON, rebuilt every 200 ms of wall time; with `?fresh=1` the run loop builds one first (within a 10 ms piece) |
| `GET /api/logs?since=SEQ&max=N&board=NAME&tail=1` | Log and UART lines after a cursor, read live ([below](#incremental-reads)) |
| `GET /api/frames?bus=NAME&since=SEQ&max=N&id=ID&mask=MASK&tail=1` | CAN frames after a cursor, read live |
| `POST /api/fault` `{"board": "esc0", "fault": "can-stuck"}` | Applies a fault, in the `--fault` syntax ([faults.md](faults.md)); `run` takes its board's name or no `board` |
| `POST /api/input` `{"board": "camera_cage", "name": "depth.pressure_pa", "value": 120000}` | Sets a physical input |
| `POST /api/pause` `{"paused": true}` | Pauses or resumes virtual time; with `"at_s": 2.5`, pausing waits for that virtual time, and resuming runs until it and pauses again |
| `GET /api/thrusters` | Built live by the run loop: `escs` (every DShot ESC: `board`, `esc`, `rpm`, `throttle`, `signal_lost`, and the vehicle `thruster` it spins with its `thrust_n`) and `vehicles` (each one's pose, `external`, and its thrusters with `board`, `esc`, `rpm`, `thrust_n`) |
| `POST /api/rewind` `{"time_s": 1.25}`, `{"checkpoint": "booted"}`, `{"save": "booted"}` | Takes the whole run (every board, bus, vehicle, the water) back to a virtual time (the last 100 µs slice boundary at or before it), or to a checkpoint saved by name, or forward within what has run; `save` keeps a checkpoint here under a name ([below](#rewinding)) |
| `POST /api/vehicle` `{"vehicle": "talos", "position_m": [2, 1, 1.5], "rpy_deg": [0, 0, 90]}` | Puts a vehicle at a pose (world north-east-down, body forward-starboard-down; `quat_wxyz` instead of `rpy_deg`; optional `velocity_mps`, `rate_dps`; `vehicle` may be left out when there is one), answered like `/api/thrusters`; an [external vehicle](vehicles.md#external-vehicles)'s only motion |

A POST is answered once the run loop has applied it and published the new state: 200 with `{"ok": true,
"message": ...}`, 400 for a malformed request (bad JSON, an unknown fault spelling), 422 when it could not be
applied (no such board or input, a fault that does not fit the board, resuming until a time already past), 503
once the run is over. Other paths give 404, other methods 405. A pause at a time takes effect at the first 10 ms
piece that reaches it, and the state is published then.

The state has `time_s`, `wall_s`, `speed` (virtual seconds per wall second), `paused`, `pause_at_s`, `realtime`,
`finished`,
`faults` (every fault applied, from the command line or the page, with its outcome) and `fault_help` at the top,
then `boards` (each with `name`, `image`, `powered`, `resets`, `cores`, `gpio`, `leds`, `strips`, `escs`,
`inputs`, `logs` (the last 120 lines), `link`, and its `bus` in a robot), `buses` (with the last 40 frames),
`links` (the mock bridges and what they decoded) and `vehicles`. With time travel, `travel` has `live` (false while
the run re-executes its recorded history), `oldest_s` and `frontier_s` (the history it can go back to),
`checkpoints`, `memory_mb` and `saved` (named checkpoints).

### Rewinding

A run with a dashboard is recorded for time travel (`--checkpoint-every`, 10 ms by default; `0` turns it off;
[running.md](running.md#reverse-debugging-and-rewinding) explains it). `POST /api/rewind` puts every board, bus,
vehicle and the water back exactly as they were at that time, by restoring the checkpoint before it and replaying
the log to it. Pause first, or the run goes on at once: going on re-executes the recorded history (log lines and
frames already shown are not shown again; the dashboard keeps its lines and frames as they were) and then runs
live. An input set or a fault applied in the past starts a new future from there. A time past the end of what
has run stops at that end. The page has no rewind control of its own: use the API, or the MCP server's `rewind`.

```bash
curl -s -X POST localhost:8080/api/pause -d '{"paused": true}'
curl -s -X POST localhost:8080/api/rewind -d '{"time_s": 1.25}'      # {"ok": true, "message": "at 1.250000s"}
curl -s -X POST localhost:8080/api/input -d '{"name": "stbd_battery", "value": 13.1}'   # another future
curl -s -X POST localhost:8080/api/pause -d '{"paused": false}'
```

### Incremental reads

`/api/logs` and `/api/frames` read what the run loop has collected so far, not the last published state, so a
client can poll them for what is new:

```bash
curl -s 'http://127.0.0.1:8080/api/logs?since=0&max=100'            # every board's lines, oldest first
curl -s 'http://127.0.0.1:8080/api/logs?board=esc0&since=412&tail=1&max=20'
curl -s 'http://127.0.0.1:8080/api/frames?bus=internal&id=0x1020800&mask=0x1ffff00&max=50'
```

| Parameter | Default | Meaning |
|---|---|---|
| `since` | `0` | Return what came after this sequence number (lines: counted across all boards; frames: the bus's frame count) |
| `max` | `200` | At most this many (at most 2000) |
| `tail` | off | The newest `max` after `since` instead of the oldest |
| `board` (logs) | all | One board's lines |
| `bus` (frames) | the only bus | Which bus; required when there are several |
| `id`, `mask` (frames) | all | Only frames with `frame_id & mask == id & mask` (hex `0x...` or decimal; the mask defaults to all bits) |

Both answer `{time_s, paused, finished, next, more, truncated, ...}` with `lines` (`seq`, `board`, `t`, `kind`:
the defmt level or `uart0`/`uart1`, `text`) or `frames` (the [JSON log](#json-log)'s `can` records plus `seq`,
with the bus's `total`). Read on with `since=next`: it neither repeats nor skips anything (for the same `board`
or filter). `more` says there was more than `max`; `truncated` that some of what came after `since` was already
dropped (2000 lines are kept per board, 2000 frames per bus). `time_s`, `paused` and `finished` are where the run
is as of the last 10 ms piece. 400 for an unknown board or bus, or a malformed number.

### How it fits the run loop

The run loop owns the boards; the server threads (std::net, one per connection) never touch them. With a
dashboard, `run` and `robot` advance in 10 ms pieces of virtual time; between pieces the loop applies posted
commands, answers them and, when due, builds a fresh state from what is already there (registers, pin levels,
device models, the mock bridges' records). With `--threads` the worker threads are done with the boards by then. A
session already runs in 100 µs slices and a robot's slices do not change, so a dashboard does not change the
run. It works with `--realtime`: pacing carries across the pieces, and resuming after a pause starts pacing afresh
so the run does not race to catch up.

## MCP server

`firmware_simulator mcp` is a [Model Context Protocol](https://modelcontextprotocol.io) server for coding agents
(Claude Code and other MCP clients): through its tools an agent starts emulator runs, reads boards' state, log
lines and CAN traffic as JSON, and drives a run (faults, inputs, pause), without opening the dashboard page. It
speaks MCP's stdio transport (JSON-RPC 2.0, one message per line on stdin and stdout; protocol revisions
2025-11-25 back to 2024-11-05) and logs to stderr (`RUST_LOG=debug` logs every tool call).

### Setup

Build the emulator first; the server runs the binary it is part of, and starts emulators from it:

```bash
cargo build --release
```

**Claude Code, in this repository.** The repository's [`.mcp.json`](../.mcp.json) declares the server as
`./target/release/firmware_simulator mcp`. Claude Code starts it in the directory `claude` was started in, which
is what that path and the paths in tool arguments are relative to, so start `claude` at the repository root. It
asks once whether to use the project's servers; `/mcp` in a session shows the server and its tools.

**Claude Code, from anywhere.** Register the binary by its absolute path (for one user, every project):

```bash
claude mcp add --scope user firmware_simulator -- $PWD/target/release/firmware_simulator mcp
claude mcp list
```

Paths in tool arguments are then relative to wherever `claude` runs; give absolute ones, or `cwd` on `run_once`
and `start`.

**Other MCP clients.** Configure a stdio server with command `/path/to/firmware_simulator`, arguments `["mcp"]`
and the repository as working directory. The server passes its environment on to the emulators, so set
`TITAN_RS_RELEASE_DIR` there if the firmware is built elsewhere.

### Tools

| Tool | What it does |
|---|---|
| `run_once` | Runs a command (`args`: `["run", IMAGE, ...]`, `["robot", ROBOT, ...]`, `can-diff`, `bridges`, ...) to the end, with a `time` limit and a wall-clock `timeout_s`; returns its console output (long output keeps its start and end; `grep` narrows it), exit status and final `stopped at` line |
| `start` | Starts `run`, `robot` or `picoboot load` in the background with its dashboard on a free port, optionally `realtime` and set to pause at `pause_at_s`; returns a session id, the boards and buses |
| `attach` | Attaches to an emulator someone started with `--dashboard ADDR`, by its URL |
| `list`, `stop` | Lists the sessions; stops one (kills the emulator, or detaches) and returns the end of its console |
| `state` | The state, compact: per board cores, LEDs, inputs, the bridge's decoded firmware status, strips, ESCs and the last log lines; buses; vehicles; faults applied. `board` and `sections` narrow it, `gpio` adds pins, `full` gives the dashboard's JSON |
| `logs` | Log and UART lines after a cursor (`since`, pass back `next`), of one board or all, filtered by `kinds` (`warn`, `error`, `uart0`, ...) and `grep` |
| `can_frames` | Frames on a bus after a cursor, filtered by `id` and `mask` |
| `inject_fault`, `set_input` | A fault in the `--fault` spelling ([faults.md](faults.md)), a physical input |
| `pause`, `resume` | Pause now or at `at_s`; resume, optionally for `for_s` or until `until_s` |
| `rewind` | Takes the whole run back to `time_s`, or to a `checkpoint` saved by name with `save` (or forward within what has run), as `POST /api/rewind` ([above](#rewinding)) |
| `wait_for` | Blocks until a board logs a line matching `pattern` (substring, or `regex`), or virtual time reaches `time_s`, or `timeout_s` of wall time passes; returns early if the run is paused or over. `pause` pauses the run when it happens |
| `console` | The console output of a started emulator: setup messages, `--candump` lines, and after the run its stop line and the unmodelled-register table |

Tool errors (an unknown board, a fault that does not fit, a run that has ended) come back as results with
`isError`, for the agent to read; unknown tools, methods and malformed messages as JSON-RPC errors. Results carry
the JSON both as text and as `structuredContent`.

### Example session

What an agent's calls look like on the wire, one line each (`>` to the server, `<` back; answers shortened to
their `structuredContent`):

```
> {"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"script","version":"0"}}}
< {"jsonrpc":"2.0","id":1,"result":{"protocolVersion":"2025-06-18","capabilities":{"tools":{"listChanged":false}},"serverInfo":{"name":"firmware_simulator",...},"instructions":"..."}}
> {"jsonrpc":"2.0","method":"notifications/initialized"}
> {"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"start","arguments":{"args":["robot","robots/talos.toml","--mock-bridge","internal:1"],"time":"60s","pause_at_s":1.5}}}
< {"session":"s1","url":"http://127.0.0.1:35505/","boards":["poac","esc0","esc1","camera_cage","actuator","led","sbc","ivc","battery"],"buses":["internal","external"],...}
> {"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"wait_for","arguments":{"time_s":1.5}}}
< {"matched":"time","time_s":1.5,"paused":true,"next":37,...}
> {"jsonrpc":"2.0","id":4,"method":"tools/call","params":{"name":"inject_fault","arguments":{"board":"camera_cage","fault":"i2c-absent:0x76"}}}
< {"ok":true,"message":"i2c-absent:0x76 applied to camera_cage"}
> {"jsonrpc":"2.0","id":5,"method":"tools/call","params":{"name":"resume","arguments":{"for_s":3}}}
< {"ok":true,"message":"running until 4.500000s"}
> {"jsonrpc":"2.0","id":6,"method":"tools/call","params":{"name":"wait_for","arguments":{"board":"camera_cage","pattern":"Depth sensor","since":37}}}
< {"matched":"log","line":{"seq":38,"board":"camera_cage","t":1.5217,"kind":"error","text":"1.389772 ERROR Depth sensor failed: Comm(NoAcknowledge)"},"next":38,...}
> {"jsonrpc":"2.0","id":7,"method":"tools/call","params":{"name":"can_frames","arguments":{"bus":"internal","max":2,"tail":true}}}
< {"frames":[{"seq":81,"t":1.400676488,"id":"0x1020801","data":"00","candump":"01020801##100","acked":true,"sender":"node 0",...},...],"next":82,...}
> {"jsonrpc":"2.0","id":8,"method":"tools/call","params":{"name":"stop","arguments":{}}}
< {"session":"s1","stopped":"killed by signal 9","console_tail":[...]}
```

In full, a tool result is `{"content":[{"type":"text","text":"<the JSON>"}],"structuredContent":<the JSON>,
"isError":false}`.

### How it works

Each tool call runs on its own thread, so a long `wait_for` or `run_once` does not hold up others. `start` runs
the emulator as a child process with `--dashboard 127.0.0.1:0 --dashboard-hold` (and `--dashboard-pause-at` for
`pause_at_s`, so that a fast run cannot get past it first), reads the port from its `dashboard on http://...`
line and waits for the first state; the other tools use the [dashboard API](#api) and its
[incremental reads](#incremental-reads). `--dashboard-hold` keeps a finished run's state readable until `stop`.
The children's console output is collected (the last 10 000 lines) and never reaches the server's stdout. When
stdin closes the server kills its emulators and exits; on Linux they are also spawned to die with it
(`PR_SET_PDEATHSIG`) if it is killed. The tools run whatever `firmware_simulator` command they are given, as the
user running the client: options such as `--vcd` and `--flash-file` write files.

## JSON log

`--log-json FILE` writes one JSON object per line, each with `kind`, virtual time `t` (seconds) and `t_ns`
(exact), and its `board` or `bus`:

```
{"kind":"log","board":"esc0","t":0.137,"t_ns":137000676,"level":"info","fw_time":"0.003398","message":"CAN controller online as client 2"}
{"kind":"can","bus":"internal","t":0.8,"t_ns":800658092,"start_ns":800600000,"id":"0x2020801","extended":true,"fd":true,"brs":true,"rtr":false,"data":"0100","acked":true,"error":null,"sender":"node 0","candump":"02020801##10100"}
{"kind":"end","t":3.0,"t_ns":3000000000,"reason":"TimeLimit"}
```

| `kind` | Fields |
|---|---|
| `log` | `board`, `level` (lower case), `fw_time` (the firmware's defmt timestamp), `message` |
| `uart` | `board`, `uart` (0 or 1), `line` |
| `usb` | `board`, `line` (a USB CDC serial line) |
| `can` | `bus`, `start_ns`, `id` (hex), `extended`, `fd`, `brs`, `rtr`, `data` (hex), `acked`, `error` (null, or the injected bit error), `sender` (`node N`, `endpoint N`, `flood N`), `candump` |
| `fault` | `board`, `fault`, `ok`, `error` |
| `end` | `reason`: why the run stopped (`TimeLimit`, or the board that stopped and why) |

```bash
jq -c 'select(.kind == "log" and .level == "error")' talos.jsonl
tail -f talos.jsonl | jq -c 'select(.kind == "log")'     # while the run goes on
```

The file can be followed while the run goes on: it is written through a buffer and flushed every 100 ms of wall
time, so a line is in the file at most 100 ms after it happened (the same goes for `--trace-file` and
`--candump-log`; what goes to stdout, the firmware's lines among it, is written line by line).

## Robot traces

`robot --trace KINDS` prints the same events as `run --trace` ([running.md](running.md#traces)) for every board,
or for the boards in `--trace-board`, each line prefixed with the board's name, to stderr or `--trace-file`. Lines
of one board are in time order. Boards take turns every 100 µs slice, so lines of different boards are in order
only to within a slice; with `--threads` they can come in any order within it (each line stays whole).

## To verify locally

Not available where this was built:

* Opening a `--vcd` file in GTKWave, Surfer and PulseView: wires, names, the 1 ns timescale, and the string
  variables of `--vcd-events`. The files were checked against the VCD standard by the emulator's own parser.
* The dashboard in a real browser against
  `robot robots/talos.toml --mock-bridge internal:1 --realtime --dashboard 127.0.0.1:8080`, with the fault, input
  and pause controls. Its script is tested under Node against a stub DOM with real states
  (`crates/sim_host/tests/dashboard_page.rs`), and the API over loopback (`crates/sim_host/tests/dashboard.rs`).
* The MCP server from Claude Code itself (`.mcp.json` approval, `/mcp`, an agent using the tools). It is tested
  end to end with a scripted JSON-RPC client over its stdio (`tests/mcp.rs`).
* Rendering the folded stacks with `inferno-flamegraph` or `flamegraph.pl`, and loading them in speedscope.
* Rendering `--coverage` output with `genhtml` and showing it with VS Code's Coverage Gutters (lcov is not
  installed here; the tracefile's records are checked line for line in `crates/sim_host/tests/coverage.rs`).
* `POST /api/rewind` from a browser session; it is tested over loopback through the MCP server (`tests/mcp.rs`).
* Rebuilding titan_firmware_rs with embassy-rp's `rom-func-cache` and profiling IVCBoard2 again.
