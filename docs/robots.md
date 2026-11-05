# Robots: many boards, buses, and the robot computer's side

A robot file puts several boards in one process, on the CAN buses they share, optionally in an acoustic
environment and in simulated vehicles. `firmware_simulator robot` runs one; `RobotBench` runs one in a test
([testing.md](testing.md)). This page also covers what connects to the buses: mock bridges, SocketCAN, CAN logs
and replay, log comparison, UDP tunnels, and the real host tools.

- [Robot files](#robot-files)
- [The `robot` command](#the-robot-command)
- [Traces, profiles, waveforms and the dashboard](#traces-profiles-waveforms-and-the-dashboard)
- [How a robot runs](#how-a-robot-runs)
- [Mock bridges](#mock-bridges)
- [Mock agents: nano-ROS and other protocols](#mock-agents-nano-ros-and-other-protocols)
- [CAN logs: recording, replaying, comparing](#can-logs-recording-replaying-comparing)
- [CAN over UDP (cannelloni)](#can-over-udp-cannelloni)
- [Without vcan: UDP](#without-vcan-udp)
- [titan_bridge command lines: `bridges`](#titan_bridge-command-lines-bridges)
- [Robot in a box: the Orin's side on vcan](#robot-in-a-box-the-orins-side-on-vcan)

## Robot files

| File | What it is |
|---|---|
| `robots/talos.toml` | Talos: eight boards on the internal CAN FD bus (PoAC, two ESC boards, camera cage, actuator, LED, SBC, IVCBoard2), the smart battery on the external classic bus, and the Talos vehicle |
| `robots/puddles.toml` | Puddles: the backplane over Ethernet, in the Puddles vehicle |
| `robots/ivc_pool.toml` | Two vehicles in `pool.toml`, each with a first-generation IVCBoard on its own bus |
| `robots/ivc2_pool.toml` | The same with IVCBoard2 |
| `robots/ivc2_interop.toml` | The Rust `ivc_board2` and the C titan_firmware IVCBoard2 (your build, micro-ROS over USB) in the pool ([acoustics.md](acoustics.md#c-and-rust-ivcboard2-interop)) |
| `robots/talos_c.toml` | Talos with titan_firmware's C board apps (your build, micro-ROS over CAN), as the robot runs them, in Talos's real thruster layout ([robot in a box](#robot-in-a-box-the-orins-side-on-vcan)) |
| `robots/nanoros_led.toml` | The nano-ROS test board (`test_programs/nanoros_led`: osu-uwrt/nano_ros's libnanoros on the Talos LED board) with its `ros_toml`, for `--mock-agent nanoros:led` ([below](#mock-agents-nano-ros-and-other-protocols); early) |
| `robots/talos_c_nereus.toml`, `robots/talos_nereus.toml` | The boards in the loop (PoAC, two ESC boards, camera cage), C or Rust, in a vehicle nereus moves ([nereus.md](nereus.md)) |
| `robots/mercury.toml` | Mercury, the vehicle UWRT's new Mercury stack (`mercury_release`) runs on: titan_firmware's C `mercury/PoHalfBoard` (client 1) and `mercury/ESCBoard` twice (clients 2 and 3), your build, micro-ROS over CAN on the internal bus; no vehicle ([below](#mercury-and-tempest)) |
| `robots/tempest.toml` | Tempest, the AUV before Talos: its MK2 camera cage (client 4) and actuator (client 5) with today's Rust images, in `vehicles/tempest.toml` (riptide's description), thrusters unwired ([below](#mercury-and-tempest)) |

```toml
[environment]                       # optional: the water the boards' transducers are in
file = "../environments/pool.toml"
# seed = 7                          # replaces the environment's noise seed

[[bus]]
name = "internal"
rates = "internal"                  # CAN FD 1/5 Mbit/s; "external" is classic 250 kbit/s;
                                    # default: the rates of the first board on the bus
# record = true                     # keep every frame (tests)

[[board]]
name = "esc1"
board = "../boards/mk2/esc_board.toml"
elf = "../external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release/esc_board"
bus = "internal"                    # default: the only bus, if there is one and the board has [can]
power_on_ms = 3                     # switched on 3 ms after the others
gpio = [{ pin = 29, level = "high" }]   # straps, replacing the board file's for those pins
# rom_boot = true                   # boot through the bootrom and boot2
# usb_cdc = "pty:/tmp/esc1-log"     # USB plugged into a host that opens CDC port 0: stdout, pty or pty:PATH
# usb_cdc2 = "pty:/tmp/esc1-ros"    # and CDC port 1 (micro-ROS over titan_dual_stdio_usb); usb = true only plugs in
# ros_toml = "../path/ros.toml"     # a nano-ROS board's topics, for --mock-agent nanoros:BOARD (docs/nanoros.md)
# position_m = [5.0, 6.0, 1.5]      # in an environment: where its transducer is,
# trajectory = { start_m = [...], velocity_mps = [...] }   # or how it moves
# [board.transducer]                # replaces the board file's transducer

[[vehicle]]                         # optional: see vehicles.md
name = "talos"
file = "../vehicles/talos.toml"
```

Paths are relative to the robot file. Unknown keys are errors. Several vehicles go in one file as several buses
(the IVC pool files). `elf` paths into `external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release`
are redirected to `$TITAN_RS_RELEASE_DIR` when it is set (a worktree, another checkout, CI).

* **Images from anywhere.** `elf` takes an ELF or a UF2 (a C build's `NAME_with_bl.uf2`, bootloader and
  application; give it `rom_boot = true` to boot it like a powered-on board), relative or absolute. `elf`, `board`
  and pty paths may start with `~` and contain environment variables, `$NAME` or `${NAME}` (an unset one is an
  error naming it). For C titan_firmware images the convention is **`TITAN_FIRMWARE_DIR`**, your titan_firmware
  checkout, with each project built in its own `build/` as titan_firmware's docs do it:
  `elf = "${TITAN_FIRMWARE_DIR}/talos/IVCBoard2/build/IVCBoard2_with_bl.uf2"`.

* **Power-on offsets.** Boards never power up in perfect lockstep; `power_on_ms` keeps a board unpowered for that
  long. Talos's second ESC board starts 3 ms late, the IVC pools' second board 37 ms late.
* **Straps.** `gpio` entries replace the board file's `[[gpio]]` for the same pins: Talos's `esc1` has its ID
  strap (GPIO 29) high, which makes it ESC board 1 (client 3); the first-generation IVC pool's surface board has its
  board ID pin (GPIO 6) high.
* **Environments and positions**: a board with a transducer in a robot with an `[environment]` needs
  `position_m`, a `trajectory` or a vehicle that carries it ([acoustics.md](acoustics.md)).
* **Vehicles**: `[[vehicle]]`, `[[vehicle.sensor]]`, `[[vehicle.transducer]]` ([vehicles.md](vehicles.md)).
* **USB**: see [below](#usb-on-robot-boards).

### Mercury and Tempest

**Mercury** is UWRT's new software stack (`osu-uwrt/mercury_release` and its `mercury_*` packages, most of them
still empty skeletons) and the robot titan_firmware builds for it (`robots/mercury.h`: namespace `mercury`, CAN FD
internal bus, classic external bus). Its boards are titan_firmware's "Triton" boards (`tools/DeviceList.jsonc`):
`mercury/PoHalfBoard` (kill switch, aux switch, depth) and `mercury/ESCBoard` twice, a copy of `talos/ESCBoard` with
the ESC pins moved, no inverted thrusters and `mercury_msgs`. Only these C apps exist (titan_firmware_rs has no
Mercury boards: its `po_half_board` is Lilboat's), so `robots/mercury.toml` takes them from `TITAN_FIRMWARE_DIR`,
and `scenarios/boot_mercury.yaml` (build tags on the UART, each client's CANmore heartbeat, no panic or reset) is
skipped until it is set. Both apps build with the micro-ROS library that has `mercury_msgs` (the Nix shell's), and
the scenario passes with them (titan_firmware 058df2a). There is no vehicle file: `mercury_descriptions` is empty
and `mercury_simulator` loads riptide's `talos.yaml`. A firmware finding, confirmed on the running image
(`firmware_finding_mercury_esc_board_reads_each_rpm_high_byte_twice` in `crates/sim_test/tests/c_boards.rs`):
`mercury/ESCBoard`'s raw CAN thruster command handler still reads each RPM's high byte twice
(`(buf[idx] << 8) | buf[idx]`), fixed for `talos/ESCBoard` only (titan_firmware PR #26). A raw command of 1200,
-1200, 300 and 0x1234 RPM becomes the target speeds 1028 (0x0404), -1029 (0xFBFB), 257 (0x0101) and 0x1212 in
core 1's `target_req`; the Talos board's are the ones sent. (Watching the DShot output as well needs a micro-ROS
agent: without one the boards stay killed and their controllers unconfigured.)

**Tempest** is the AUV before Talos, not a new one: riptide's launch files default to `robot:=tempest` because they
were carried over from the earlier stack, and `riptide_descriptions/config/tempest.yaml` has not changed since. Its
boards are electric_boogaloo's `TempestBoards`: the MK1 coprocessor on the backplane breakout board (micro-ROS over
CAN, PWM thrusters), MK1 power distribution, and the MK2 camera cage and actuator boards that Talos carries now.
titan_firmware built `Copro/`, `Actuator/` and `CameraCageBB/` for `UWRT_ROBOT tempest` until February 2023
(2f24ddb removed them); today only the MK2 boards have firmware, Talos's. `robots/tempest.toml` runs those two
Rust images (they report themselves as Talos boards) in `vehicles/tempest.toml`, with the camera cage's depth
sensor at tempest.yaml's depth pose and the thrusters unwired.

## The `robot` command

```bash
firmware_simulator robot robots/talos.toml --mock-bridge internal:1 --mock-bridge external:21 --time 2s
firmware_simulator robot robots/ivc2_pool.toml --mock-bridge talos:8 --mock-bridge surface:8 --time 15s
firmware_simulator robot robots/puddles.toml --eth-mock-bridge backplane --time 10s --vehicle-csv puddles.csv
```

Each board's log lines are prefixed with its name (`[0.200300s] poac: 0.068290 INFO  ROS connected`, UART lines
as `[t] NAME uart0: ...`); the run ends with the stop time, the wall time and each vehicle's pose.

| Option | Default | What it does |
|---|---|---|
| `-t, --time T` | `5s` | Virtual time to run |
| `--mock-bridge BUS:CLIENT` | | A mock bridge for one client on one bus (repeatable, one per client) |
| `--eth-mock-bridge BOARD` | | A mock bridge on an Ethernet board's cable (Puddles) |
| `--mock-agent PROTOCOL[:BUS][:BOARD][=CONFIG]` | | A scripted host side in the board's protocol: `nanoros:led` (a nano-ROS agent with the board's `ros_toml`), `nanoros:led=ros.toml`, `titan_link:poac` (`--mock-bridge` for the board's client); repeatable ([below](#mock-agents-nano-ros-and-other-protocols)) |
| `--udp BOARD:LOCAL=PEER` | | Relays an Ethernet board's UDP through a host socket, e.g. `backplane:127.0.0.1:2203=127.0.0.1:2204` |
| `--can BUS=IFACE` | | Bridges a bus to a SocketCAN interface, e.g. `internal=vcan0` (repeatable) |
| `--candump` | | Prints every frame with its bus |
| `--candump-log FILE`, `--replay FILE`, `--replay-filter`, `--replay-offset`, `--replay-iface IFACE=BUS` | | [Below](#can-logs-recording-replaying-comparing) |
| `--cannelloni BUS=LOCAL,PEER`, `--cannelloni BUS=LOCAL` | | CAN over UDP: a tunnel to one peer, or a hub for any number of host tools ([below](#can-over-udp-cannelloni)) |
| `--micro-ros-agent BUS=HOST[:PORT]` | | Relays the bus's micro-ROS boards to a micro-ROS agent over UDP, with no vcan ([below](#without-vcan-udp)) |
| `--arrivals-log FILE`, `--replay-arrivals FILE` | | Logs where each frame from `--can`/`--cannelloni` entered a bus; replays such a log at exactly those times ([below](#repeating-a-run-with-host-tools)) |
| `--usb-cdc BOARD=SINK` | | Opens a board's USB CDC port (pico-sdk stdio_usb, titan_dual_stdio_usb's text log): `stdout` prints its lines as `[t] BOARD usb: ...`, `pty` or `pty:PATH` puts it on a terminal (repeatable; replaces the robot file's `usb_cdc`) |
| `--usb-cdc2 BOARD=SINK` | | Opens its second CDC port (titan_firmware's micro-ROS USB transport) on `pty` or `pty:PATH` (repeatable; replaces `usb_cdc2`) |
| `--ivc-send TIME@BOARD:BYTE` | | Makes an IVC board transmit a byte at a virtual time through its mock bridge, e.g. `4s@talos_ivc:0x5a` (repeatable; [below](#mock-bridges)) |
| `--fault TIME@BOARD:FAULT` | | A fault on one board, e.g. `1.5s@camera_cage:can-unplug` (repeatable; bridge faults act on the first mock bridge on the board's bus; [faults.md](faults.md)) |
| `--noise-db L` | | Replaces every hydrophone's ambient noise with white noise at `L` dB re 1 µPa²/Hz |
| `--vehicle-csv FILE` | | Writes each vehicle's state every 10 ms (`NAME` in the file name for several vehicles) |
| `--external-vehicle NAME` | | Lets another simulator move the vehicle through the dashboard (`external = true`; needs `--dashboard`; [vehicles.md](vehicles.md#external-vehicles)) |
| `--realtime` | off | Paces virtual time to the wall clock; warns when the robot falls behind it ([running.md](running.md#options)) |
| `--threads N` | 0 | Runs the boards on up to `N` threads (0: up to one per core; 1: in turn on one) ([below](#how-a-robot-runs)) |
| `--no-run-ahead` | off | On threads, keeps the boards in step slice by slice rather than letting them run ahead of each other (same results; [below](#how-a-robot-runs)) |
| `--trace KINDS`, `--trace-board NAMES`, `--trace-file FILE` | | Traces the boards ([below](#traces-profiles-waveforms-and-the-dashboard)) |
| `--profile-dir DIR`, `--profile-interval T` | `10us` | Profiles every board's firmware |
| `--vcd FILE`, `--vcd-pins SPEC`, `--vcd-events` | all pins | A VCD waveform with a scope per board (and per bus) |
| `--dashboard ADDR`, `--dashboard-hold`, `--dashboard-pause-at T` | | A live dashboard of all boards, buses and vehicles |
| `--log-json FILE` | | Every log line, UART line, CAN frame and fault as JSON lines |
| `--gdb BOARD=ADDR` | | Waits for GDB on one board, e.g. `esc0=127.0.0.1:3333`; continuing runs the whole robot ([running.md](running.md#debugging-with-gdb)) |
| `--unmodelled MODE` | `summary` | Registers the emulator does not model, as for `run` ([running.md](running.md#traces)): one note, then a line per board at the end listing them; `first`, `every` or `off` |

A board whose core locks up runs on until its watchdog resets it, as on the chip (`--stop-on-lockup` stops the
robot at the first lockup instead; [faults.md](faults.md#what-the-models-do)). It exits with an error if a board
stops before `--time` (a lockup nothing can end, say). Ctrl-C or SIGTERM ends the run cleanly
at the next slice boundary, as for `run` ([running.md](running.md#exit-status)): logs, waveform, profiles, candump
log and vehicle CSV are written, and the stop line says `(Interrupted)`.

### USB on robot boards

A board's USB port is plugged into its own host controller model ([running.md](running.md#usb-bootsel-picoboot-uf2-and-cdc))
by `usb = true`, `usb_cdc` or `usb_cdc2` on its `[[board]]`, or by `--usb-cdc BOARD=SINK` / `--usb-cdc2 BOARD=SINK`,
which replace the file's setting for that port. The host opens the CDC ports asked for (115200 8N1, DTR and RTS),
so pico-sdk's stdio_usb and titan_firmware's `titan_dual_stdio_usb` (text on port 0, micro-ROS on port 1) talk to
it as to a laptop. A port on `pty:PATH` gets a pseudo terminal with a symlink at `PATH` (replacing a symlink an
earlier run left), which the micro-ROS agent or `picocom` open; the paths are printed at the start:

```bash
firmware_simulator robot robots/ivc2_interop.toml --mock-bridge talos:8 --realtime --time 600s \
    --usb-cdc c_ivc=stdout --usb-cdc2 c_ivc=pty:/tmp/c_ivc-ros
#   c_ivc: USB CDC port 1 on /dev/pts/4 (linked from /tmp/c_ivc-ros)
#   [0.209900s] c_ivc usb: ...
```

The ports are polled after every slice, like the UARTs, on the calling thread, so a robot on `--threads` runs the
same; what arrives from a terminal reaches the board at the next slice boundary after it was written, which depends
on the wall clock (as with any host tool). Snapshots (`Robot::snapshot`) keep each board's USB host and its
ports' settings but not the terminals; `Robot::open_usb_ptys` opens new ones for the copy. In code:
`Robot::set_usb_cdc(board, port, sink)`, `Robot::plug_usb`, `Robot::usb`.

## Traces, profiles, waveforms and the dashboard

The developer tools of `run` work on robots too ([tools.md](tools.md) has their formats and how they work):

| Option | Default | What it does |
|---|---|---|
| `--trace KINDS` | | Trace categories as for `run` (`instr`, `exc`, `gpio`, `unmodelled`, `device`; [running.md](running.md#traces)), each line prefixed with the board's name |
| `--trace-board NAMES` | all boards | Only trace these boards (comma separated) |
| `--trace-file FILE` | stderr | Where the trace goes |
| `--profile-dir DIR` | | Profiles every board: `DIR/BOARD.txt` (functions by time per core) and `DIR/BOARD.folded` (folded stacks) |
| `--profile-interval T` | `10us` | Mean virtual time between samples |
| `--vcd FILE` | | One VCD file: a scope per board with its GPIOs, wires named after each board's `[pins]` |
| `--vcd-pins SPEC` | `all` | Pins to record on every board: numbers, ranges, pin names |
| `--vcd-events` | off | Adds each board's logs, UART lines, CAN frames and device reports, and a scope per bus with its frames |
| `--dashboard ADDR` | | Serves the live dashboard: every board, every bus's traffic and load, the mock bridges' decoded firmware status, the vehicles; faults and inputs are posted per board |
| `--dashboard-hold` | off | Keeps serving after the run ends |
| `--dashboard-pause-at T` | | Pauses virtual time when it reaches `T` (`0`: from the start); resume from the page |
| `--log-json FILE` | | JSON lines with each record's board or bus |

```bash
firmware_simulator robot robots/talos.toml --mock-bridge internal:1 --time 5s --log-json talos.jsonl \
    --trace gpio --trace-board led --trace-file led.trace
firmware_simulator robot robots/talos.toml --mock-bridge internal:1 --time 3s --profile-dir prof/
firmware_simulator robot robots/talos.toml --mock-bridge internal:1 --time 2s --vcd talos.vcd --vcd-pins 0-15
firmware_simulator robot robots/talos.toml --mock-bridge internal:1 --realtime --time 600s --dashboard 127.0.0.1:8080
```

Trace lines of one board are in time order; boards take turns every 100 µs slice, so lines of different boards
are ordered only to within a slice (with `--threads`, in any order within it). Profiles, waveforms and the JSON log
come out the same with any `--threads`. With a dashboard the robot runs in 10 ms pieces of virtual time, which
leaves its slices, and so the run, as they were.

## How a robot runs

All boards run to the same virtual time in 100 µs slices. After each slice every bus arbitrates and delivers what
its boards and host endpoints sent, the boards' devices update their pins, vehicles step, and each board's
progress is reported to the acoustic medium. The slice bounds how late a frame can reach another board, and keeps
runs deterministic: the same robot file and inputs give the same run, frame for frame.

**Threads.** Within a slice the boards do not see each other, so they can run side by side, and each slice ends
exactly as if they had taken turns (tests check Talos frame by frame on any number of threads). `--threads N`
(`Robot::threads` in code) allows up to `N` threads; the default, 0, allows one per core. Each board stays on one
thread for many slices, grouped by what it was measured to cost: on Talos the two ESC boards each get a thread and
the other seven share a third. Only as many threads are used as make the slices shorter, so a robot of light
boards stays on one, and if the cores are busy with other programs (the slices are not shorter on threads) the
boards go back to taking turns for a while. Robots in an acoustic environment always take turns, since a
hydrophone can hear another board within the slice. Emulator log lines from different boards can interleave
differently with threads; what the boards do does not change. `Robot::threads` defaults to 1 in code (tests).

**Running ahead.** On threads, the boards also run ahead of each other between the slices where they can see each
other: a CAN frame can reach another board no sooner than it finishes on the wire, and a mock bridge knows when it
will next send, so each board runs on alone through the slices where no frame can finish (up to 64 at a time),
and stops where its own next frame could; the others stop there too. A board that went past such a point before it
knew is rolled back and run again, and the calling thread replays every bus, vehicle and host hook in slice order
as it goes, so the run is exactly the one taking turns gives (tests check Talos frame by frame, log line by log line
and state by state, also with boards rolled back thousands of times). Boards whose inputs a vehicle drives (the
camera cage's depth sensor) run slice by slice on the calling thread, after the vehicle (not for an external
vehicle, which stays where `POST /api/vehicle` put it during a run). This is what lets the two ESC boards, whose
heavy slices do not line up, stop waiting for each other every slice.

Host tools run ahead too. What arrives on `--can` (SocketCAN) or `--cannelloni` cannot be known in advance, but it
enters the bus where taking turns would put it: at the slice boundary whose bus step reads it. Running ahead reads
it at every boundary as it commits them, and boards that already ran past where the frame can arrive are rolled
back to before it, so a frame never lands in a board's past. With `--realtime`, windows also end no more than 2 ms
ahead of the wall clock, so host frames are read, and the boards' answers go out, about when they are due. A
terminal (a pty on a USB CDC port) is read by its board at each boundary the same way and what it read is kept for
a rerun; `--vcd`, `--trace` (except `--trace instructions`) and `--profile-dir` see only what the boards finally
did. Runs with real host tools differ only in when frames arrive: `--arrivals-log FILE` logs where each frame from
`--can`/`--cannelloni` entered a bus, and `--replay-arrivals FILE` (in place of those options) repeats the run from
it exactly, running ahead or not. With an Ethernet cable (`--udp`, `--eth-mock-bridge`), `--usbip`, instruction
tracing or an acoustic environment the boards stay in step slice by slice, as with `--no-run-ahead`.
`Robot::run_ahead` switches it in code.
A run recorded for time travel (`--checkpoint-every`, `--dashboard`) runs ahead too, live and when it replays
(checkpoints are taken where every board is committed, and the recorded bridges tell when they send from the
recording); with `--gdb` the boards take turns, so stops are as without threads. `--coverage` counts only what the
committed run executed: what a board ran before it was rolled back does not count.

A board alone on a bus with nothing to acknowledge its frames (Talos's smart battery on the external bus when no
bridge is there for client 21) retries each frame without end, as a CAN controller does (the firmware does since
titan_firmware_rs c59474f; before, its transmit queue gave up after three attempts and fell silent). Each retry is a
frame on the bus, every 284 µs, and each ends a window, so running ahead gains nothing: Talos on four threads took
16 s of wall time for 2 s instead of 1.8 s. Give such a bus a bridge (`--mock-bridge external:21`, as the bench and
the run-ahead tests do), which is also what the robot has: the Orin's CAN interface acknowledges every frame.

**Speed.** On one core of the 4-core machine it was measured on, Talos (nine boards) runs at about 0.54x real
time, and at about 1.2-1.25x on three or four threads (2.2-2.3x faster), so `--realtime` with all of Talos keeps
up when three cores are free for it. Two threads give about 0.8x (the ESC boards then share a thread with the
others). More threads than three or four add nothing: each ESC board needs about 0.7 s of host time per virtual
second on its own, and with running ahead the costlier of them sets the pace (slice by slice, without running
ahead, it was about 0.8x on three threads: their bursts, the DShot frames, do not line up). The template runs
about 75x faster than real time, IVCBoard2 about 3.6x and the ESC board about 1.5x on its own. `cargo run --release -p sim_test --example bench` measures it ([internals.md](internals.md#performance)). Without
`--realtime` a robot runs as fast as it can, so a host tool next to it (titan_bridge, a micro-ROS agent) needs
timeouts with room for a robot slower than real time; with `--realtime` the robot warns
(`real-time mode: emulation is 1.23s behind the wall clock`, at most every 5 s) when it cannot keep up.

## Mock bridges

A mock bridge is a scripted stand-in for `titan_bridge` built on the real `canmore` and `titan_link` crates. On a
CAN bus it serves one client: a `Ping` every 100 ms, the `state/kill` kill switch forwarded with each (released,
so the vehicle is enabled), and a record of everything the board sends (titan_link messages, CANmore heartbeats,
utility frames, decoding errors). On an Ethernet cable (`--eth-mock-bridge`, or `run --mock-bridge` on Puddles) it
speaks the Puddles UDP transport from the robot computer's address. Like a Linux CAN socket it queues at most 10
frames and drops pings that do not fit.

**IVC bytes.** Each mock bridge of `robot` prints the IVC bytes its board received acoustically
(`DeviceToHost::Ivc2(RxNew)` from IVCBoard2, `DeviceToHost::Ivc(Rx)` from the first-generation IVCBoard), named
after the board: `[12.200999s] surface_ivc: IVC received 0x5a`. `--ivc-send TIME@BOARD:BYTE` (BYTE as `90` or
`0x5a`) makes the board's bridge send `Ivc2(EnqueueData(BYTE))` to an `ivc_board2` image or `Ivc(Tx(BYTE))` to an
`ivc_board` image at that virtual time; the board needs a `--mock-bridge` for its bus and client. IVCBoard2 hears
nothing until its noise floor has settled (about 2.5 s after power-on), and a packet takes 9 s:

```bash
firmware_simulator robot robots/ivc2_pool.toml --mock-bridge talos:8 --mock-bridge surface:8 \
    --ivc-send 3.5s@talos_ivc:0x5a --ivc-send 13s@surface_ivc:0xc3 --time 23s
#   [3.561700s] talos_ivc: 3.426997 INFO  Transmitting 0x5a
#   [12.200999s] surface_ivc: IVC received 0x5a
#   [13.051400s] surface_ivc: 12.879642 INFO  Transmitting 0xc3
#   [21.689465s] talos_ivc: IVC received 0xc3
```

In code it is `sim_host::mock_bridge::MockBridge` (`PuddlesBridge` for Ethernet): `send` a `HostToDevice` now or
`send_at` a virtual time, read `state().received` (or get each message as it arrives with `on_message`),
`stop`/`start` it, `set_kill`, `fork` it for a snapshot. `IvcModem::from_image(name).transmit(byte)` and
`ivc_received(msg)` are the IVC messages above.

## Mock agents: nano-ROS and other protocols

**Early and experimental: nano-ROS is still changing ([nanoros.md](nanoros.md)).** A mock agent is the scripted
host side of one board's CAN link in whatever protocol its firmware speaks, behind one interface
(`sim_host::host_protocol::HostAgent`) that the command line, [scenarios](scenarios.md#actions), the
[dashboard](tools.md#dashboard) and the [MCP server](tools.md#mcp-server) use: messages from the board by
topic, sending one by topic, a status. Two protocols speak it: `nanoros`, a stand-in for osu-uwrt/nano_ros's
`nano_ros_agent` for titan_firmware C boards built with libnanoros, and `titan_link`, the mock bridge above (its
messages' variants are its topics).

`--mock-agent nanoros:BOARD` puts a nano-ROS agent on the board's bus for its CANmore client (from its board file),
reading the topic table from the robot file's `ros_toml` for the board, or `nanoros:BOARD=ros.toml` from that file
(`nanoros:BUS:CLIENT=ros.toml` for a client no board file names). It discovers the board, takes its topic
registrations, checks them against the `ros.toml`, and prints each message the board publishes, decoded:

```bash
firmware_simulator robot robots/nanoros_led.toml --mock-agent nanoros:led --time 2s
#   [INFO  sim_host::nanoros] [0.249318s] nano-ROS: client 6 registered 5 topic(s)
#   [0.314943s] led <- imu_data {"angular_velocity":{"x":0.0,...},"header":{"frame_id":"imu_link",...},...}
#   [1.055491s] led <- firmware_status {"board_name":"nanoros_led","bus_id":1,"client_id":6,...,"uptime_ms":1040,...}
#   led: nanoros agent connected, 19 message(s) from the board, 0 sent, 0 error(s)
firmware_simulator run test_programs/nanoros_led/nanoros_led.elf --board boards/talos/led_board.toml \
    --mock-agent nanoros=test_programs/nanoros_led/ros.toml --time 2s
```

Sending to the board's subscribers goes through a scenario's `send: {board, topic, msg}`, the dashboard's
`POST /api/agent` or the MCP `agent_send` tool. In code: `host_protocol::attach(&mut robot, &spec, dir)` (or
`sim_host::nanoros::NanoRosAgent::new`), then `send(topic, &json, at)`, `messages(from)`, `on_message`, `status`,
`stop`/`start` (the agent restarting), `fork`.

## CAN logs: recording, replaying, comparing

These options work with `run` (a board's own bus) and `robot` (every bus).

### Recording

`--candump-log FILE` writes the traffic in `candump -l` format: every acknowledged frame (what a listening
interface would receive), stamped with the virtual time its last bit was sent, with the bus name as the
interface. Unacknowledged frames (sent before anyone else was up) are counted and left out. The file is flushed
every 100 ms of wall time, so it can be followed while the run goes on, and a Ctrl-C'd run ends cleanly with the
log complete.

```
(0000000000.135357) can 06F#C0
(0000000000.200059) can 01420802##10002
```

### Replaying

`--replay FILE` puts recorded traffic back on a bus as a host endpoint. It reads `candump -l` logs and candump's
text output (`candump can0`, with `-t a`, `-x`, `-a`). Frames keep their spacing: a log with Unix timestamps (from
hardware) starts at virtual time 0, one written by `--candump-log` replays at its own times, and
`--replay-offset TIME` says when its first frame goes out. Each frame is made ready its on-wire duration before
its logged time, so on a free bus it ends exactly when it ended in the log.

`--replay-filter SPEC` selects frames by CANmore fields: `key=value` terms joined by `,` (all must hold; `!`
negates a term; `|` separates alternative values). Repeated filters are alternatives.

| Key | Values |
|---|---|
| `sender` | `host` (what the robot computer sent) or `board` |
| `client` | CANmore client ID, e.g. `1` or `1\|8` |
| `type` | `msg` or `util` |
| `channel` | utility channel (implies `type=util`) |
| `iface` | interface in the log |
| `id` | raw identifier `0x123`, or with a mask `0x100/0x700` |

Interfaces: with `run`, a log with one interface replays it, otherwise pick one with `--replay-iface IFACE`. In a
robot an interface goes to the bus of the same name, or the one `--can` bridges to that interface name; map others
with `--replay-iface can0=internal`.

```bash
# What the robot computer sent to the power board, replayed to the emulated one
firmware_simulator run $FW/power_board --board boards/mk2/power_board.toml \
    --replay hw.log --replay-filter sender=host,client=1 --candump-log sim.log --time 30s
# Everything on a recorded Talos bus except ESC board 0 (client 2), which the emulator plays
firmware_simulator robot robots/talos.toml --replay hw.log --replay-iface can0=internal \
    --replay-filter sender=host --replay-filter '!client=2'
```

Because the emulator is deterministic, replaying the bridge's side of an emulator log into a fresh run reproduces
it exactly; `crates/sim_test/tests/can_log.rs` does this, and replays a recorded upload through the C Titan
bootloader into a fresh board, which reflashes it.

### Repeating a run with host tools

With `--can` or `--cannelloni`, what the host tools send reaches the bus whenever it arrives, so two runs differ.
Each frame enters the bus at the slice boundary whose bus step read it; `robot --arrivals-log FILE` writes every
such frame with that boundary's time, to the nanosecond, as a `candump -l` log with the bus name as the interface:

```
(0000000000.512300000) internal 00000123##10102
```

`robot --replay-arrivals FILE`, in place of the `--can` and `--cannelloni` options, hands each frame in at exactly
that time again (each interface to the bus of that name), so the run is repeated exactly: the same frames at the
same times, the same logs, the same state, with or without `--threads` and `--no-run-ahead` and without the host
tools, vcan or ROS. (A frame the boards answered is answered the same; what the host tools would have sent in
reply to something different is not recomputed.)

### Comparing: `can-diff`

`firmware_simulator can-diff A B` compares two logs by what the frames mean. CANmore messages are re-assembled per
client and direction and their `titan_link` payloads decoded; each kind of message is a stream (`can client 1
board->host Power::Electrical::ElectricalReadings`, `... host->board Ping`, `... board->host heartbeat` for the
CANmore heartbeat, compared without its rolling counter). Two runs of the power board, the second with the port
battery at 13 V:

```bash
firmware_simulator run $FW/power_board --board boards/mk2/power_board.toml --mock-bridge 1 --candump-log a.log --time 3s
firmware_simulator run $FW/power_board --board boards/mk2/power_board.toml --mock-bridge 1 --candump-log c.log --time 3s \
    --input port_battery=13
firmware_simulator can-diff a.log c.log
```

```
frames: A 119, B 119; aligned on their timestamps as logged: B = A +0.000000 s; compared 0.135357 s to 2.905483 s of A

stream                                                               A      B paired   miss  extra payload         period A ms         period B ms   offset ms    max skew
can client 1 board->host FirmwareStatus                              2      2      2      0      0       0     1000.00 ± 0.000     1000.00 ± 0.000      +0.000       0.000
can client 1 board->host Heartbeat                                  27     27     27      0      0       0      100.00 ± 0.020      100.00 ± 0.020      +0.000       0.000
can client 1 board->host Power::Electrical::ElectricalReadings       2      2      2      0      0       2     1001.16 ± 0.000     1001.16 ± 0.000      +0.000       0.000
...
differences (times in A's timeline):
  can client 1 board->host Power::Electrical::ElectricalReadings:
    2 with different payloads
      1.203556  port_voltage: 16.195385 -> 12.995312
      2.204719  port_voltage: 16.195385 -> 12.995312

DIFFERENT: 0 missing, 0 extra, 2 payload difference(s)
```

The logs are lined up (two emulator logs on their own timestamps, otherwise on their first frames), each stream
is lined up on its own (its offset is reported), and events are paired in order within a tolerance; only the time
both logs cover is compared. It exits with 0 when the logs agree, 1 when they differ, 2 on errors, for CI.

| Option | What it does |
|---|---|
| `--align start\|none\|auto` | First frames at time 0, timestamps as logged, or search for the offset that pairs the most events |
| `--offset T` | B's timestamps minus A's (e.g. `-1.5s`), instead of `--align` |
| `--tolerance T` | How far apart paired events may be (20 ms, at most half a stream's period) |
| `--no-stream-align` | Pair on the global alignment only |
| `--full` | Also count events outside the time both logs cover |
| `--ignore-payload STREAM` | Compare a stream's presence and timing only (e.g. `FirmwareStatus`, whose uptime differs between a board and its emulation) |
| `--only STREAM` | Compare only streams whose name contains this |
| `--raw` | Compare frames by identifier, without decoding |
| `--iface-map OLD=NEW` | Renames interfaces (two logs with one interface each are matched without it) |
| `--details N` | Examples shown per difference (5) |
| `--max-skew T` | Also fail if paired events are further apart than this |
| `--robot FILE` | Names each client's board in the report (`internal client 1 (poac) ...`) |

`replay-pool-test` does all of this for a whole robot's pool test, lines a hardware log's clock up on the
boards' uptimes, and reports faults and the emulated boards' errors too ([calibration.md](calibration.md#replaying-a-pool-test)).
Holding a real board against the emulator (PLAN.md, "testing against hardware"; **to verify locally**, no
hardware log was available here): record the board with `candump -l can0 > hw.log` while titan_bridge runs,
replay the robot computer's side into the emulated board (`--replay hw.log --replay-filter sender=host
--candump-log sim.log`), and compare with `can-diff hw.log sim.log --align auto --ignore-payload FirmwareStatus`.

## CAN over UDP (cannelloni)

`--cannelloni LOCAL,PEER` (`run`) or `--cannelloni BUS=LOCAL,PEER` (`robot`) tunnels a bus over UDP in the
framing of [cannelloni](https://github.com/mguentner/cannelloni) (protocol version 2), for host tools on a
machine, VM or container that cannot share the emulator's vcan. On that side `firmware_simulator can-tunnel vcan0
LOCAL,PEER` connects a local vcan to it. Use `--realtime`.

```bash
firmware_simulator robot robots/talos.toml --realtime --cannelloni internal=0.0.0.0:20000,10.0.0.2:20000
# on 10.0.0.2, after scripts/vcan_up.sh:
firmware_simulator can-tunnel vcan0 0.0.0.0:20000,10.0.0.1:20000
```

With `LOCAL` alone (`--cannelloni internal=127.0.0.1:20000`) the emulator is a **hub**: it needs no peer
address, learns its peers from the datagrams they send (an empty one, frame count 0, is a keepalive), sends each
the bus's frames and every other peer's, as on a shared vcan, and forgets a peer it has not heard from for 10 s.
Host tools that speak cannelloni themselves connect with no vcan anywhere: titan_bridge's `--interface
cannelloni:127.0.0.1:20000`, or `can-tunnel` for a far vcan (it sends keepalives; the `cannelloni` program does not,
so use a tunnel with it). A hub binds the address it is given; `0.0.0.0:20000` takes peers from other machines,
VMs and containers too (`host.docker.internal:20000` from Docker Desktop).

Tested between two emulated buses over loopback (`can_log.rs`: a power board reaches a mock bridge through a
tunnel and through a hub, with a second peer listening), from the command line (`cli_live.rs`: a peer that only
knows the hub's address gets every Talos board's heartbeat), and with the real titan_bridge over the hub
([below](#without-vcan-udp)). **To verify locally**: `can-tunnel` itself (needs vcan) and interoperation with the
real cannelloni program (the framing was written from the protocol description).

## Without vcan: UDP

vcan needs Linux with the `vcan` module and root (or sudo) to make the interfaces. WSL2's stock kernel has no
`vcan`, macOS has no SocketCAN, and a laptop without sudo cannot make them. For those, everything the robot
computer runs reaches the emulated buses over UDP on localhost instead, and the firmware is unchanged:

| Robot computer's side | On vcan | Without vcan |
|---|---|---|
| C boards' micro-ROS sessions | `micro_ros_agent can -D can0` (osu-uwrt's fork) | `--micro-ros-agent internal=127.0.0.1:8888` and any micro-ROS agent's `udp4` transport |
| Rust boards | `titan_bridge --interface can0` | `--cannelloni internal=127.0.0.1:20000` and `titan_bridge --interface cannelloni:127.0.0.1:20000` |
| The controller's raw thruster frames | written to `can0` | none (no `can0`): `command/thruster_rpm` instead ([nereus.md](nereus.md#the-bridge-node)'s `--rpm-from-forces`) |
| candump, canmore_cli, upload_tool | on `can0` | not available (they are SocketCAN tools) |

**`--micro-ros-agent BUS=HOST[:PORT]`** (`HOST[:PORT]` with `run`) plays the CAN micro-ROS agent on the bus:
titan_firmware's micro-ROS CAN transport is packet oriented (one XRCE-DDS packet per CANmore message), so it
reassembles each board's messages and sends every packet to the agent as one UDP datagram, from a socket of that
board's own (the agent sees each board as its own UDP client, as boards on a network would be), and hands what the
agent sends to that socket back to the board as CANmore messages, classic or CAN FD as the board sends. It does
nothing else: titan_link messages (Rust boards), heartbeats and thruster commands are left alone. Any micro-ROS
agent with the `udp4` transport serves it: riptide's `micro_ros_agent` package (`ros2 run micro_ros_agent
micro_ros_agent udp4 --port 8888`), or a standalone `MicroXRCEAgent udp4 --port 8888`
([nereus.md](nereus.md#the-micro-ros-agent) has how to build one). Use `--realtime`: the agent's timeouts run on
the wall clock.

```bash
firmware_simulator robot robots/talos_c.toml --micro-ros-agent internal=127.0.0.1:8888 --micro-ros-agent external=127.0.0.1:8888 --realtime --time 100000s
MicroXRCEAgent udp4 --port 8888        # or: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
ros2 topic list | grep /talos          # /talos/state/firmware, /talos/state/kill, ...

firmware_simulator robot robots/talos.toml --cannelloni internal=127.0.0.1:20000 --cannelloni external=127.0.0.1:20001 --realtime --time 100000s
firmware_simulator bridges robots/talos.toml --can internal=cannelloni:127.0.0.1:20000 --can external=cannelloni:127.0.0.1:20001 --format launch -o bridges.launch.py
ros2 launch ./bridges.launch.py
```

titan_bridge's `cannelloni:` interface is in titan_firmware_rs's `m12-titan-bridge-udp` branch (not yet in the
submodule this repository pins, which only the emulator's tests use; the bridge you run is your own build). The
scripts choose by themselves: `scripts/robot_in_a_box.sh` and `scripts/nereus_firmware_sim.sh` take `--transport
auto|vcan|udp` (`--no-vcan` is `--transport udp`), and `auto` uses vcan when `can0` and `can1` are vcan interfaces
already or the module is there (with root or sudo), and UDP otherwise; they print which and why.

What runs where with UDP is up to you: the emulator's addresses can be another machine's. On a Mac, for example,
the emulator can run natively while ROS runs in a Linux VM: `--micro-ros-agent internal=VM_IP:8888` and an agent in
the VM, or `--cannelloni internal=0.0.0.0:20000` and the VM's titan_bridges on `cannelloni:MAC_IP:20000`.

Verified here, with no vcan (ROS 2 Humble from uwrt_nix's `ros` shell, Fast DDS): the C PoAC, both ESC boards and
the camera cage (`robots/talos_c_nereus.toml`) through `--micro-ros-agent` and a stock Micro-XRCE-DDS-Agent v2.4.2
(`MicroXRCEAgent udp4`): all four sessions, every firmware topic in `ros2 topic list`, `state/firmware` and
`state/kill` echoed, the software kill asserted and released through `command/software_kill`, and
`command/thruster_rpm` spinning the ESCs (`state/thrusters/rpm`, the dashboard). The same with the Rust images and
four real titan_bridges on a hub. In the tests: `microros_udp.rs` (sessions per client, errors), `cli_live.rs` (the
real C PoAC's packets reach a UDP socket, when `TITAN_FIRMWARE_DIR` has it built).

## titan_bridge command lines: `bridges`

`firmware_simulator bridges ROBOT` prints the `titan_bridge` command for each Rust board of a robot file:

```
$ firmware_simulator bridges robots/talos.toml
# Emulator: firmware_simulator robot robots/talos.toml --can internal=vcan0 --can external=vcan1 --realtime
# poac (poac_board) on internal (vcan0)
titan_bridge --board poac --interface vcan0 --namespace talos --client-id 1
# esc0 (esc_board) on internal (vcan0)
titan_bridge --board esc0 --interface vcan0 --namespace talos --client-id 2
# esc1 (esc_board) on internal (vcan0)
titan_bridge --board esc1 --interface vcan0 --namespace talos --client-id 3
...
# ivc (ivc_board2) on internal (vcan0)
titan_bridge --board ivc2 --interface vcan0 --namespace talos --client-id 8
# battery (smart_battery) on external (vcan1)
titan_bridge --board smart_battery --interface vcan1 --namespace talos --client-id 21
```

The `--board` profile comes from the firmware image's name (titan_bridge's profiles are mirrored, and a test
checks them against titan_bridge's sources), the ESC board's ID from its GPIO 29 strap and the smart battery's
client ID from the serial number in its board file's `[[flash]]`. The Nth bus of the file is `vcanN` unless
`--can BUS=IFACE` says otherwise. Boards without a Rust bridge (C images, examples, Puddles over UDP) get a note.

| Option | What it does |
|---|---|
| `--format lines\|script\|launch` | One command per line (default), a bash script that checks the interfaces and starts them all, or a ROS 2 Python launch file |
| `-o, --output FILE` | Write to a file (a script is made executable) |
| `--can BUS=IFACE` | Interface of a bus |
| `--namespace NS` | ROS namespace for every bridge instead of each board's default |
| `--program PATH` | The titan_bridge program |
| `--probe`, `--probe-time T` | Boot the robot for `T` (1.5 s) and take each board's client ID from its CANmore heartbeats |

## Robot in a box: the Orin's side on vcan

(Or on UDP where there is no vcan: `--transport udp`, the default without the module; see
[Without vcan](#without-vcan-udp). The controller's raw thruster frames then go nowhere.)

> **To verify locally.** None of this could run where this guide was written (no vcan module, no
> `CAP_NET_ADMIN`, no ROS 2, no Docker daemon). The scripts and generated files are checked statically
> (`tests/host_cli.rs`: the vcan script against stand-ins for `ip` and `modprobe`, `robot_in_a_box.sh` and
> `nereus_firmware_sim.sh` in `--dry-run`, the bridges script with `bash -n`, the launch file with `py_compile`,
> the compose file's consistency); the C robot files load with stand-in images (`tests/cli.rs`). Since then, in a
> Nix shell with ARM GCC and the micro-ROS library (still no vcan or Docker daemon), all eight C images of
> `robots/talos_c.toml` were built and run: every board boots through its bootloader into its app and sends its
> CANmore heartbeat (`crates/sim_test/tests/c_boards.rs`), except the SBC board (see "The C images" below); four of
> them have run with a real micro-ROS agent over UDP ([Without vcan](#without-vcan-udp)). Still to verify:
> everything on vcan, `can1`'s agent and the smart battery's session, and riptide.

**What the Orin does.** Talos's robot computer (an NVIDIA Jetson AGX Orin, JetPack with ROS 2 Humble, the
`riptide_release` workspace) talks to the boards over plain Linux SocketCAN through the `mttcan` driver: `can0` is
the internal bus (1 Mbit/s; titan_firmware runs it as CAN FD with a 5 Mbit/s data phase) and `can1` the external
bus (250 kbit/s), set up by `riptide_release/scripts/config_target/configure_jetpack.bash`. The boards run
**titan_firmware's C apps with micro-ROS over CAN**, and riptide_hardware2's `copro_agent.launch.py` (part of
`hardware.launch.py`) serves them with `micro_ros_agent can -D can0` and, on Talos, `can -D can1` (osu-uwrt's
Micro-XRCE-DDS-Agent fork with the `can` transport). The controller (`complete_controller`, riptide_control's
Simulink model) does not go through ROS for the thrusters: `send_thruster_cmd_canbus.c` writes raw CANmore
utility frames to `can0` (channel 0, four big-endian RPMs per ESC board, clients 2 and 3), which both the C and the
Rust ESC firmware take. `titan_bridge` (titan_firmware_rs's host side) is not used by any riptide repository yet.

**In a box.** `scripts/robot_in_a_box.sh` puts the emulated robot on vcan interfaces named `can0` and `can1`, so
all of that runs unchanged:

```bash
export TITAN_FIRMWARE_DIR=~/osu-uwrt/titan_firmware    # the C images, built with scripts/build_c_boards.sh
scripts/robot_in_a_box.sh                    # robots/talos_c.toml, plus the micro-ROS agents if ROS is sourced
scripts/robot_in_a_box.sh --stack bringup    # also riptide's bringup (hardware:=none): controller, EKF, autonomy
scripts/robot_in_a_box.sh --rust             # titan_firmware_rs images (robots/talos.toml) and one titan_bridge each
scripts/robot_in_a_box.sh --dry-run          # what it would run
```

It refuses a `can0` that is not a vcan interface (a real adapter would get the emulator's frames), creates the
interfaces with `scripts/vcan_up.sh can0 can1` (the CAN FD MTU of 72 bytes, which the internal bus needs), starts

```bash
firmware_simulator robot robots/talos_c.toml --can internal=can0 --can external=can1 --realtime --time 1000000s --dashboard 127.0.0.1:8080
```

in the background, and then, when it finds a sourced ROS 2 with the riptide workspace, the robot computer's side:
`ros2 launch riptide_hardware2 copro_agent.launch.py robot:=talos` for the C images, or the launch file
`firmware_simulator bridges ROBOT --format launch` writes for the Rust ones; without ROS it prints those commands.
`ros2 topic list` then shows the boards' topics under `/talos` (`state/firmware`, `state/kill`,
`state/thrusters/rpm`, `state/depth/raw`, ...), and `candump can0` the traffic.

**The C images.** `robots/talos_c.toml` runs the PoAC, both ESC boards, the camera cage, the actuator, LED, SBC
and smart battery apps from `${TITAN_FIRMWARE_DIR}`, each booted through the bootrom and its CAN bootloader as on a
powered board, in Talos's real thruster layout (`vehicles/talos_nereus.toml`, the emulator's own physics). Each
project builds in its own `build/` directory into `NAME_with_bl.uf2`; the apps need titan_firmware's micro-ROS
library, which is built in Docker once:

```bash
git -C $TITAN_FIRMWARE_DIR submodule update --init --recursive
(cd $TITAN_FIRMWARE_DIR/lib/micro_ros_pico && ./build.sh)                      # Docker
scripts/build_c_boards.sh                                                      # all of them, or
cd $TITAN_FIRMWARE_DIR/talos/ESCBoard && mkdir -p build && cd build && cmake .. && make   # one by hand
```

| Board | Project | Image |
|---|---|---|
| `poac` | `talos/PoacBoard` | `poac_board_with_bl.uf2` |
| `esc0`, `esc1` | `talos/ESCBoard` | `esc_board_with_bl.uf2` (one image; GPIO 29 strapped high makes ESC board 1) |
| `camera_cage` | `talos/CameraCageBB` | `camera_cage_bb_with_bl.uf2` |
| `actuator` | `talos/Actuator` | `actuator_mk2_with_bl.uf2` |
| `led` | `talos/LEDBoard` | `led_board_with_bl.uf2` |
| `sbc` | `SBCBoard` | `sbc_board_with_bl.uf2` |
| `battery` | `talos/SmartBattery` | `smart_battery_with_bl.uf2` (external bus) |

Built at titan_firmware 058df2a with the Nix shell's micro-ROS library (`ln -s $MICRO_ROS_LIB_DIR
$TITAN_FIRMWARE_DIR/lib/micro_ros_pico/libmicroros`), two apps need help, which `scripts/build_c_boards.sh` gives
without changing the checkout: `SBCBoard` includes `driver/canbus.h` and `micro_ros_pico/transport_can.h` but its
CMakeLists.txt links neither library (the script links them with a CMake project include), and `talos/Actuator`
uses `riptide_msgs2/ActuatorStatus.CLAW_CLOSED`, which the library's newer riptide_msgs2 split into
`CLAW_BOTTLE_CLOSED` and `CLAW_SPOON_CLOSED` (the script defines it as the first; the later states moved up by one,
so the actuator's claw states read differently against today's messages). With no agent on the bus, each board
boots through its bootloader (its heartbeat about 80 ms after power-on) into its app, prints its build tag and
`Clean Boot: Power-on Reset` on UART0, and sends a CANmore heartbeat every 500 ms and micro-ROS's session requests;
the smart battery, alone on the external bus without the Orin to acknowledge, retransmits each heartbeat three
times. The SBC board's app (work in progress) never calls `canbus_tick`, so it falls silent once its bootloader
hands over ([faults.md](faults.md#firmware-findings-so-far)). The eight C boards are much slower to emulate than the
Rust images (their main loops never sleep): about 0.1x real time on four busy cores, the four of
`talos_c_nereus.toml` about 0.2x.

The micro-ROS agent's sessions and timeouts are on the wall clock and the firmware's on virtual time; Talos as a
whole runs slower than real time on one core (see [Speed](#how-a-robot-runs)), so watch the emulator's "behind the
wall clock" warnings. On threads (the scripts' default `--threads 0`) the boards run ahead of each other with the
vcan interfaces too: Talos keeps up with three free cores (about 1.1x before pacing; slice by slice, as before
this, it was about 0.7x with a live endpoint), and the emulator stays within 2 ms of the wall clock. Add
`--arrivals-log FILE` to keep a session's host traffic: `--replay-arrivals FILE` in place of `--can` repeats it,
without vcan or ROS. With nereus as the physics instead of the emulator's own, see [nereus.md](nereus.md).

**The Rust images and titan_bridge.** The same with `--rust`, or by hand:

```bash
scripts/vcan_up.sh
firmware_simulator robot robots/talos.toml --can internal=vcan0 --can external=vcan1 --realtime --time 100000s
firmware_simulator bridges robots/talos.toml --format script -o bridges.sh   # run it in riptide-shell
firmware_simulator bridges robots/talos.toml --format launch -o bridges.launch.py   # or: ros2 launch
candump -l vcan0                    # the internal bus
canmore_cli                         # lists the boards on every can*/vcan* interface
upload_tool -b talos/ivc_board path/to/ivc_board   # over CAN, to a board running the C Titan bootloader
```

For a single board: `run IMAGE --board ... --can vcan0 --realtime`. `titan_bridge` drops the link after 700 ms
without pings, so check the emulator's "behind the wall clock" warnings (a robot on threads runs its boards ahead
of each other with `--can` as well; see [Running ahead](#how-a-robot-runs)).

`docker/robot_computer/` has a ROS 2 Humble image with titan_bridge, upload_tool and canmore_cli and a compose
file that runs it next to the emulator on the host's vcan interfaces (both with `network_mode: host`); see
[its README](../docker/robot_computer/README.md) for usage, the riptide message dependencies, arm64 under
qemu-user, and the UDP tunnel for setups without a shared vcan. It has never been built or run.
