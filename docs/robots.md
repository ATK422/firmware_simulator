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
- [CAN logs: recording, replaying, comparing](#can-logs-recording-replaying-comparing)
- [CAN over UDP (cannelloni)](#can-over-udp-cannelloni)
- [titan_bridge command lines: `bridges`](#titan_bridge-command-lines-bridges)
- [Robot in a box: vcan, titan_bridge, Docker](#robot-in-a-box-vcan-titan_bridge-docker)

## Robot files

| File | What it is |
|---|---|
| `robots/talos.toml` | Talos: eight boards on the internal CAN FD bus (PoAC, two ESC boards, camera cage, actuator, LED, SBC, IVCBoard2), the smart battery on the external classic bus, and the Talos vehicle |
| `robots/puddles.toml` | Puddles: the backplane over Ethernet, in the Puddles vehicle |
| `robots/ivc_pool.toml` | Two vehicles in `pool.toml`, each with a first-generation IVCBoard on its own bus |
| `robots/ivc2_pool.toml` | The same with IVCBoard2 |

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

* **Power-on offsets.** Boards never power up in perfect lockstep; `power_on_ms` keeps a board unpowered for that
  long. Talos's second ESC board starts 3 ms late, the IVC pools' second board 37 ms late.
* **Straps.** `gpio` entries replace the board file's `[[gpio]]` for the same pins: Talos's `esc1` has its ID
  strap (GPIO 29) high, which makes it ESC board 1 (client 3); the first-generation IVC pool's surface board has its
  board ID pin (GPIO 6) high.
* **Environments and positions**: a board with a transducer in a robot with an `[environment]` needs
  `position_m`, a `trajectory` or a vehicle that carries it ([acoustics.md](acoustics.md)).
* **Vehicles**: `[[vehicle]]`, `[[vehicle.sensor]]`, `[[vehicle.transducer]]` ([vehicles.md](vehicles.md)).

## The `robot` command

```bash
firmware_simulator robot robots/talos.toml --mock-bridge internal:1 --mock-bridge external:21 --time 2s
firmware_simulator robot robots/ivc2_pool.toml --mock-bridge talos:8 --mock-bridge surface:8 --time 15s
firmware_simulator robot robots/puddles.toml --eth-mock-bridge backplane --time 10s --vehicle-csv puddles.csv
```

Each board's log lines are prefixed with its name (`[0.200400s] poac: 0.068333 INFO  ROS connected`, UART lines
as `[t] NAME uart0: ...`); the run ends with the stop time, the wall time and each vehicle's pose.

| Option | Default | What it does |
|---|---|---|
| `-t, --time T` | `5s` | Virtual time to run |
| `--mock-bridge BUS:CLIENT` | | A mock bridge for one client on one bus (repeatable, one per client) |
| `--eth-mock-bridge BOARD` | | A mock bridge on an Ethernet board's cable (Puddles) |
| `--udp BOARD:LOCAL=PEER` | | Relays an Ethernet board's UDP through a host socket, e.g. `backplane:127.0.0.1:2203=127.0.0.1:2204` |
| `--can BUS=IFACE` | | Bridges a bus to a SocketCAN interface, e.g. `internal=vcan0` (repeatable) |
| `--candump` | | Prints every frame with its bus |
| `--candump-log FILE`, `--replay FILE`, `--replay-filter`, `--replay-offset`, `--replay-iface IFACE=BUS`, `--cannelloni BUS=LOCAL,PEER` | | [Below](#can-logs-recording-replaying-comparing) |
| `--fault TIME@BOARD:FAULT` | | A fault on one board, e.g. `1.5s@camera_cage:can-unplug` (repeatable; bridge faults act on the first mock bridge on the board's bus; [faults.md](faults.md)) |
| `--noise-db L` | | Replaces every hydrophone's ambient noise with white noise at `L` dB re 1 µPa²/Hz |
| `--vehicle-csv FILE` | | Writes each vehicle's state every 10 ms (`NAME` in the file name for several vehicles) |
| `--realtime` | off | Paces virtual time to the wall clock |
| `--threads N` | 1 | Runs the boards on `N` threads within each slice (0: one per core) |
| `--trace KINDS`, `--trace-board NAMES`, `--trace-file FILE` | | Traces the boards ([below](#traces-profiles-waveforms-and-the-dashboard)) |
| `--profile-dir DIR`, `--profile-interval T` | `10us` | Profiles every board's firmware |
| `--vcd FILE`, `--vcd-pins SPEC`, `--vcd-events` | all pins | A VCD waveform with a scope per board (and per bus) |
| `--dashboard ADDR`, `--dashboard-hold` | | A live dashboard of all boards, buses and vehicles |
| `--log-json FILE` | | Every log line, UART line, CAN frame and fault as JSON lines |

It exits with an error if a board stops (a lockup, say) before `--time`.

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

**Threads.** Within a slice the boards do not see each other, so `--threads N` (`Robot::threads` in code) runs
them side by side on a pool of worker threads, and each slice ends exactly as if they had taken turns (a test
checks Talos frame by frame). Robots in an acoustic environment always take turns, since a hydrophone can hear
another board within the slice. Handing a board to a worker costs about 15-20 µs of CPU per board and slice, so
it only pays off with a few heavy boards on an otherwise idle multi-core machine; on the 4-core shared machine it
was measured on, Talos was not faster with 4 threads. The default is 1. Emulator log lines from different boards
can interleave differently with threads; what the boards do does not change.

**Speed.** Talos (nine boards) runs at about 0.4x real time on one core of the shared machine it was measured
on, so `--realtime` with all of Talos is not expected to keep up; the template runs about 60x faster than real
time, IVCBoard2 about 3.4x and the ESC board about 1.1x. `cargo run --release -p sim_test --example bench` measures it
([internals.md](internals.md#performance)).

## Mock bridges

A mock bridge is a scripted stand-in for `titan_bridge` built on the real `canmore` and `titan_link` crates. On a
CAN bus it serves one client: a `Ping` every 100 ms, the `state/kill` kill switch forwarded with each (released,
so the vehicle is enabled), and a record of everything the board sends (titan_link messages, CANmore heartbeats,
utility frames, decoding errors). On an Ethernet cable (`--eth-mock-bridge`, or `run --mock-bridge` on Puddles) it
speaks the Puddles UDP transport from the robot computer's address. Like a Linux CAN socket it queues at most 10
frames and drops pings that do not fit.

In code it is `sim_host::mock_bridge::MockBridge` (`PuddlesBridge` for Ethernet): `send` a `HostToDevice`, read
`state().received`, `stop`/`start` it, `set_kill`, `fork` it for a snapshot.

## CAN logs: recording, replaying, comparing

These options work with `run` (a board's own bus) and `robot` (every bus).

### Recording

`--candump-log FILE` writes the traffic in `candump -l` format: every acknowledged frame (what a listening
interface would receive), stamped with the virtual time its last bit was sent, with the bus name as the
interface. Unacknowledged frames (sent before anyone else was up) are counted and left out. The file is flushed
every 0.5 s of wall time, so a Ctrl-C'd real-time run leaves a usable log.

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

Tested between two emulated buses over loopback (`can_log.rs`: a power board reaches a mock bridge through the
tunnel). **To verify locally**: `can-tunnel` itself (needs vcan) and interoperation with the real cannelloni
program (the framing was written from the protocol description).

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

## Robot in a box: vcan, titan_bridge, Docker

> **To verify locally.** None of this could run where this guide was written (no vcan module, no
> `CAP_NET_ADMIN`, no ROS 2, no Docker daemon). The scripts and generated files are checked statically
> (`tests/host_cli.rs`: the vcan script against stand-ins for `ip` and `modprobe`, the bridges script with
> `bash -n`, the launch file with `py_compile`, the compose file's consistency).

`scripts/vcan_up.sh` loads the vcan module and creates `vcan0` and `vcan1` (or the interfaces named) with the CAN FD
MTU (72) and a longer transmit queue; `--down` removes them. Then the emulated robot is on those interfaces and
the real host tools run against it unchanged, with one `titan_bridge` per Rust board:

```bash
scripts/vcan_up.sh
firmware_simulator robot robots/talos.toml --can internal=vcan0 --can external=vcan1 --realtime --time 100000s
firmware_simulator bridges robots/talos.toml --format script -o bridges.sh   # run it in riptide-shell
firmware_simulator bridges robots/talos.toml --format launch -o bridges.launch.py   # or: ros2 launch
candump -l vcan0                    # the internal bus
canmore_cli                         # lists the boards on every can*/vcan* interface
upload_tool -b talos/ivc_board path/to/ivc_board   # over CAN, to a board running the C Titan bootloader
```

For a single board: `run IMAGE --board ... --can vcan0 --realtime`. Talos as a whole runs slower than real time
on one core (see [Speed](#how-a-robot-runs)); `titan_bridge` drops the link after 700 ms without pings, so check
the emulator's "behind the wall clock" warnings.

`docker/robot_computer/` has a ROS 2 Humble image with titan_bridge, upload_tool and canmore_cli and a compose
file that runs it next to the emulator on the host's vcan interfaces (both with `network_mode: host`); see
[its README](../docker/robot_computer/README.md) for usage, the riptide message dependencies, arm64 under
qemu-user, and the UDP tunnel for setups without a shared vcan. It has never been built or run.
