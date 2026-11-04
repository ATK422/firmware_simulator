# Scenario tests in YAML, and firmware A/B comparisons

A scenario file describes a firmware integration test without Rust: what to run (a robot file, or one board and its
image), the mock `titan_bridge`s, the inputs, how long, and a list of steps. Steps are actions (faults, inputs,
titan_link messages, IVC bytes) and checks on what the boards log, what the bridges decode, the frames on the buses
and the boards' state. `firmware_simulator scenario` runs them and says for each step whether it held, when, and
on what evidence; `firmware_simulator compare` runs them with two firmware builds and reports what changed.

- [A first scenario](#a-first-scenario)
- [Running scenarios](#running-scenarios): the command, its output, JUnit and JSON, CI, from Rust
- [The file](#the-file): top-level keys, paths, several scenarios in one file
- [Steps and time](#steps-and-time): the cursor, `at`, `after`, `within`, `until`
- [Actions](#actions): `fault`, `input`, `send`, `ivc_send`, `pause`
- [Checks](#checks): `expect`, `never`, `always`, and what they look at
- [Cases: one boot, many branches](#cases-one-boot-many-branches)
- [The shipped scenarios](#the-shipped-scenarios), and writing ones that outlive a firmware change
- [Firmware A/B comparisons](#firmware-ab-comparisons)

## A first scenario

`scenarios/can_cable_pull.yaml` pulls the camera cage's CAN cable on a three-board Talos and plugs it back in:

```yaml
name: Camera cage recovers after its CAN cable is pulled
robot: {file: robots/talos.toml, boards: [poac, camera_cage, sbc]}
bridges: all
time: 17s
steps:
  - expect: {board: camera_cage, log: ROS connected, within: 1s}
  - at: 2s
    fault: camera_cage:can-unplug
  - expect: {board: camera_cage, log: Lost connection to ROS, within: 1s}
  - expect: {board: camera_cage, link: down, within: 1s}
  - at: 4s
    fault: camera_cage:can-plug
  - expect: {board: camera_cage, log: ROS connected, within: 500ms}
  - expect: {board: camera_cage, link: up, within: 12s}
  - expect: {bus: internal, frame: {id: 0x12F, extended: false}, count: {min: 2}, within: 1.5s}
  - expect: {board: camera_cage, status: {board_name: mk2/camera_cage_bb, clear: [FAULT_CAN_INTERNAL_ERROR]}, within: 1.5s}
  - never: {board: poac, log: Lost connection to ROS}
  - never: {board: sbc, log: Lost connection to ROS}
  - always: {board: poac, link: up, at: 1s}
  - never: {board: camera_cage, log: PANIC}
```

```bash
cargo build --release
firmware_simulator scenario scenarios/can_cable_pull.yaml
```

```
● Camera cage recovers after its CAN cable is pulled  (scenarios/can_cable_pull.yaml)
  ✓      0.200400s  expect camera_cage log "ROS connected" within 1s
                      [0.200400s] camera_cage: 0.068458 INFO  ROS connected
  ✓      2.000000s  fault camera_cage: can-unplug at 2s
                      applied to camera_cage
  ✓      2.600400s  expect camera_cage log "Lost connection to ROS" within 1s
                      [2.600400s] camera_cage: 2.468468 INFO  Lost connection to ROS
  ✓      2.601000s  expect camera_cage link down within 1s
                      last link heartbeat at 1.900643s (down at 2.601000s)
  ✓      4.000000s  fault camera_cage: can-plug at 4s
                      applied to camera_cage
  ✓      4.000800s  expect camera_cage log "ROS connected" within 500ms
                      [4.000800s] camera_cage: 3.868862 INFO  ROS connected
  ✓      4.001000s  expect camera_cage link up within 12s
                      last link heartbeat at 4.000812s (up at 4.001000s)
  ✓      5.501000s  expect frame 0x12f on internal count >= 2 within 1.5s
                      3 matching from 4.001000s to 5.501000s (count >= 2)
  ✓      6.001145s  expect camera_cage status clear=FAULT_CAN_INTERNAL_ERROR board_name=mk2/camera_cage_bb within 1.5s
                      [6.001145s] internal:4 status: mk2/camera_cage_bb client 4: faults 0x0 (none), uptime 5.87 s, enabled
  ✓     17.000000s  never poac log "Lost connection to ROS" over the whole run
                      none from 0.000000s to 17.000000s
  ...
  PASS: 17.000000s of virtual time in 2.57s wall
1 scenario(s): 1 passed, 0 failed, 0 skipped
```

Each line is a step: ✓ held, ✗ did not, · was never reached (the run ended first). The time is when it was
decided: the matching line's time, the sample that satisfied it, the end of its window. Under it is the evidence:
the line, frame, status or state that decided it, or for a failure what was there instead (the window searched,
the board's last lines, the last status). The camera cage hears ROS again at once after the replug (4.0 s), and its
own traffic is back straight away too. With titan_firmware_rs before c59474f (the build pinned until 94a107e) that
traffic only came back at 11.6 s, when the CAN driver's offline reset freed the transmit queue that stalled while
unplugged ([faults.md](faults.md#firmware-findings-so-far), finding 1); the scenario's 12 s window passes with both,
and [the A/B section](#firmware-ab-comparisons) shows the comparison that catches the difference.

## Running scenarios

```bash
firmware_simulator scenario scenarios/                     # every *.yaml under it, on every core
firmware_simulator scenario scenarios/crash_record.yaml --junit results.xml --json results.json
firmware_simulator scenario scenarios/ --quiet --jobs 2    # only what failed, on two threads
firmware_simulator scenario scenarios/ --image /path/to/other/release   # another firmware build
firmware_simulator scenario --check scenarios/             # parse and list the steps, run nothing
```

| Option | Default | What it does |
|---|---|---|
| `FILE_OR_DIR...` | | Scenario files, or directories searched for `*.yaml` and `*.yml` (recursively, in name order) |
| `-j, --jobs N` | 0 | Scenarios run side by side on `N` threads (0: one per core); each prints as soon as it and those before it are done, so the output is in file order on any `N` |
| `--junit FILE` | | JUnit XML: a test suite per file, a test case per scenario or case, a `<failure>` naming the failed steps, the step list as its output |
| `--json FILE` | | The results as JSON (`-` prints it instead of the text): per scenario `name`, `case`, `file`, `passed`, `error`, `skipped`, `end_s`, `wall_s`, `images`, and `steps` (`number`, `kind`, `description`, `status`: `pass`/`fail`/`not_reached`, `time`, `evidence`) |
| `--image DIR\|IMAGE\|BOARD=IMAGE` | | Runs another firmware build (repeatable): a directory of images replaces every board's image whose file name is in it, an image every board running one of that file name, `BOARD=IMAGE` one board's |
| `-q, --quiet` | | Prints only the scenarios that did not pass, and the summary |
| `--check` | | Reads every file and lists its scenarios and steps as the runner describes them; exits 2 on a mistake |
| `--from-checkpoint NAME` | | Scenarios that reach the `pause: NAME` step the same way share one run up to it ([below](#checkpoints-one-boot-many-scenarios)) |
| `--coverage DIR` | | Firmware code coverage over every scenario: `DIR/lcov.info` (every image), `DIR/IMAGE.info` per image, `DIR/summary.txt` ([tools.md](tools.md#code-coverage)) |

**Exit status**: 0 when every scenario passed, 1 when one failed (or could not be set up: a missing image, an
unknown board), 2 when a file could not be read or has a mistake. Mistakes are found before anything runs, with the
file, the document, the step and the key ("scenarios/x.yaml: step 3: expect: the condition: unknown key \`withn\` (expected one of: ...)").

**Determinism.** Everything in a run is virtual time: the same files and the same images give the same steps,
times and evidence on any machine, with any `--jobs` and any scenario's `threads` (only `wall_s` in the JSON
differs). `crates/sim_test/tests/scenarios.rs` checks that on one, two and four threads.

**Images.** Robot files' images are redirected to `$TITAN_RS_RELEASE_DIR` when it is set, as for `robot`
([robots.md](robots.md#robot-files)); a `board:` scenario's `image: esc_board` is found there too. `--image` comes
on top of that.

### In `cargo test` and CI

`crates/sim_test/tests/scenarios.rs` runs every scenario in `scenarios/` as `the_shipped_scenarios_pass`, so
`cargo test --release --workspace` covers them, printing each one's steps; a scenario whose images are not built is
skipped (`skipping: ...`), or fails with `SIM_REQUIRE_TITAN_RS` set ([testing.md](testing.md#running-the-tests)).
The CI test job also runs them with the command, which writes `scenarios.xml` (JUnit) as an artifact of the run:

```bash
firmware_simulator scenario scenarios/ --junit scenarios.xml --quiet
```

### From Rust

`sim_test::scenario` is the same runner as a library: load files, run them, read the outcomes.

```rust,ignore
use sim_test::scenario::{self, Options};

let opts = Options { skip_missing: true, ..Default::default() };
let outcomes = scenario::run_paths(&["scenarios/crash_record.yaml".into()], &opts, 4).unwrap();
for o in &outcomes {
    print!("{}", scenario::human(o));       // or junit(&outcomes), json(&outcomes)
    assert!(o.passed(), "{}", o.title());
}
scenario::assert_pass(&[sim_test::repo().join("scenarios")], 0);   // what the test above does
```

`scenario::parse_str(text, path)` takes a scenario from a string (tests write small ones inline);
`Options::images` takes the same overrides as `--image`; `Options::record` keeps what the run produced (lines,
frames, statuses, resets, ESC speeds) for [comparisons](#firmware-ab-comparisons).

## The file

```yaml
name: ESC board recovers after its CAN cable is pulled   # required
description: Anything you like                          # optional
robot: robots/talos.toml                                 # a robot file, or
robot: {file: robots/talos.toml, boards: [esc0, poac]}   #   only some of its boards, or
board: {image: esc_board, file: mk2/esc_board}           # one board (give robot or board)
bridges: [internal:1, internal:2]                        # mock titan_bridges; or `all`
agents: [nanoros:led]                                    # mock agents in the board's protocol (nano-ROS)
inputs: {camera_cage.depth.pressure_pa: 101325}          # physical inputs at time 0
time: 15s                                                # virtual time to run (required)
threads: 1                                               # threads the boards may use (results are the same)
steps: [...]                                             # actions and checks, in order
cases: [...]                                             # branches from a snapshot after `steps`
```

| Key | Meaning |
|---|---|
| `name` | Shown in the output, the JUnit test case and the A/B report |
| `robot` | A robot file ([robots.md](robots.md#robot-files)), by path, or by name (`talos` is `robots/talos.toml`). With `{file, boards}` only the boards listed run, and vehicle thrusters, sensors and transducers wired to the others are left out: a two-board scenario of Talos runs several times faster than all nine boards |
| `board` | One board: `image` (a path, or a titan_firmware_rs image name such as `esc_board`), `file` (a board file: `mk2/esc_board` is `boards/mk2/esc_board.toml`, or a path), and optionally `name` (the board's name in checks; default the image's) and `rom_boot: true`. It is on a bus called `can` if its board file has a CAN controller |
| `bridges` | Mock bridges ([robots.md](robots.md#mock-bridges)), as on the command line: `BUS:CLIENT` for a CANmore client on a bus (`CLIENT` alone when there is one bus), `eth:BOARD` on an Ethernet board's cable (Puddles). `all` puts one on every board's client on its bus and on every Ethernet board. The ESC board's client comes from its ID strap and the smart battery's from its serial number, as `bridges` works them out ([robots.md](robots.md#titan_bridge-command-lines-bridges)). Each bridge pings every 100 ms with the kill switch released |
| `agents` | Mock agents ([robots.md](robots.md#mock-agents-nano-ros-and-other-protocols)), as `--mock-agent`: `nanoros:BOARD` for a nano-ROS board (its `ros_toml` from the robot file), `nanoros:BOARD=PATH` with its `ros.toml` (looked for from the scenario's directory up), `titan_link:BOARD` the same as a bridge for the board. Early: nano-ROS is still changing ([nanoros.md](nanoros.md)) |
| `inputs` | `BOARD.INPUT: VALUE` (or `INPUT` with one board), as in the board files ([boards.md](boards.md#inputs)); set before the boards start. A vehicle drives its sensors' inputs itself ([vehicles.md](vehicles.md)) |
| `time` | How long to run: `15s`, `1500ms`, or a number of seconds |
| `threads` | As `robot --threads` ([robots.md](robots.md#how-a-robot-runs)); 1 by default. Robots in water always take turns |
| `steps`, `cases` | Below |

**Paths** (robot files, board files, images) are looked for from the scenario's directory and then each
directory above it, so `robots/talos.toml` works from `scenarios/` and from any folder under the repository;
absolute paths, `~` and `$VARIABLES` work as in robot files.

**Several scenarios in one file**: separate them with a `---` line (YAML documents).
`scenarios/boot_boards.yaml` has one per board image.

**Times** are `1.5s`, `200ms`, `50us`, `10ns` or a bare number of seconds. **Numbers** may be hex (`0x76`).
**Ranges** are a number (exactly), `[min, max]` (inclusive), `{min: 5}`, `{max: 3}` or `{min: 1, max: 3}`.

## Steps and time

Steps run in order along a **cursor**: the time the previous step was decided. An action happens at the cursor
unless it says when; a check starts looking at the cursor unless it says from when. A step's time comes from four
keys, which may sit next to the step's kind or inside its map (`- at: 5s` + `fault: ...`, or
`expect: {..., within: 2s}`):

| Key | Meaning |
|---|---|
| `at: T` | Starts at virtual time `T` |
| `after: D` | Starts `D` after the cursor |
| `within: D` | The window ends `D` after the start |
| `until: T` | The window ends at virtual time `T` |

| Step | No window (`within`/`until`) | With a window |
|---|---|---|
| action | happens at the start | only `send` with `every` |
| `expect` | checked at the start: a state then; the latest status or message then; a line, frame or byte at any time before (from 0) | passes at the first moment it holds in the window, fails when the window closes; the cursor moves to that moment |
| `never`, `always` | over the whole run (from `at`/`after` if given) | over the window (from the cursor if no `at`/`after`) |

So `expect: {log: X, within: 2s}` waits up to 2 s from the cursor for a line, `expect: {log: X, at: 3s}` asks
whether it has been logged by 3 s, and `expect: {status: {faults: 0}, at: 14s}` looks at the last status received
by 14 s. `never` and `always` do not move the cursor, and those with a fixed window (`at`, or none: the whole run)
are watched from the start of the run, wherever they are in the list, so they are judged even when an earlier step
holds the timeline up.

**Actions take effect at the start of the next 100 µs slice** (the robot's slice) at or after their time, so
stopping for one never changes how the boards run ([robots.md](robots.md#how-a-robot-runs)). Lines, frames and
messages are judged by their own times; states (link, GPIO, ESC, vehicle, resets) are sampled every millisecond
while a check looks at one.

When a window reaches past `time`, the run ends first and the check fails with `the run ended at T before the
window closed at T'`; checks never reached are marked `·` with `the run ended at T first`. A failed check does not
stop the run: the steps after it go on from the end of its window.

Every step may have a `name:`, shown instead of the generated description.

## Actions

| Step | Example | What it does |
|---|---|---|
| `fault` | `fault: esc0:can-unplug`, `fault: {board: esc0, fault: can-unplug}`, `fault: brownout:50ms` (one board) | A fault in the `--fault` spelling ([faults.md](faults.md#every-fault)). Bus faults (`can-stuck`, `can-error-rate:P`, `can-flood:...`) may leave out the board when there is one bus. Bridge and kill switch faults act on the board's own mock bridge |
| `input` | `input: {camera_cage.depth.pressure_pa: 120000, port_battery: 13}` | Sets physical inputs |
| `send` | `send: {board: esc1, message: {Esc: {ThrusterRpm: [0, 0, 0, 0, 1200, 1200, 1200, 1200]}}}` | The board's bridge (or `bridge: internal:3`) sends a titan_link `HostToDevice` message, written as its variant names and fields (`{KillSwitch: {asserting_kill: true}}`, `{Power: {ElectricalCommand: 6}}`). With `every: 100ms` and `within`/`until`, again and again until then, the way titan_bridge forwards a topic |
| `send` (a topic) | `send: {board: led, topic: bit_sub, msg: {data: 5}}`, `send: {board: led, topic: imu_cmd, msg: {orientation: {z: 0.6, w: 0.8}}, every: 100ms, until: 3s}` | The board's mock agent (`agents`) sends a message on one of the board's subscribed topics, its fields as the `.msg` names them (missing ones are zero); before the board has registered it waits for it. On a board with only a bridge, the topic is a titan_link `HostToDevice` variant and `msg` its contents (`topic: KillSwitch, msg: {asserting_kill: true}`). `every` as above |
| `ivc_send` | `ivc_send: talos_ivc:0x5a`, `ivc_send: {board: surface_ivc, byte: 195}` | Makes an IVC board transmit a byte through its bridge, as `robot --ivc-send` ([robots.md](robots.md#mock-bridges)) |
| `pause` | `pause: after the replug` | A checkpoint, named by its text: its evidence is the state of every board (resets, what each core is doing, the last status, the last line) and the frames so far on each bus; with `--from-checkpoint` scenarios share the run up to it ([below](#checkpoints-one-boot-many-scenarios)) |

## Checks

`expect` must become true, `never` must not happen, `always` must hold throughout:

```yaml
- expect: {board: esc0, log: "ROS connected", within: 1s}
- expect: {board: esc0, log: {regex: "CAN offline for \\d+ ms"}, level: warn, within: 12s}
- expect: {board: camera_cage, status: {raised: [FAULT_DEPTH_INIT_ERROR]}, within: 2s}
- expect: {bridge: internal:1, status: {faults: 0}, at: 14s}
- expect: {bus: internal, frame: {id: 0x1020800, mask: 0x1FFFF00}, rate: [8, 12], within: 1s}
- always: {board: esc1, esc: {name: esc1, rpm: [-1, 1]}, until: 4.9s}
- expect: {vehicle: {depth: {min: 0.25}, roll: [-2, 2]}, within: 1.5s}
- expect: {board: surface_ivc, ivc: 0x5a, within: 10s}
- expect: {board: camera_cage, message: CameraCage.Depth, value: [-0.05, 0.05], count: {min: 15}, within: 1s}
- expect: {board: led, topic: imu_data, msg: {header: {frame_id: imu_link}, linear_acceleration: {z: {min: 9.7}}}, within: 1s}
- never: {board: esc0, log: PANIC}
- always: {board: poac, resets: 0}
```

A check names what it looks at with one of the keys below, and where with `board` (a board's name), `bridge`
(`BUS:CLIENT` or `eth:BOARD`; for status, link, IVC and messages a `board` means its own bridge) or `bus`. With one
board, or one bridge, these may be left out.

| Key | Kind | What it looks at |
|---|---|---|
| `log: TEXT` or `log: {regex: RE}` | occurrence | A line the board printed whose text contains `TEXT` (or matches `RE`, Rust regex syntax): defmt lines (their message, without the firmware's timestamp and level), UART and USB serial lines. `level: info` (`trace`, `debug`, `warn`, `error`) takes only defmt lines of that level |
| `status: {...}` | value stream | Each `FirmwareStatus` the bridge decodes (one a second); every check given must hold: `faults: 0` (the bitmask, exactly), `raised: [FAULT_X, ...]` and `clear: [...]` (by name, with or without `FAULT_`, from the board's own `FAULT_NAMES` table, or bit numbers), `killed: true` (a kill switch asserts kill), `kill_timed_out: true`, `uptime: {min: 5}` (seconds), `board_name`, `client_id`, and `max_age` (2.5 s: at an instant, how old the latest may be); `status: any` is any status |
| `link: up` / `down` | state | Whether the board's titan_link heartbeats reach its bridge (one in the last 300 ms) |
| `frame: {id, mask, extended, data}` | occurrence | An acknowledged frame on the bus with `frame_id & mask == id & mask` (the mask defaults to every bit), standard or extended, whose data starts with `data` (hex, `"01 0A"`); `frame: 0x12F` is the id alone, `frame: any` any frame. With `board`, only frames that board sent |
| `gpio: {pin, level}` | state | A pin's level, `high` or `low`; `pin` is a number or a name from the board file's `[pins]` (`status_ledg`) |
| `esc: {name, rpm}` | state | A DShot ESC's propeller speed (signed mechanical rpm) in a range; `name` from the board file (`esc1`) |
| `vehicle: {name, depth, north, east, speed, roll, pitch, yaw}` | state | A vehicle's position (m, depth positive down), speed (m/s) or attitude (degrees), each a range; `name` may be left out with one vehicle |
| `resets: RANGE` | state | Chip resets of the board since the start: watchdog resets and the power-on reset after a brownout |
| `lockups: RANGE` | state | Core lockups since the start ([faults.md](faults.md#what-the-models-do)) |
| `ivc: BYTE` (or `any`) | occurrence | A byte the board's IVC modem received acoustically, as its bridge decodes it |
| `topic: NAME`, `msg: {...}` | value stream | Each message on the topic the board's mock agent decodes (`agents`; on a board with only a bridge, its titan_link messages of that variant, `topic: FirmwareStatus`), whose fields match `msg`: each field given must be there and match (others are not looked at), nested messages as maps, lists item by item, `{min, max}` (either) a number in a range, numbers equal to within a millionth (a float32's rounding), anything else exactly; without `msg`, any message on the topic. A topic the board's `ros.toml` does not have is an error |
| `message: PATH`, `value: ...` | value stream | Each titan_link message from the board whose JSON form has `PATH` (`CameraCage.Depth`, `Power.Electrical.port_voltage`, `Heartbeat`), with the value there in a range (`value: [15, 17]`) or equal to a value (`value: true`, `value: "text"`); without `value`, any such message |

**`count` and `rate`** go with `expect` on occurrences and value streams: how many matching ones the window
holds (`count: 0` is "none", `count: {min: 3}`), or how many per second; they are judged when the window closes.

| | occurrence (log, frame, ivc) | value stream (status, message, topic) | state |
|---|---|---|---|
| `expect` | one matches | one matches (at an instant: the latest) | it holds at a sample |
| `never` | none matches | none matches | it holds at no sample |
| `always` | (use `never`) | every one matches, and there is at least one | it holds at every sample |

## Cases: one boot, many branches

`cases` runs several continuations of one run: the common `steps` run once, a snapshot of everything (boards,
buses, bridges, what was collected) is taken when they are done, and each case runs its own steps from its own
copy to the end of `time`. Each case is reported as `NAME / CASE`, with the common steps in it.

```yaml
name: Crash records after a hang, a panic and a brownout
board: {image: poac_board, file: talos/poac_board, name: poac}
bridges: [1]
time: 5s
steps:
  - expect: {log: ROS connected, within: 1s}
  - expect: {status: {faults: 0}, at: 1.5s}
cases:
  - name: hang
    steps:
      - fault: hang:0
      - expect: {resets: 1, within: 1s}
      - expect: {log: {regex: "Watchdog timeout after \\d+ ms"}, level: error, within: 500ms}
  - name: brownout
    steps:
      - fault: brownout:50ms
      - expect: {log: "Crash counts since power on: CrashCounts { watchdog_timeouts: 0, panics: 0 }", within: 500ms}
```

Steps are numbered through: a case's first step follows the last common one.

## Checkpoints: one boot, many scenarios

`cases` share a boot within one scenario. `--from-checkpoint NAME` does it across scenarios and files: every
scenario with a `pause: NAME` step that gets there the same way (the same setup, bridges, inputs, images, `time`,
`threads` and steps up to the pause) shares one run up to it. The first to get there keeps a checkpoint of
everything at the pause (boards, buses, bridges, what was collected, as `cases` do); the others go on from copies
of it, so their boot is not run again:

```yaml
# scenarios/unplug.yaml                 # scenarios/battery.yaml
name: Unplugged                         name: Battery sag
board: {image: power_board, file: mk2/power_board}   # (the same setup)
bridges: [1]                            bridges: [1]
time: 4s                                time: 4s
steps:                                  steps:
  - expect: {log: ROS connected, within: 1s}           - expect: {log: ROS connected, within: 1s}
  - pause: booted                         - pause: booted
  - fault: can-unplug                     - input: {stbd_battery: 11.0}
  - expect: {link: down, within: 1s}      - expect: {status: {faults: 0}, after: 1s}
```

```bash
firmware_simulator scenario scenarios/unplug.yaml scenarios/battery.yaml --from-checkpoint booted
```

The results are the same as without it, step for step and time for time (`crates/sim_test/tests/scenarios.rs`
checks that); only the wall time changes. A scenario whose later steps include a `never` or `always` watched
from the start of the run (no `after`, or a fixed `at`) runs on its own, as a copy taken at the pause could not
have watched them from the start.

A run with a dashboard keeps named checkpoints of its own to come back to: `POST /api/rewind {"save": NAME}`, then
`{"checkpoint": NAME}`, or the MCP server's `rewind` with `save` and `checkpoint` ([tools.md](tools.md#rewinding)).

## The shipped scenarios

`scenarios/` holds scenarios that pass with the current images. They assert what the firmware is meant to do, in
ways both the titan_firmware_rs pinned now (94a107e) and the one pinned before its fixes (342ee0d) share (where
those differ, a comment says how).

| File | What it checks |
|---|---|
| `boot_talos.yaml` | All nine Talos boards connect, report their identity with no fault, keep their links up, put their CANmore heartbeats on the internal bus, and do not panic |
| `boot_boards.yaml` | The same for every other image (power board, template, LED board 2, both IVC boards' other images, H-bridge, PO half board), and the Puddles backplane over Ethernet |
| `can_cable_pull.yaml` | A pulled and replugged CAN cable: ROS lost and regained, the link and the board's traffic back, the other boards unaffected |
| `depth_sensor_fault.yaml` | The depth sensor lost after zeroing (FAULT_DEPTH_ERROR, no depth published) or from power-on (FAULT_DEPTH_INIT_ERROR); working, depth at the surface throughout |
| `kill_switch.yaml` | The kill switch asserted, released and timed out, in the firmware status |
| `ivc_pool.yaml` | An IVC byte across the pool and back between two IVCBoard2s |
| `crash_record.yaml` | Crash records after a hang (watchdog timeout), a panic and a brownout (none) |
| `thrusters_dive.yaml` | Thruster commands spin ESC board 1's motors after its 5 s hold, the vehicle dives, the camera cage reports it, the kill switch stops the motors |

Writing one that lasts:

* **Check behaviour, not incidentals.** A log line's text is part of the firmware's interface only as far as you
  want it to be; prefer statuses, link state and messages where they say the same thing.
* **Give windows room** for what may change: `within: 12s` for the link after a replug passes with the firmware
  that stalls for 10 s and with the one that does not; the A/B comparison is where the difference shows.
* **Use `at`** (instead of a chain of `within`s) for things that happen in no particular order, such as boards
  connecting.
* **Run few boards**: `robot: {file: ..., boards: [...]}`. Busy boards (the ESC boards, IVCBoard2) run near or
  below real time; idle ones fast-forward.

## Firmware A/B comparisons

`firmware_simulator compare` runs each scenario twice, with build A and build B, and reports what changed:

```bash
# Two titan_firmware_rs builds: the pinned images, and those of another commit built in a scratch target dir
(cd ~/titan_firmware_rs/firmware && CARGO_TARGET_DIR=/tmp/fw-b cargo build --workspace --release)
firmware_simulator compare --a external/titan_firmware_rs/firmware/target/thumbv6m-none-eabi/release \
    --b /tmp/fw-b/thumbv6m-none-eabi/release --a-label pinned --b-label "my branch" scenarios/ -o ab.md --json ab.json
# One board's image only, against the scenario's own images
firmware_simulator compare --b esc0=/tmp/esc_board scenarios/thrusters_dive.yaml
```

| Option | Default | What it does |
|---|---|---|
| `--a`, `--b` `DIR\|IMAGE\|BOARD=IMAGE` | the scenario's images | The two builds, as `scenario --image` (repeatable) |
| `--a-label`, `--b-label TEXT` | what `--a`/`--b` gave | Their names in the report |
| `-o, --output FILE` | stdout | The Markdown report |
| `--json FILE` | | The comparison as JSON |
| `--tolerance T` | `20ms` | Times closer than this are the same |
| `--details N` | 8 | Examples shown per kind of difference |
| `--profile`, `--profile-interval T` | off, `50us` | Also profiles every board and compares CPU load ([tools.md](tools.md#profiler)) |
| `-j, --jobs N` | 0 | Runs side by side (A and B of a scenario run at the same time) |
| `--fail-on-regression` | | Exits with 1 when a step that passes with A does not with B (otherwise 0, 2 on unreadable files) |

The report (Markdown, for a pull request comment; details folded in `<details>`) has, per scenario (and case):

* **Images**: which boards ran different images (by a hash of the file) and which the same.
* **Checks**: how many steps passed in each, and every step whose result or time differs.
* **Boot and connect**: per board, when it first logged, brought its CAN controller online, connected to ROS and
  sent its first status, where those differ.
* **Faults raised**: each fault whose "Fault X raised" line, first appearance in a status, or presence at the end
  differs.
* **Resets and lockups**, where they differ.
* **Log lines**: per board, lines added, removed, changed (the same message with other numbers in it) and retimed
  (the same line more than the tolerance apart), lined up by content with a longest-common-subsequence diff over the
  lines' text with their numbers masked.
* **CAN traffic**, per bus, from [`can-diff`](robots.md#comparing-can-diff)'s comparison of the two runs' frames (by
  CANmore message: counts, periods, first-seen times, missing, extra, payload changes), every stream that differs.
* **ESC speeds**: mean, min, max and last rpm (sampled every 10 ms) of each ESC whose speeds differ.
* **CPU load** with `--profile`: each core's running share and the functions whose sample counts differ.

The JSON has all of it, unfiltered. Runs are deterministic, so two runs of the same build compare equal (the
report says "Every step has the same result at the same time", and so on); with `--profile`, profiling cuts the
runs at sample times, which on boards that use both cores can shift how the cores interleave.

What the comparison of titan_firmware_rs 342ee0d (pinned before) and 4d86d47 (its fixes, merged; now pinned as
94a107e, which only adds the toolchain pin and CI) found, from the scenarios here (`--profile` on the IVC pool):

* `can_cable_pull.yaml`: the camera cage's link is back 4.0 s after the replug instead of 11.6 s, and
  `FAULT_WATCHDOG_WARNING` is no longer raised (and latched) when ROS is lost; the `CAN offline for 10000 ms` line
  is gone; the camera cage's depth readings reach the bus from 4.0 s instead of 11.6 s (CAN retries without limit,
  findings 1 and 2).
* `crash_record.yaml`, case `panic`: `Watchdog timeout after 1360 ms` becomes `Reset by panic after 1360 ms`, the
  crash counts read `panics: 1` instead of `watchdog_timeouts: 1`, and the board is back 142 ms sooner (finding 3).
* `ivc_pool.yaml`: IVCBoard2's core 0 runs 82 % of the time instead of 86 %: `rom_table_lookup` drops from 62 000
  samples to none (`rom-func-cache`), and the surface board connects 82 ms sooner.
* `thrusters_dive.yaml`: the first `RpmFeedback` after the kill (7.10 s) reports the ESC commands of the tick
  before the kill, ±300, where 342ee0d reported 0. Expected, not a regression: the bridge forwards the kill with its
  next ping (7.100119 s), the thruster command comes 85 µs later and the feedback 0.67 ms after that, built from
  the last *completed* control-loop tick; whether core 1 has finished a tick with the kill latched by then is a
  race of a fraction of a millisecond, which the faster soft float (`rom-func-cache`) moves. The DShot frames go
  neutral within a control tick of the kill either way (`faults.rs::esc_thrusters_go_neutral_when_killed`), the
  next feedback (7.20 s) reports 0, and the ESCs spin down the same. Images from rustc 1.97.0 and 1.99.0 of the same
  source agree on it.
* Every image's version line changes (`git 342ee0d` to `git 4d86d47`).

Comparing builds of the *same* source from two compilers (`--b` a directory of images from rustc 1.97.0 against
the pinned 1.99.0) shows no check, boot, connect, fault or log difference beyond the 20 ms tolerance in any shipped
scenario; only payload details move (an `Esc::Telemetry` tick count by one, depth readings in the fourth digit,
IVC2 heartbeats and statuses a few ms apart). Bench fingerprints differ, as they do for any change to the images
([testing.md](testing.md)).
