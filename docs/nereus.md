# nereus: the firmware drives the vehicle

[nereus](https://github.com/osu-uwrt/nereus) is the team's simulator for the riptide stack: Talos in a pool, its
IMU, FOG, DVL, depth and cameras, the course, a 3D viewer. Normally the controller's `thruster_forces` push its
vehicle directly. Here they go the way they go on the robot instead: riptide's controller sends RPM commands over
`can0` to the **emulated ESC boards**, their firmware's DShot output spins the emulator's motor models, and the
thrust those propeller speeds make is what nereus applies. nereus moves the vehicle, the stack sees it through
nereus's sensors, and the camera cage firmware feels nereus's depth. You watch it in nereus's viewer.

> **To verify locally.** This was written where there is no ROS 2, vcan, nereus build or GPU. What is tested here:
> the emulator's side (external vehicles, `/api/thrusters` and `/api/vehicle`, the Rust ESC boards spinning to the
> controller's raw CAN frames: `cargo test --release`), the bridge's logic and the pack generator
> (`python3 -I -m unittest discover -s integrations/nereus -p 'test_*.py'`), the generated packs against nereus's
> own validator, and the scripts in `--dry-run`. Nothing has run with ROS. [Below](#to-verify-locally) is the
> checklist.

- [How it fits together](#how-it-fits-together)
- [Setting up the laptop](#setting-up-the-laptop)
- [Running it](#running-it)
- [What you should see](#what-you-should-see)
- [By hand](#by-hand)
- [The pieces](#the-pieces)
- [Time](#time)
- [Limitations](#limitations)
- [Troubleshooting](#troubleshooting)
- [To verify locally](#to-verify-locally)

## How it fits together

```
  riptide stack (unchanged, use_sim_time)                       nereus (nereus-sim + viewer)
  complete_controller --- raw CANmore thruster frames ---+      6-DOF Talos, pool, sensors, cameras
  EKF, mapping, autonomy                                 |        ^ thrust per thruster      | IMU, FOG, DVL,
        ^   ^ depth/pose (depth_converter)               |        | simulator/firmware_      | cameras, /clock
        |   |                                            |        | thruster_forces          v
        |   +--- state/depth/raw <--+                    |      firmware_bridge.py  <- simulator/ground_truth
        |                           |                    |        |  POST /api/vehicle (pose)  ^ GET thrust back
        +---- micro_ros_agent can -D can0 (C images)     |        |  sets real_time_factor     |
              or titan_bridge (Rust images)              |        v                            |
                    ^                                    v      firmware_simulator --dashboard
                    +------------------ vcan can0 (CAN FD) -->  PoAC, ESC board 0 and 1, camera cage
                                                                ESC DShot -> motor models -> thrust curves
                                                                external vehicle: pose from nereus -> MS5837
```

| Process | What it does | Clock |
|---|---|---|
| `firmware_simulator robot robots/talos_c_nereus.toml --can internal=can0 --dashboard ...` | The four boards in the loop on `can0`; the vehicle is **external** (moved by nereus, see [vehicles.md](vehicles.md#external-vehicles)) | Virtual, at most real time (`--realtime`) |
| `micro_ros_agent can -D can0` (riptide_hardware2's `copro_agent.launch.py`) | The C boards' micro-ROS sessions, as on the Orin | Wall |
| nereus `sim.launch.py` with the generated scenario | nereus-sim, the riptide stack (bringup with `hardware:=none`, `use_sim_time`), the viewer | nereus's `/clock` |
| `integrations/nereus/firmware_bridge.py` | Pose to the emulator, thrust to nereus, nereus's speed to the emulator's | Wall |

The boards are the ones in the loop: the PoAC (the kill switch: its `state/kill` is nereus's kill too), the two
ESC boards, and the camera cage (depth). Fewer boards keep the emulator closer to real time.

## Setting up the laptop

Ubuntu 22.04 with ROS 2 Humble, as nereus wants it. Once:

1. **The emulator**: this repository, `cargo build --release` ([README](../README.md#quick-start)).
2. **The riptide workspace** (`osu-uwrt/riptide_release`, built with colcon as usual), with
   `micro_ros_agent` built against **osu-uwrt/Micro-XRCE-DDS-Agent's `can-support` branch** (the agent's `can`
   transport; `riptide_release/scripts/meta_files/humble_robot_jetson.repos` pins it for the Orin). Check with
   `ros2 run micro_ros_agent micro_ros_agent can --help`.
3. **nereus**: clone `osu-uwrt/nereus` (or use `riptide_release`'s `nereus` submodule) and, with ROS and the
   riptide workspace sourced, run its `./setup.sh` and `./build.sh`.
4. **The C board apps** (titan_firmware, what the robot runs): a checkout of `osu-uwrt/titan_firmware` with its
   submodules, micro-ROS built once in Docker, then the four projects:

   ```bash
   export TITAN_FIRMWARE_DIR=~/osu-uwrt/titan_firmware
   git -C $TITAN_FIRMWARE_DIR submodule update --init --recursive
   (cd $TITAN_FIRMWARE_DIR/lib/micro_ros_pico && ./build.sh)
   scripts/build_c_boards.sh talos/PoacBoard talos/ESCBoard talos/CameraCageBB
   ```

   For the Rust images instead: `(cd external/titan_firmware_rs/firmware && cargo build --workspace --release)` and
   `titan_bridge` on `PATH` (riptide-shell, or `docker/robot_computer`), then add `--rust` below.
5. **vcan**: the kernel's `vcan` module (most desktop kernels have it). The script creates `can0` and `can1` with
   sudo; a real CAN adapter named `can0` must be unplugged first.

## Running it

In a terminal with ROS 2 and the riptide workspace sourced:

```bash
scripts/nereus_firmware_sim.sh --nereus ~/osu-uwrt/release/nereus
```

It checks the pieces above (and says which is missing), then:

1. creates `can0` and `can1` (`scripts/vcan_up.sh can0 can1`: CAN FD MTU, since titan_firmware's internal bus runs
   CAN FD at 1/5 Mbit/s);
2. writes the nereus packs for this run from your nereus checkout (`integrations/nereus/make_packs.py`) into
   `/tmp/nereus_firmware/<time>/packs`;
3. starts the emulator in the background (log in `/tmp/nereus_firmware/<time>/emulator.log`, dashboard on
   `http://127.0.0.1:8080/`);
4. `ros2 launch integrations/nereus/nereus_firmware.launch.py`: nereus with the stack and the viewer, the micro-ROS
   agents, and the bridge.

Ctrl-C (or closing the viewer) stops everything. Options: `--rust` (Rust images and titan_bridge), `--pool rpac`,
`--no-viewer`, `--no-cameras`, `--rmw rmw_zenoh_cpp`, `--threads N`, `--dry-run` (print the commands), and the
ones in the script's `--help`.

## What you should see

1. The emulator's log: each board boots through its bootloader, then `CAN controller online` and, once the agent
   is up, the boards' micro-ROS sessions. Its dashboard shows four boards, the `internal` bus busy, and the vehicle
   card "talos, vehicle, moved externally".
2. `ros2 topic list` has the firmware's topics next to nereus's: `/talos/state/firmware`, `/talos/state/kill`,
   `/talos/state/depth/raw`, `/talos/state/thrusters/rpm`, `/talos/state/thrusters/telemetry`.
3. The bridge logs `emulator on http://127.0.0.1:8080`, `poac: physical kill switch in`, then every 5 s a status
   line: `emulator 42.1s, 0.62x real time, nereus rtf 0.62, clock drift +0.01s, thrust [...] N`.
4. In nereus's viewer, Talos sits where the scenario put it, killed (nereus starts killed, and follows the PoAC
   firmware's `state/kill` from its first message). Unkill it as usual (the viewer's kill control or `command/software_kill`): the PoAC firmware
   publishes `state/kill` false, nereus unkills, the ESC boards arm. Give the controller a goal (the viewer's motion
   control, or autonomy); the ESC boards spin (dashboard: each ESC's RPM; ROS: `/talos/state/thrusters/rpm`),
   `/talos/simulator/firmware_thruster_forces` shows the thrust, and the vehicle moves in the viewer.
5. `/talos/state/depth/raw` follows the vehicle's depth in nereus (the firmware's reading of the pressure at its
   port); `/talos/depth/pose` is riptide's `depth_converter` on it.
6. Kill from the viewer: the thrusters stop within the firmware's kill handling, and the vehicle coasts.

## By hand

What the script runs, for when one piece needs restarting (each in its own terminal, ROS sourced):

```bash
scripts/vcan_up.sh can0 can1
python3 integrations/nereus/make_packs.py ~/osu-uwrt/release/nereus /tmp/nf/packs
firmware_simulator robot robots/talos_c_nereus.toml --can internal=can0 --realtime --time 100000s --dashboard 127.0.0.1:8080
ros2 launch integrations/nereus/nereus_firmware.launch.py nereus_dir:=$HOME/osu-uwrt/release/nereus \
    scenario:=/tmp/nf/packs/scenarios/talos_firmware_robosub
```

or, without that launch file: `ros2 launch nereus sim.launch.py scenario:=/tmp/nf/packs/scenarios/talos_firmware_robosub`,
`ros2 launch riptide_hardware2 copro_agent.launch.py robot:=talos` and
`python3 integrations/nereus/firmware_bridge.py`.

## The pieces

### The emulator's side

`robots/talos_c_nereus.toml` (C) and `robots/talos_nereus.toml` (Rust) have the four boards on the internal bus and
the vehicle `talos` with `external = true`, from [`vehicles/talos_nereus.toml`](../vehicles/talos_nereus.toml):
Talos's real thrusters in the controller's order (thruster k on ESC board k / 4, ESC k % 4 + 1, the way
`complete_controller` addresses them), the boards' invert mask, and a thrust curve fitted to riptide's own
`force_to_rpm_coefficents`, so the thrust reported for a propeller speed is the force the controller chose that
speed for. The depth sensor's mount is `talos.yaml`'s depth pose from `base_link`, the point whose pose nereus
reports. The bridge uses two dashboard endpoints ([tools.md](tools.md#api)): `POST /api/vehicle` to put the
vehicle where nereus has it (answered with the thrust), `GET /api/thrusters` for the thrust alone.

### The bridge pack

nereus is not ours to change, so `make_packs.py` derives a variant of its `uwrt_talos` bridge pack from your
checkout (a copy generated from nereus 58763df is in
[`integrations/nereus/packs`](../integrations/nereus/packs/bridges/uwrt_talos_firmware/bridge.yaml) for reference),
and a scenario that uses it:

| Stream | In `uwrt_talos` | In `uwrt_talos_firmware` |
|---|---|---|
| thrusters (`command:thrusters.set_forces`) | the controller's `thruster_forces` | `simulator/firmware_thruster_forces`, from the bridge (same order) |
| kill (`command:robot.set_killed`) | `command/software_kill` from the stack | the PoAC firmware's `state/kill` (best effort, as titan_bridge publishes it) |
| nereus's kill state | `state/kill` | `simulator/kill_state` (the PoAC publishes `state/kill`) |
| ESC telemetry | fake `state/thrusters/telemetry` | none: the ESC boards publish theirs |
| depth | `depth/pose` | none: `depth_converter` makes it from the camera cage's `state/depth/raw` (`--depth nereus` keeps nereus's) |
| actuator status | `state/actuator/status`, `busy`, `cmd_status` | under `simulator/nereus/` (the PoAC firmware publishes them; `--actuators nereus` keeps nereus's) |

Everything else (IMU, FOG, DVL, cameras, TF, `/clock`, mechanisms, scoring) is unchanged. The scenario also
switches off nereus's thruster lag (`thruster_overrides`: no delay, rise or slew; 60 N limits), since the emulated
ESCs and motors are the lag now.

### The bridge node

`integrations/nereus/firmware_bridge.py` (rclpy; its logic is in `firmware_bridge_logic.py`, which imports without
ROS). Every tick (50 per second, or as fast as the emulator answers) it posts nereus's `simulator/ground_truth`
(converted from ENU/FLU to the emulator's NED/FRD) to the external vehicle and publishes the thrust it gets back.
For the first 4 s of emulator time it holds the vehicle at the surface, where the camera cage zeroes its depth
sensor at boot (as on the dock). At the start it puts the PoAC's physical kill switch in (GPIO 1 high), so the
software kill decides; `simulator/physical_kill` (`std_msgs/Bool`, true pulls it) pulls it again. Status, as JSON,
on `simulator/firmware_bridge/status`.

| Option | Default | What it does |
|---|---|---|
| `--dashboard ADDR` | `127.0.0.1:8080` | The emulator's dashboard |
| `--rate HZ` | 50 | Ticks per second |
| `--max-rtf X` | 1.0 | The fastest nereus may run |
| `--no-clock-sync` | | Leave nereus's `real_time_factor` alone ([Time](#time)) |
| `--zero-hold S` | 4 | Emulator seconds at the surface first |
| `--no-kill-switch` | | Leave the physical kill switch out |
| `--rpm-from-forces` | | Also turn the controller's `thruster_forces` into `command/thruster_rpm` (`DshotCommand`, riptide's force-to-RPM curve) for a controller that does not write `can0` itself |

## Time

The emulator runs the firmware as fast as it can, never faster than the wall clock (`--realtime`). On threads
(the script's default) its boards run ahead of each other between the moments they can see each other, with can0
as well: what titan_bridge, the micro-ROS agent and the controller write to can0 is read at the slice boundary
where taking turns would read it, and a board that already ran past where such a frame can reach it is rolled
back to before it. For these four boards that is about 0.9-1.1x real time before pacing on a shared 4-core
machine with three cores free (`nereus_t3` in the benchmark; slice by slice, as before, it was about 0.6-0.75x: the ESC
boards' DShot PIO is the cost, the other boards and the Orin's tools are cheap), so the emulator about keeps up,
and more on a laptop with more cores; it stays within 2 ms of the wall clock, so host frames are not held back. On fewer free cores, or a slower
machine, it may still fall behind. If nereus ran at 1x anyway, the vehicle would move on thrust the firmware has
not made yet, and the lag would grow without bound.

So **nereus is slowed down to the emulator's speed** when the emulator is slower than real time (and runs at 1x
when it keeps up): the bridge measures the emulator's virtual seconds per wall
second and sets nereus's `real_time_factor` parameter (on `/talos/physics_simulator`) to it, with a correction that
keeps the two clocks' simulated time equal since the bridge started (the "clock drift" in its status). Pausing the
emulator (its dashboard) pauses nereus too. The riptide stack runs on nereus's `/clock` (`use_sim_time`), so the
controller, EKF and autonomy slow down with it, and the whole simulated world runs at one speed.

What stays on the wall clock, and why that is fine or not:

* The micro-ROS agent, titan_bridge and DDS move bytes; their timeouts are generous for a robot running at 0.6x.
* When a frame from the Orin's side arrives depends on the wall clock, so two runs differ. `--arrivals-log FILE`
  on the emulator logs where each one entered the bus; `firmware_simulator robot robots/talos_nereus.toml
  --replay-arrivals FILE --dashboard ...` repeats the firmware's side of the run exactly, without vcan or ROS
  (the external vehicle's poses come from whoever posts them, so post the same ones, or none).
* A controller on wall-clock timers sends commands more often per simulated second; the latest command wins.
* **The C firmware's message stamps.** A micro-ROS board syncs its clock with the agent's (wall) clock once, when
  its ROS session starts, and then counts its own virtual time. If the emulator ran slower than real time before
  that sync, its stamps are ahead of `/clock` by a constant: the EKF holds such a depth reading until its stamp
  comes up, so depth arrives that much late. The bridge measures it (`firmware stamps +1.20s vs /clock` in its
  status, and a warning above 0.5 s). Keep it small by starting nereus and the bridge right after the emulator (the
  script does), or take depth from nereus (`--depth nereus`).

Alternatives considered: running nereus at 1x and reporting the lag (simple, but the firmware falls ever further
behind the vehicle and the loop goes unstable), and stepping both in lockstep (the emulator can be paused and
resumed from the dashboard, but nereus's pacing loop has no external step, and an HTTP round trip per 2 ms physics
step would be slower than either). Matching the speeds keeps every part of the loop consistent with only
nereus's existing parameter.

## Limitations

* **24 N ahead is out of the boards' reach.** The ESC boards' throttle hard limit (725 of 999) stops a motor at
  3068 RPM ahead, which riptide's curve makes 22.4 N, against the 3171 RPM it chooses for its 24 N per-thruster
  limit; astern, 24 N (2932 RPM) is within the limit's 3100 RPM. This is the real system's: the emulated ESCs turn
  at the speed the firmware's feed-forward fit (from the same bench runs as riptide's curve) gives for a throttle
  ([boards.md](boards.md#the-esc-boards-speed-curve)), so every other target, ahead or astern, settles within a few
  RPM (`crates/sim_test/tests/vehicle_nereus.rs`). Before that calibration the ESC model was linear (4000 RPM at
  full throttle): reverse targets settled about 330 RPM short and the limit was 2904 RPM.
* Only the boards in the loop run. The actuator board (claw), LED, SBC, smart battery and IVC are left out; nereus
  keeps simulating the mechanisms, and the PoAC firmware reports actuator state.
* nereus's own thruster dynamics are off and its propellers' surface fade still applies to the firmware's thrust.

## Troubleshooting

* **No firmware topics.** The agent must be the osu-uwrt fork with the `can` transport; `ip -details link show
  can0` must say `vcan` and `mtu 72` (an MTU of 16 drops every CAN FD frame); `candump can0` shows the boards'
  heartbeats either way.
* **Discovery.** Use the same RMW everywhere (`--rmw rmw_zenoh_cpp`, which nereus recommends, with
  `ros2 run rmw_zenoh_cpp rmw_zenohd` running; or a Fast DDS discovery server: riptide_release's
  `scripts/dds_scripts`). The agent runs with `-m rmw`, so it follows `RMW_IMPLEMENTATION` like the rest.
* **`ROS_DOMAIN_ID` does not separate micro-ROS boards.** The client (the firmware) picks the domain, not the
  agent's environment, so two people running this on one network see each other's `/talos` topics whatever their
  `ROS_DOMAIN_ID`. Use one discovery server (or Zenoh router) per laptop, or a network of your own.
* **`could not load the vcan module` / permission denied.** `sudo modprobe vcan`; the script needs sudo for `ip
  link`. In a container: `--network host` and the interfaces created on the host.
* **The vehicle does not move.** Check the kill (`ros2 topic echo /talos/state/kill`), that the controller writes
  can0 (`candump can0` shows frames to `0B0` and `0F0`, the thruster commands to clients 2 and 3), and the ESC RPMs
  on the dashboard. A controller that only publishes `thruster_forces` (riptide_mpc) needs `--rpm-from-forces`.
* **It runs very slowly.** The bridge's status gives the emulator's speed; `--threads` (default: as many as help)
  puts the two ESC boards on their own cores. Close other heavy programs; nereus's cameras (`--no-cameras`) cost
  GPU and CPU.
* **The emulator's `behind the wall clock` warnings** are expected when it is slower than real time: nereus
  waits for it. Running ahead needs the boards on threads (not `--threads 1`, not `--no-run-ahead`); with
  `RUST_LOG=info` nothing more is printed, but the bridge's status shows the emulator's speed.

## To verify locally

| Step | Command | Expect |
|---|---|---|
| The pieces are there | `scripts/nereus_firmware_sim.sh --dry-run` | No missing pieces listed |
| vcan | `scripts/vcan_up.sh can0 can1 && ip -details link show can0` | `vcan`, `mtu 72` |
| Boards and agent | `scripts/robot_in_a_box.sh --robot robots/talos_c_nereus.toml` (add `--dashboard 127.0.0.1:8080`), then `ros2 topic list` | The firmware topics of [What you should see](#what-you-should-see) |
| Packs | `uv run nereus-packs validate /tmp/nereus_firmware/<time>/packs/scenarios/talos_firmware_robosub` (in nereus) | `OK scenario` |
| nereus up | the script | The viewer, nereus's topics, `/clock` |
| Bridge | its log and `ros2 topic echo /talos/simulator/firmware_bridge/status` | Status lines with the emulator's speed and the drift near 0 |
| Kill | unkill from the viewer | `state/kill` false, nereus unkilled |
| Thrust | a goal from the viewer | ESC RPMs on the dashboard, `/talos/simulator/firmware_thruster_forces` non-zero, the vehicle moves |
| Depth | dive | `/talos/state/depth/raw` follows nereus's depth within a few cm |
| Rust | `scripts/nereus_firmware_sim.sh --rust` | The same through titan_bridge |
