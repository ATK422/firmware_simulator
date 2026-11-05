# Robot computer in a container

The robot computer side of a "robot in a box" (PLAN.md's Orin option 1): a ROS 2 Humble container with
`titan_bridge`, `upload_tool` and `canmore_cli` built from the titan_firmware_rs checkout, next to the emulator
running a robot file, both on the host's vcan interfaces.

> **Unverified.** These files were written in an environment without a Docker daemon, network access, the vcan
> module or ROS 2. `docker compose config` accepts the compose file and the shell scripts pass `bash -n`; nothing
> has been built or run. Expect to fix things on first use, most likely in the robot computer image (rclrs's build
> requirements, the riptide message package).

## Files

| File | What it is |
|---|---|
| `compose.yaml` | The emulator and the robot computer, both with `network_mode: host`, sharing a volume for the generated bridges script |
| `Dockerfile` | `ros:humble-ros-base` + Rust; builds `riptide_msgs2` (from `osu-uwrt/riptide_core`, see below) and anything in `ros_ws/`, then `titan_bridge` (and the tools) against that environment, as `riptide-shell` does |
| `entrypoint.sh` | Sources ROS and the workspace; `wait-for FILE` waits for the bridges script and runs it |
| `ros_ws/` | ROS packages to build into the image (placeholder for the riptide stack) |
| `../emulator/Dockerfile` | The emulator plus the titan_firmware_rs images (built in the image) |

## Running

```bash
scripts/vcan_up.sh                                          # vcan0 + vcan1 with the CAN FD MTU
docker compose -f docker/robot_computer/compose.yaml up --build
```

The emulator service runs `firmware_simulator bridges $ROBOT --format script` into the shared volume, then
`firmware_simulator robot $ROBOT --can internal=vcan0 --can external=vcan1 --realtime`; the robot computer
service waits for the script and starts one `titan_bridge` per board with its `--board` and `--client-id`. On the
host (or `docker compose ... run --rm tools`):

```bash
candump -l vcan0                    # the internal bus
ros2 topic echo /talos/state/firmware
canmore_cli                         # pick a board from the list
```

Settings: `ROBOT=robots/ivc2_pool.toml`, `CANDUMP_LOG=/shared/robot.log` (a `candump -l` log of every bus, written
by the emulator; it grows about 0.3 GB an hour), `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`.

`network_mode: host` is what makes the vcan interfaces visible: they are network interfaces of the host's network
namespace, not device nodes, so there is nothing to pass with `devices:`, and SocketCAN raw sockets need no added
capabilities. It also puts the bridges' ROS nodes on the host's DDS network.

## The riptide stack

`titan_bridge` publishes with rclrs dynamic messages, so at run time it needs the type support of the packages it
uses: `std_msgs` and `std_srvs` (from the ROS image) and `riptide_msgs2` (`ActuatorStatus`, `BatteryStatus`,
`Depth`, `DshotCommand`, `DshotPartialTelemetry`, `DshotRPMFeedback`, `DynamixelStatus`, `ElectricalCommand`,
`ElectricalReadings`, `FirmwareStatus`, `IvcRx`, `KillSwitchReport`, `LedCommand`). The image clones
`RIPTIDE_CORE_REPO` (default `https://github.com/osu-uwrt/riptide_core`, where titan_firmware's micro-ROS library
generation also gets `riptide_msgs`) at `RIPTIDE_CORE_REF` and builds its `riptide_msgs` directory. To use a fork,
a private copy, or the whole riptide software stack, build with `--build-arg RIPTIDE_CORE_REPO=` and put the
packages in `ros_ws/`; everything there is built with colcon before titan_bridge. Start the rest of the stack from
the service's `command`.

## Orin architecture

The `ros:humble` images are multi-arch. To run the robot computer image as arm64 (the Orin's userspace
architecture) under qemu-user, register binfmt once (`docker run --privileged --rm tonistiigi/binfmt --install
arm64`) and build and run with `DOCKER_DEFAULT_PLATFORM=linux/arm64`. Building rclrs under emulation is slow. An
actual L4T image (`nvcr.io/nvidia/l4t-*`) as the base is the next step towards the real Orin userspace; it is not
done here.

## Without vcan on the same kernel

When the bridges run somewhere that cannot share the emulator's vcan (Docker Desktop's VM, another machine, a VM
with its own kernel), tunnel the bus over UDP instead. On the emulator's machine:

```bash
firmware_simulator robot robots/talos.toml --realtime \
    --cannelloni internal=0.0.0.0:20000,ROBOT_COMPUTER:20000 --cannelloni external=0.0.0.0:20001,ROBOT_COMPUTER:20001
```

and on the robot computer, which has its own `vcan0`/`vcan1` (`scripts/vcan_up.sh`), connect them with
`firmware_simulator can-tunnel vcan0 0.0.0.0:20000,EMULATOR:20000` (and the same for vcan1 on 20001), or with
`cannelloni -I vcan0 -R EMULATOR -r 20000 -l 20000` (the tunnel uses cannelloni's UDP framing; compatibility with the
cannelloni program itself is untested).

With no vcan on either side, a titan_bridge that has the `cannelloni:` interface (titan_firmware_rs's
`m12-titan-bridge-udp` branch or later) talks to the emulator's bus directly: run the emulator with a hub,
`--cannelloni internal=0.0.0.0:20000 --cannelloni external=0.0.0.0:20001`, and each bridge with
`--interface cannelloni:EMULATOR:20000` (`firmware_simulator bridges robots/talos.toml --can
internal=cannelloni:EMULATOR:20000 --can external=cannelloni:EMULATOR:20001` writes them; from Docker Desktop,
`EMULATOR` is `host.docker.internal`). The image builds titan_bridge from the submodule, which does not have it yet.
