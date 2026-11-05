"""nereus + the riptide stack + the robot computer's side of the emulated boards + the emulator <-> nereus bridge.

    ros2 launch integrations/nereus/nereus_firmware.launch.py nereus_dir:=~/osu-uwrt/release/nereus \\
        scenario:=/tmp/nereus_firmware/packs/scenarios/talos_firmware_robosub [firmware:=c|rust] \\
        [dashboard:=127.0.0.1:8080] [viewer:=true] [cameras:=true] [rmw:=rmw_zenoh_cpp] [rpm_from_forces:=false]
        [bridges_launch:=<titan_bridge launch file, for firmware:=rust>] [agent:=can|udp] [agent_cmd:=...]

scripts/nereus_firmware_sim.sh starts the emulator (firmware_simulator robot robots/talos_c_nereus.toml --can
internal=can0 --dashboard ...), writes the scenario (make_packs.py) and runs this. It starts

* nereus's integrations/uwrt/launch/sim.launch.py with that scenario: the simulator, the UWRT stack (riptide's
  bringup with hardware:=none and use_sim_time) and the pool viewer;
* for firmware:=c, riptide_hardware2's copro_agent.launch.py, as hardware.launch.py runs it on the robot
  (`micro_ros_agent can -D can0`, under the robot's namespace): the emulated boards' micro-ROS sessions;
  with agent:=udp (no vcan: the emulator's --micro-ros-agent relays the sessions over UDP) a micro-ROS agent's
  udp4 transport instead, agent_cmd or else `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888` or
  `MicroXRCEAgent udp4 --port 8888`, whichever is there;
  for firmware:=rust, the titan_bridge launch file `firmware_simulator bridges ... --format launch` wrote;
* firmware_bridge.py, which moves the emulator's vehicle to where nereus has it and gives nereus the thrust.
"""

import os
import shlex
import shutil
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import PushRosNamespace

HERE = Path(__file__).resolve().parent


def _truthy(v):
    return v.lower() in ("true", "1", "yes")


def _processes(context):
    nereus = Path(os.path.expanduser(LC("nereus_dir").perform(context)))
    sim = nereus / "integrations/uwrt/launch/sim.launch.py"
    if not sim.exists():
        raise RuntimeError(f"{sim} not found: pass nereus_dir:= your nereus checkout")
    scenario = LC("scenario").perform(context)
    if not scenario:
        raise RuntimeError("pass scenario:= (integrations/nereus/make_packs.py writes one)")
    firmware = LC("firmware").perform(context)
    nereus_args = {
        "scenario": scenario,
        "viewer": LC("viewer").perform(context),
        "cameras": LC("cameras").perform(context),
        "stack": LC("stack").perform(context),
    }
    for k in ("rmw", "output", "active_control_model"):
        if v := LC(k).perform(context):
            nereus_args[k] = v
    # The same RMW for the agents and the bridge as for nereus and the stack (nereus sets it for its own processes)
    rmw = LC("rmw").perform(context)
    actions = [SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw)] if rmw else []
    actions.append(
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim)), launch_arguments=nereus_args.items())
    )
    agent = LC("agent").perform(context)
    if agent not in ("can", "udp"):
        raise RuntimeError(f"agent:= can or udp (got {agent!r})")
    if firmware == "c" and agent == "udp":
        actions.append(ExecuteProcess(cmd=udp_agent(LC("agent_cmd").perform(context)), output="screen", name="agent"))
    elif firmware == "c":
        from ament_index_python.packages import get_package_share_directory as share

        agent = Path(share("riptide_hardware2")) / "launch/copro_agent.launch.py"
        actions.append(
            GroupAction(
                [
                    PushRosNamespace(LC("robot")),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(str(agent)), launch_arguments={"robot": LC("robot")}.items()
                    ),
                ],
                scoped=True,
            )
        )
    elif firmware == "rust":
        bridges = LC("bridges_launch").perform(context)
        if not bridges:
            raise RuntimeError("firmware:=rust needs bridges_launch:= (firmware_simulator bridges --format launch)")
        actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(bridges)))
    elif firmware != "none":
        raise RuntimeError(f"firmware:= c, rust or none (got {firmware!r})")
    cmd = [
        sys.executable,
        str(HERE / "firmware_bridge.py"),
        "--dashboard",
        LC("dashboard").perform(context),
        "--namespace",
        "/" + LC("robot").perform(context).strip("/"),
    ]
    if _truthy(LC("rpm_from_forces").perform(context)):
        cmd.append("--rpm-from-forces")
    if not _truthy(LC("clock_sync").perform(context)):
        cmd.append("--no-clock-sync")
    actions.append(ExecuteProcess(cmd=cmd, output="screen", name="firmware_bridge"))
    return actions


def udp_agent(agent_cmd="", port=8888):
    """The micro-ROS agent for agent:=udp: agent_cmd, or riptide's micro_ros_agent package (any build of it has the
    udp4 transport), or a standalone MicroXRCEAgent on PATH."""
    if agent_cmd.strip():
        return shlex.split(agent_cmd)
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_prefix

        try:
            get_package_prefix("micro_ros_agent")
            return ["ros2", "run", "micro_ros_agent", "micro_ros_agent", "udp4", "--port", str(port)]
        except PackageNotFoundError:
            pass
    except ImportError:
        pass
    if shutil.which("MicroXRCEAgent"):
        return ["MicroXRCEAgent", "udp4", "--port", str(port)]
    raise RuntimeError(
        "agent:=udp needs a micro-ROS agent: the micro_ros_agent package or MicroXRCEAgent on PATH "
        "(docs/nereus.md#the-micro-ros-agent), or agent_cmd:="
    )


def generate_launch_description():
    args = {
        "nereus_dir": ("~/osu-uwrt/release/nereus", "your nereus checkout (built: ./build.sh)"),
        "scenario": ("", "scenario folder from make_packs.py"),
        "firmware": ("c", "c: titan_firmware boards (micro-ROS agent), rust: titan_firmware_rs (titan_bridge), none"),
        "bridges_launch": ("", "titan_bridge launch file for firmware:=rust"),
        "agent": ("can", "firmware:=c: can (micro_ros_agent can -D can0) or udp (a udp4 agent, no vcan)"),
        "agent_cmd": ("", "agent:=udp: the agent's command line (default: micro_ros_agent or MicroXRCEAgent, port 8888)"),
        "robot": ("talos", "robot name and ROS namespace"),
        "dashboard": ("127.0.0.1:8080", "the emulator's --dashboard address"),
        "viewer": ("true", "nereus's pool viewer"),
        "cameras": ("true", "nereus's camera rendering"),
        "stack": ("true", "the UWRT stack (riptide bringup, hardware:=none)"),
        "rmw": ("", "RMW for every process (empty: the shell's)"),
        "output": ("", "nereus run records directory"),
        "active_control_model": ("", "riptide controller (empty: bringup's default)"),
        "rpm_from_forces": ("false", "the bridge also turns thruster_forces into command/thruster_rpm"),
        "clock_sync": ("true", "slow nereus down to the emulator's speed"),
    }
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=d, description=h) for k, (d, h) in args.items()]
        + [OpaqueFunction(function=_processes)]
    )
