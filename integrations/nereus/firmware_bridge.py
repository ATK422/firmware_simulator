#!/usr/bin/env python3
"""Bridges firmware_simulator's emulated Talos and nereus, so the firmware moves nereus's vehicle.

    python3 integrations/nereus/firmware_bridge.py [--dashboard 127.0.0.1:8080] [--namespace /talos] ...

The riptide stack runs unchanged against nereus. Its controller sends RPM commands over can0 to the emulated ESC
boards (raw CANmore frames from complete_controller, or riptide_msgs2/DshotCommand on command/thruster_rpm through
the micro-ROS agent or titan_bridge); their DShot outputs spin the emulator's motor models. Every tick this node

* posts nereus's ground truth pose of base_link (simulator/ground_truth) to the emulator's external vehicle
  (POST /api/vehicle), so the camera cage's depth sensor feels nereus's depth;
* gets back each ESC's thrust (the emulator's thruster curves on the propeller speeds) and publishes it in the
  controller's thruster order on simulator/firmware_thruster_forces, which the uwrt_talos_firmware bridge pack
  feeds to nereus's thrusters instead of the controller's thruster_forces;
* keeps nereus's clock running with the emulator's virtual time (nereus's real_time_factor parameter), since the
  emulator can run slower than real time.

It also puts the PoAC's physical kill switch in at the start (--kill-switch), as on the dock; the stack's
software kill and nereus's viewer then kill and unkill through the PoAC firmware, whose state/kill is nereus's
kill state too. simulator/physical_kill (std_msgs/Bool, true = pulled) pulls it out again.

docs/nereus.md has the whole setup. Status goes to simulator/firmware_bridge/status (std_msgs/String, JSON).
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import firmware_bridge_logic as fb  # noqa: E402


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--dashboard", default="127.0.0.1:8080", help="the emulator's --dashboard address")
    p.add_argument("--vehicle", default="talos", help="the robot file's vehicle (external = true)")
    p.add_argument("--namespace", default="/talos", help="the robot's ROS namespace")
    p.add_argument("--rate", type=float, default=50.0, help="ticks per wall second")
    p.add_argument("--esc-boards", default="esc0,esc1", help="ESC boards in thruster order (4 ESCs each)")
    p.add_argument(
        "--forces-topic",
        default="simulator/firmware_thruster_forces",
        help="where the firmware's thrust goes (the uwrt_talos_firmware pack's thruster stream)",
    )
    p.add_argument("--truth-topic", default="simulator/ground_truth", help="nereus's nav_msgs/Odometry of base_link")
    p.add_argument(
        "--nereus-node",
        default="/talos/physics_simulator",
        help="the nereus node with the real_time_factor parameter",
    )
    p.add_argument(
        "--max-rtf",
        type=float,
        default=1.0,
        help="the fastest nereus may run (1.0: real time, as the emulator's --realtime)",
    )
    p.add_argument("--no-clock-sync", action="store_true", help="leave nereus's real_time_factor alone")
    p.add_argument(
        "--zero-hold",
        type=float,
        default=4.0,
        help="emulator seconds the vehicle is held at the surface first, while the depth sensor zeroes",
    )
    p.add_argument("--kill-switch-board", default="poac", help="board with the physical kill switch ('' for none)")
    p.add_argument("--kill-switch-pin", type=int, default=1, help="its GPIO (high: switch in, not killing)")
    p.add_argument(
        "--no-kill-switch",
        action="store_true",
        help="do not put the physical kill switch in at the start",
    )
    p.add_argument(
        "--rpm-from-forces",
        action="store_true",
        help="also turn the controller's thruster_forces into command/thruster_rpm (DshotCommand), for a "
        "controller that does not write can0 itself",
    )
    p.add_argument("--depth-topic", default="state/depth/raw", help="the camera cage's riptide_msgs2/Depth")
    p.add_argument("--status-every", type=float, default=5.0, help="wall seconds between status log lines")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    # ROS only from here on, so the logic above imports without it
    import rclpy
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool, Float32MultiArray, String

    class FirmwareBridge(Node):
        def __init__(self):
            super().__init__("firmware_bridge", namespace=args.namespace)
            self.emu = fb.EmulatorClient(args.dashboard)
            self.slots = fb.esc_slots(tuple(b for b in args.esc_boards.split(",") if b))
            self.lock = threading.Lock()
            self.truth = None
            self.clock_s = None
            self.stamp_offset = None
            self.governor = fb.ClockGovernor(max_rtf=args.max_rtf)
            self.rtf_sent = None
            self.rtf_pending = None
            self.problems_seen = set()
            self.last_status = 0.0
            self.status = {}
            self.forces_pub = self.create_publisher(Float32MultiArray, args.forces_topic, 10)
            self.status_pub = self.create_publisher(String, "simulator/firmware_bridge/status", 10)
            self.create_subscription(Odometry, args.truth_topic, self.on_truth, 10)
            self.create_subscription(Clock, "/clock", self.on_clock, 10)
            self.create_subscription(Bool, "simulator/physical_kill", self.on_physical_kill, 10)
            self.rtf_client = self.create_client(SetParameters, args.nereus_node.rstrip("/") + "/set_parameters")
            self.rpm_pub = None
            if args.rpm_from_forces:
                from riptide_msgs2.msg import DshotCommand

                self.DshotCommand = DshotCommand
                self.rpm_pub = self.create_publisher(DshotCommand, "command/thruster_rpm", qos_profile_sensor_data)
                self.create_subscription(Float32MultiArray, "thruster_forces", self.on_controller_forces, 10)
            try:
                from riptide_msgs2.msg import Depth

                best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
                self.create_subscription(Depth, args.depth_topic, self.on_depth, best_effort)
            except ImportError:
                self.get_logger().warn("riptide_msgs2 not found: not checking the firmware's depth stamps")
            self.Parameter, self.ParameterType, self.ParameterValue = Parameter, ParameterType, ParameterValue
            self.SetParameters = SetParameters
            self.String, self.Float32MultiArray = String, Float32MultiArray
            self.stop = threading.Event()
            self.wait_for_emulator()
            if args.kill_switch_board and not args.no_kill_switch:
                self.set_kill_switch(True)
            self.thread = threading.Thread(target=self.loop, name="emulator", daemon=True)
            self.thread.start()

        def wait_for_emulator(self):
            while rclpy.ok():
                try:
                    api = self.emu.thrusters()
                    names = [v["name"] for v in api.get("vehicles", [])]
                    ext = [v["name"] for v in api.get("vehicles", []) if v.get("external")]
                    if args.vehicle not in ext:
                        self.get_logger().error(
                            f"the emulator's vehicles are {names}, external ones {ext}: run a robot file whose "
                            f"vehicle {args.vehicle!r} is external (robots/talos_c_nereus.toml), or add "
                            f"--external-vehicle {args.vehicle}"
                        )
                    self.get_logger().info(f"emulator on {self.emu.base} at {api.get('time_s', 0):.2f}s")
                    return
                except fb.EmulatorError as e:
                    self.get_logger().info(f"waiting for the emulator's dashboard ({e})", throttle_duration_sec=5)
                    time.sleep(0.5)

        def set_kill_switch(self, inserted):
            fault = fb.kill_switch_fault(inserted, args.kill_switch_pin)
            try:
                self.emu.fault(args.kill_switch_board, fault)
                what = "in (not killing)" if inserted else "out (killing)"
                self.get_logger().info(f"{args.kill_switch_board}: physical kill switch {what}")
            except fb.EmulatorError as e:
                self.get_logger().error(f"could not set the kill switch: {e}")

        def on_physical_kill(self, msg):
            self.set_kill_switch(not msg.data)

        def on_truth(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            lin, ang = msg.twist.twist.linear, msg.twist.twist.angular
            with self.lock:
                self.truth = ([p.x, p.y, p.z], [q.w, q.x, q.y, q.z], [lin.x, lin.y, lin.z], [ang.x, ang.y, ang.z])

        def on_clock(self, msg):
            with self.lock:
                self.clock_s = msg.clock.sec + msg.clock.nanosec * 1e-9

        def on_depth(self, msg):
            with self.lock:
                clock = self.clock_s
            if clock is not None:
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.stamp_offset = fb.stamp_offset_s(stamp, clock)

        def on_controller_forces(self, msg):
            cmd = self.DshotCommand()
            values = fb.dshot_command_values(list(msg.data)[:8])
            cmd.values = values + [0] * (8 - len(values))
            self.rpm_pub.publish(cmd)

        def problem(self, text):
            if text not in self.problems_seen:
                self.problems_seen.add(text)
                self.get_logger().warn(text)

        def tick(self):
            with self.lock:
                truth, clock = self.truth, self.clock_s
            try:
                if truth is None:
                    api = self.emu.thrusters()
                else:
                    emu_t = self.status.get("emulator_s", 0.0)
                    hold = emu_t < args.zero_hold
                    body = fb.vehicle_post_body(*truth, vehicle=args.vehicle, hold_at_surface=hold)
                    api = self.emu.post_vehicle(body)
            except fb.EmulatorError as e:
                # nereus's thrusters coast to zero after its command timeout
                self.problem(f"emulator: {e}")
                return
            forces, problems = fb.forces_from_thrusters(api, self.slots)
            for p in problems:
                self.problem(p)
            self.forces_pub.publish(self.Float32MultiArray(data=[float(f) for f in forces]))
            emu_t = float(api.get("time_s", 0.0))
            rtf = self.governor.update(time.monotonic(), emu_t, clock)
            if not args.no_clock_sync and fb.should_set(rtf, self.rtf_sent):
                self.set_rtf(rtf)
            self.status = {
                "emulator_s": emu_t,
                "paused": api.get("paused"),
                "nereus_s": clock,
                "speed": self.governor.speed,
                "rtf": self.rtf_sent,
                "drift_s": self.governor.drift_s if clock is not None else None,
                "stamp_offset_s": self.stamp_offset,
                "holding_at_surface": truth is not None and emu_t < args.zero_hold,
                "forces": forces,
            }

        def set_rtf(self, rtf):
            if not self.rtf_client.service_is_ready():
                self.problem(f"nereus's {args.nereus_node}/set_parameters is not up yet: not slowing nereus down")
                return
            if self.rtf_pending is not None and not self.rtf_pending.done():
                return
            value = self.ParameterValue(type=self.ParameterType.PARAMETER_DOUBLE, double_value=float(rtf))
            req = self.SetParameters.Request(parameters=[self.Parameter(name="real_time_factor", value=value)])
            self.rtf_pending = self.rtf_client.call_async(req)
            self.rtf_sent = rtf

        def loop(self):
            period = 1.0 / args.rate
            next_t = time.monotonic()
            while rclpy.ok() and not self.stop.is_set():
                try:
                    self.tick()
                except Exception:
                    # Publishing fails once ROS has shut down (Ctrl-C): that is the end, not an error
                    if not rclpy.ok() or self.stop.is_set():
                        return
                    raise
                now = time.monotonic()
                if now - self.last_status >= args.status_every:
                    self.last_status = now
                    self.get_logger().info(fb.status_line(self.status))
                    if (self.status.get("stamp_offset_s") or 0) > 0.5:
                        self.problem(
                            "the firmware's micro-ROS stamps run ahead of /clock (the emulator was slower than "
                            "real time before the boards synced their clocks): the EKF takes depth that much late; "
                            "docs/nereus.md#time"
                        )
                try:
                    self.status_pub.publish(self.String(data=json.dumps(self.status)))
                except Exception:
                    if not rclpy.ok() or self.stop.is_set():
                        return
                    raise
                next_t += period
                time.sleep(max(0.0, next_t - time.monotonic()))
                if time.monotonic() - next_t > 1.0:
                    next_t = time.monotonic()

    # SIGINT and SIGTERM (what `ros2 launch` sends on shutdown) stop the node here, with its emulator thread
    # joined before the context goes: rclpy's own handlers would shut the context down under that thread and spin
    # (an ExternalShutdownException, or an RCLError from the wait set, and the thread's publish failing)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = None
    try:
        node = FirmwareBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop.set()
            if hasattr(node, "thread"):
                try:
                    node.thread.join(timeout=2.0)
                except KeyboardInterrupt:
                    # A second Ctrl-C (ros2 launch passes one on as well): stop waiting
                    pass
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
