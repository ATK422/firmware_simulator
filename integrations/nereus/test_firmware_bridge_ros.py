"""firmware_bridge.py as a ROS 2 node, against a stand-in for the emulator's dashboard and for nereus.

Needs ROS 2 (rclpy, nav_msgs, rosgraph_msgs, rcl_interfaces) sourced, and is skipped otherwise:

    python3 -m unittest discover -s integrations/nereus -p 'test_firmware_bridge_ros.py'

The dashboard stand-in answers `/api/thrusters` with what the emulator answered in a run of
robots/talos_nereus.toml with the controller's raw CAN thruster commands, and records the bridge's POSTs. The test
node plays nereus: `/clock` at half the wall clock's speed, `simulator/ground_truth` 1.2 m down, facing north, and
the `real_time_factor` parameter of `/talos/physics_simulator`. The bridge runs in its own process (as `ros2 launch`
starts it), on a DDS domain of its own, on localhost only.
"""

import http.server
import importlib.util
import json
import math
import os
import random
import signal
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent

# The emulator's ESCs in that run: thrust per ESC for the RPMs the controller chose 12, -5, 24, 0, -5, -24, 16 and
# -8 N for (24 N ahead stops at the boards' hard limit, 22.5 N)
ESCS = [
    ("esc0", "esc1", "VUS", 12.35),
    ("esc0", "esc2", "VUP", -5.52),
    ("esc0", "esc3", "HUS", 22.47),
    ("esc0", "esc4", "HUP", 0.0),
    ("esc1", "esc1", "HLS", -5.52),
    ("esc1", "esc2", "HLP", -23.93),
    ("esc1", "esc3", "VLS", 16.14),
    ("esc1", "esc4", "VLP", -8.23),
]


def have_ros():
    modules = ("rclpy", "nav_msgs", "rosgraph_msgs", "rcl_interfaces")
    return all(importlib.util.find_spec(m) is not None for m in modules)


class Dashboard:
    """The emulator's dashboard API as the bridge uses it; virtual time runs at half the wall clock's speed."""

    def __init__(self):
        self.posts = []
        self.t0 = time.monotonic()
        dash = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def reply(self, body):
                data = json.dumps(body).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                self.reply(dash.thrusters())

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                dash.posts.append((time.monotonic(), self.path, body))
                self.reply(dash.thrusters() if self.path == "/api/vehicle" else {"ok": True, "message": "ok"})

        self.server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def thrusters(self):
        escs = [{"board": b, "esc": e, "thruster": n, "vehicle": "talos", "thrust_n": f} for b, e, n, f in ESCS]
        vehicle = {"name": "talos", "external": True, "thrusters": [{"name": n, "thrust_n": f} for _, _, n, f in ESCS]}
        t = 0.5 * (time.monotonic() - self.t0)
        return {"ok": True, "time_s": t, "paused": False, "escs": escs, "vehicles": [vehicle]}

    def close(self):
        self.server.shutdown()
        self.server.server_close()


@unittest.skipUnless(have_ros(), "needs ROS 2 (rclpy, nav_msgs, rosgraph_msgs, rcl_interfaces) sourced")
class BridgeNode(unittest.TestCase):
    def test_bridge_between_the_dashboard_and_nereus_stopped_by_sigterm(self):
        self.bridge_between_the_dashboard_and_nereus(signal.SIGTERM)

    def test_bridge_between_the_dashboard_and_nereus_stopped_by_sigint(self):
        self.bridge_between_the_dashboard_and_nereus(signal.SIGINT)

    def bridge_between_the_dashboard_and_nereus(self, stop_signal):
        import rclpy
        from nav_msgs.msg import Odometry
        from rcl_interfaces.msg import SetParametersResult
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Bool, Float32MultiArray, String

        domain = str(random.randint(100, 200))
        env = dict(os.environ, ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY="1", PYTHONUNBUFFERED="1")
        os.environ.update(ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY="1")
        dash = Dashboard()
        self.addCleanup(dash.close)
        port = dash.server.server_address[1]
        bridge = subprocess.Popen(
            [sys.executable, str(HERE / "firmware_bridge.py"), "--dashboard", f"127.0.0.1:{port}", "--zero-hold", "1"],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.addCleanup(lambda: bridge.poll() is None and bridge.kill())

        rclpy.init()
        self.addCleanup(rclpy.try_shutdown)
        got = {"forces": [], "status": [], "rtf": []}

        class Physics(Node):
            def __init__(self):
                super().__init__("physics_simulator", namespace="/talos")
                self.declare_parameter("real_time_factor", 1.0)
                self.add_on_set_parameters_callback(self.on_set)

            def on_set(self, params):
                got["rtf"] += [p.value for p in params if p.name == "real_time_factor"]
                return SetParametersResult(successful=True)

        class Nereus(Node):
            def __init__(self):
                super().__init__("nereus", namespace="/talos")
                self.t0 = time.monotonic()
                self.clock = self.create_publisher(Clock, "/clock", 10)
                self.truth = self.create_publisher(Odometry, "simulator/ground_truth", 10)
                self.kill = self.create_publisher(Bool, "simulator/physical_kill", 10)
                forces = "simulator/firmware_thruster_forces"
                self.create_subscription(Float32MultiArray, forces, lambda m: got["forces"].append(list(m.data)), 10)
                self.create_subscription(
                    String, "simulator/firmware_bridge/status", lambda m: got["status"].append(json.loads(m.data)), 10
                )
                self.create_timer(0.02, self.tick)

            def tick(self):
                t = 0.5 * (time.monotonic() - self.t0)
                c = Clock()
                c.clock.sec, c.clock.nanosec = int(t), int(t % 1 * 1e9)
                self.clock.publish(c)
                o = Odometry()
                o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = 3.0, 4.0, -1.2
                # Facing north: yaw 90 degrees in ENU
                o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(math.pi / 4), math.cos(math.pi / 4)
                o.twist.twist.linear.x = 0.5
                self.truth.publish(o)

        physics, nereus = Physics(), Nereus()
        executor = MultiThreadedExecutor()
        executor.add_node(physics)
        executor.add_node(nereus)
        threading.Thread(target=executor.spin, daemon=True).start()

        def wait_for(what, timeout=30.0):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                if what():
                    return True
                time.sleep(0.1)
            return False

        def posted(path, pred=lambda b: True):
            return [b for _, p, b in dash.posts if p == path and pred(b)]

        expected = [f for *_, f in ESCS]
        try:
            # The forces, in the controller's order, from the emulator's thrust per ESC
            self.assertTrue(wait_for(lambda: got["forces"]), "no simulator/firmware_thruster_forces")
            for a, b in zip(got["forces"][-1], expected):
                self.assertAlmostEqual(a, b, places=4)
            # The physical kill switch put in at the start
            self.assertTrue(posted("/api/fault", lambda b: b == {"board": "poac", "fault": "gpio:1:high"}))
            # nereus's pose (ENU, FLU) as the emulator's (NED, FRD), after the hold at the surface
            self.assertTrue(wait_for(lambda: posted("/api/vehicle", lambda b: b["position_m"][2] > 1.0)))
            body = posted("/api/vehicle", lambda b: b["position_m"][2] > 1.0)[-1]
            for a, b in zip(body["position_m"], [4.0, 3.0, 1.2]):
                self.assertAlmostEqual(a, b, places=6)
            # Facing north is yaw 0 in NED: the identity quaternion; 0.5 m/s ahead stays ahead
            for a, b in zip(body["quat_wxyz"], [1.0, 0.0, 0.0, 0.0]):
                self.assertAlmostEqual(abs(a), b, places=6)
            self.assertAlmostEqual(body["velocity_mps"][0], 0.5, places=6)
            held = posted("/api/vehicle", lambda b: b.get("velocity_mps") == [0.0, 0.0, 0.0])
            self.assertTrue(held and held[0]["position_m"][2] == 0.0, "held at the surface first")
            # nereus slowed to the emulator's speed (half the wall clock's)
            self.assertTrue(wait_for(lambda: any(r < 0.8 for r in got["rtf"])), got["rtf"])
            self.assertTrue(wait_for(lambda: got["status"] and got["status"][-1].get("speed")))
            status = got["status"][-1]
            self.assertAlmostEqual(status["speed"], 0.5, delta=0.15)
            self.assertFalse(status["holding_at_surface"])
            # Pulling the physical kill switch out
            nereus.kill.publish(Bool(data=True))
            self.assertTrue(wait_for(lambda: posted("/api/fault", lambda b: b["fault"] == "gpio:1:low"), 10))
            # `ros2 launch` stops it with SIGINT (then SIGTERM); either ends it cleanly
            bridge.send_signal(stop_signal)
            out, _ = bridge.communicate(timeout=15)
            self.assertEqual(bridge.returncode, 0, out)
            self.assertNotIn("Traceback", out)
        finally:
            executor.shutdown()
            physics.destroy_node()
            nereus.destroy_node()


if __name__ == "__main__":
    unittest.main()
