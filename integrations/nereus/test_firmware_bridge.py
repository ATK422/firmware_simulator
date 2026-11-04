"""Tests of the emulator <-> nereus bridge's logic and pack generation, without ROS or nereus:

    python3 -I -m unittest discover -s integrations/nereus -p 'test_*.py'

(also run by `cargo test --release --test host_cli`).
"""

import http.server
import json
import math
import os
import sys
import threading
import tomllib
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

import firmware_bridge_logic as fb  # noqa: E402
import make_packs  # noqa: E402

try:
    import yaml
except ImportError:  # PyYAML comes with ROS 2; the pack tests that read YAML skip without it
    yaml = None


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b, strict=True))


class ThrusterOrderAndThrust(unittest.TestCase):
    def test_slots_follow_the_controllers_order(self):
        slots = fb.esc_slots()
        self.assertEqual(len(slots), 8)
        self.assertEqual(slots[0], ("esc0", "esc1"))
        self.assertEqual(slots[3], ("esc0", "esc4"))
        self.assertEqual(slots[4], ("esc1", "esc1"))
        self.assertEqual(slots[7], ("esc1", "esc4"))
        self.assertEqual(fb.NEREUS_ORDER, ["VUS", "VUP", "HUS", "HUP", "HLS", "HLP", "VLS", "VLP"])

    def test_riptide_force_to_rpm(self):
        self.assertAlmostEqual(fb.riptide_rpm(24.0), 3170.95, delta=0.05)
        self.assertAlmostEqual(fb.riptide_rpm(-24.0), -2932.03, delta=0.05)
        self.assertAlmostEqual(fb.riptide_rpm(12.0), 2274.6, delta=0.1)
        self.assertEqual(fb.riptide_rpm(0.0), 0.0)
        self.assertEqual(fb.riptide_rpm(float("nan")), 0.0)
        # Near zero the positive curve's offset would turn the propeller backwards: no command instead
        self.assertEqual(fb.riptide_rpm(0.01), 0.0)
        self.assertEqual(fb.dshot_command_values([24, -24, 0, 12]), [3171, -2932, 0, 2275])
        self.assertEqual(fb.dshot_command_values([1e9])[0], 32767)

    def test_the_vehicle_files_curve_gives_back_riptides_force(self):
        """vehicles/talos_nereus.toml's curve and inversions: the thrust the emulator reports for the speed
        riptide picked for a force is that force (within the curve fit), with the inverted thrusters turning the
        other way, as the ESC boards' invert mask makes them."""
        vehicle = tomllib.loads((REPO / "vehicles/talos_nereus.toml").read_text())
        m = vehicle["thruster_model"]
        names = [t["name"] for t in vehicle["thruster"]]
        self.assertEqual(names, fb.NEREUS_ORDER)
        inverted = [t.get("inverted", False) for t in vehicle["thruster"]]
        self.assertEqual(inverted, [k % 4 in (0, 3) for k in range(8)])
        for k, inv in enumerate(inverted):
            curve = fb.ThrustCurve(m["forward_n"], m["reverse_n"], m["at_rpm"], inv)
            for force, within in ((24.0, 0.04), (16.0, 0.04), (12.0, 0.04), (5.0, 0.11), (-5.0, 0.11), (-12.0, 0.04)):
                motor = fb.riptide_rpm(force) * (-1 if inv else 1)
                self.assertLess(abs(curve.thrust(motor) - force), within * abs(force), (k, force))
            self.assertEqual(curve.thrust(0.0), 0.0)

    def test_forces_come_from_the_emulators_thrust_per_esc(self):
        api = {
            "time_s": 6.0,
            "escs": [
                {"board": "esc0", "esc": f"esc{i + 1}", "rpm": 100.0 * i, "thrust_n": float(i)} for i in range(4)
            ]
            + [{"board": "esc1", "esc": "esc1", "rpm": 2000.0, "thrust_n": None}],
        }
        forces, problems = fb.forces_from_thrusters(api, fb.esc_slots())
        self.assertEqual(forces[:5], [0.0, 1.0, 2.0, 3.0, 0.0])
        self.assertEqual(len(problems), 3)
        self.assertIn("no ESC esc2 on board esc1", problems[0])
        # An ESC on no vehicle thruster: a curve of the bridge's own
        curves = [fb.ThrustCurve()] * 8
        forces, _ = fb.forces_from_thrusters(api, fb.esc_slots(), curves)
        self.assertAlmostEqual(forces[4], 24.0 * (2000 / 3170.95) ** 2)
        bad = {"escs": [{"board": "esc0", "esc": "esc1", "thrust_n": float("inf")}]}
        self.assertEqual(fb.forces_from_thrusters(bad, fb.esc_slots()[:1])[0], [0.0])


class Frames(unittest.TestCase):
    def test_quaternions_round_trip(self):
        for q in [(1, 0, 0, 0), (0.7071068, 0, 0, 0.7071068), (0.5, 0.5, -0.5, 0.5), (0, 1, 0, 0), (0, 0, 0, 1)]:
            back = fb.matrix_to_quat(fb.quat_to_matrix(q))
            n = math.sqrt(sum(c * c for c in q))
            q = [c / n for c in q]
            self.assertTrue(close(back, q, 1e-6) or close(back, [-c for c in q], 1e-6), (q, back))
        with self.assertRaises(ValueError):
            fb.quat_to_matrix((0, 0, 0, 0))

    def test_enu_flu_poses_become_ned_frd(self):
        # Level, facing east (ENU yaw 0), 1.5 m down: north 2 (ENU y), east 1 (ENU x), depth 1.5, NED yaw 90
        pos, q = fb.pose_to_emulator([1.0, 2.0, -1.5], [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(pos, [2.0, 1.0, 1.5])
        self.assertTrue(close(fb.rpy_deg(q), [0.0, 0.0, 90.0], 1e-9))
        # Facing north (ENU yaw +90, counter-clockwise from east): NED yaw 0
        s = math.sqrt(0.5)
        _, q = fb.pose_to_emulator([0, 0, 0], [s, 0, 0, s])
        self.assertTrue(close(fb.rpy_deg(q), [0.0, 0.0, 0.0], 1e-6))
        # Facing east, nose up 10 degrees (a negative turn about FLU's left axis): NED pitch +10
        h = math.radians(-10) / 2
        _, q = fb.pose_to_emulator([0, 0, 0], [math.cos(h), 0, math.sin(h), 0])
        self.assertTrue(close(fb.rpy_deg(q), [0.0, 10.0, 90.0], 1e-6), fb.rpy_deg(q))
        # Starboard side down 20 degrees: a positive turn about FLU's forward axis (left side up) and FRD's
        h = math.radians(20) / 2
        _, q = fb.pose_to_emulator([0, 0, 0], [math.cos(h), math.sin(h), 0, 0])
        self.assertTrue(close(fb.rpy_deg(q), [20.0, 0.0, 90.0], 1e-6), fb.rpy_deg(q))

    def test_post_bodies(self):
        body = fb.vehicle_post_body([1.0, 2.0, -1.5], [1, 0, 0, 0], [0.5, 0.1, -0.2], [0.0, 0.0, 0.1])
        self.assertEqual(body["vehicle"], "talos")
        self.assertEqual(body["position_m"], [2.0, 1.0, 1.5])
        self.assertEqual(body["velocity_mps"], [0.5, -0.1, 0.2])
        self.assertTrue(close(body["rate_dps"], [0.0, 0.0, -math.degrees(0.1)]))
        self.assertEqual(len(body["quat_wxyz"]), 4)
        json.dumps(body)
        held = fb.vehicle_post_body([1.0, 2.0, -1.5], [1, 0, 0, 0], [0.5, 0, 0], None, vehicle="v", hold_at_surface=True)
        self.assertEqual((held["vehicle"], held["position_m"]), ("v", [2.0, 1.0, 0.0]))
        self.assertEqual(held["velocity_mps"], [0.0, 0.0, 0.0])
        self.assertNotIn("rate_dps", fb.vehicle_post_body([0, 0, 0], [1, 0, 0, 0]))

    def test_pressure(self):
        self.assertEqual(fb.pressure_pa(0.0), 101_325.0)
        self.assertEqual(fb.pressure_pa(-1.0), 101_325.0)
        self.assertAlmostEqual(fb.pressure_pa(2.0), 101_325.0 + 997.0 * 9.80665 * 2.0)


class Time(unittest.TestCase):
    def run_world(self, speed_of, steps=600, dt=0.05, nereus_start_rtf=1.0):
        """The emulator runs at speed_of(t) of real time; nereus runs at the rtf the governor last gave."""
        g = fb.ClockGovernor()
        wall = emu = 0.0
        nereus = 1000.0
        rtf = nereus_start_rtf
        out = []
        for _ in range(steps):
            wall += dt
            emu += speed_of(wall) * dt
            nereus += rtf * dt
            r = g.update(wall, emu, nereus)
            if r is not None:
                rtf = r
            out.append((wall, emu, nereus, rtf, g))
        return out

    def test_nereus_follows_a_slow_emulator(self):
        out = self.run_world(lambda t: 0.55)
        wall, emu, nereus, rtf, g = out[-1]
        self.assertAlmostEqual(g.speed, 0.55, places=6)
        self.assertAlmostEqual(rtf, 0.55, delta=0.01)
        # Both clocks covered the same simulated time since the governor took over, within a tenth of a second
        self.assertLess(abs(g.drift_s), 0.1)

    def test_a_paused_emulator_pauses_nereus_and_real_time_is_the_cap(self):
        out = self.run_world(lambda t: 0.0 if 10 < t < 20 else 0.8)
        at_15 = next(o for o in out if o[0] >= 15.0)
        self.assertEqual(at_15[3], 0.0)
        wall, emu, nereus, rtf, g = out[-1]
        self.assertAlmostEqual(rtf, 0.8, delta=0.02)
        self.assertLess(abs(g.drift_s), 0.1, "caught up again after the pause")
        out = self.run_world(lambda t: 1.5)
        self.assertEqual(out[-1][3], 1.0, "never faster than real time")

    def test_nothing_until_both_clocks_are_known(self):
        g = fb.ClockGovernor()
        self.assertIsNone(g.update(0.0, 0.0, 5.0))
        self.assertIsNone(g.update(0.1, 0.05, 5.1), "a quarter window first")
        self.assertIsNone(g.update(1.0, 0.5, None), "no /clock yet")
        self.assertAlmostEqual(g.speed, 0.5)
        self.assertIsNotNone(g.update(1.1, 0.55, 5.0))
        g.reset()
        self.assertIsNone(g.speed)

    def test_when_to_send(self):
        self.assertFalse(fb.should_set(None, 0.5))
        self.assertTrue(fb.should_set(0.5, None))
        self.assertFalse(fb.should_set(0.51, 0.5))
        self.assertTrue(fb.should_set(0.55, 0.5))
        self.assertTrue(fb.should_set(0.0, 0.01), "pausing always goes out")
        self.assertAlmostEqual(fb.stamp_offset_s(101.5, 100.0), 1.5)

    def test_status_line(self):
        line = fb.status_line(
            {"emulator_s": 12.0, "speed": 0.61, "rtf": 0.6, "drift_s": 0.02, "stamp_offset_s": 1.2, "forces": [1, -2]}
        )
        self.assertIn("0.61x real time", line)
        self.assertIn("firmware stamps +1.20s", line)
        self.assertIn("[+1.0 -2.0] N", line)
        self.assertEqual(fb.status_line({}), "emulator 0.0s")


class Dashboard(unittest.TestCase):
    """EmulatorClient against a stand-in for the emulator's dashboard."""

    def setUp(self):
        self.posted = []
        test = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def reply(self, code, body):
                data = json.dumps(body).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                if self.path == "/api/thrusters":
                    self.reply(200, {"ok": True, "time_s": 1.5, "escs": [], "vehicles": []})
                else:
                    self.reply(404, {"ok": False, "error": "not found"})

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                test.posted.append((self.path, body))
                if body.get("vehicle") == "nope":
                    self.reply(422, {"ok": False, "error": 'no vehicle "nope"; there are: talos'})
                else:
                    self.reply(200, {"ok": True, "time_s": 2.0, "message": "ok"})

        self.server = http.server.HTTPServer(("127.0.0.1", 0), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.client = fb.EmulatorClient(f"127.0.0.1:{self.server.server_address[1]}")

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def test_requests(self):
        self.assertEqual(self.client.thrusters()["time_s"], 1.5)
        self.assertEqual(self.client.post_vehicle({"vehicle": "talos"})["time_s"], 2.0)
        self.client.fault("poac", fb.kill_switch_fault(True))
        self.assertEqual(self.posted[-1], ("/api/fault", {"board": "poac", "fault": "gpio:1:high"}))
        self.assertEqual(fb.kill_switch_fault(False, 3), "gpio:3:low")
        with self.assertRaises(fb.EmulatorError) as e:
            self.client.post_vehicle({"vehicle": "nope"})
        self.assertIn('no vehicle "nope"', str(e.exception))
        with self.assertRaises(fb.EmulatorError):
            self.client.state()

    def test_no_emulator(self):
        port = self.server.server_address[1]
        self.tearDown()
        with self.assertRaises(fb.EmulatorError):
            fb.EmulatorClient(f"http://127.0.0.1:{port}", timeout=0.5).thrusters()
        self.setUp()


def upstream_bridge():
    """The parts of nereus's uwrt_talos bridge.yaml the variant touches (and one it keeps)."""

    def stream(sid, direction, topic, native, message_type="std_msgs/msg/Bool"):
        return {
            "id": sid,
            "direction": direction,
            "topic": topic,
            "message_type": message_type,
            "native": native,
            "frame_id": "",
            "rate_hz": 0,
            "qos": {"history": "keep_last", "depth": 10, "reliability": "reliable", "durability": "volatile"},
            "fields": {},
        }

    kill = stream("software_kill", "subscribe", "command/software_kill", "command:robot.set_killed")
    kill["accept_if"] = [{"field": "kill_switch_id", "equals": 1}]
    return {
        "kind": "bridge",
        "id": "uwrt_talos",
        "description": "Talos.",
        "namespace": "/talos",
        "clock": {"topic": "/clock", "real_time_factor": 1.0},
        "thrusters": {"order": list(fb.NEREUS_ORDER)},
        "kill": {"command_stream": "software_kill", "state_stream": "firmware_kill"},
        "streams": [
            stream("thruster_forces", "subscribe", "thruster_forces", "command:thrusters.set_forces"),
            kill,
            stream("firmware_kill", "publish", "state/kill", "event:robot.kill_changed"),
            stream("t03", "publish", "state/thrusters/telemetry", "timer"),
            stream("t47", "publish", "state/thrusters/telemetry", "timer"),
            stream("depth", "publish", "depth/pose", "sensor:depth"),
            stream("imu", "publish", "vectornav/imu", "sensor:imu"),
            stream("actuator_status", "publish", "state/actuator/status", "state:mechanisms"),
            stream("torpedo", "subscribe", "command/actuator/torpedo", "command:mechanisms.torpedo_launcher.fire"),
        ],
    }


class Packs(unittest.TestCase):
    def test_the_firmware_bridge(self):
        up = upstream_bridge()
        pack = make_packs.firmware_bridge_pack(up, real_time_factor=0.5)
        self.assertEqual(up["streams"][0]["topic"], "thruster_forces", "the upstream pack is left alone")
        self.assertEqual(pack["id"], "uwrt_talos_firmware")
        self.assertEqual(pack["clock"]["real_time_factor"], 0.5)
        streams = {s["id"]: s for s in pack["streams"]}
        self.assertEqual(streams["thruster_forces"]["topic"], "simulator/firmware_thruster_forces")
        kill = streams["software_kill"]
        self.assertEqual((kill["topic"], kill["message_type"]), ("state/kill", "std_msgs/msg/Bool"))
        self.assertEqual(kill["fields"], {"killed": {"from": "data"}})
        self.assertEqual(kill["qos"]["reliability"], "best_effort")
        self.assertNotIn("accept_if", kill)
        self.assertEqual(streams["firmware_kill"]["topic"], "simulator/kill_state")
        for gone in ("t03", "t47", "depth"):
            self.assertNotIn(gone, streams)
        self.assertEqual(streams["imu"]["topic"], "vectornav/imu")
        self.assertEqual(streams["actuator_status"]["topic"], "simulator/nereus/state/actuator/status")
        self.assertEqual(streams["torpedo"]["topic"], "command/actuator/torpedo", "nereus still fires its torpedo")
        # nereus can keep the depth and the actuators instead
        pack = make_packs.firmware_bridge_pack(up, depth="nereus", actuators="nereus")
        streams = {s["id"]: s for s in pack["streams"]}
        self.assertEqual(streams["depth"]["topic"], "depth/pose")
        self.assertEqual(streams["actuator_status"]["topic"], "state/actuator/status")
        with self.assertRaises(make_packs.PackError):
            make_packs.firmware_bridge_pack(up, depth="maybe")

    def test_a_changed_upstream_is_refused(self):
        up = upstream_bridge()
        up["kill"]["command_stream"] = "renamed"
        with self.assertRaises(make_packs.PackError) as e:
            make_packs.firmware_bridge_pack(up)
        self.assertIn("no stream 'renamed'", str(e.exception))
        up = upstream_bridge()
        up["streams"] = up["streams"][1:]
        with self.assertRaises(make_packs.PackError):
            make_packs.firmware_bridge_pack(up)
        pack = make_packs.firmware_bridge_pack(upstream_bridge())
        pack["streams"].append(upstream_bridge()["streams"][3])
        with self.assertRaises(make_packs.PackError) as e:
            make_packs.check_firmware_pack(pack)
        self.assertIn("state/thrusters/telemetry", str(e.exception))

    def test_the_scenario(self):
        up = {
            "kind": "scenario",
            "id": "talos_uwrt",
            "robot": "../../robots/talos",
            "pool": "../../pools/robosub_2026",
            "bridge": "../../bridges/uwrt_talos",
            "initial": {"position_m": [0, 0, -1]},
        }
        nereus = Path("/opt/nereus/content/packs/scenarios/talos_uwrt")
        out = Path("/tmp/run/scenarios/talos_firmware_robosub")
        sc = make_packs.firmware_scenario(up, nereus, Path("/tmp/run/bridges/uwrt_talos_firmware"), "robosub", out)
        self.assertEqual(sc["bridge"], "../../bridges/uwrt_talos_firmware")
        self.assertEqual(sc["robot"], "../../../../opt/nereus/content/packs/robots/talos")
        self.assertFalse(any(sc[r].startswith("/") for r in ("robot", "pool", "bridge")), "nereus wants relative paths")
        self.assertEqual(sc["initial"], up["initial"])
        p = sc["thruster_overrides"][-1]["parameters"]
        self.assertEqual((p["delay_s"], p["rise_time_s"], p["slew_rate_n_s"]), (0.0, 0.0, 0.0))
        self.assertGreater(p["forward_limit_n"], 30.0)

    @unittest.skipIf(yaml is None, "no PyYAML")
    def test_the_shipped_reference_pack_is_a_firmware_pack(self):
        pack = yaml.safe_load((HERE / "packs/bridges/uwrt_talos_firmware/bridge.yaml").read_text())
        make_packs.check_firmware_pack(pack)
        self.assertEqual(pack["thrusters"]["order"], fb.NEREUS_ORDER)

    @unittest.skipIf(yaml is None, "no PyYAML")
    def test_writing_from_a_nereus_checkout(self):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            nereus = Path(d) / "nereus"
            packs = nereus / "content/packs"
            (packs / "bridges/uwrt_talos").mkdir(parents=True)
            (packs / "scenarios/talos_uwrt").mkdir(parents=True)
            (packs / "bridges/uwrt_talos/bridge.yaml").write_text(yaml.safe_dump(upstream_bridge()))
            (packs / "scenarios/talos_uwrt/scenario.yaml").write_text(
                yaml.safe_dump({"kind": "scenario", "id": "s", "robot": "../../robots/talos", "bridge": "../../bridges/uwrt_talos"})
            )
            out = Path(d) / "out"
            scenario = make_packs.write(nereus, out)
            self.assertEqual(scenario, out / "scenarios/talos_firmware_robosub")
            sc = yaml.safe_load((scenario / "scenario.yaml").read_text())
            self.assertEqual(os.path.normpath(scenario / sc["robot"]), str(packs / "robots/talos"))
            bridge = (out / "bridges/uwrt_talos_firmware/bridge.yaml").read_text()
            self.assertTrue(bridge.startswith("# Generated by firmware_simulator"))
            make_packs.check_firmware_pack(yaml.safe_load(bridge))
            self.assertEqual(make_packs.main([str(nereus), str(out), "--pool", "rpac"]), 1, "no rpac scenario there")


class Node(unittest.TestCase):
    def test_the_node_imports_without_ros(self):
        import firmware_bridge

        a = firmware_bridge.parse_args([])
        self.assertEqual((a.dashboard, a.namespace, a.vehicle, a.rate), ("127.0.0.1:8080", "/talos", "talos", 50.0))
        self.assertEqual(a.forces_topic, make_packs.FORCES_TOPIC)
        self.assertEqual(firmware_bridge.parse_args(["--rpm-from-forces", "--max-rtf", "0.8"]).max_rtf, 0.8)


if __name__ == "__main__":
    unittest.main()
