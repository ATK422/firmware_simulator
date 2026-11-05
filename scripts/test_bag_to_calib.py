"""Tests for bag_to_calib.py's conversion logic (no ROS needed): `python3 -m unittest discover -s scripts`.

`ReadBag` reads a bag through the real ROS 2 message classes and serialization (rclpy, rosidl_runtime_py and the
riptide_msgs2 interfaces sourced; skipped otherwise), with rosbag2_py's reader stood in for, as no rosbag2 was there
to write a real bag."""

import math
import os
import sys
import tempfile
import types
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bag_to_calib as b  # noqa: E402


class Conversion(unittest.TestCase):
    def test_frames(self):
        # Level, facing north in ENU (yaw 90 deg): yaw 0 in NED
        q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        roll, pitch, yaw = b.flu_to_frd_rpy(*b.quat_to_rpy(*q))
        self.assertAlmostEqual(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(yaw, 0.0)
        # Facing east in ENU (yaw 0): 90 deg in NED
        self.assertAlmostEqual(b.flu_to_frd_rpy(0.0, 0.0, 0.0)[2], 90.0)
        # Nose up in FLU (pitch about +y left: positive pitch puts the nose down) is nose up in FRD
        self.assertAlmostEqual(b.flu_to_frd_rpy(0.0, -0.1, 0.0)[1], math.degrees(0.1))
        self.assertEqual(b.enu_to_ned((1.0, 2.0, -3.0)), (2.0, 1.0, 3.0))
        self.assertEqual(b.flu_to_frd((1.0, 2.0, 3.0)), (1.0, -2.0, -3.0))

    def test_rows_hold_the_last_value(self):
        streams = {
            "rpm": [(10.0, [100.0, -200.0]), (10.12, [150.0, -250.0])],
            "depth": [(10.0, 1.0), (10.06, 1.1)],
            "imu": [(10.0, ((0.0, 1.0, 90.0), (0.0, 0.0, 5.0)))],
            "dvl": [(10.05, (0.2, 0.0, 0.01))],
        }
        header, rows = b.vehicle_rows(streams, ["a", "b"], 20.0)
        self.assertEqual(header[:3], ["time_s", "rpm_a", "rpm_b"])
        self.assertEqual(header[3:], b.CHANNELS)
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0][0], "10.000000")
        self.assertEqual(rows[1][1:4], ["100", "-200", "1"])
        self.assertEqual(rows[2][1:4], ["100", "-200", "1.1"])
        # The DVL arrives at 10.05: empty before, then held
        u = header.index("u_mps")
        self.assertEqual((rows[0][u], rows[1][u]), ("", "0.2"))
        self.assertEqual(rows[0][header.index("r_dps")], "5")

    def test_names_and_topics(self):
        self.assertEqual(b.thruster_names("", 3), ["0", "1", "2"])
        here = os.path.dirname(os.path.abspath(__file__))
        names = b.thruster_names(os.path.join(here, "..", "vehicles", "talos.toml"), 8)
        if sys.version_info >= (3, 11):
            self.assertEqual(names[0], "fwd_stbd")
            self.assertEqual(len(names), 8)
        self.assertEqual(b.topic("/talos", "depth/pose"), "/talos/depth/pose")
        self.assertEqual(b.topic("/talos", "/clock"), "/clock")
        a = b.parse_args(["bag", "--rate", "10"])
        self.assertEqual((a.bag, a.rate, a.rpm_field), ("bag", 10.0, "rpm"))

    def test_partial_arrays_merge_by_their_valid_mask(self):
        # ESC board 0 publishes thrusters 0-3 (mask 0x0f), board 1 4-7 (0xf0), the others zero
        held = b.merge_partial(None, [1.0, 2.0, 3.0, 4.0, 0.0, 0.0, 0.0, 0.0], 0x0F)
        self.assertEqual(held, [1.0, 2.0, 3.0, 4.0, None, None, None, None])
        held = b.merge_partial(held, [0.0, 0.0, 0.0, 0.0, 5.0, 6.0, 7.0, 8.0], 0xF0)
        self.assertEqual(held, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])
        # One thruster without telemetry keeps its last value
        held = b.merge_partial(held, [9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0x01)
        self.assertEqual(held, [9.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])
        # A message without a mask is whole
        self.assertEqual(b.merge_partial(held, [1.0, 2.0], None), [1.0, 2.0])
        # Rows start once every thruster has a value
        streams = {"rpm": [(1.0, [1.0, None]), (1.05, [1.0, 2.0])], "depth": [(1.0, 0.5)]}
        _, rows = b.vehicle_rows(streams, ["a", "b"], 20.0)
        self.assertEqual([r[1:3] for r in rows], [["1", "2"]])

    def test_depth_is_a_height_by_default(self):
        self.assertTrue(b.parse_args(["bag"]).depth_up)
        self.assertFalse(b.parse_args(["bag", "--no-depth-up"]).depth_up)


def ros_messages():
    try:
        from rclpy.serialization import serialize_message  # noqa: F401
        from rosidl_runtime_py.utilities import get_message  # noqa: F401
        import riptide_msgs2.msg  # noqa: F401
        import nav_msgs.msg  # noqa: F401
        import sensor_msgs.msg  # noqa: F401

        return True
    except ImportError:
        return False


@unittest.skipUnless(ros_messages(), "needs ROS 2 (rclpy, rosidl_runtime_py) and riptide_msgs2 sourced")
class ReadBag(unittest.TestCase):
    """The topics a Talos pool test records, as the robot's nodes publish them, through `read_bag` and `main`."""

    def bag(self):
        from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
        from nav_msgs.msg import Odometry
        from rclpy.serialization import serialize_message
        from riptide_msgs2.msg import DshotRPMFeedback
        from sensor_msgs.msg import Imu

        def rpm(board, values, t):
            m = DshotRPMFeedback()
            rpms = [0] * 8
            rpms[board * 4 : board * 4 + 4] = values
            m.rpm = rpms
            m.rpm_valid_mask = 0x0F << (board * 4)
            return ("/talos/state/thrusters/rpm", m, t)

        depth = PoseWithCovarianceStamped()
        depth.pose.pose.position.z = -1.5  # depth_converter: odom's z, up
        imu = Imu()
        imu.orientation.z, imu.orientation.w = math.sin(math.pi / 4), math.cos(math.pi / 4)  # facing north
        imu.angular_velocity.z = 0.1  # turning left (to port)
        dvl = TwistWithCovarianceStamped()
        dvl.twist.twist.linear.x, dvl.twist.twist.linear.y = 0.3, 0.1  # ahead, and to port
        odom = Odometry()
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = 2.0, 5.0, -1.5
        odom.pose.pose.orientation.w = 1.0
        msgs = [
            rpm(0, [100, 200, 300, 400], 10.00),
            rpm(1, [500, 600, 700, 800], 10.01),
            ("/talos/depth/pose", depth, 10.02),
            ("/talos/vectornav/imu", imu, 10.03),
            ("/talos/dvl/twist", dvl, 10.04),
            ("/talos/odometry/filtered", odom, 10.05),
            rpm(0, [110, 210, 310, 410], 10.10),
            ("/talos/depth/pose", depth, 10.12),
        ]
        types_ = {
            "/talos/state/thrusters/rpm": "riptide_msgs2/msg/DshotRPMFeedback",
            "/talos/depth/pose": "geometry_msgs/msg/PoseWithCovarianceStamped",
            "/talos/vectornav/imu": "sensor_msgs/msg/Imu",
            "/talos/dvl/twist": "geometry_msgs/msg/TwistWithCovarianceStamped",
            "/talos/odometry/filtered": "nav_msgs/msg/Odometry",
            "/talos/unrelated": "std_msgs/msg/String",
        }
        return [(n, serialize_message(m), int(t * 1e9)) for n, m, t in msgs], types_

    def fake_rosbag2(self, records, topic_types):
        mod = types.ModuleType("rosbag2_py")

        class SequentialReader:
            def open(self, storage, converter):
                self.records = list(records)

            def get_all_topics_and_types(self):
                return [types.SimpleNamespace(name=n, type=t) for n, t in topic_types.items()]

            def has_next(self):
                return bool(self.records)

            def read_next(self):
                return self.records.pop(0)

        mod.SequentialReader = SequentialReader
        mod.StorageOptions = lambda uri, storage_id: (uri, storage_id)
        mod.ConverterOptions = lambda a, b: (a, b)
        return mod

    def test_a_talos_bag(self):
        records, topic_types = self.bag()
        saved = sys.modules.get("rosbag2_py")
        sys.modules["rosbag2_py"] = self.fake_rosbag2(records, topic_types)
        try:
            streams, pose = b.read_bag(b.parse_args(["bag"]))
            with tempfile.TemporaryDirectory() as d:
                b.main(["bag", "--out", d, "--rate", "20"])
                with open(os.path.join(d, "vehicle.csv")) as f:
                    rows = [line.strip().split(",") for line in f]
                with open(os.path.join(d, "pose.csv")) as f:
                    pose_rows = [line.strip().split(",") for line in f]
        finally:
            if saved is None:
                del sys.modules["rosbag2_py"]
            else:
                sys.modules["rosbag2_py"] = saved
        # The two boards' halves make one array of eight; the third message updates board 0's half only
        self.assertEqual(streams["rpm"][1][1], [100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0])
        self.assertEqual(streams["rpm"][2][1], [110.0, 210.0, 310.0, 410.0, 500.0, 600.0, 700.0, 800.0])
        self.assertEqual(streams["depth"][0], (10.02, 1.5), "1.5 m down")
        (roll, pitch, yaw), (p, q, r) = streams["imu"][0][1]
        self.assertAlmostEqual(yaw, 0.0, places=6)
        self.assertAlmostEqual(r, -math.degrees(0.1), places=6)
        self.assertEqual(streams["dvl"][0][1], (0.3, -0.1, -0.0))
        self.assertEqual(pose[0][:4], ["10.050000", "5", "2", "1.5"])
        header = rows[0]
        self.assertEqual(header[:9], ["time_s"] + [f"rpm_{k}" for k in range(8)])
        # From the first row with all eight thrusters (10.01 s, on the 20 Hz grid from 10.00 s: 10.05)
        self.assertEqual(rows[1][0], "10.050000")
        self.assertEqual(rows[1][1:9], ["100", "200", "300", "400", "500", "600", "700", "800"])
        self.assertEqual(rows[1][header.index("depth_m")], "1.5")
        self.assertEqual(rows[-1][1], "110")
        self.assertEqual(pose_rows[0], ["time_s"] + b.POSE)


if __name__ == "__main__":
    unittest.main()
