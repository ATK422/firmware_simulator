"""Tests for bag_to_calib.py's conversion logic (no ROS needed): `python3 -m unittest discover -s scripts`."""

import math
import os
import sys
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


if __name__ == "__main__":
    unittest.main()
