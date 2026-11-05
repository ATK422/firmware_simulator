#!/usr/bin/env python3
"""Converts a ROS 2 bag from a pool test into the CSV files firmware_simulator's calibration reads.

Writes, into --out:

* vehicle.csv for `firmware_simulator calibrate vehicle`: time_s, one rpm_<thruster> (measured RPM) and/or
  cmd_<thruster> (commanded RPM) column per thruster, and depth_m, roll_deg, pitch_deg, yaw_deg, u_mps, v_mps,
  w_mps, p_dps, q_dps, r_dps, sampled on a fixed grid (--rate) by holding each topic's last value;
* pose.csv for `firmware_simulator replay-pool-test --pose`: time_s, north_m, east_m, depth_m, roll_deg,
  pitch_deg, yaw_deg from the odometry topic.

Times are Unix seconds (the bag's receive times, the robot computer's clock, as candump stamps its logs). ROS
messages are in REP 103 frames (world east-north-up, body forward-left-up); the CSVs are in the simulator's
(world north-east-down, body forward-starboard-down), converted here. Topic names and the fields that hold the
values are options: check them with `ros2 bag info BAG` and `ros2 interface show TYPE`.

Needs ROS 2 (rosbag2_py, rclpy, rosidl_runtime_py) sourced to read a bag; --help and the conversion logic do not.
"""

import argparse
import csv
import math
import os
import sys

CHANNELS = ["depth_m", "roll_deg", "pitch_deg", "yaw_deg", "u_mps", "v_mps", "w_mps", "p_dps", "q_dps", "r_dps"]
POSE = ["north_m", "east_m", "depth_m", "roll_deg", "pitch_deg", "yaw_deg"]


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("bag", help="the bag directory (ros2 bag record -o ...)")
    p.add_argument("--out", default="calibration_data", help="directory for vehicle.csv and pose.csv")
    p.add_argument("--namespace", default="/talos", help="prefix of the relative topic names below")
    p.add_argument("--rpm-topic", default="state/thrusters/rpm", help="measured thruster RPM (ESC telemetry)")
    p.add_argument("--rpm-field", default="rpm", help="the array field of the RPM message")
    p.add_argument("--cmd-topic", default="", help="commanded thruster RPM, if recorded (written as cmd_ columns)")
    p.add_argument("--cmd-field", default="data", help="the array field of the command message")
    p.add_argument("--depth-topic", default="depth/pose", help="depth (PoseWithCovarianceStamped or riptide Depth)")
    p.add_argument("--depth-field", default="pose.pose.position.z", help="field holding the depth")
    p.add_argument(
        "--depth-up",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="the depth field is a height, z up, and is negated (riptide's depth/pose and state/depth/raw both are: "
        "negative below the surface); --no-depth-up for a field that is already positive down",
    )
    p.add_argument("--imu-topic", default="vectornav/imu", help="sensor_msgs/Imu: attitude and body rates")
    p.add_argument("--dvl-topic", default="dvl/twist", help="DVL body velocity (TwistWithCovarianceStamped)")
    p.add_argument("--dvl-field", default="twist.twist.linear", help="field holding the velocity vector")
    p.add_argument("--odom-topic", default="odometry/filtered", help="nav_msgs/Odometry for pose.csv")
    p.add_argument("--rate", type=float, default=20.0, help="vehicle.csv rows per second")
    p.add_argument("--vehicle", default="", help="vehicle file (TOML): column names from its thrusters, in order")
    return p.parse_args(argv)


def topic(ns, name):
    if not name or name.startswith("/"):
        return name
    return ns.rstrip("/") + "/" + name


def field(msg, path):
    """A dotted attribute path of a message (`pose.pose.position.z`)."""
    for part in path.split("."):
        msg = getattr(msg, part)
    return msg


def quat_to_rpy(x, y, z, w):
    """Roll, pitch, yaw (rad) of a quaternion, ZYX."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def flu_to_frd_rpy(roll, pitch, yaw_enu):
    """Attitude of a forward-left-up body in east-north-up, as forward-starboard-down in north-east-down (deg)."""
    yaw = math.degrees(math.pi / 2 - yaw_enu)
    yaw = (yaw + 180.0) % 360.0 - 180.0
    return math.degrees(roll), -math.degrees(pitch), yaw


def flu_to_frd(v):
    """A body-frame vector from forward-left-up to forward-starboard-down."""
    return (v[0], -v[1], -v[2])


def enu_to_ned(p):
    return (p[1], p[0], -p[2])


def thruster_names(path, count):
    if path:
        try:
            import tomllib
        except ImportError:  # Python < 3.11
            tomllib = None
        if tomllib is not None:
            with open(path, "rb") as f:
                names = [t["name"] for t in tomllib.load(f).get("thruster", [])]
            if names:
                return names
    return [str(k) for k in range(count)]


def merge_partial(last, values, mask):
    """An array message that carries only some elements (each ESC board publishes its four thrusters of the eight on
    state/thrusters/rpm and /cmd, the others zero, with `rpm_valid_mask` saying which): the held array with the valid
    ones replaced. `mask` None: every element is valid."""
    if mask is None:
        return list(values)
    out = list(last) if last is not None else [None] * len(values)
    out += [None] * (len(values) - len(out))
    for j, v in enumerate(values):
        if mask >> j & 1:
            out[j] = v
    return out


def hold(series, grid):
    """Each grid time's value: the last sample at or before it (None before the first)."""
    out, k = [], -1
    for t in grid:
        while k + 1 < len(series) and series[k + 1][0] <= t:
            k += 1
        out.append(series[k][1] if k >= 0 else None)
    return out


def vehicle_rows(streams, names, rate):
    """vehicle.csv's header and rows from time-sorted streams: `rpm` and `cmd` [(t, [rpm, ...])], `depth`
    [(t, d)], `imu` [(t, (rpy_deg, pqr_dps))], `dvl` [(t, (u, v, w))]."""
    times = [t for s in streams.values() for t, _ in s]
    if not times:
        return [], []
    t0, t1 = min(times), max(times)
    n = int((t1 - t0) * rate) + 1
    grid = [t0 + k / rate for k in range(n)]
    header = ["time_s"]
    cols = []
    for kind in ("rpm", "cmd"):
        s = streams.get(kind) or []
        if s:
            held = hold(s, grid)
            for j, name in enumerate(names):
                header.append(f"{kind}_{name}")
                cols.append([None if v is None or j >= len(v) else v[j] for v in held])
    depth = hold(streams.get("depth") or [], grid)
    imu = hold(streams.get("imu") or [], grid)
    dvl = hold(streams.get("dvl") or [], grid)
    header += CHANNELS
    cols.append(depth)
    for j in range(3):
        cols.append([None if v is None else v[0][j] for v in imu])
    for j in range(3):
        cols.append([None if v is None else v[j] for v in dvl])
    for j in range(3):
        cols.append([None if v is None else v[1][j] for v in imu])
    rows = [[f"{t:.6f}"] + ["" if c[i] is None else f"{c[i]:.6g}" for c in cols] for i, t in enumerate(grid)]
    # Rows before every input has a value say nothing
    first = next((i for i, r in enumerate(rows) if all(x != "" for x in r[1 : 1 + len(names)])), len(rows))
    return header, rows[first:]


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def read_bag(args):
    """Reads the topics from the bag: (streams, pose rows)."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        sys.exit(f"reading a bag needs ROS 2 sourced (rosbag2_py, rclpy): {e}")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = {
        "rpm": topic(args.namespace, args.rpm_topic),
        "cmd": topic(args.namespace, args.cmd_topic),
        "depth": topic(args.namespace, args.depth_topic),
        "imu": topic(args.namespace, args.imu_topic),
        "dvl": topic(args.namespace, args.dvl_topic),
        "odom": topic(args.namespace, args.odom_topic),
    }
    for kind, name in want.items():
        if name and name not in types:
            print(f"warning: no {name} in the bag ({kind}); have: {', '.join(sorted(types))}", file=sys.stderr)
    by_topic = {v: k for k, v in want.items() if v in types}
    classes = {t: get_message(types[t]) for t in by_topic}
    streams = {k: [] for k in want}
    last = {"rpm": None, "cmd": None}
    pose = []
    while reader.has_next():
        name, data, stamp = reader.read_next()
        kind = by_topic.get(name)
        if kind is None:
            continue
        msg = deserialize_message(data, classes[name])
        t = stamp / 1e9
        if kind in ("rpm", "cmd"):
            values = [float(x) for x in field(msg, args.rpm_field if kind == "rpm" else args.cmd_field)]
            last[kind] = merge_partial(last[kind], values, getattr(msg, "rpm_valid_mask", None))
            streams[kind].append((t, last[kind]))
        elif kind == "depth":
            d = float(field(msg, args.depth_field))
            streams["depth"].append((t, -d if args.depth_up else d))
        elif kind == "imu":
            q = msg.orientation
            rpy = flu_to_frd_rpy(*quat_to_rpy(q.x, q.y, q.z, q.w))
            w = flu_to_frd((msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
            streams["imu"].append((t, (rpy, tuple(math.degrees(x) for x in w))))
        elif kind == "dvl":
            v = field(msg, args.dvl_field)
            streams["dvl"].append((t, flu_to_frd((v.x, v.y, v.z))))
        elif kind == "odom":
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            n, e, d = enu_to_ned((p.x, p.y, p.z))
            pose.append([f"{t:.6f}"] + [f"{x:.6g}" for x in (n, e, d, *flu_to_frd_rpy(*quat_to_rpy(q.x, q.y, q.z, q.w)))])
    return streams, pose


def main(argv=None):
    args = parse_args(argv)
    streams, pose = read_bag(args)
    os.makedirs(args.out, exist_ok=True)
    count = max([len(v) for _, v in (streams.get("rpm") or streams.get("cmd") or [])] or [0])
    names = thruster_names(args.vehicle, count)
    header, rows = vehicle_rows({k: v for k, v in streams.items() if k != "odom"}, names, args.rate)
    if rows:
        write_csv(os.path.join(args.out, "vehicle.csv"), header, rows)
        print(f"vehicle.csv: {len(rows)} rows, {len(header)} columns")
    else:
        print("no thruster or sensor messages: no vehicle.csv", file=sys.stderr)
    if pose:
        write_csv(os.path.join(args.out, "pose.csv"), ["time_s"] + POSE, pose)
        print(f"pose.csv: {len(pose)} rows")


if __name__ == "__main__":
    main()
