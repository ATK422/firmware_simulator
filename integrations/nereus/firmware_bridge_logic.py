"""The pure part of the emulator <-> nereus bridge (firmware_bridge.py): frames, thrust, the clock governor and
the dashboard client. Nothing here imports ROS, so it runs and is tested without it
(`python3 -I -m unittest discover -s integrations/nereus -p 'test_*.py'`).

Frames: nereus (and ROS) use world ENU with z up and 0 at the water surface, body FLU (x forward, y left, z up);
the emulator uses world NED (north, east, depth) and body FRD (forward, starboard, down).
"""

from __future__ import annotations

import json
import math
import urllib.error
import urllib.request
from dataclasses import dataclass, field

# nereus's uwrt_talos bridge `thrusters.order`: index k of the controller's force array is this robot-pack
# thruster, and the controller sends thruster k's RPM to ESC board k // 4, ESC k % 4 + 1
# (riptide_controllers' send_thruster_cmd_canbus.c).
NEREUS_ORDER = ["VUS", "VUP", "HUS", "HUP", "HLS", "HLP", "VLS", "VLP"]


def esc_slots(boards=("esc0", "esc1"), per_board=4):
    """(board, ESC name) of controller thruster k, in order: ESC board k // 4's ESC k % 4 + 1."""
    return [(b, f"esc{i + 1}") for b in boards for i in range(per_board)]


# riptide_descriptions talos.yaml `thruster_solver.force_to_rpm_coefficents`: four for the positive curve, four for
# the negative one, each 1, x, tanh(x), x^(1/4).
RIPTIDE_FORCE_TO_RPM = [
    -475.886186,
    32.327429,
    -324.744664,
    1443.832105,
    257.460623,
    42.948545,
    -24.402056,
    -986.338071,
]


def riptide_rpm(force_n: float, coefficients=RIPTIDE_FORCE_TO_RPM) -> float:
    """The RPM riptide's thruster solver sends for a force (our reading of its curve: the linear and tanh terms on
    the signed force, the fourth root on its size). Zero force, and a curve offset that would turn the propeller
    the wrong way for a small force, give 0."""
    if not math.isfinite(force_n) or force_n == 0.0:
        return 0.0
    c = coefficients[0:4] if force_n > 0 else coefficients[4:8]
    rpm = c[0] + c[1] * force_n + c[2] * math.tanh(force_n) + c[3] * abs(force_n) ** 0.25
    return rpm if rpm * force_n > 0 else 0.0


def dshot_command_values(forces, coefficients=RIPTIDE_FORCE_TO_RPM, limit=32767):
    """riptide_msgs2/DshotCommand `values` (int16[8], RPM targets) for controller forces."""
    out = []
    for f in forces:
        r = round(riptide_rpm(float(f), coefficients))
        out.append(max(-limit, min(limit, r)))
    return out


@dataclass
class ThrustCurve:
    """The emulator's thruster model (sim_physics): thrust = k rpm |rpm|, separate forward and reverse points."""

    forward_n: float = 24.0
    reverse_n: float = 28.07
    at_rpm: float = 3170.95
    inverted: bool = False

    def thrust(self, rpm: float) -> float:
        rpm = -rpm if self.inverted else rpm
        k = (self.forward_n if rpm >= 0 else self.reverse_n) / (self.at_rpm * self.at_rpm)
        return math.copysign(k * rpm * rpm, rpm) if rpm else 0.0


def forces_from_thrusters(api: dict, slots, curves=None) -> tuple[list[float], list[str]]:
    """Per-thruster force in the controller's order from the emulator's `/api/thrusters` answer, and the problems
    found. Each slot (board, ESC) takes the `thrust_n` the emulator computed for the vehicle thruster that ESC
    spins; an ESC wired to no thruster uses `curves[k]` (a ThrustCurve) on its RPM, or gives 0. A slot with no ESC
    gives 0 and a problem."""
    by_slot = {(e.get("board"), e.get("esc")): e for e in api.get("escs", [])}
    forces, problems = [], []
    for k, slot in enumerate(slots):
        e = by_slot.get(tuple(slot))
        if e is None:
            forces.append(0.0)
            problems.append(f"thruster {k}: the emulator has no ESC {slot[1]} on board {slot[0]}")
            continue
        f = e.get("thrust_n")
        if f is None and curves is not None:
            f = curves[k].thrust(float(e.get("rpm") or 0.0))
        f = float(f or 0.0)
        forces.append(f if math.isfinite(f) else 0.0)
    return forces, problems


# ---- frames ----

# World ENU -> NED and body FLU -> FRD swap axes; both matrices are their own inverse.
_ENU_TO_NED = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
_FLU_TO_FRD = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))


def _matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def quat_to_matrix(q):
    """Rotation matrix of a unit quaternion (w, x, y, z)."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if not n > 1e-9:
        raise ValueError("not a rotation")
    w, x, y, z = w / n, x / n, y / n, z / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def matrix_to_quat(m):
    """Unit quaternion (w, x, y, z) of a rotation matrix (Shepperd's method), w >= 0."""
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)
    return tuple(-c for c in q) if q[0] < 0 else q


def rpy_deg(q):
    """Roll, pitch, yaw in degrees of a body-to-world quaternion (the z-y-x sequence, as sim_physics)."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def enu_to_ned(v):
    """A world vector (or position) from ENU to NED: north = y, east = x, down = -z."""
    return [v[1], v[0], -v[2]]


def flu_to_frd(v):
    """A body vector from FLU to FRD."""
    return [v[0], -v[1], -v[2]]


def pose_to_emulator(position_enu, quat_enu_flu):
    """nereus's pose of base_link (ENU, FLU body) as the emulator's (NED position, FRD-body-to-NED quaternion)."""
    r = _matmul(_matmul(_ENU_TO_NED, quat_to_matrix(quat_enu_flu)), _FLU_TO_FRD)
    return enu_to_ned(position_enu), matrix_to_quat(r)


def vehicle_post_body(
    position_enu,
    quat_enu_flu,
    linear_flu=None,
    angular_flu=None,
    vehicle="talos",
    hold_at_surface=False,
):
    """The JSON body of the emulator's `POST /api/vehicle` for nereus's ground truth (nav_msgs/Odometry of
    base_link: world-frame pose, body-frame twist). `hold_at_surface` puts it at depth 0 (where the firmware's
    depth sensor zeroes at boot) with the same attitude, standing still."""
    position, q = pose_to_emulator(position_enu, quat_enu_flu)
    body = {"vehicle": vehicle, "position_m": position, "quat_wxyz": list(q)}
    if hold_at_surface:
        body["position_m"] = [position[0], position[1], 0.0]
        body["velocity_mps"] = [0.0, 0.0, 0.0]
        body["rate_dps"] = [0.0, 0.0, 0.0]
        return body
    if linear_flu is not None:
        body["velocity_mps"] = flu_to_frd(linear_flu)
    if angular_flu is not None:
        body["rate_dps"] = [math.degrees(a) for a in flu_to_frd(angular_flu)]
    return body


def pressure_pa(depth_m, density=997.0, gravity=9.80665, surface_pa=101_325.0):
    """Absolute pressure at a depth (the emulator's water: fresh, as titan_firmware's depth driver assumes)."""
    return surface_pa + density * gravity * max(0.0, depth_m)


# ---- time ----


@dataclass
class ClockGovernor:
    """Keeps nereus's simulated time running with the emulator's virtual time, by setting nereus's
    `real_time_factor` (simulated seconds per wall second; 0 pauses it).

    The emulator runs the firmware as fast as it can, at most at wall-clock speed (`--realtime`), which for Talos is
    often slower (0.4x to 1x, depending on the machine and the boards). nereus would otherwise run at 1x and the
    vehicle would move on thrust the firmware has not produced yet. Each update measures the emulator's speed over
    `window_s` of wall time and adds a correction for the drift between the two clocks since the start (`gain` per
    second of drift), so both cover the same simulated time; the result is clamped to [`min_rtf`, `max_rtf`].
    """

    gain: float = 0.5
    min_rtf: float = 0.0
    max_rtf: float = 1.0
    window_s: float = 2.0
    _origin: tuple | None = None
    _samples: list = field(default_factory=list)
    speed: float | None = None
    drift_s: float = 0.0

    def update(self, wall_s: float, emulator_s: float, nereus_s: float | None):
        """Returns the real_time_factor to run nereus at, or None until it can tell (no nereus clock yet, or
        less than a quarter of a window measured)."""
        self._samples.append((wall_s, emulator_s))
        while len(self._samples) > 2 and wall_s - self._samples[1][0] >= self.window_s:
            self._samples.pop(0)
        (w0, e0) = self._samples[0]
        if wall_s - w0 < self.window_s / 4:
            return None
        self.speed = max(0.0, (emulator_s - e0) / (wall_s - w0))
        if nereus_s is None:
            return None
        if self._origin is None:
            self._origin = (emulator_s, nereus_s)
        # Positive: nereus is behind the firmware
        self.drift_s = (emulator_s - self._origin[0]) - (nereus_s - self._origin[1])
        rtf = self.speed + self.gain * self.drift_s
        return max(self.min_rtf, min(self.max_rtf, rtf))

    def reset(self):
        """Starts over (the emulator or nereus restarted)."""
        self._origin = None
        self._samples.clear()
        self.speed = None
        self.drift_s = 0.0


def should_set(new: float | None, last: float | None, threshold=0.02) -> bool:
    """Whether a new real_time_factor differs enough from the last one sent to be worth sending."""
    if new is None:
        return False
    return last is None or abs(new - last) >= threshold or (new == 0.0) != (last == 0.0)


def stamp_offset_s(stamp_s: float, clock_s: float) -> float:
    """How far a firmware message's header stamp is ahead of ROS time (/clock). micro-ROS boards stamp with the
    agent's wall clock at their time sync plus their own (virtual) time since, so with the emulator slower than
    real time before that sync their stamps run ahead of nereus's slowed-down clock by a constant."""
    return stamp_s - clock_s


# ---- the emulator's dashboard ----


class EmulatorError(RuntimeError):
    pass


class EmulatorClient:
    """The emulator's dashboard API (firmware_simulator robot ... --dashboard ADDR)."""

    def __init__(self, base_url="http://127.0.0.1:8080", timeout=2.0):
        if "://" not in base_url:
            base_url = "http://" + base_url
        self.base = base_url.rstrip("/")
        self.timeout = timeout

    def _request(self, method, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.base + path, data=data, method=method)
        if data is not None:
            req.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return json.loads(r.read().decode())
        except urllib.error.HTTPError as e:
            try:
                msg = json.loads(e.read().decode()).get("error", str(e))
            except (ValueError, AttributeError):
                msg = str(e)
            raise EmulatorError(f"{method} {path}: {msg}") from None
        except (urllib.error.URLError, OSError, ValueError) as e:
            raise EmulatorError(f"{method} {path}: {e}") from None

    def thrusters(self):
        return self._request("GET", "/api/thrusters")

    def post_vehicle(self, body):
        """Puts the vehicle at a pose; answers like thrusters()."""
        return self._request("POST", "/api/vehicle", body)

    def fault(self, board, fault):
        return self._request("POST", "/api/fault", {"board": board, "fault": fault})

    def state(self):
        return self._request("GET", "/api/state")


def kill_switch_fault(inserted: bool, pin=1):
    """The fault that drives the PoAC's physical kill switch pin: high while the switch is in (not killing)."""
    return f"gpio:{pin}:{'high' if inserted else 'low'}"


def status_line(s: dict) -> str:
    """One line for the log from the bridge's status."""
    parts = [f"emulator {s.get('emulator_s', 0):.1f}s"]
    if s.get("speed") is not None:
        parts.append(f"{s['speed']:.2f}x real time")
    if s.get("rtf") is not None:
        parts.append(f"nereus rtf {s['rtf']:.2f}")
    if s.get("drift_s") is not None:
        parts.append(f"clock drift {s['drift_s']:+.2f}s")
    if s.get("stamp_offset_s") is not None:
        parts.append(f"firmware stamps {s['stamp_offset_s']:+.2f}s vs /clock")
    forces = s.get("forces")
    if forces:
        parts.append("thrust [" + " ".join(f"{f:+.1f}" for f in forces) + "] N")
    return ", ".join(parts)
