//! A vehicle as a rigid body in water, in six degrees of freedom.
//!
//! The body frame's origin is the centre of gravity, its axes the principal axes of inertia (forward, starboard,
//! down). The equations are Kirchhoff's (Fossen's, for a diagonal mass matrix): with the linear and angular
//! momenta of the body and the water it carries along, `p = (m + M_A) v` and `h = (I + I_A) w`,
//!
//! ```text
//! dp/dt = F - w x p
//! dh/dt = T - w x h - v x p
//! ```
//!
//! so the added mass shows up both as inertia and as the Munk moment (`v x p`) that turns a body moving through
//! water broadside. That moment is the potential-flow value; an open frame sheds the flow and feels much less of
//! it, so a vehicle file can scale it down (`munk_moment_scale`): at full strength a bluff vehicle whose heave added
//! mass is well above its surge added mass pitches over at speed unless its buoyancy is far above its weight.
//! `F` and `T` are the sum of
//! * gravity, `m g` down at the centre of gravity;
//! * buoyancy, `rho g V` up at the centre of buoyancy (above the centre of gravity for a stable vehicle, which then
//!   rights itself in roll and pitch). Near the surface only part of the hull is under water: the vehicle is taken as
//!   a prism of `height_m` around its centre of buoyancy, so the buoyancy falls linearly as the centre of buoyancy
//!   rises through the last `height_m / 2`, and a positively buoyant vehicle floats with part of it out of the water;
//! * drag on the velocity through the water, linear plus quadratic per body axis (`-(D_l + D_q |v|) v`), scaled
//!   by the fraction under water;
//! * the thrusters' forces at their mounting points;
//! * the bottom, if the water has one (`bottom_m`): once the hull (taken as `height_m / 2` below the centre of
//!   gravity) reaches it, a stiff spring and damper hold it up and friction stops it sliding.
//!
//! The state is integrated with fixed-step fourth-order Runge-Kutta (1 ms by default), the thrust held constant
//! over the step, so a run is a pure function of its inputs and reproduces exactly.

use serde::Deserialize;

use crate::math::{Quat, Vec3, add, cross, mul, norm, scale, sub};
use crate::thruster::{Thruster, ThrusterSpec};

/// The water the vehicle is in.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields, default)]
pub struct Water {
    /// 997 kg/m³ is fresh water at 25 °C (and what titan_firmware's depth driver assumes); sea water is about 1025.
    pub density_kg_m3: f64,
    pub gravity_mps2: f64,
    /// Air pressure at the surface.
    pub surface_pa: f64,
    /// A uniform current (world frame, m/s): drag acts on the velocity through the water.
    pub current_mps: Vec3,
    /// Depth of the bottom (none: bottomless). A robot in an acoustic environment takes its pool or lake depth.
    pub bottom_m: Option<f64>,
}

impl Default for Water {
    fn default() -> Self {
        Water {
            density_kg_m3: 997.0,
            gravity_mps2: 9.80665,
            surface_pa: 101_325.0,
            current_mps: [0.0; 3],
            bottom_m: None,
        }
    }
}

impl Water {
    /// Absolute pressure at `depth` metres (the air pressure above the surface).
    pub fn pressure_at(&self, depth: f64) -> f64 {
        self.surface_pa + self.density_kg_m3 * self.gravity_mps2 * depth.max(0.0)
    }
}

/// The physical parameters of a vehicle (a vehicle file without its thrusters).
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Params {
    pub mass_kg: f64,
    /// Volume of water displaced when fully under water; the buoyancy is `rho g V`.
    pub volume_m3: f64,
    /// Moments of inertia about the body axes through the centre of gravity (roll, pitch, yaw), kg m².
    pub inertia_kg_m2: Vec3,
    /// Centre of buoyancy relative to the centre of gravity, body frame (negative `z` is above it).
    pub cob_m: Vec3,
    /// Vertical extent of the hull around the centre of buoyancy, for the buoyancy near the surface.
    pub height_m: f64,
    /// Added mass and inertia, diagonal: surge, sway, heave (kg), roll, pitch, yaw (kg m²).
    pub added_mass: [f64; 6],
    /// Linear damping per axis: N/(m/s) for surge, sway, heave, N m/(rad/s) for roll, pitch, yaw.
    pub linear_drag: [f64; 6],
    /// Quadratic damping per axis: N/(m/s)², N m/(rad/s)².
    pub quadratic_drag: [f64; 6],
    /// How much of the potential-flow Munk moment acts (1, the default, is all of it).
    #[serde(default = "one")]
    pub munk_moment_scale: f64,
}

fn one() -> f64 {
    1.0
}

impl Params {
    /// Buoyancy minus weight when fully submerged, N (positive floats up).
    pub fn net_buoyancy_n(&self, water: &Water) -> f64 {
        (water.density_kg_m3 * self.volume_m3 - self.mass_kg) * water.gravity_mps2
    }
}

/// A vehicle file (`vehicles/*.toml`): the body and its thrusters.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VehicleFile {
    pub name: String,
    #[serde(flatten)]
    pub params: Params,
    /// The thrust curve every thruster uses unless it gives its own.
    pub thruster_model: Option<crate::thruster::ThrustCurve>,
    #[serde(default)]
    pub thruster: Vec<ThrusterSpec>,
}

impl VehicleFile {
    pub fn load(path: &std::path::Path) -> Result<VehicleFile, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Self::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
    }

    pub fn parse(text: &str) -> Result<VehicleFile, String> {
        let f: VehicleFile = toml::from_str(text).map_err(|e| e.to_string())?;
        let mut names: Vec<&str> = f.thruster.iter().map(|t| t.name.as_str()).collect();
        names.sort();
        if names.windows(2).any(|w| w[0] == w[1]) {
            return Err("thruster names must be unique".into());
        }
        if f.params.mass_kg <= 0.0 || f.params.inertia_kg_m2.iter().any(|i| *i <= 0.0) || f.params.height_m <= 0.0 {
            return Err("mass, inertia and height must be positive".into());
        }
        // Thrust coefficients are thrust / at_rpm²: a zero (or negative, or non-finite) speed or thrust would make
        // them, and with them the whole state, NaN or backwards
        let curve_ok = |c: &crate::thruster::ThrustCurve| {
            c.at_rpm > 0.0
                && c.at_rpm.is_finite()
                && [c.forward_n, c.reverse_n].iter().all(|n| *n >= 0.0 && n.is_finite())
        };
        if f.thruster_model.as_ref().is_some_and(|c| !curve_ok(c)) {
            return Err("thruster_model: a thrust curve needs at_rpm > 0 and thrusts >= 0".into());
        }
        for t in &f.thruster {
            if t.curve.as_ref().is_some_and(|c| !curve_ok(c)) {
                return Err(format!("thruster {}: a thrust curve needs at_rpm > 0 and thrusts >= 0", t.name));
            }
            if t.curve.is_none() && f.thruster_model.is_none() {
                return Err(format!("thruster {}: no thrust curve (give one or a [thruster_model])", t.name));
            }
            if norm(t.direction) == 0.0 {
                return Err(format!("thruster {}: zero direction", t.name));
            }
        }
        Ok(f)
    }

    /// The thrusters, each with its curve.
    pub fn thrusters(&self) -> Vec<Thruster> {
        self.thruster.iter().map(|t| Thruster::new(t, self.thruster_model.as_ref())).collect()
    }
}

/// Where the vehicle is and how it moves.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct State {
    /// Centre of gravity, world frame (north, east, depth), m.
    pub position: Vec3,
    /// Body to world.
    pub attitude: Quat,
    /// Velocity, body frame, m/s (surge, sway, heave).
    pub velocity: Vec3,
    /// Angular rate, body frame, rad/s (roll, pitch, yaw rates).
    pub rate: Vec3,
}

impl State {
    /// At rest at `position` with roll, pitch and yaw `rpy` (radians).
    pub fn at_rest(position: Vec3, rpy: Vec3) -> State {
        State { position, attitude: Quat::from_euler(rpy[0], rpy[1], rpy[2]), ..Default::default() }
    }

    /// Velocity in the world frame.
    pub fn world_velocity(&self) -> Vec3 {
        self.attitude.rotate(self.velocity)
    }

    fn add_scaled(&self, d: &Deriv, k: f64) -> State {
        State {
            position: add(self.position, scale(d.position, k)),
            attitude: self.attitude.add_scaled(&d.attitude, k),
            velocity: add(self.velocity, scale(d.velocity, k)),
            rate: add(self.rate, scale(d.rate, k)),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Deriv {
    position: Vec3,
    attitude: Quat,
    velocity: Vec3,
    rate: Vec3,
}

/// A force and moment about the centre of gravity, body frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Wrench {
    pub force: Vec3,
    pub torque: Vec3,
}

impl Wrench {
    /// The wrench of force `f` (body frame) acting at `at` (body frame, from the centre of gravity).
    pub fn at(f: Vec3, at: Vec3) -> Wrench {
        Wrench { force: f, torque: cross(at, f) }
    }

    pub fn add(&self, o: &Wrench) -> Wrench {
        Wrench { force: add(self.force, o.force), torque: add(self.torque, o.torque) }
    }
}

/// What an IMU at the centre of gravity would read.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Imu {
    /// Specific force (acceleration minus gravity), body frame, m/s²: about `[0, 0, -9.8]` at rest and level.
    pub accel_mps2: Vec3,
    /// Body angular rate, rad/s.
    pub gyro_rps: Vec3,
    /// Roll, pitch, yaw, rad.
    pub rpy: Vec3,
}

/// A vehicle in the water.
#[derive(Clone, Debug)]
pub struct Vehicle {
    pub name: String,
    pub params: Params,
    pub water: Water,
    pub thrusters: Vec<Thruster>,
    pub state: State,
    /// Seconds simulated.
    pub time: f64,
    /// Integration step, s.
    pub dt: f64,
    /// Each thruster's thrust in the last step, N (negative is reverse).
    pub thrust_n: Vec<f64>,
    /// Acceleration in the last step: linear (body frame, of the centre of gravity) and angular.
    accel: (Vec3, Vec3),
}

impl Vehicle {
    pub fn new(file: &VehicleFile, water: Water, state: State) -> Vehicle {
        let thrusters = file.thrusters();
        Vehicle {
            name: file.name.clone(),
            params: file.params.clone(),
            water,
            thrust_n: vec![0.0; thrusters.len()],
            thrusters,
            state,
            time: 0.0,
            dt: 1e-3,
            accel: ([0.0; 3], [0.0; 3]),
        }
    }

    /// World position of a point given in the body frame (from the centre of gravity).
    pub fn point(&self, body: Vec3) -> Vec3 {
        add(self.state.position, self.state.attitude.rotate(body))
    }

    /// Absolute water pressure at a point of the body (air pressure when it is out of the water).
    pub fn pressure_at(&self, body: Vec3) -> f64 {
        self.water.pressure_at(self.point(body)[2])
    }

    /// The depth of the centre of gravity at which the vehicle floats at its current attitude, if it is
    /// positively buoyant.
    pub fn floating_depth(&self) -> Option<f64> {
        let p = &self.params;
        let fraction = p.mass_kg / (self.water.density_kg_m3 * p.volume_m3);
        if fraction >= 1.0 {
            return None;
        }
        let cob_depth = (fraction - 0.5) * p.height_m;
        Some(cob_depth - self.state.attitude.rotate(p.cob_m)[2])
    }

    /// How much of the hull is under water, 0 to 1.
    pub fn submerged(&self) -> f64 {
        submerged(&self.params, &self.state)
    }

    pub fn imu(&self) -> Imu {
        let s = &self.state;
        // The acceleration of the centre of gravity in the world frame, seen in the body frame, is v' + w x v
        let a = add(self.accel.0, cross(s.rate, s.velocity));
        let g = s.attitude.rotate_inv([0.0, 0.0, self.water.gravity_mps2]);
        Imu { accel_mps2: sub(a, g), gyro_rps: s.rate, rpy: s.attitude.euler() }
    }

    /// The thrusters' wrench at `state` for the given RPMs (signed mechanical RPM, one per thruster). A thruster
    /// out of the water pushes nothing; within 5 cm of the surface its thrust fades (it draws air).
    fn thrust_wrench(&mut self, state: &State, rpm: &[f64]) -> Wrench {
        let mut w = Wrench::default();
        for (i, t) in self.thrusters.iter().enumerate() {
            let depth = add(state.position, state.attitude.rotate(t.position))[2];
            let wet = (depth / 0.05).clamp(0.0, 1.0);
            let f = t.thrust(rpm.get(i).copied().unwrap_or(0.0)) * wet;
            self.thrust_n[i] = f;
            w = w.add(&Wrench::at(scale(t.direction, f), t.position));
        }
        w
    }

    /// Advances one step of `dt` with the thrusters at `rpm`.
    pub fn step(&mut self, rpm: &[f64]) {
        let s0 = self.state;
        let thrust = self.thrust_wrench(&s0, rpm);
        let (p, w, dt) = (&self.params, &self.water, self.dt);
        let k1 = deriv(p, w, &s0, &thrust);
        let k2 = deriv(p, w, &s0.add_scaled(&k1, dt / 2.0), &thrust);
        let k3 = deriv(p, w, &s0.add_scaled(&k2, dt / 2.0), &thrust);
        let k4 = deriv(p, w, &s0.add_scaled(&k3, dt), &thrust);
        let sum = |f: fn(&Deriv) -> Vec3| {
            let v = add(add(f(&k1), scale(f(&k2), 2.0)), add(scale(f(&k3), 2.0), f(&k4)));
            scale(v, dt / 6.0)
        };
        let q = [&k1, &k2, &k3, &k4]
            .iter()
            .zip([1.0, 2.0, 2.0, 1.0])
            .fold(s0.attitude, |q, (k, c)| q.add_scaled(&k.attitude, c * dt / 6.0));
        let s = State {
            position: add(s0.position, sum(|d| d.position)),
            attitude: q.normalized(),
            velocity: add(s0.velocity, sum(|d| d.velocity)),
            rate: add(s0.rate, sum(|d| d.rate)),
        };
        self.accel = (scale(sub(s.velocity, s0.velocity), 1.0 / dt), scale(sub(s.rate, s0.rate), 1.0 / dt));
        self.state = s;
        self.time += dt;
    }

    /// Steps until `t` seconds (to the last whole step before it), with RPMs from `rpm` at the start of each step.
    pub fn advance_to(&mut self, t: f64, mut rpm: impl FnMut(f64) -> Vec<f64>) -> usize {
        let mut n = 0;
        // The small margin keeps a step that ends exactly at `t` from being lost to rounding
        while self.time + self.dt <= t + 1e-9 {
            let r = rpm(self.time);
            self.step(&r);
            n += 1;
        }
        n
    }

    /// The wrench of everything but the thrusters (gravity, buoyancy, drag) at the current state.
    pub fn environment_wrench(&self) -> Wrench {
        environment_wrench(&self.params, &self.water, &self.state)
    }
}

fn submerged(p: &Params, s: &State) -> f64 {
    let cob_depth = s.position[2] + s.attitude.rotate(p.cob_m)[2];
    (0.5 + cob_depth / p.height_m).clamp(0.0, 1.0)
}

fn environment_wrench(p: &Params, w: &Water, s: &State) -> Wrench {
    let wet = submerged(p, s);
    let weight = s.attitude.rotate_inv([0.0, 0.0, p.mass_kg * w.gravity_mps2]);
    let buoyancy = s.attitude.rotate_inv([0.0, 0.0, -w.density_kg_m3 * w.gravity_mps2 * p.volume_m3 * wet]);
    let through_water = sub(s.velocity, s.attitude.rotate_inv(w.current_mps));
    let drag = |i: usize, v: f64| -(p.linear_drag[i] + p.quadratic_drag[i] * v.abs()) * v * wet;
    let damping = Wrench {
        force: [drag(0, through_water[0]), drag(1, through_water[1]), drag(2, through_water[2])],
        torque: [drag(3, s.rate[0]), drag(4, s.rate[1]), drag(5, s.rate[2])],
    };
    let total = Wrench::at(weight, [0.0; 3]).add(&Wrench::at(buoyancy, p.cob_m)).add(&damping);
    match w.bottom_m {
        Some(bottom) => total.add(&bottom_contact(p, s, bottom)),
        None => total,
    }
}

/// The bottom pushing back on the hull: a spring and damper (natural frequency 20 rad/s, damping ratio 0.7, so
/// the hull sinks in by about `m g / k`, 2.5 cm per unit of weight in water) and friction that stops a sliding
/// vehicle within a second, at the centre of gravity so it doesn't tip the vehicle.
fn bottom_contact(p: &Params, s: &State, bottom: f64) -> Wrench {
    let penetration = s.position[2] + p.height_m / 2.0 - bottom;
    if penetration <= 0.0 {
        return Wrench::default();
    }
    let m = p.mass_kg + p.added_mass[2];
    let k = m * 400.0;
    let c = 2.0 * 0.7 * (k * m).sqrt();
    let v = s.world_velocity();
    let up = (k * penetration + c * v[2]).max(0.0);
    let friction = 2.0 * (p.mass_kg + p.added_mass[0]);
    let world = [-friction * v[0], -friction * v[1], -up];
    Wrench::at(s.attitude.rotate_inv(world), [0.0; 3])
}

fn deriv(p: &Params, w: &Water, s: &State, thrust: &Wrench) -> Deriv {
    let total = environment_wrench(p, w, s).add(thrust);
    let m_lin = [p.mass_kg + p.added_mass[0], p.mass_kg + p.added_mass[1], p.mass_kg + p.added_mass[2]];
    let m_rot = [
        p.inertia_kg_m2[0] + p.added_mass[3],
        p.inertia_kg_m2[1] + p.added_mass[4],
        p.inertia_kg_m2[2] + p.added_mass[5],
    ];
    let lin = mul(m_lin, s.velocity);
    let ang = mul(m_rot, s.rate);
    let dlin = sub(total.force, cross(s.rate, lin));
    let munk = scale(cross(s.velocity, lin), p.munk_moment_scale);
    let dang = sub(sub(total.torque, cross(s.rate, ang)), munk);
    Deriv {
        position: s.attitude.rotate(s.velocity),
        attitude: s.attitude.derivative(s.rate),
        velocity: [dlin[0] / m_lin[0], dlin[1] / m_lin[1], dlin[2] / m_lin[2]],
        rate: [dang[0] / m_rot[0], dang[1] / m_rot[1], dang[2] / m_rot[2]],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::thruster::ThrustCurve;

    const RHO: f64 = 997.0;
    const G: f64 = 9.80665;

    /// A 30 kg box-ish vehicle displacing `buoyancy` times its weight, with thrusters given as (position, direction).
    fn vehicle(buoyancy: f64, thrusters: &[(Vec3, Vec3)]) -> Vehicle {
        let file = VehicleFile {
            name: "test".into(),
            params: Params {
                mass_kg: 30.0,
                volume_m3: 30.0 * buoyancy / RHO,
                inertia_kg_m2: [1.0, 2.0, 2.0],
                cob_m: [0.0, 0.0, -0.05],
                height_m: 0.4,
                added_mass: [10.0, 20.0, 30.0, 0.5, 1.0, 1.0],
                linear_drag: [20.0, 30.0, 40.0, 3.0, 4.0, 4.0],
                quadratic_drag: [40.0, 60.0, 80.0, 1.0, 1.0, 1.0],
                munk_moment_scale: 1.0,
            },
            thruster_model: Some(ThrustCurve { forward_n: 50.0, reverse_n: 40.0, at_rpm: 4000.0 }),
            thruster: thrusters
                .iter()
                .enumerate()
                .map(|(i, (p, d))| ThrusterSpec {
                    name: format!("t{i}"),
                    position_m: *p,
                    direction: *d,
                    curve: None,
                    inverted: false,
                })
                .collect(),
        };
        Vehicle::new(&file, Water::default(), State::at_rest([0.0, 0.0, 2.0], [0.0; 3]))
    }

    /// Runs for `secs` with constant RPMs.
    fn run(v: &mut Vehicle, secs: f64, rpm: &[f64]) {
        let end = v.time + secs;
        v.advance_to(end, |_| rpm.to_vec());
    }

    /// The speed at which linear plus quadratic drag balance `force`.
    fn terminal(force: f64, lin: f64, quad: f64) -> f64 {
        (-lin + (lin * lin + 4.0 * quad * force).sqrt()) / (2.0 * quad)
    }

    #[test]
    fn neutral_buoyancy_holds_depth_and_attitude() {
        let mut v = vehicle(1.0, &[]);
        assert!(v.params.net_buoyancy_n(&v.water).abs() < 1e-9);
        run(&mut v, 20.0, &[]);
        assert!((v.state.position[2] - 2.0).abs() < 1e-9, "{:?}", v.state);
        assert!(norm(v.state.velocity) < 1e-9 && norm(v.state.rate) < 1e-12);
        assert_eq!(v.submerged(), 1.0);
        // At rest and level an IMU reads gravity's reaction (up, negative z)
        let imu = v.imu();
        assert!(norm(sub(imu.accel_mps2, [0.0, 0.0, -G])) < 1e-6, "{imu:?}");
    }

    #[test]
    fn positive_buoyancy_rises_and_floats_at_the_surface() {
        // 2 % positive (6 N): rises, then floats with the hull out of the water just enough to shed the excess
        let mut v = vehicle(1.02, &[]);
        let mut highest = f64::MAX;
        let mut rising = Vec::new();
        for _ in 0..600 {
            run(&mut v, 0.1, &[]);
            highest = highest.min(v.state.position[2]);
            rising.push(v.state.position[2]);
        }
        // Rising at the terminal speed where drag takes the 6 N of net buoyancy
        let expected = terminal(0.02 * 30.0 * G, 40.0, 80.0);
        let speed = (rising[79] - rising[99]) / 2.0;
        assert!((speed - expected).abs() < 0.002, "{speed} vs {expected}");
        // Afloat: the submerged fraction carries exactly the weight
        let f = 1.0 / 1.02;
        let cob_depth = (f - 0.5) * 0.4;
        let z = v.state.position[2] - 0.05;
        assert!((z - cob_depth).abs() < 1e-3, "{z} vs {cob_depth}");
        assert!((v.submerged() - f).abs() < 1e-3);
        // It never leaves the water (the centre of buoyancy stays below the top of the hull's range)
        assert!(highest - 0.05 > -0.2, "{highest}");
        assert!(v.state.velocity[2].abs() < 1e-3);
        // Where it floats is known up front, and a vehicle put there stays
        let depth = v.floating_depth().unwrap();
        assert!((depth - v.state.position[2]).abs() < 1e-3);
        let mut w = vehicle(1.02, &[]);
        w.state.position[2] = depth;
        run(&mut w, 5.0, &[]);
        assert!((w.state.position[2] - depth).abs() < 1e-9);
        assert_eq!(vehicle(1.0, &[]).floating_depth(), None);
    }

    #[test]
    fn negative_buoyancy_sinks_at_the_drag_terminal_velocity() {
        let mut v = vehicle(0.97, &[]);
        run(&mut v, 30.0, &[]);
        let expected = terminal(0.03 * 30.0 * G, 40.0, 80.0);
        assert!((v.state.velocity[2] - expected).abs() < 1e-6, "{} vs {expected}", v.state.velocity[2]);
        assert!(v.state.position[2] > 2.0 + expected * 25.0);
    }

    #[test]
    fn forward_thrust_reaches_the_surge_terminal_velocity() {
        let fwd = [1.0, 0.0, 0.0];
        let mut v = vehicle(1.0, &[([-0.3, 0.2, 0.0], fwd), ([-0.3, -0.2, 0.0], fwd)]);
        run(&mut v, 30.0, &[2000.0, 2000.0]);
        // 12.5 N each at half speed (thrust goes with RPM squared)
        assert!((v.thrust_n[0] - 12.5).abs() < 1e-9);
        let expected = terminal(25.0, 20.0, 40.0);
        assert!((v.state.velocity[0] - expected).abs() < 1e-6, "{} vs {expected}", v.state.velocity[0]);
        // Straight ahead: no turning, no sideways drift
        assert!(v.state.attitude.euler().iter().all(|a| a.abs() < 1e-9));
        assert!((v.state.position[0] - expected * 30.0).abs() < 1.0);
        // Reverse is weaker
        run(&mut v, 30.0, &[-2000.0, -2000.0]);
        let back = terminal(20.0, 20.0, 40.0);
        assert!((v.state.velocity[0] + back).abs() < 1e-6, "{} vs {back}", v.state.velocity[0]);
    }

    #[test]
    fn differential_thrust_yaws() {
        let fwd = [1.0, 0.0, 0.0];
        let mut v = vehicle(1.0, &[([-0.3, 0.2, 0.0], fwd), ([-0.3, -0.2, 0.0], fwd)]);
        // Starboard thruster ahead, port astern: the bow swings to port (negative yaw, z is down)
        run(&mut v, 20.0, &[2000.0, -2000.0]);
        let r = v.state.rate[2];
        assert!(r < 0.0);
        // Steady turn rate: yaw drag balances the couple (12.5 N and 10 N, 0.2 m either side)
        let couple = 0.2 * 12.5 + 0.2 * 10.0;
        // The unequal thrusts also push it forwards a little, which the turn spins around (Coriolis): small sway
        assert!((r.abs() - terminal(couple, 4.0, 1.0)).abs() < 0.01, "{r}");
        assert!(v.state.rate[0].abs() < 1e-9 && v.state.rate[1].abs() < 1e-9);
        assert!(v.state.position[2] == 2.0);
    }

    #[test]
    fn buoyancy_above_gravity_rights_a_roll() {
        let mut v = vehicle(1.0, &[]);
        v.params.linear_drag[3] = 0.2;
        v.params.quadratic_drag[3] = 0.0;
        v.state.attitude = Quat::from_euler(30f64.to_radians(), 0.0, 0.0);
        // Rolled to starboard, the righting moment (buoyancy 5 cm above gravity) rolls it back to port
        let w = v.environment_wrench();
        let expected = -(30.0 * G) * 0.05 * 30f64.to_radians().sin();
        assert!((w.torque[0] - expected).abs() < 1e-9, "{w:?}");
        // A pendulum: period 2 pi sqrt((I + I_A) / (B GM)) for small angles; time the zero crossings
        v.state.attitude = Quat::from_euler(5f64.to_radians(), 0.0, 0.0);
        let mut crossings = Vec::new();
        let mut last = v.state.attitude.euler()[0];
        for _ in 0..20_000 {
            v.step(&[]);
            let roll = v.state.attitude.euler()[0];
            if last > 0.0 && roll <= 0.0 {
                crossings.push(v.time);
            }
            last = roll;
        }
        let period = (crossings[3] - crossings[0]) / 3.0;
        let expected = 2.0 * std::f64::consts::PI * (1.5 / (30.0 * G * 0.05)).sqrt();
        assert!((period / expected - 1.0).abs() < 0.01, "{period} vs {expected}");
        // And with the drag it settles level
        v.params.linear_drag[3] = 3.0;
        run(&mut v, 30.0, &[]);
        assert!(v.state.attitude.euler()[0].abs() < 1e-3);
    }

    #[test]
    fn pitch_is_righted_too_and_the_depth_holds() {
        let mut v = vehicle(1.0, &[]);
        v.state.attitude = Quat::from_euler(0.0, -20f64.to_radians(), 0.0);
        run(&mut v, 40.0, &[]);
        let [roll, pitch, _] = v.state.attitude.euler();
        assert!(pitch.abs() < 1e-3 && roll.abs() < 1e-9, "{pitch}");
        assert!((v.state.position[2] - 2.0).abs() < 0.01);
    }

    #[test]
    fn thrusters_out_of_the_water_push_nothing() {
        let down = [0.0, 0.0, 1.0];
        // Floating: the thruster on top of the hull is in the air, the one below is wet
        let mut v = vehicle(1.0, &[([0.0, 0.0, -0.5], down), ([0.0, 0.0, 0.3], down)]);
        v.state.position = [0.0, 0.0, 0.1];
        v.step(&[4000.0, 4000.0]);
        assert_eq!(v.thrust_n[0], 0.0);
        assert!((v.thrust_n[1] - 50.0).abs() < 1e-9);
        // Thrusting up at the surface doesn't lift it out of the water: the buoyancy is lost instead
        let mut v = vehicle(1.0, &[([0.0, 0.0, 0.1], [0.0, 0.0, -1.0])]);
        v.state.position = [0.0, 0.0, 1.0];
        let mut highest = f64::MAX;
        for _ in 0..300 {
            run(&mut v, 0.1, &[4000.0]);
            highest = highest.min(v.state.position[2]);
        }
        assert!(highest > -0.3, "{highest}");
    }

    #[test]
    fn a_sinking_vehicle_comes_to_rest_on_the_bottom() {
        let mut v = vehicle(0.95, &[([0.0; 3], [1.0, 0.0, 0.0])]);
        v.water.bottom_m = Some(3.0);
        v.state.velocity = [0.3, 0.0, 0.0];
        run(&mut v, 30.0, &[]);
        // Resting with the hull pressed in just enough for the contact spring to carry its 15 N of weight in water
        let weight = 0.05 * 30.0 * G;
        let k = 60.0 * 400.0;
        let expected = 3.0 - 0.2 + weight / k;
        assert!((v.state.position[2] - expected).abs() < 1e-4, "{} vs {expected}", v.state.position[2]);
        assert!(norm(v.state.velocity) < 1e-4, "{:?}", v.state.velocity);
        // Friction slows it more than the water alone would, but full thrust still drives it along the bottom
        let x = v.state.position[0];
        run(&mut v, 5.0, &[4000.0]);
        assert!(v.state.position[0] - x > 1.0, "{}", v.state.position[0] - x);
        assert!(v.state.position[2] < 3.0);
    }

    #[test]
    fn the_munk_moment_turns_a_moving_body_broadside() {
        // Moving ahead and a little upwards relative to the hull (the nose below the path): with 30 kg of heave
        // added mass against 10 kg in surge, the moment (Z_w' - X_u') u w turns the nose further down
        let mut v = vehicle(1.0, &[]);
        v.state.velocity = [1.0, 0.0, -0.05];
        let d = deriv(&v.params, &v.water, &v.state, &Wrench::default());
        let expected = -(30.0 - 10.0) * 1.0 * 0.05 / (2.0 + 1.0);
        assert!((d.rate[1] - expected).abs() < 1e-9, "{} vs {expected}", d.rate[1]);
        v.params.munk_moment_scale = 0.25;
        let d = deriv(&v.params, &v.water, &v.state, &Wrench::default());
        assert!((d.rate[1] - expected / 4.0).abs() < 1e-9);
    }

    #[test]
    fn a_current_carries_a_neutral_vehicle_along() {
        let mut v = vehicle(1.0, &[]);
        v.water.current_mps = [0.0, 0.3, 0.0];
        v.state.attitude = Quat::from_euler(0.0, 0.0, 0.4);
        run(&mut v, 60.0, &[]);
        let vw = v.state.world_velocity();
        assert!(norm(sub(vw, [0.0, 0.3, 0.0])) < 1e-3, "{vw:?}");
    }

    #[test]
    fn sensors_on_the_hull_see_its_depth_and_attitude() {
        let mut v = vehicle(1.0, &[]);
        v.state.position = [1.0, 2.0, 3.0];
        // Pitched nose down 90 degrees: a sensor 0.5 m forward is 0.5 m deeper
        v.state.attitude = Quat::from_euler(0.0, -std::f64::consts::FRAC_PI_2, 0.0);
        let p = v.point([0.5, 0.0, 0.0]);
        assert!(norm(sub(p, [1.0, 2.0, 3.5])) < 1e-12, "{p:?}");
        assert!((v.pressure_at([0.5, 0.0, 0.0]) - (101_325.0 + RHO * G * 3.5)).abs() < 1e-6);
        // Above the surface: air pressure
        v.state.position = [0.0, 0.0, -1.0];
        assert_eq!(v.pressure_at([0.0; 3]), 101_325.0);
    }

    #[test]
    fn runs_reproduce_exactly() {
        let fwd = [1.0, 0.0, 0.0];
        let go = || {
            let mut v = vehicle(1.01, &[([-0.3, 0.2, 0.1], fwd), ([-0.3, -0.2, 0.1], [0.0, 0.7, 0.7])]);
            v.advance_to(10.0, |t| vec![3000.0 * (t * 2.0).sin(), -1500.0]);
            v.state
        };
        assert_eq!(go(), go());
    }

    #[test]
    fn vehicle_files_are_checked() {
        let ok = r#"
name = "x"
mass_kg = 10.0
volume_m3 = 0.01
inertia_kg_m2 = [1.0, 1.0, 1.0]
cob_m = [0.0, 0.0, -0.01]
height_m = 0.3
added_mass = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_drag = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
quadratic_drag = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
thruster_model = { forward_n = 50.0, reverse_n = 40.0, at_rpm = 3800.0 }
[[thruster]]
name = "a"
position_m = [0.0, 0.0, 0.0]
direction = [1.0, 0.0, 0.0]
"#;
        let f = VehicleFile::parse(ok).unwrap();
        assert_eq!(f.thrusters()[0].direction, [1.0, 0.0, 0.0]);
        assert!(VehicleFile::parse(&ok.replace("thruster_model", "#")).unwrap_err().contains("no thrust curve"));
        assert!(
            VehicleFile::parse(&format!(
                "{ok}[[thruster]]\nname = \"a\"\nposition_m = [0.0, 0.0, 0.0]\ndirection = [0.0, 0.0, 1.0]\n"
            ))
            .unwrap_err()
            .contains("unique")
        );
        assert!(VehicleFile::parse(&ok.replace("mass_kg = 10.0", "mass_kg = 0.0")).is_err());
        assert!(VehicleFile::parse(&ok.replace("direction = [1.0, 0.0, 0.0]", "direction = [0.0, 0.0, 0.0]")).is_err());
        // A thrust curve at 0 RPM has no coefficient (it would make every thrust, and the whole state, NaN)
        let zero_rpm = ok.replace("at_rpm = 3800.0", "at_rpm = 0.0");
        assert!(VehicleFile::parse(&zero_rpm).unwrap_err().contains("thrust curve"));
        let own = format!("{ok}curve = {{ forward_n = 50.0, reverse_n = 40.0, at_rpm = 0.0 }}\n");
        assert!(VehicleFile::parse(&own).unwrap_err().contains("thrust curve"));
        assert!(VehicleFile::parse(&ok.replace("forward_n = 50.0", "forward_n = -50.0")).is_err());
    }
}
