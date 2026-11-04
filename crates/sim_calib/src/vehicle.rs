//! Vehicle calibration from a pool test: the body's drag, added mass, buoyancy (and anything else in a vehicle
//! file's parameters) fitted by simulating the recorded thruster speeds with [`sim_physics::Vehicle`] and
//! minimising the difference to the recorded depth, attitude, velocities and rates.
//!
//! The log is a CSV table (`docs/calibration.md`; `scripts/bag_to_calib.py` writes one from a ROS 2 bag, and
//! `robot --vehicle-csv` writes one from the emulator): `time_s`, one input column per thruster (`rpm_NAME` motor
//! RPM, `cmd_NAME` commanded RPM, which goes through the ESC's lag, or `NAME_n` thrust in N), and any of the
//! measured `north_m`, `east_m`, `depth_m`, `roll_deg`, `pitch_deg`, `yaw_deg`, `u_mps`, `v_mps`, `w_mps`, `p_dps`,
//! `q_dps`, `r_dps`.
//!
//! The log is cut into segments (multiple shooting): each starts from the measured state, so an error early on
//! does not grow over the whole log, and unmeasured parts of the state (north and east in a pool without
//! positioning) do not matter beyond a segment.

use std::fmt::Write as _;

use sim_physics::{Params, Quat, State, Thruster, Vehicle, VehicleFile, Water};

use crate::fit::{self, Fit, Options, Param};
use crate::table::{Series, Table, markdown, svg_plot};

/// The measured channels: column, standard deviation used to weigh it, and how to read it from a state.
pub const CHANNELS: [(&str, f64); 12] = [
    ("north_m", 0.10),
    ("east_m", 0.10),
    ("depth_m", 0.02),
    ("roll_deg", 1.0),
    ("pitch_deg", 1.0),
    ("yaw_deg", 2.0),
    ("u_mps", 0.03),
    ("v_mps", 0.03),
    ("w_mps", 0.03),
    ("p_dps", 2.0),
    ("q_dps", 2.0),
    ("r_dps", 2.0),
];

fn read_state(s: &State) -> [f64; 12] {
    let rpy = s.attitude.euler();
    [
        s.position[0],
        s.position[1],
        s.position[2],
        rpy[0].to_degrees(),
        rpy[1].to_degrees(),
        rpy[2].to_degrees(),
        s.velocity[0],
        s.velocity[1],
        s.velocity[2],
        s.rate[0].to_degrees(),
        s.rate[1].to_degrees(),
        s.rate[2].to_degrees(),
    ]
}

const AXES: [&str; 6] = ["surge", "sway", "heave", "roll", "pitch", "yaw"];

/// The vehicle parameters calibration can fit, by name: `mass_kg`, `volume_m3`, `height_m`, `munk_moment_scale`,
/// `cob_m.x|y|z`, `inertia.roll|pitch|yaw`, and `added_mass.AXIS`, `linear_drag.AXIS`, `quadratic_drag.AXIS` with
/// AXIS one of surge, sway, heave, roll, pitch, yaw.
pub fn param_mut<'a>(p: &'a mut Params, name: &str) -> Option<&'a mut f64> {
    let axis = |s: &str| AXES.iter().position(|a| *a == s);
    let (head, tail) = name.split_once('.').unwrap_or((name, ""));
    match (head, tail) {
        ("mass_kg", "") => Some(&mut p.mass_kg),
        ("volume_m3", "") => Some(&mut p.volume_m3),
        ("height_m", "") => Some(&mut p.height_m),
        ("munk_moment_scale", "") => Some(&mut p.munk_moment_scale),
        ("cob_m", c) => ["x", "y", "z"].iter().position(|a| *a == c).map(|i| &mut p.cob_m[i]),
        ("inertia", c) => ["roll", "pitch", "yaw"].iter().position(|a| *a == c).map(|i| &mut p.inertia_kg_m2[i]),
        ("added_mass", c) => axis(c).map(|i| &mut p.added_mass[i]),
        ("linear_drag", c) => axis(c).map(|i| &mut p.linear_drag[i]),
        ("quadratic_drag", c) => axis(c).map(|i| &mut p.quadratic_drag[i]),
        _ => None,
    }
}

/// The TOML key and index a parameter is stored at in a vehicle file.
fn toml_place(name: &str) -> (&'static str, Option<usize>) {
    let (head, tail) = name.split_once('.').unwrap_or((name, ""));
    let idx = |list: &[&str]| list.iter().position(|a| *a == tail);
    match head {
        "mass_kg" => ("mass_kg", None),
        "volume_m3" => ("volume_m3", None),
        "height_m" => ("height_m", None),
        "munk_moment_scale" => ("munk_moment_scale", None),
        "cob_m" => ("cob_m", idx(&["x", "y", "z"])),
        "inertia" => ("inertia_kg_m2", idx(&["roll", "pitch", "yaw"])),
        "added_mass" => ("added_mass", idx(&AXES)),
        "linear_drag" => ("linear_drag", idx(&AXES)),
        _ => ("quadratic_drag", idx(&AXES)),
    }
}

/// What the defaults fit: buoyancy, and drag and added mass on the axes a pool test usually excites.
pub const DEFAULT_FIT: &[&str] = &[
    "volume_m3",
    "linear_drag.surge",
    "quadratic_drag.surge",
    "added_mass.surge",
    "linear_drag.heave",
    "quadratic_drag.heave",
    "added_mass.heave",
    "linear_drag.yaw",
    "quadratic_drag.yaw",
    "added_mass.yaw",
];

/// A pool-test log, read for a vehicle's thrusters.
#[derive(Clone, Debug)]
pub struct Log {
    pub time: Vec<f64>,
    /// Motor RPM per row and thruster (vehicle file order).
    pub rpm: Vec<Vec<f64>>,
    /// Measured channel values per row (`NaN` where not measured), in [`CHANNELS`] order.
    pub meas: Vec<[f64; 12]>,
    /// Which channels the log has at all.
    pub has: [bool; 12],
    pub notes: Vec<String>,
}

/// The motor RPM a thruster needs for a thrust along its direction (the curve inverted).
fn rpm_for_thrust(t: &Thruster, f: f64) -> f64 {
    let along = if f >= 0.0 { (f / t.k_forward).sqrt() } else { -(-f / t.k_reverse).sqrt() };
    if t.inverted { -along } else { along }
}

/// Reads a log table for the vehicle's thrusters; `esc_tau_s` is the lag commanded RPMs (`cmd_` columns) go
/// through.
pub fn read_log(t: &Table, file: &VehicleFile, esc_tau_s: f64) -> Result<Log, String> {
    let time = t.col("time_s").ok_or("no time_s column")?;
    if time.windows(2).any(|w| w[1].partial_cmp(&w[0]) != Some(std::cmp::Ordering::Greater)) {
        return Err("time_s must increase from row to row".into());
    }
    let thrusters = file.thrusters();
    let n = time.len();
    let mut rpm = vec![vec![0.0; thrusters.len()]; n];
    let mut notes = Vec::new();
    let mut any = false;
    for (k, th) in thrusters.iter().enumerate() {
        let name = &th.name;
        let by = |prefix: &str, suffix: &str| {
            t.col(&format!("{prefix}{name}{suffix}")).or_else(|| t.col(&format!("{prefix}{k}{suffix}")))
        };
        if let Some(c) = by("rpm_", "") {
            for (r, v) in rpm.iter_mut().zip(c) {
                r[k] = if v.is_finite() { v } else { 0.0 };
            }
        } else if let Some(c) = by("cmd_", "") {
            // The ESC's first-order lag, exact for a command held between rows
            let mut x = 0.0;
            for i in 0..n {
                rpm[i][k] = x;
                let dt = if i + 1 < n { time[i + 1] - time[i] } else { 0.0 };
                let target = if c[i].is_finite() { c[i] } else { 0.0 };
                x += (target - x) * (1.0 - (-dt / esc_tau_s).exp());
            }
            notes.push(format!("{name}: commanded RPM through a {:.0} ms lag", esc_tau_s * 1000.0));
        } else if let Some(c) = by("", "_n") {
            for (r, v) in rpm.iter_mut().zip(c) {
                r[k] = if v.is_finite() { rpm_for_thrust(th, v) } else { 0.0 };
            }
        } else {
            notes.push(format!("{name}: no rpm_{name}, cmd_{name} or {name}_n column, taken as stopped"));
            continue;
        }
        any = true;
    }
    if !any {
        return Err("no thruster columns (rpm_NAME, cmd_NAME or NAME_n for the vehicle's thrusters)".into());
    }
    let cols: Vec<Option<Vec<f64>>> = CHANNELS.iter().map(|(c, _)| t.col(c)).collect();
    let mut has = [false; 12];
    for (i, c) in cols.iter().enumerate() {
        has[i] = c.as_ref().is_some_and(|c| c.iter().any(|v| v.is_finite()));
    }
    if !has.iter().any(|h| *h) {
        return Err("no measured columns (depth_m, roll_deg, ..., see docs/calibration.md)".into());
    }
    let meas = (0..n)
        .map(|i| {
            let mut m = [f64::NAN; 12];
            for (j, c) in cols.iter().enumerate() {
                if let Some(c) = c {
                    m[j] = c[i];
                }
            }
            m
        })
        .collect();
    Ok(Log { time, rpm, meas, has, notes })
}

/// The state a segment starts from: the measured one, with what was not measured taken as zero (level, at rest).
fn start_state(m: &[f64; 12]) -> State {
    let v = |i: usize| if m[i].is_finite() { m[i] } else { 0.0 };
    State {
        position: [v(0), v(1), v(2)],
        attitude: Quat::from_euler(v(3).to_radians(), v(4).to_radians(), v(5).to_radians()),
        velocity: [v(6), v(7), v(8)],
        rate: [v(9).to_radians(), v(10).to_radians(), v(11).to_radians()],
    }
}

/// Options of a vehicle calibration.
#[derive(Clone, Debug)]
pub struct VehicleOptions {
    pub fit: Vec<String>,
    /// Segment length (each starts from the measured state).
    pub segment_s: f64,
    pub water: Water,
    pub method: fit::Method,
    pub max_iter: usize,
}

impl Default for VehicleOptions {
    fn default() -> Self {
        VehicleOptions {
            fit: DEFAULT_FIT.iter().map(|s| s.to_string()).collect(),
            segment_s: 4.0,
            water: Water::default(),
            method: fit::Method::LevenbergMarquardt,
            max_iter: 60,
        }
    }
}

/// Simulates the log with `params`, segment by segment: the simulated channels at every row (a segment's first
/// row is its measured start).
pub fn simulate(file: &VehicleFile, params: &Params, water: &Water, log: &Log, segment_s: f64) -> Vec<[f64; 12]> {
    let n = log.time.len();
    let mut out = vec![[f64::NAN; 12]; n];
    let mut i0 = 0;
    while i0 < n {
        let mut i1 = i0 + 1;
        while i1 < n && log.time[i1] - log.time[i0] < segment_s {
            i1 += 1;
        }
        let mut v = Vehicle::new(file, water.clone(), start_state(&log.meas[i0]));
        v.params = params.clone();
        out[i0] = read_state(&v.state);
        for (k, o) in out.iter_mut().enumerate().take(i1).skip(i0 + 1) {
            let rpm = log.rpm[k - 1].clone();
            v.advance_to(log.time[k] - log.time[i0], |_| rpm.clone());
            *o = read_state(&v.state);
        }
        i0 = i1;
    }
    out
}

fn residuals(log: &Log, sim: &[[f64; 12]]) -> Vec<f64> {
    let mut r = Vec::new();
    for (m, s) in log.meas.iter().zip(sim) {
        for j in 0..12 {
            if !log.has[j] {
                continue;
            }
            // Every row and measured channel gives a residual (zero where a value is missing), so the count is
            // the same for every parameter set
            let mut d = if m[j].is_finite() && s[j].is_finite() { s[j] - m[j] } else { 0.0 };
            if (3..6).contains(&j) {
                d = (d + 180.0).rem_euclid(360.0) - 180.0;
            }
            r.push(d / CHANNELS[j].1);
        }
    }
    r
}

/// What a vehicle calibration found.
#[derive(Clone, Debug)]
pub struct Calibration {
    pub fit: Fit,
    pub params: Params,
    /// RMS error per measured channel (column, before, after).
    pub channels: Vec<(String, f64, f64)>,
    pub report: String,
    /// The vehicle file with the fitted values.
    pub toml: String,
    pub plots: Vec<(String, String)>,
}

/// Fits the log. `source_toml` is the vehicle file's text (the fitted file keeps its other keys).
pub fn calibrate(
    file: &VehicleFile,
    source_toml: &str,
    log: &Log,
    opts: &VehicleOptions,
) -> Result<Calibration, String> {
    let mut start = file.params.clone();
    let mut params = Vec::new();
    for name in &opts.fit {
        let v = *param_mut(&mut start, name).ok_or_else(|| format!("no vehicle parameter {name:?}"))?;
        params.push(if name.starts_with("cob_m") {
            Param::new(name, v, 1e-3)
        } else if v > 0.0 {
            Param::positive(name, v, 1e-3)
        } else {
            Param::new(name, v, 1e-2).bounded(0.0, f64::INFINITY)
        });
    }
    let make = |v: &[f64]| -> Params {
        let mut p = file.params.clone();
        for (name, x) in opts.fit.iter().zip(v) {
            *param_mut(&mut p, name).unwrap() = *x;
        }
        p
    };
    let resid = |v: &[f64]| residuals(log, &simulate(file, &make(v), &opts.water, log, opts.segment_s));
    let f = fit::fit(&params, resid, &Options { method: opts.method, max_iter: opts.max_iter, ..Default::default() });
    let fitted = make(&f.params.iter().map(|p| p.value).collect::<Vec<_>>());
    let before = simulate(file, &file.params, &opts.water, log, opts.segment_s);
    let after = simulate(file, &fitted, &opts.water, log, opts.segment_s);
    let rms_of = |sim: &[[f64; 12]], j: usize| {
        let d: Vec<f64> = log
            .meas
            .iter()
            .zip(sim)
            .filter(|(m, s)| m[j].is_finite() && s[j].is_finite())
            .map(|(m, s)| {
                let d = s[j] - m[j];
                if (3..6).contains(&j) { (d + 180.0).rem_euclid(360.0) - 180.0 } else { d }
            })
            .collect();
        fit::rms(&d)
    };
    let channels: Vec<(String, f64, f64)> = (0..12)
        .filter(|&j| log.has[j])
        .map(|j| (CHANNELS[j].0.to_string(), rms_of(&before, j), rms_of(&after, j)))
        .collect();
    let toml = render_toml(source_toml, &f)?;
    let mut plots = Vec::new();
    for (j, title) in [(2, "Depth"), (5, "Yaw"), (6, "Surge velocity"), (8, "Heave velocity")] {
        if !log.has[j] {
            continue;
        }
        let pts =
            |sim: &[[f64; 12]]| -> Vec<(f64, f64)> { log.time.iter().zip(sim).map(|(t, s)| (*t, s[j])).collect() };
        let meas: Vec<(f64, f64)> = log.time.iter().zip(&log.meas).map(|(t, m)| (*t, m[j])).collect();
        plots.push((
            format!("{}.svg", CHANNELS[j].0),
            svg_plot(
                &format!("{title}: measured and simulated ({:.0} s segments from the measured state)", opts.segment_s),
                "time (s)",
                CHANNELS[j].0,
                &[
                    Series::dots("measured", meas),
                    Series::line("fitted", pts(&after)),
                    Series::line("before", pts(&before)),
                ],
                false,
            ),
        ));
    }
    let report = render_report(file, log, &f, &channels, opts);
    Ok(Calibration { fit: f, params: fitted, channels, report, toml, plots })
}

fn render_toml(source: &str, f: &Fit) -> Result<String, String> {
    let mut doc: toml::Table = toml::from_str(source).map_err(|e| e.to_string())?;
    for p in &f.params {
        let (key, idx) = toml_place(&p.name);
        // Eight significant digits: a volume needs them (1e-6 m³ is 0.01 N of buoyancy)
        let v = toml::Value::Float(format!("{:.7e}", p.value).parse().unwrap());
        match idx {
            None => {
                doc.insert(key.into(), v);
            }
            Some(i) => {
                let arr =
                    doc.get_mut(key).and_then(|a| a.as_array_mut()).ok_or_else(|| format!("{key} is not an array"))?;
                arr[i] = v;
            }
        }
    }
    let names: Vec<&str> = f.params.iter().map(|p| p.name.as_str()).collect();
    Ok(format!(
        "# Fitted by `firmware_simulator calibrate vehicle` ({}); see report.md. The other values are the original\n\
         # file's (whose comments say where they came from).\n{}",
        names.join(", "),
        toml::to_string(&doc).map_err(|e| e.to_string())?
    ))
}

fn render_report(
    file: &VehicleFile,
    log: &Log,
    f: &Fit,
    channels: &[(String, f64, f64)],
    opts: &VehicleOptions,
) -> String {
    let mut o = format!("# Vehicle calibration: {}\n\n", file.name);
    let dur = log.time.last().unwrap_or(&0.0) - log.time.first().unwrap_or(&0.0);
    let _ = writeln!(
        o,
        "{} rows over {dur:.1} s, simulated in {:.1} s segments that each start from the measured state; {} \
         evaluations, {} iterations.\n",
        log.time.len(),
        opts.segment_s,
        f.evaluations,
        f.iterations
    );
    for n in &log.notes {
        let _ = writeln!(o, "* {n}");
    }
    if !log.notes.is_empty() {
        o.push('\n');
    }
    o += "## Parameters\n\n";
    let mut start = file.params.clone();
    let rows: Vec<Vec<String>> = f
        .params
        .iter()
        .zip(&f.std_err)
        .map(|(p, se)| {
            let s = *param_mut(&mut start, &p.name).unwrap();
            vec![
                format!("`{}`", p.name),
                format!("{s:.4}"),
                format!("{:.4}", p.value),
                if se.is_finite() { format!("{se:.4}") } else { "-".into() },
            ]
        })
        .collect();
    o += &markdown(&["parameter", "before", "fitted", "± (1 sigma)"], &rows);
    o += "\nA standard error as large as the value means the log does not excite that axis enough to tell it: fit \
          fewer parameters, or record a manoeuvre that moves the vehicle along it.\n\n";
    for (a, b, c) in f.degenerate_pairs(0.95) {
        let _ = writeln!(o, "* `{a}` and `{b}` are {c:+.2} correlated: the log cannot tell them apart.");
    }
    o += "\n## Trajectory error (RMS)\n\n";
    let rows: Vec<Vec<String>> =
        channels.iter().map(|(c, b, a)| vec![format!("`{c}`"), format!("{b:.4}"), format!("{a:.4}")]).collect();
    o += &markdown(&["channel", "before", "fitted"], &rows);
    let mut water = String::new();
    let _ = write!(water, "{} kg/m³", opts.water.density_kg_m3);
    let _ = writeln!(
        o,
        "\nWater density {water}. Plots: depth_m.svg, yaw_deg.svg, u_mps.svg, w_mps.svg (those the log has). Use \
         `vehicle.toml` as the vehicle file."
    );
    o
}

/// A pool test from the emulator's own vehicle model with known parameters, `params`: a minute of thruster steps
/// (dives of two strengths, rises, surges, yaws, each held 5 s, sampled at 20 Hz) from 1.5 m down, logged as a
/// calibration CSV (motor RPM in, depth, attitude, DVL velocities and gyro rates out, no north/east), with
/// Gaussian noise scaled by `noise` (1: 1 cm, 0.3°, 1 cm/s, 0.5 °/s) from `seed`.
pub fn synthetic_pool_test(file: &VehicleFile, params: &Params, water: &Water, noise: f64, seed: u64) -> Table {
    let th = file.thrusters();
    // Effective commands (pushing along each thruster's direction) by phase: [horizontal x4, vertical x4]
    let phases: [[f64; 8]; 12] = [
        [0.0, 0.0, 0.0, 0.0, 1500.0, 1500.0, 1500.0, 1500.0],
        [0.0, 0.0, 0.0, 0.0, 3000.0, 3000.0, 3000.0, 3000.0],
        [0.0; 8],
        [1500.0, 1500.0, 1500.0, 1500.0, 0.0, 0.0, 0.0, 0.0],
        [3200.0, 3200.0, 3200.0, 3200.0, 0.0, 0.0, 0.0, 0.0],
        [0.0; 8],
        [-1800.0, 1800.0, -1800.0, 1800.0, 0.0, 0.0, 0.0, 0.0],
        [-3000.0, 3000.0, -3000.0, 3000.0, 0.0, 0.0, 0.0, 0.0],
        [0.0; 8],
        [0.0, 0.0, 0.0, 0.0, -1200.0, -1200.0, -1200.0, -1200.0],
        [0.0, 0.0, 0.0, 0.0, 2500.0, 2500.0, 2500.0, 2500.0],
        [0.0; 8],
    ];
    let mut v = Vehicle::new(file, water.clone(), State::at_rest([5.0, 6.0, 1.5], [0.0; 3]));
    v.params = params.clone();
    let mut columns = vec!["time_s".to_string()];
    columns.extend(th.iter().map(|t| format!("rpm_{}", t.name)));
    columns.extend(CHANNELS[2..].iter().map(|(c, _)| c.to_string()));
    let sig = [0.01, 0.3, 0.3, 0.3, 0.01, 0.01, 0.01, 0.5, 0.5, 0.5];
    let mut rows = Vec::new();
    let dt = 0.05;
    let mut n = 0u64;
    for (k, ph) in phases.iter().enumerate() {
        let rpm: Vec<f64> = th.iter().zip(ph.iter().cycle()).map(|(t, e)| if t.inverted { -e } else { *e }).collect();
        for i in 0..100 {
            // A row has the state at its time and the speeds from then until the next row
            let t = (k * 100 + i) as f64 * dt;
            let s = read_state(&v.state);
            let mut row = vec![t];
            row.extend(&rpm);
            for (j, x) in s[2..].iter().enumerate() {
                row.push(x + noise * sig[j] * sim_acoustics::noise::gaussian(seed, n));
                n += 1;
            }
            rows.push(row);
            v.advance_to(t + dt, |_| rpm.clone());
        }
    }
    Table { columns, rows }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn talos() -> (VehicleFile, String) {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../vehicles/talos.toml");
        let text = std::fs::read_to_string(&path).unwrap();
        (VehicleFile::parse(&text).unwrap(), text)
    }

    const FIT: [&str; 8] = [
        "volume_m3",
        "linear_drag.heave",
        "quadratic_drag.heave",
        "added_mass.heave",
        "quadratic_drag.surge",
        "added_mass.surge",
        "quadratic_drag.yaw",
        "added_mass.yaw",
    ];

    /// Talos with the drag, added mass and buoyancy changed from the file's values.
    fn truth(file: &VehicleFile) -> Params {
        let mut p = file.params.clone();
        for (name, k) in FIT.iter().zip([1.0015, 1.4, 0.7, 1.3, 0.8, 1.5, 1.3, 0.75]) {
            *param_mut(&mut p, name).unwrap() *= k;
        }
        p
    }

    fn opts() -> VehicleOptions {
        VehicleOptions { fit: FIT.iter().map(|s| s.to_string()).collect(), ..Default::default() }
    }

    #[test]
    fn parameter_names_reach_every_value() {
        let (file, _) = talos();
        let mut p = file.params.clone();
        *param_mut(&mut p, "quadratic_drag.yaw").unwrap() = 9.0;
        *param_mut(&mut p, "cob_m.z").unwrap() = -0.05;
        *param_mut(&mut p, "inertia.pitch").unwrap() = 3.0;
        assert_eq!((p.quadratic_drag[5], p.cob_m[2], p.inertia_kg_m2[1]), (9.0, -0.05, 3.0));
        assert!(param_mut(&mut p, "drag.surge").is_none() && param_mut(&mut p, "added_mass.up").is_none());
        for name in DEFAULT_FIT {
            assert!(param_mut(&mut p, name).is_some(), "{name}");
        }
    }

    #[test]
    fn recovers_drag_added_mass_and_buoyancy_from_a_synthetic_pool_test() {
        let (file, text) = talos();
        let t = truth(&file);
        let data = synthetic_pool_test(&file, &t, &Water::default(), 0.0, 1);
        let log = read_log(&data, &file, 0.06).unwrap();
        let cal = calibrate(&file, &text, &log, &opts()).unwrap();
        let mut truth_p = t.clone();
        for p in &cal.fit.params {
            let want = *param_mut(&mut truth_p, &p.name).unwrap();
            let tol = if p.name == "volume_m3" { 1e-4 } else { 0.01 };
            assert!((p.value - want).abs() <= tol * want.abs(), "{}: {} vs {want}\n{}", p.name, p.value, cal.report);
        }
        // The fitted file loads and carries the values
        let fitted = VehicleFile::parse(&cal.toml).unwrap();
        assert!((fitted.params.added_mass[2] - t.added_mass[2]).abs() < 0.02 * t.added_mass[2]);
        assert_eq!(fitted.thruster.len(), 8);
        assert!(cal.channels.iter().all(|(_, before, after)| after <= before));
        assert_eq!(cal.plots.len(), 4);
    }

    #[test]
    fn noisy_measurements_and_commanded_rpm() {
        let (file, text) = talos();
        let t = truth(&file);
        let data = synthetic_pool_test(&file, &t, &Water::default(), 1.0, 3);
        let log = read_log(&data, &file, 0.06).unwrap();
        let cal = calibrate(&file, &text, &log, &opts()).unwrap();
        let mut truth_p = t.clone();
        for p in &cal.fit.params {
            let want = *param_mut(&mut truth_p, &p.name).unwrap();
            let tol = if p.name == "volume_m3" { 0.002 } else { 0.15 };
            assert!((p.value - want).abs() <= tol * want.abs(), "{}: {} vs {want}\n{}", p.name, p.value, cal.report);
        }
        // Commanded RPM goes through the lag; thrust columns through the inverted curve
        let mut cmd = data.clone();
        for c in cmd.columns.iter_mut() {
            if let Some(n) = c.strip_prefix("rpm_") {
                *c = format!("cmd_{n}");
            }
        }
        let l = read_log(&cmd, &file, 0.06).unwrap();
        assert!(l.rpm[1][5] > 0.0 && l.rpm[1][5] < l.rpm[50][5], "{:?}", &l.rpm[..3]);
        assert!(l.notes[0].contains("60 ms lag"));
        let th = file.thrusters();
        for (k, t) in th.iter().enumerate() {
            for rpm in [-2000.0, 0.0, 1500.0] {
                assert!((rpm_for_thrust(t, t.thrust(rpm)) - rpm).abs() < 1e-6, "{k}");
            }
        }
        assert!(read_log(&Table::parse("time_s,depth_m\n0,1\n").unwrap(), &file, 0.06).is_err());
        assert!(read_log(&Table::parse("time_s,rpm_0\n0,1\n").unwrap(), &file, 0.06).is_err());
    }
}
