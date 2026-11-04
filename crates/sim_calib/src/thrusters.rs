//! Thruster calibration from a test stand: the thrust curve a vehicle file uses (`thruster_model`) and the DShot
//! ESC's motor model (`[[dshot_esc]]` in a board file: throttle to RPM, the motor's lag).
//!
//! Predictions go through the emulator's own models, not a copy of their equations: the ESC through
//! [`sim_devices::dshot::Esc::command`] and [`Esc::rpm_at`], the thrust through [`sim_physics::Thruster::thrust`].
//! The ESC's fittable parameters are listed in [`ESC_PARAMS`] (name, how to read and set it on an `Esc`, its board
//! file key); a change to the ESC model's equations is picked up by the fit as it is, and a new parameter of the
//! model needs one entry there.
//!
//! The stand data is a CSV table (see `docs/calibration.md`): a command column (`dshot`, `throttle` or `pwm_us`),
//! and any of `time_s`, `rpm`, `thrust_n` (or `thrust_kgf`), `voltage_v`. With `time_s` the rows are a time
//! series and the ESC's lag can be fitted; without it every row is a steady point.

use std::fmt::Write as _;

use sim_core::time::Time;
use sim_devices::dshot::Esc;
use sim_physics::{ThrustCurve, Thruster, ThrusterSpec};

use crate::fit::{self, Fit, Options, Param};
use crate::table::{Series, Table, markdown, svg_plot};

/// A parameter of the ESC model that calibration can fit.
pub struct EscParam {
    pub name: &'static str,
    /// The key in a board file's `[[dshot_esc]]`.
    pub board_key: &'static str,
    pub get: fn(&Esc) -> f64,
    pub set: fn(&mut Esc, f64),
    pub lo: f64,
    pub hi: f64,
}

/// The ESC model's fittable parameters.
pub const ESC_PARAMS: &[EscParam] = &[
    EscParam {
        name: "max_rpm",
        board_key: "max_rpm",
        get: |e| e.max_rpm,
        set: |e, v| e.max_rpm = v,
        lo: 100.0,
        hi: 100_000.0,
    },
    EscParam {
        name: "tau_ms",
        board_key: "tau_ms",
        get: |e| e.tau.as_secs_f64() * 1000.0,
        set: |e, v| e.tau = Time::from_secs_f64(v / 1000.0),
        lo: 0.5,
        hi: 5000.0,
    },
];

fn esc_param(name: &str) -> Result<&'static EscParam, String> {
    ESC_PARAMS.iter().find(|p| p.name == name).ok_or_else(|| {
        format!(
            "no ESC parameter {name:?} (the model has {})",
            ESC_PARAMS.iter().map(|p| p.name).collect::<Vec<_>>().join(", ")
        )
    })
}

/// DShot 3D throttle for a signed fraction of full throttle, as the ESC model reads it: 48-1047 forward,
/// 1049-2047 reverse, 0 to stop.
pub fn dshot_for(fraction: f64) -> u16 {
    let f = fraction.clamp(-1.0, 1.0);
    if f > 0.0 {
        47 + ((f * 1000.0).round() as u16).max(1)
    } else if f < 0.0 {
        1048 + ((-f * 999.0).round() as u16).max(1)
    } else {
        0
    }
}

/// The signed fraction a DShot 3D throttle stands for.
pub fn fraction_of(dshot: u16) -> f64 {
    match dshot {
        48..=1047 => (dshot - 47) as f64 / 1000.0,
        1049..=2047 => -((dshot - 1048) as f64) / 999.0,
        _ => 0.0,
    }
}

/// One row of test-stand data.
#[derive(Clone, Debug, PartialEq)]
pub struct StandRow {
    pub time_s: Option<f64>,
    pub dshot: u16,
    /// Mechanical RPM (signed).
    pub rpm: Option<f64>,
    /// Thrust, N (negative in reverse).
    pub thrust_n: Option<f64>,
    pub voltage_v: Option<f64>,
}

/// Reads a stand table into rows (signs of all-positive RPM or thrust columns are taken from the command).
pub fn read_stand(t: &Table) -> Result<Vec<StandRow>, String> {
    let n = t.rows.len();
    let cmd: Vec<u16> = if let Some(c) = t.col("dshot") {
        c.iter().map(|v| v.round().clamp(0.0, 2047.0) as u16).collect()
    } else if let Some(c) = t.col("throttle") {
        c.iter().map(|&v| dshot_for(v)).collect()
    } else if let Some(c) = t.col("pwm_us") {
        // Servo PWM as Blue Robotics' ESCs take it: 1500 µs stop, +-400 µs full
        c.iter().map(|&v| dshot_for((v - 1500.0) / 400.0)).collect()
    } else {
        return Err("no command column: dshot, throttle (-1..1) or pwm_us".into());
    };
    let signed = |v: Option<Vec<f64>>| -> Option<Vec<f64>> {
        let v = v?;
        let all_positive = v.iter().all(|x| x.is_nan() || *x >= 0.0);
        Some(v.iter().zip(&cmd).map(|(&x, &c)| if all_positive && fraction_of(c) < 0.0 { -x } else { x }).collect())
    };
    let rpm = signed(t.col("rpm"));
    let thrust =
        signed(t.col("thrust_n").or_else(|| t.col("thrust_kgf").map(|c| c.iter().map(|k| k * 9.80665).collect())));
    if rpm.is_none() && thrust.is_none() {
        return Err("no measurement column: rpm and/or thrust_n (or thrust_kgf)".into());
    }
    let time = t.col("time_s");
    let volt = t.col("voltage_v");
    let opt = |c: &Option<Vec<f64>>, i: usize| c.as_ref().map(|c| c[i]).filter(|v| v.is_finite());
    Ok((0..n)
        .map(|i| StandRow {
            time_s: opt(&time, i),
            dshot: cmd[i],
            rpm: opt(&rpm, i),
            thrust_n: opt(&thrust, i),
            voltage_v: opt(&volt, i),
        })
        .collect())
}

/// A series of rows from one file: a time series if every row has a time, otherwise independent steady points.
#[derive(Clone, Debug)]
pub struct Run {
    pub name: String,
    pub rows: Vec<StandRow>,
}

impl Run {
    fn timed(&self) -> bool {
        !self.rows.is_empty() && self.rows.iter().all(|r| r.time_s.is_some())
    }
}

/// The ESC model with these parameter values (the others at the model's defaults or `base`'s).
fn esc_with(base: &Esc, names: &[&str], values: &[f64]) -> Esc {
    let mut e = base.clone();
    e.signal_loss_timeout = None;
    for (n, v) in names.iter().zip(values) {
        (esc_param(n).unwrap().set)(&mut e, *v);
    }
    e
}

/// The ESC model's RPM for each row: a time series replays the commands as they change and reads the motor at
/// each row's time; steady rows read the motor long after their command.
pub fn predict_rpm(esc: &Esc, run: &Run) -> Vec<f64> {
    let mut out = Vec::with_capacity(run.rows.len());
    if run.timed() {
        let mut e = esc.clone();
        let t0 = run.rows[0].time_s.unwrap();
        let mut last = None;
        for r in &run.rows {
            let t = Time::from_secs_f64((r.time_s.unwrap() - t0).max(0.0));
            if last != Some(r.dshot) {
                e.command(t, r.dshot);
                last = Some(r.dshot);
            }
            out.push(e.rpm_at(t));
        }
    } else {
        for r in &run.rows {
            let mut e = esc.clone();
            e.command(Time::ZERO, r.dshot);
            out.push(e.rpm_at(Time::from_secs(1000)));
        }
    }
    out
}

/// Rows at least `settle_s` after their command last changed (all rows of an untimed run): steady thrust.
fn steady(run: &Run, settle_s: f64) -> Vec<bool> {
    if !run.timed() {
        return vec![true; run.rows.len()];
    }
    let mut since = run.rows[0].time_s.unwrap();
    let mut prev = None;
    run.rows
        .iter()
        .map(|r| {
            let t = r.time_s.unwrap();
            if prev != Some(r.dshot) {
                since = t;
                prev = Some(r.dshot);
            }
            t - since >= settle_s
        })
        .collect()
}

fn thruster(curve: &ThrustCurve) -> Thruster {
    let spec = ThrusterSpec {
        name: "stand".into(),
        position_m: [0.0; 3],
        direction: [1.0, 0.0, 0.0],
        curve: Some(curve.clone()),
        inverted: false,
    };
    Thruster::new(&spec, None)
}

#[derive(Clone, Debug)]
pub struct StandOptions {
    /// ESC parameters to fit (empty: every one the data can tell, see [`calibrate`]).
    pub fit: Vec<String>,
    /// Starting ESC (a board file's `[[dshot_esc]]` values or the model's defaults).
    pub esc: Esc,
    /// Starting thrust curve; its `at_rpm` is kept.
    pub curve: ThrustCurve,
    /// Rows count as steady this long after a command change.
    pub settle_s: f64,
    /// The supply voltage the output is for (the largest group of rows by default).
    pub voltage_v: Option<f64>,
    pub method: fit::Method,
}

impl Default for StandOptions {
    fn default() -> Self {
        StandOptions {
            fit: Vec::new(),
            esc: Esc::new("stand", 0, None),
            curve: ThrustCurve { forward_n: 51.5, reverse_n: 40.2, at_rpm: 3800.0 },
            settle_s: 0.5,
            voltage_v: None,
            method: fit::Method::LevenbergMarquardt,
        }
    }
}

/// What a calibration found.
#[derive(Clone, Debug)]
pub struct Calibration {
    /// The fitted thrust curve, if the data had thrust.
    pub curve: Option<ThrustCurve>,
    pub curve_fit: Option<Fit>,
    /// The ESC fit for the chosen voltage, and for every voltage group: (voltage, fit).
    pub esc_fit: Option<Fit>,
    pub per_voltage: Vec<(Option<f64>, Fit)>,
    pub voltage_v: Option<f64>,
    pub notes: Vec<String>,
    /// Markdown report, the TOML to paste, and SVG plots (file name, contents).
    pub report: String,
    pub toml: String,
    pub plots: Vec<(String, String)>,
}

fn voltage_key(v: Option<f64>) -> Option<i64> {
    v.map(|v| (v * 10.0).round() as i64)
}

/// Fits the stand runs.
pub fn calibrate(runs: &[Run], opts: &StandOptions) -> Result<Calibration, String> {
    let mut notes = Vec::new();
    let has_rpm = runs.iter().any(|r| r.rows.iter().any(|r| r.rpm.is_some()));
    let has_thrust = runs.iter().any(|r| r.rows.iter().any(|r| r.thrust_n.is_some()));
    let timed = runs.iter().any(Run::timed);
    // Which ESC parameters: those asked for, or what the data tell (the lag needs a time series)
    let names: Vec<String> = if !opts.fit.is_empty() {
        for n in &opts.fit {
            esc_param(n)?;
        }
        opts.fit.clone()
    } else if has_rpm {
        ESC_PARAMS.iter().map(|p| p.name.to_string()).filter(|n| timed || n != "tau_ms").collect()
    } else {
        Vec::new()
    };
    if !timed && names.iter().any(|n| n == "tau_ms") {
        return Err("fitting tau_ms needs a time series (a time_s column)".into());
    }
    if !has_rpm && !names.is_empty() {
        return Err("fitting the ESC model needs measured RPM (an rpm column)".into());
    }
    let names_ref: Vec<&str> = names.iter().map(String::as_str).collect();
    // Voltage groups
    let mut groups: Vec<Option<i64>> = Vec::new();
    for r in runs.iter().flat_map(|r| &r.rows) {
        let k = voltage_key(r.voltage_v);
        if !groups.contains(&k) {
            groups.push(k);
        }
    }
    groups.sort();
    let count = |k: Option<i64>| runs.iter().flat_map(|r| &r.rows).filter(|r| voltage_key(r.voltage_v) == k).count();
    let chosen = match opts.voltage_v {
        Some(v) => {
            let k = voltage_key(Some(v));
            if !groups.contains(&k) {
                return Err(format!("no rows at {v} V (the data have {})", fmt_groups(&groups)));
            }
            k
        }
        None => groups.iter().copied().max_by_key(|&k| (count(k), std::cmp::Reverse(k))).unwrap_or(None),
    };
    let fopts = Options { method: opts.method, max_iter: 200, ..Default::default() };
    let mut per_voltage = Vec::new();
    let mut esc_fit = None;
    if !names.is_empty() {
        for &g in &groups {
            let sub: Vec<Run> = runs
                .iter()
                .map(|r| Run {
                    name: r.name.clone(),
                    rows: r.rows.iter().filter(|x| voltage_key(x.voltage_v) == g).cloned().collect(),
                })
                .filter(|r| r.rows.iter().any(|x| x.rpm.is_some()))
                .collect();
            if sub.is_empty() {
                continue;
            }
            let params: Vec<Param> = names_ref
                .iter()
                .map(|n| {
                    let p = esc_param(n).unwrap();
                    Param::positive(n, (p.get)(&opts.esc), 1e-3).bounded(p.lo, p.hi)
                })
                .collect();
            let scale =
                sub.iter().flat_map(|r| &r.rows).filter_map(|r| r.rpm).fold(0.0f64, |m, v| m.max(v.abs())).max(1.0);
            let resid = |v: &[f64]| -> Vec<f64> {
                let e = esc_with(&opts.esc, &names_ref, v);
                let mut out = Vec::new();
                for run in &sub {
                    let pred = predict_rpm(&e, run);
                    for (r, p) in run.rows.iter().zip(pred) {
                        if let Some(m) = r.rpm {
                            out.push((p - m) / (0.01 * scale));
                        }
                    }
                }
                out
            };
            let f = fit::fit(&params, resid, &fopts);
            if g == chosen {
                esc_fit = Some(f.clone());
            }
            per_voltage.push((g.map(|k| k as f64 / 10.0), f));
        }
    }
    let esc = match &esc_fit {
        Some(f) => esc_with(&opts.esc, &names_ref, &f.params.iter().map(|p| p.value).collect::<Vec<_>>()),
        None => esc_with(&opts.esc, &[], &[]),
    };
    // The thrust curve, from steady rows: at measured RPM, or at the ESC model's RPM for the command
    let mut curve_fit = None;
    let mut curve = None;
    let mut thrust_points: Vec<(f64, f64)> = Vec::new();
    if has_thrust {
        for run in runs {
            let st = steady(run, opts.settle_s);
            let model_rpm = predict_rpm(&esc, run);
            for ((r, s), mr) in run.rows.iter().zip(st).zip(model_rpm) {
                if let (true, Some(t)) = (s, r.thrust_n) {
                    thrust_points.push((r.rpm.unwrap_or(mr), t));
                }
            }
        }
        if !has_rpm {
            notes.push(format!(
                "no measured RPM: the thrust curve is fitted at the ESC model's RPM for each command (max_rpm \
                 {:.0}), so it is only as good as max_rpm; measure RPM (ESC telemetry or an optical tachometer) to \
                 separate the two",
                esc.max_rpm
            ));
        }
        let fwd = thrust_points.iter().any(|p| p.0 > 0.0);
        let rev = thrust_points.iter().any(|p| p.0 < 0.0);
        if !fwd || !rev {
            notes.push(format!(
                "no {} thrust points: {} thrust is left at its starting value",
                if fwd { "reverse" } else { "forward" },
                if fwd { "reverse" } else { "forward" }
            ));
        }
        let mut params = Vec::new();
        if fwd {
            params.push(Param::positive("forward_n", opts.curve.forward_n, 1e-3));
        }
        if rev {
            params.push(Param::positive("reverse_n", opts.curve.reverse_n, 1e-3));
        }
        let pts = &thrust_points;
        let base = opts.curve.clone();
        let make = move |v: &[f64]| -> ThrustCurve {
            let mut c = base.clone();
            let mut k = 0;
            if fwd {
                c.forward_n = v[k];
                k += 1;
            }
            if rev {
                c.reverse_n = v[k];
            }
            c
        };
        let resid = |v: &[f64]| -> Vec<f64> {
            let t = thruster(&make(v));
            pts.iter().map(|&(rpm, m)| (t.thrust(rpm) - m) / (0.02 * m.abs() + 0.2)).collect()
        };
        let f = fit::fit(&params, resid, &fopts);
        curve = Some(make(&f.params.iter().map(|p| p.value).collect::<Vec<_>>()));
        curve_fit = Some(f);
    }
    let mut cal = Calibration {
        curve,
        curve_fit,
        esc_fit,
        per_voltage,
        voltage_v: chosen.map(|k| k as f64 / 10.0),
        notes,
        report: String::new(),
        toml: String::new(),
        plots: Vec::new(),
    };
    cal.toml = render_toml(&cal);
    cal.plots = plots(runs, &cal, &esc, &thrust_points, opts);
    cal.report = render_report(runs, &cal, &esc, opts);
    Ok(cal)
}

fn fmt_groups(g: &[Option<i64>]) -> String {
    g.iter()
        .map(|k| k.map_or("no voltage".to_string(), |k| format!("{:.1} V", k as f64 / 10.0)))
        .collect::<Vec<_>>()
        .join(", ")
}

fn render_toml(cal: &Calibration) -> String {
    let mut o =
        String::from("# Fitted by `firmware_simulator calibrate thrusters`; see report.md for the residuals.\n");
    if let Some(c) = &cal.curve {
        let _ = writeln!(
            o,
            "\n# In a vehicle file (vehicles/*.toml), for every thruster of this kind:\nthruster_model = {{ forward_n = \
             {:.3}, reverse_n = {:.3}, at_rpm = {:.1} }}",
            c.forward_n, c.reverse_n, c.at_rpm
        );
    }
    if let Some(f) = &cal.esc_fit {
        let v = cal.voltage_v.map_or(String::new(), |v| format!(" at {v:.1} V"));
        let _ = writeln!(o, "\n# In each [[dshot_esc]] of the ESC's board file (boards/overlays/...){v}:");
        for p in &f.params {
            let key = esc_param(&p.name).unwrap().board_key;
            let _ = writeln!(o, "{key} = {:.3}", p.value);
        }
    }
    o
}

fn plots(
    runs: &[Run],
    cal: &Calibration,
    esc: &Esc,
    thrust_points: &[(f64, f64)],
    opts: &StandOptions,
) -> Vec<(String, String)> {
    let mut out = Vec::new();
    if let Some(c) = &cal.curve {
        let t = thruster(c);
        let t0 = thruster(&opts.curve);
        let max = thrust_points.iter().fold(c.at_rpm, |m, p| m.max(p.0.abs()));
        let line =
            |t: &Thruster| (-50..=50).map(|i| i as f64 / 50.0 * max).map(|r| (r, t.thrust(r))).collect::<Vec<_>>();
        out.push((
            "thrust.svg".into(),
            svg_plot(
                "Thrust against motor speed",
                "RPM",
                "thrust (N)",
                &[
                    Series::dots("measured", thrust_points.to_vec()),
                    Series::line("fitted", line(&t)),
                    Series::line("before", line(&t0)),
                ],
                false,
            ),
        ));
    }
    if cal.esc_fit.is_some() {
        let mut meas = Vec::new();
        let mut model = Vec::new();
        for run in runs {
            let pred = predict_rpm(esc, run);
            for (r, p) in run.rows.iter().zip(pred) {
                if let Some(m) = r.rpm {
                    meas.push((fraction_of(r.dshot), m));
                    model.push((fraction_of(r.dshot), p));
                }
            }
        }
        model.sort_by(|a, b| a.0.total_cmp(&b.0));
        out.push((
            "rpm.svg".into(),
            svg_plot(
                "Motor speed against throttle",
                "throttle (fraction, DShot 3D)",
                "RPM",
                &[Series::dots("measured", meas), Series::dots("ESC model, fitted", model)],
                false,
            ),
        ));
        if let Some(run) = runs.iter().find(|r| r.timed() && r.rows.iter().any(|x| x.rpm.is_some())) {
            let pred = predict_rpm(esc, run);
            let t0 = run.rows[0].time_s.unwrap();
            let meas: Vec<(f64, f64)> =
                run.rows.iter().filter_map(|r| Some((r.time_s.unwrap() - t0, r.rpm?))).collect();
            let model: Vec<(f64, f64)> = run.rows.iter().zip(pred).map(|(r, p)| (r.time_s.unwrap() - t0, p)).collect();
            out.push((
                "rpm_time.svg".into(),
                svg_plot(
                    &format!("Motor speed over time ({})", run.name),
                    "time (s)",
                    "RPM",
                    &[Series::dots("measured", meas), Series::line("ESC model, fitted", model)],
                    false,
                ),
            ));
        }
    }
    out
}

fn fit_table(f: &Fit, start: &dyn Fn(&str) -> f64) -> String {
    let rows: Vec<Vec<String>> = f
        .params
        .iter()
        .zip(&f.std_err)
        .map(|(p, se)| {
            vec![
                format!("`{}`", p.name),
                format!("{:.4}", start(&p.name)),
                format!("{:.4}", p.value),
                if se.is_finite() { format!("{se:.3}") } else { "-".into() },
            ]
        })
        .collect();
    markdown(&["parameter", "before", "fitted", "± (1 sigma)"], &rows)
}

fn render_report(runs: &[Run], cal: &Calibration, esc: &Esc, opts: &StandOptions) -> String {
    let mut o = String::from("# Thruster calibration\n\n");
    let rows: usize = runs.iter().map(|r| r.rows.len()).sum();
    let _ = writeln!(
        o,
        "{} file(s), {rows} row(s): {}.\n",
        runs.len(),
        runs.iter()
            .map(|r| format!(
                "{} ({}, {} rows)",
                r.name,
                if r.timed() { "time series" } else { "steady points" },
                r.rows.len()
            ))
            .collect::<Vec<_>>()
            .join(", ")
    );
    for n in &cal.notes {
        let _ = writeln!(o, "* Note: {n}");
    }
    if !cal.notes.is_empty() {
        o.push('\n');
    }
    if let (Some(c), Some(f)) = (&cal.curve, &cal.curve_fit) {
        let _ = writeln!(
            o,
            "## Thrust curve\n\n`thrust = k rpm |rpm|`, with `k` from the thrust at {:.0} RPM in each direction.\n",
            c.at_rpm
        );
        o += &fit_table(f, &|n| if n == "forward_n" { opts.curve.forward_n } else { opts.curve.reverse_n });
        let _ = writeln!(
            o,
            "\nResiduals (in units of 2 % of the thrust + 0.2 N): RMS {:.2} at the start, {:.2} fitted, over {} \
             steady point(s).\n",
            (2.0 * f.initial_cost / f.residuals.len().max(1) as f64).sqrt(),
            f.rms(),
            f.residuals.len()
        );
    }
    if let Some(f) = &cal.esc_fit {
        let v = cal.voltage_v.map_or(String::new(), |v| format!(" at {v:.1} V"));
        let _ = writeln!(o, "## ESC and motor model{v}\n");
        o += &fit_table(f, &|n| (esc_param(n).unwrap().get)(&opts.esc));
        let _ = writeln!(
            o,
            "\nRPM residuals (in units of 1 % of the largest RPM): RMS {:.2} at the start, {:.2} fitted. By direction:\n",
            (2.0 * f.initial_cost / f.residuals.len().max(1) as f64).sqrt(),
            f.rms()
        );
        let mut by_dir = [(0.0, 0usize, "forward"), (0.0, 0, "reverse")];
        for run in runs {
            for (r, p) in run.rows.iter().zip(predict_rpm(esc, run)) {
                if let Some(m) = r.rpm {
                    let d = if fraction_of(r.dshot) < 0.0 { 1 } else { 0 };
                    by_dir[d].0 += (p - m) * (p - m);
                    by_dir[d].1 += 1;
                }
            }
        }
        let rows: Vec<Vec<String>> = by_dir
            .iter()
            .filter(|d| d.1 > 0)
            .map(|d| vec![d.2.to_string(), d.1.to_string(), format!("{:.1}", (d.0 / d.1 as f64).sqrt())])
            .collect();
        o += &markdown(&["direction", "rows", "RMS error (RPM)"], &rows);
        o += "\nA much larger error in one direction means the model cannot follow the motor there (for example a \
              different top speed in reverse): the model needs another parameter, not a better fit.\n\n";
        if cal.per_voltage.len() > 1 {
            let _ = writeln!(o, "By supply voltage:\n");
            let names: Vec<&str> = f.params.iter().map(|p| p.name.as_str()).collect();
            let mut header = vec!["voltage"];
            header.extend(&names);
            let rows: Vec<Vec<String>> = cal
                .per_voltage
                .iter()
                .map(|(v, f)| {
                    let mut r = vec![v.map_or("-".into(), |v| format!("{v:.1} V"))];
                    r.extend(f.params.iter().map(|p| format!("{:.2}", p.value)));
                    r
                })
                .collect();
            o += &markdown(&header, &rows);
            o += "\nThe ESC model has no supply voltage: the output is for the voltage above. A motor's top speed \
                  grows about in proportion to its voltage (its Kv).\n\n";
        }
        for (a, b, c) in f.degenerate_pairs(0.95) {
            let _ = writeln!(o, "* `{a}` and `{b}` are {c:+.2} correlated: the data cannot tell them apart.");
        }
    }
    o += "\n## To use it\n\nPaste `thrusters.toml` into the vehicle file and the ESC board's overlay (then rerun \
          `python3 scripts/gen_boards.py`). Plots: `thrust.svg`, `rpm.svg`, `rpm_time.svg`.\n";
    o
}

/// Test-stand data from the emulator's own models with known parameters: a staircase of throttle steps (forward
/// and reverse, each held 1.5 s, sampled at 50 Hz) through the ESC model and the thrust curve, with Gaussian noise
/// of `rpm_noise` RPM and `thrust_noise` N (deterministic from `seed`), at `voltage_v`.
pub fn synthetic_stand(
    esc: &Esc,
    curve: &ThrustCurve,
    rpm_noise: f64,
    thrust_noise: f64,
    voltage_v: f64,
    seed: u64,
) -> Table {
    let levels = [0.1, 0.25, 0.4, 0.6, 0.8, 1.0, 0.0, -0.1, -0.3, -0.5, -0.7, -1.0, 0.0, 0.5];
    let mut e = esc.clone();
    e.signal_loss_timeout = None;
    let t = thruster(curve);
    let mut rows = Vec::new();
    let mut n = 0u64;
    for (k, &lvl) in levels.iter().enumerate() {
        let d = dshot_for(lvl);
        let t_start = k as f64 * 1.5;
        e.command(Time::from_secs_f64(t_start), d);
        for i in 0..75 {
            let time = t_start + i as f64 * 0.02;
            let rpm = e.rpm_at(Time::from_secs_f64(time));
            let g1 = sim_acoustics::noise::gaussian(seed, n);
            let g2 = sim_acoustics::noise::gaussian(seed ^ 0x7417, n);
            n += 1;
            rows.push(vec![time, d as f64, rpm + rpm_noise * g1, t.thrust(rpm) + thrust_noise * g2, voltage_v]);
        }
    }
    Table { columns: ["time_s", "dshot", "rpm", "thrust_n", "voltage_v"].map(String::from).to_vec(), rows }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn truth() -> (Esc, ThrustCurve) {
        let mut e = Esc::new("truth", 0, None);
        e.max_rpm = 3550.0;
        e.tau = Time::from_ms(85);
        (e, ThrustCurve { forward_n: 44.0, reverse_n: 33.5, at_rpm: 3800.0 })
    }

    #[test]
    fn dshot_fractions_round_trip() {
        assert_eq!(dshot_for(1.0), 1047);
        assert_eq!(dshot_for(-1.0), 2047);
        assert_eq!(dshot_for(0.0), 0);
        assert_eq!(dshot_for(0.0001), 48);
        for d in [48u16, 300, 1047, 1049, 1500, 2047] {
            assert_eq!(dshot_for(fraction_of(d)), d);
        }
    }

    #[test]
    fn recovers_the_esc_and_thrust_curve_from_clean_synthetic_data() {
        let (esc, curve) = truth();
        let data = synthetic_stand(&esc, &curve, 0.0, 0.0, 16.0, 1);
        let runs = [Run { name: "stand".into(), rows: read_stand(&data).unwrap() }];
        let cal = calibrate(&runs, &StandOptions::default()).unwrap();
        let f = cal.esc_fit.as_ref().unwrap();
        assert!((f.value("max_rpm").unwrap() - 3550.0).abs() < 0.5, "{:?}", f.params);
        assert!((f.value("tau_ms").unwrap() - 85.0).abs() < 0.2, "{:?}", f.params);
        let c = cal.curve.unwrap();
        assert!((c.forward_n - 44.0).abs() < 0.01 && (c.reverse_n - 33.5).abs() < 0.01, "{c:?}");
        assert_eq!(c.at_rpm, 3800.0);
        assert!(cal.toml.contains("thruster_model = { forward_n = 44.000") && cal.toml.contains("tau_ms = 85.0"));
        assert!(cal.report.contains("## ESC and motor model at 16.0 V"));
        assert_eq!(cal.plots.len(), 3);
    }

    #[test]
    fn noisy_data_still_lands_close() {
        let (esc, curve) = truth();
        let data = synthetic_stand(&esc, &curve, 40.0, 0.8, 16.0, 7);
        let runs = [Run { name: "stand".into(), rows: read_stand(&data).unwrap() }];
        let cal = calibrate(&runs, &StandOptions::default()).unwrap();
        let f = cal.esc_fit.as_ref().unwrap();
        assert!((f.value("max_rpm").unwrap() - 3550.0).abs() < 15.0, "{:?}", f.params);
        assert!((f.value("tau_ms").unwrap() - 85.0).abs() < 8.0, "{:?}", f.params);
        let c = cal.curve.unwrap();
        assert!((c.forward_n - 44.0).abs() < 0.6 && (c.reverse_n - 33.5).abs() < 0.6, "{c:?}");
        // The uncertainty covers the error
        let se = &cal.curve_fit.unwrap().std_err;
        assert!(se.iter().all(|s| s.is_finite() && *s > 0.0));
    }

    #[test]
    fn voltage_groups_steady_tables_and_missing_rpm() {
        let (mut esc, curve) = truth();
        let a = synthetic_stand(&esc, &curve, 0.0, 0.0, 16.0, 1);
        esc.max_rpm = 3100.0;
        let b = synthetic_stand(&esc, &curve, 0.0, 0.0, 14.0, 1);
        let runs = [
            Run { name: "16V".into(), rows: read_stand(&a).unwrap() },
            Run { name: "14V".into(), rows: read_stand(&b).unwrap() },
        ];
        let cal = calibrate(&runs, &StandOptions { voltage_v: Some(14.0), ..Default::default() }).unwrap();
        assert_eq!(cal.per_voltage.len(), 2);
        assert!((cal.esc_fit.unwrap().value("max_rpm").unwrap() - 3100.0).abs() < 0.5);
        assert!(cal.report.contains("By supply voltage"));
        assert!(calibrate(&runs, &StandOptions { voltage_v: Some(12.0), ..Default::default() }).is_err());
        // A table of steady points with thrust only (Blue Robotics' published data is like that): the curve at the
        // model's RPM, and a note
        let t = Table::parse("throttle,thrust_kgf\n0.5,1.0\n1.0,4.0\n-1.0,3.0\n").unwrap();
        let runs = [Run { name: "datasheet".into(), rows: read_stand(&t).unwrap() }];
        let opts = StandOptions {
            esc: {
                let mut e = Esc::new("x", 0, None);
                e.max_rpm = 4000.0;
                e
            },
            ..Default::default()
        };
        let cal = calibrate(&runs, &opts).unwrap();
        let c = cal.curve.unwrap();
        // Thrust at 4000 RPM forward is 4 kgf, so at 3800 RPM 4 kgf (3800/4000)^2
        assert!((c.forward_n - 4.0 * 9.80665 * 0.9025).abs() < 0.05, "{c:?}");
        assert!(cal.esc_fit.is_none() && cal.notes.iter().any(|n| n.contains("no measured RPM")));
        assert!(calibrate(&runs, &StandOptions { fit: vec!["tau_ms".into()], ..Default::default() }).is_err());
        assert!(read_stand(&Table::parse("a,b\n1,2\n").unwrap()).is_err());
    }
}
