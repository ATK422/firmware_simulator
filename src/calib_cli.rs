//! The calibration subcommands (`calibrate acoustics|thrusters|vehicle`) and `replay-pool-test`.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use clap::Subcommand;

use sim_calib::fit::{Method, Options};
use sim_calib::table::Table;
use sim_calib::{acoustics, pool, thrusters, vehicle};
use sim_core::time::Time;

#[derive(Subcommand)]
pub enum CalibrateCommand {
    /// Fits a board's receive chain (gain, sensitivity, filter corners, ADC offset and clipping) and the environment
    /// (noise, wall/bottom/surface reflection, source level) to hydrophone recordings, from a manifest (TOML)
    /// naming the board file, the environment, the parameters to fit and each recording's geometry. Writes the
    /// fitted `[transducer]` overlay, environment file, a report with residuals, plots and the simulated captures.
    Acoustics(AcousticsArgs),
    /// Fits the thrust curve and the DShot ESC's motor model (top speed, lag) to test-stand CSV files (a command
    /// column `dshot`, `throttle` or `pwm_us`; any of `time_s`, `rpm`, `thrust_n`/`thrust_kgf`, `voltage_v`).
    Thrusters(ThrustersArgs),
    /// Fits a vehicle file's drag, added mass, buoyancy (any of its parameters) to pool-test CSV logs (thruster
    /// RPM against depth, attitude, DVL velocities, gyro rates) by simulating the recorded thruster speeds.
    Vehicle(VehicleArgs),
}

fn parse_method(s: &str) -> Result<Method, String> {
    s.parse()
}

fn parse_curve(s: &str) -> Result<thrusters::CurveForm, String> {
    s.parse()
}

#[derive(clap::Args)]
pub struct AcousticsArgs {
    /// The manifest (see docs/calibration.md).
    manifest: PathBuf,
    /// Directory for the outputs.
    #[arg(short, long, default_value = "calibration/acoustics")]
    out: PathBuf,
    /// Parameters to fit, replacing the manifest's `fit` (comma separated), e.g. `gain_db,lowpass_hz,noise_db`.
    #[arg(long, value_delimiter = ',')]
    fit: Vec<String>,
    /// `lm` (Levenberg-Marquardt) or `nelder-mead`.
    #[arg(long, value_parser = parse_method)]
    method: Option<Method>,
    /// Most iterations.
    #[arg(long, default_value_t = 60)]
    max_iter: usize,
}

#[derive(clap::Args)]
pub struct ThrustersArgs {
    /// Test-stand CSV files (each one run: a time series with `time_s`, or a table of steady points).
    #[arg(required = true)]
    csv: Vec<PathBuf>,
    /// Vehicle file whose `thruster_model` is the starting curve (its `at_rpm` is kept).
    #[arg(long)]
    vehicle: Option<PathBuf>,
    /// Board file whose first `[[dshot_esc]]` is the starting ESC model.
    #[arg(long)]
    board: Option<PathBuf>,
    /// The speed the fitted curve gives its thrusts at (instead of the vehicle file's or 3800 RPM).
    #[arg(long)]
    at_rpm: Option<f64>,
    /// The form of ESC speed curve to fit: `linear` (`max_rpm`) or `fit` (`throttle_fit`, a quadratic per
    /// direction); by default the board file's (or linear).
    #[arg(long, value_parser = parse_curve)]
    curve: Option<thrusters::CurveForm>,
    /// ESC parameters to fit (comma separated; by default all the data can tell: the curve's (`max_rpm`, or
    /// `forward_c0`..`reverse_c2`), and `tau_ms` from a time series).
    #[arg(long, value_delimiter = ',')]
    fit: Vec<String>,
    /// The supply voltage to write the ESC model for (by default the voltage with the most rows).
    #[arg(long)]
    voltage: Option<f64>,
    /// Rows count as steady thrust this long after their command changed.
    #[arg(long, default_value = "500ms", value_parser = super::parse_time)]
    settle: Time,
    /// `lm` or `nelder-mead`.
    #[arg(long, value_parser = parse_method)]
    method: Option<Method>,
    #[arg(short, long, default_value = "calibration/thrusters")]
    out: PathBuf,
}

#[derive(clap::Args)]
pub struct VehicleArgs {
    /// Pool-test CSV logs (scripts/bag_to_calib.py writes them from a ROS 2 bag; `robot --vehicle-csv` from a run).
    #[arg(required = true)]
    csv: Vec<PathBuf>,
    /// The vehicle file to start from.
    #[arg(long)]
    vehicle: PathBuf,
    /// Parameters to fit (comma separated), e.g. `volume_m3,quadratic_drag.heave,added_mass.heave`; by default
    /// buoyancy and drag and added mass in surge, heave and yaw.
    #[arg(long, value_delimiter = ',')]
    fit: Vec<String>,
    /// Length of the segments the log is simulated in, each from the measured state.
    #[arg(long, default_value = "4s", value_parser = super::parse_time)]
    segment: Time,
    /// The ESC lag commanded RPMs (`cmd_` columns) go through, ms.
    #[arg(long, default_value_t = 60.0)]
    esc_tau_ms: f64,
    /// Water density, kg/m³ (997 fresh water, 1025 sea water).
    #[arg(long, default_value_t = 997.0)]
    density: f64,
    /// `lm` or `nelder-mead`.
    #[arg(long, value_parser = parse_method)]
    method: Option<Method>,
    #[arg(long, default_value_t = 60)]
    max_iter: usize,
    #[arg(short, long, default_value = "calibration/vehicle")]
    out: PathBuf,
}

fn write_outputs(out: &Path, files: &[(String, Vec<u8>)]) -> Result<()> {
    std::fs::create_dir_all(out).with_context(|| format!("creating {}", out.display()))?;
    for (name, bytes) in files {
        let p = out.join(name);
        std::fs::write(&p, bytes).with_context(|| format!("writing {}", p.display()))?;
    }
    eprintln!(
        "wrote {} file(s) to {}: {}",
        files.len(),
        out.display(),
        files.iter().map(|f| f.0.as_str()).collect::<Vec<_>>().join(", ")
    );
    Ok(())
}

fn print_fit(f: &sim_calib::fit::Fit) {
    for (p, se) in f.params.iter().zip(&f.std_err) {
        let se = if se.is_finite() { format!(" ± {se:.3}") } else { String::new() };
        println!("  {:<24} {:.4}{se}", p.name, p.value);
    }
}

pub fn calibrate(cmd: CalibrateCommand) -> Result<()> {
    match cmd {
        CalibrateCommand::Acoustics(a) => {
            let mut setup = acoustics::Setup::load(&a.manifest).map_err(anyhow::Error::msg)?;
            if !a.fit.is_empty() {
                setup.manifest.fit = a.fit.clone();
                // Re-check against the new list
                setup = acoustics::Setup::new(
                    setup.manifest,
                    setup.transducer,
                    setup.environment,
                    setup.env_dir,
                    setup.wavs,
                    setup.names,
                )
                .map_err(anyhow::Error::msg)?;
            }
            let method = match (a.method, setup.manifest.method.as_deref()) {
                (Some(m), _) => m,
                (None, Some(m)) => m.parse().map_err(anyhow::Error::msg)?,
                (None, None) => Method::LevenbergMarquardt,
            };
            let cal = acoustics::calibrate(&setup, &Options { method, max_iter: a.max_iter, ..Default::default() })
                .map_err(anyhow::Error::msg)?;
            println!(
                "acoustics: {} recording(s), residual RMS {:.2} -> {:.2}",
                cal.recordings.len(),
                (2.0 * cal.fit.initial_cost / cal.fit.residuals.len().max(1) as f64).sqrt(),
                cal.fit.rms()
            );
            print_fit(&cal.fit);
            let mut files = vec![
                ("report.md".to_string(), cal.report.into_bytes()),
                ("overlay.toml".into(), cal.overlay.into_bytes()),
                ("environment.toml".into(), cal.environment.into_bytes()),
            ];
            files.extend(cal.plots.into_iter().map(|(n, s)| (n, s.into_bytes())));
            files.extend(cal.wavs.into_iter().map(|(n, w)| (n, w.to_bytes())));
            write_outputs(&a.out, &files)
        }
        CalibrateCommand::Thrusters(a) => {
            let mut opts = thrusters::StandOptions::default();
            if let Some(v) = &a.vehicle {
                let f = sim_physics::VehicleFile::load(v).map_err(anyhow::Error::msg)?;
                match f.thruster_model.or_else(|| f.thruster.iter().find_map(|t| t.curve.clone())) {
                    Some(c) => opts.curve = c,
                    None => bail!("{} has no thrust curve", v.display()),
                }
            }
            if let Some(r) = a.at_rpm {
                // The same thrust law, given at another speed
                let k = (r / opts.curve.at_rpm).powi(2);
                opts.curve = sim_physics::ThrustCurve {
                    forward_n: opts.curve.forward_n * k,
                    reverse_n: opts.curve.reverse_n * k,
                    at_rpm: r,
                };
            }
            if let Some(b) = &a.board {
                let board = sim_host::board::Board::load(b).map_err(anyhow::Error::msg)?;
                let e = board.dshot_esc.first().with_context(|| format!("{} has no [[dshot_esc]]", b.display()))?;
                if let Some(f) = &e.throttle_fit {
                    opts.esc.curve = sim_devices::dshot::SpeedCurve::Fit { forward: f.forward, reverse: f.reverse };
                } else if let Some(max_rpm) = e.max_rpm {
                    opts.esc.curve = sim_devices::dshot::SpeedCurve::Linear { max_rpm };
                }
                if let Some(ms) = e.tau_ms {
                    opts.esc.tau = Time::from_secs_f64(ms / 1000.0);
                }
            }
            if let Some(form) = a.curve {
                opts.esc = thrusters::in_form(&opts.esc, form);
            }
            opts.fit = a.fit.clone();
            opts.voltage_v = a.voltage;
            opts.settle_s = a.settle.as_secs_f64();
            opts.method = a.method.unwrap_or_default();
            let mut runs = Vec::new();
            for p in &a.csv {
                let t = Table::load(p).map_err(anyhow::Error::msg)?;
                let rows = thrusters::read_stand(&t).map_err(|e| anyhow::anyhow!("{}: {e}", p.display()))?;
                runs.push(thrusters::Run { name: p.display().to_string(), rows });
            }
            let cal = thrusters::calibrate(&runs, &opts).map_err(anyhow::Error::msg)?;
            if let Some(c) = &cal.curve {
                println!(
                    "thrust curve: {:.3} N forward, {:.3} N reverse at {:.0} RPM",
                    c.forward_n, c.reverse_n, c.at_rpm
                );
            }
            if let Some(f) = &cal.esc_fit {
                println!("ESC model{}:", cal.voltage_v.map_or(String::new(), |v| format!(" at {v:.1} V")));
                print_fit(f);
            }
            for n in &cal.notes {
                println!("note: {n}");
            }
            let mut files = vec![
                ("report.md".to_string(), cal.report.into_bytes()),
                ("thrusters.toml".into(), cal.toml.into_bytes()),
            ];
            files.extend(cal.plots.into_iter().map(|(n, s)| (n, s.into_bytes())));
            write_outputs(&a.out, &files)
        }
        CalibrateCommand::Vehicle(a) => {
            let text =
                std::fs::read_to_string(&a.vehicle).with_context(|| format!("reading {}", a.vehicle.display()))?;
            let file =
                sim_physics::VehicleFile::parse(&text).map_err(|e| anyhow::anyhow!("{}: {e}", a.vehicle.display()))?;
            let seg = a.segment.as_secs_f64();
            // Several logs: one after the other, a segment apart, so no segment spans two
            let mut tables = Vec::new();
            let mut shift = 0.0;
            for p in &a.csv {
                let mut t = Table::load(p).map_err(anyhow::Error::msg)?;
                let ti = t.index("time_s").with_context(|| format!("{}: no time_s column", p.display()))?;
                let first = t.rows.first().map_or(0.0, |r| r[ti]);
                for r in &mut t.rows {
                    r[ti] = r[ti] - first + shift;
                }
                shift = t.rows.last().map_or(shift, |r| r[ti]) + seg + 1.0;
                tables.push(t);
            }
            let table = Table::concat(tables).map_err(anyhow::Error::msg)?;
            let log = vehicle::read_log(&table, &file, a.esc_tau_ms / 1000.0).map_err(anyhow::Error::msg)?;
            let mut opts = vehicle::VehicleOptions {
                segment_s: seg,
                method: a.method.unwrap_or_default(),
                max_iter: a.max_iter,
                ..Default::default()
            };
            opts.water.density_kg_m3 = a.density;
            if !a.fit.is_empty() {
                opts.fit = a.fit.clone();
            }
            let cal = vehicle::calibrate(&file, &text, &log, &opts).map_err(anyhow::Error::msg)?;
            println!("vehicle {}: {} row(s)", file.name, log.time.len());
            print_fit(&cal.fit);
            for (c, b, after) in &cal.channels {
                println!("  {c:<10} RMS error {b:.4} -> {after:.4}");
            }
            let mut files = vec![
                ("report.md".to_string(), cal.report.into_bytes()),
                ("vehicle.toml".into(), cal.toml.into_bytes()),
            ];
            files.extend(cal.plots.into_iter().map(|(n, s)| (n, s.into_bytes())));
            write_outputs(&a.out, &files)
        }
    }
}

#[derive(clap::Args)]
pub struct ReplayArgs {
    /// candump logs from the robot computer (`candump -l can0`, candump's text output too); several are merged.
    #[arg(required = true)]
    logs: Vec<PathBuf>,
    /// The robot file whose boards stand in for the real ones.
    #[arg(long, default_value = "robots/talos.toml")]
    robot: PathBuf,
    /// Which bus a log interface is: `IFACE=BUS` (repeatable). By default an interface named like a bus goes to
    /// it, and can0, can1, ... to the robot file's first, second, ... bus.
    #[arg(long = "iface", value_name = "IFACE=BUS")]
    ifaces: Vec<String>,
    /// Emulator time at which the log's first frame is replayed, instead of lining the log up on the boards'
    /// FirmwareStatus uptimes.
    #[arg(long, value_parser = super::parse_time)]
    log_start: Option<Time>,
    /// The boards' clock drift against the log's clock in ppm, instead of fitting it from the uptimes.
    #[arg(long, allow_hyphen_values = true)]
    drift_ppm: Option<f64>,
    /// Leave out what boards that are not in the robot file sent (by default it is replayed as recorded).
    #[arg(long)]
    no_other_boards: bool,
    /// How far apart a recorded and an emulated message may be and still pair up.
    #[arg(long, default_value = "20ms", value_parser = super::parse_time)]
    tolerance: Time,
    /// Compare only from this long after the log's first frame.
    #[arg(long, default_value = "0s", value_parser = super::parse_time)]
    compare_after: Time,
    /// Don't compare the payloads of streams whose name contains this (repeatable).
    #[arg(long, value_name = "STREAM")]
    ignore_payload: Vec<String>,
    /// Only compare streams whose name contains this (repeatable), e.g. `client 2`.
    #[arg(long, value_name = "STREAM")]
    only: Vec<String>,
    /// Stop the emulator at this time at the latest.
    #[arg(short, long, value_parser = super::parse_time)]
    time: Option<Time>,
    /// Threads for the boards (as `robot --threads`).
    #[arg(long, default_value_t = 0)]
    threads: usize,
    /// A recorded pose CSV (`time_s`, `north_m`, `east_m`, `depth_m`, `roll_deg`, `pitch_deg`, `yaw_deg`;
    /// scripts/bag_to_calib.py writes one): the vehicle follows it, so its depth sensors see the recorded depth.
    #[arg(long)]
    pose: Option<PathBuf>,
    /// With `--pose`: let the emulator's physics move the vehicle and compare it with the pose instead.
    #[arg(long, requires = "pose")]
    pose_compare: bool,
    /// The vehicle the pose is for (the robot file's first by default).
    #[arg(long)]
    vehicle: Option<String>,
    /// Write the report (Markdown) to this file.
    #[arg(long)]
    report: Option<PathBuf>,
    /// Write the emulated boards' frames as a candump log (bus names as interfaces, emulator time).
    #[arg(long)]
    candump_log: Option<PathBuf>,
    /// Examples shown per difference.
    #[arg(long, default_value_t = 5)]
    details: usize,
}

/// Replays and reports; returns whether the emulated boards sent what the real ones did.
pub fn replay_pool_test(a: ReplayArgs) -> Result<bool> {
    let spec = sim_host::bridges::read_spec(&a.robot).map_err(anyhow::Error::msg)?;
    let dir = a.robot.parent().unwrap_or(Path::new("."));
    let logs = a.logs.iter().map(|p| pool::LogFile::read(p).map_err(anyhow::Error::msg)).collect::<Result<Vec<_>>>()?;
    let mut iface_map = Vec::new();
    for m in &a.ifaces {
        let (i, b) = m.split_once('=').context("--iface IFACE=BUS")?;
        iface_map.push((i.to_string(), b.to_string()));
    }
    let pose = match &a.pose {
        Some(p) => {
            let t = Table::load(p).map_err(anyhow::Error::msg)?;
            let track = pool::PoseTrack::from_table(&t).map_err(anyhow::Error::msg)?;
            Some((track, if a.pose_compare { pool::PoseMode::Compare } else { pool::PoseMode::Drive }))
        }
        None => None,
    };
    let opts = pool::PoolOptions {
        iface_map,
        log_start: a.log_start,
        drift_ppm: a.drift_ppm,
        replay_other_boards: !a.no_other_boards,
        tolerance: a.tolerance,
        ignore_payload: a.ignore_payload.clone(),
        only: a.only.clone(),
        max_time: a.time,
        threads: a.threads,
        pose,
        vehicle: a.vehicle.clone(),
        details: a.details,
        compare_after: a.compare_after,
    };
    let res = pool::replay(&spec, dir, &logs, &opts).map_err(anyhow::Error::msg)?;
    print!("{}", res.markdown);
    if let Some(p) = &a.report {
        std::fs::write(p, &res.markdown).with_context(|| format!("writing {}", p.display()))?;
        eprintln!("report: {}", p.display());
    }
    if let Some(p) = &a.candump_log {
        std::fs::write(p, sim_host::canlog::to_text(&res.emulated))
            .with_context(|| format!("writing {}", p.display()))?;
        eprintln!("emulated boards' frames: {} in {}", res.emulated.len(), p.display());
    }
    Ok(res.identical)
}
