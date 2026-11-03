//! A record of a vehicle's state over time, for tests and for plotting a run (`--vehicle-csv`).

use std::fmt::Write as _;

use crate::math::Vec3;
use crate::vehicle::Vehicle;

/// One sample of the vehicle's state.
#[derive(Clone, Debug, PartialEq)]
pub struct Row {
    pub time_s: f64,
    /// World frame, m.
    pub position: Vec3,
    /// Roll, pitch, yaw, rad.
    pub rpy: Vec3,
    /// Body frame, m/s.
    pub velocity: Vec3,
    /// Body frame, rad/s.
    pub rate: Vec3,
    /// Per thruster, N.
    pub thrust_n: Vec<f64>,
}

/// Samples taken every `interval_s` of vehicle time.
#[derive(Clone, Debug)]
pub struct StateLog {
    pub interval_s: f64,
    pub rows: Vec<Row>,
    pub thruster_names: Vec<String>,
    next: f64,
}

impl StateLog {
    pub fn new(interval_s: f64, vehicle: &Vehicle) -> StateLog {
        StateLog {
            interval_s,
            rows: Vec::new(),
            thruster_names: vehicle.thrusters.iter().map(|t| t.name.clone()).collect(),
            next: 0.0,
        }
    }

    /// Takes a sample if one is due.
    pub fn record(&mut self, v: &Vehicle) {
        // Half a step of slack: a sample due at a step's time is taken at that step despite rounding
        if v.time + v.dt / 2.0 < self.next {
            return;
        }
        self.next += self.interval_s;
        let s = &v.state;
        self.rows.push(Row {
            time_s: v.time,
            position: s.position,
            rpy: s.attitude.euler(),
            velocity: s.velocity,
            rate: s.rate,
            thrust_n: v.thrust_n.clone(),
        });
    }

    /// Comma separated, one header line then one line per sample; angles in degrees.
    pub fn to_csv(&self) -> String {
        let mut out = String::from(
            "time_s,north_m,east_m,depth_m,roll_deg,pitch_deg,yaw_deg,u_mps,v_mps,w_mps,p_dps,q_dps,r_dps",
        );
        for n in &self.thruster_names {
            write!(out, ",{n}_n").unwrap();
        }
        out.push('\n');
        for r in &self.rows {
            write!(out, "{:.4}", r.time_s).unwrap();
            for v in r.position {
                write!(out, ",{v:.5}").unwrap();
            }
            for v in r.rpy {
                write!(out, ",{:.3}", v.to_degrees()).unwrap();
            }
            for v in r.velocity {
                write!(out, ",{v:.5}").unwrap();
            }
            for v in r.rate {
                write!(out, ",{:.3}", v.to_degrees()).unwrap();
            }
            for v in &r.thrust_n {
                write!(out, ",{v:.3}").unwrap();
            }
            out.push('\n');
        }
        out
    }
}
