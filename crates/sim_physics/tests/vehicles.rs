//! The shipped vehicle files: their parameters are assumed (see the files), so this checks that what they imply is
//! plausible for an AUV of that size with T200 thrusters, and prints it (`--nocapture`).

use sim_physics::{Quat, State, Vehicle, VehicleFile, Water};

fn load(name: &str) -> Vehicle {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("../../vehicles/{name}.toml"));
    let f = VehicleFile::load(&path).unwrap();
    Vehicle::new(&f, Water::default(), State::at_rest([0.0, 0.0, 5.0], [0.0; 3]))
}

/// Steady state after 30 s at constant RPMs (inverted thrusters get the opposite sign, so every thruster pushes
/// along its direction), with the vehicle trimmed neutral so only the thrust moves it, and started pitched 2
/// degrees so an unstable trim would show.
fn steady(name: &str, rpm: impl Fn(usize, &str) -> f64) -> Vehicle {
    let mut v = load(name);
    v.params.volume_m3 = v.params.mass_kg / v.water.density_kg_m3;
    v.state.attitude = Quat::from_euler(0.0, 2f64.to_radians(), 0.0);
    let r: Vec<f64> =
        v.thrusters.iter().enumerate().map(|(i, t)| rpm(i, &t.name) * if t.inverted { -1.0 } else { 1.0 }).collect();
    v.advance_to(30.0, |_| r.clone());
    v
}

#[test]
fn shipped_vehicles_perform_plausibly() {
    for (name, horizontal) in [("talos", 0..4), ("puddles", 0..4)] {
        let full = 4000.0;
        let ahead = steady(name, |i, _| if horizontal.contains(&i) { full } else { 0.0 });
        let dive = steady(name, |i, _| if horizontal.contains(&i) { 0.0 } else { full });
        // Turning: the starboard pair (0, 2) ahead, the port pair (1, 3) astern
        let turn = steady(name, |i, _| match i {
            0 | 2 => full,
            1 | 3 => -full,
            _ => 0.0,
        });
        let surge = ahead.state.velocity[0];
        let heave = dive.state.velocity[2];
        let yaw_rate = turn.state.rate[2].to_degrees();
        // Roll period from a 10 degree release
        let mut v = load(name);
        v.params.volume_m3 = v.params.mass_kg / v.water.density_kg_m3;
        v.state.attitude = Quat::from_euler(10f64.to_radians(), 0.0, 0.0);
        let mut crossings = Vec::new();
        let mut last = 1.0;
        for _ in 0..20_000 {
            v.step(&[]);
            let roll = v.state.attitude.euler()[0];
            if last > 0.0 && roll <= 0.0 {
                crossings.push(v.time);
            }
            last = roll;
        }
        let period = crossings[1] - crossings[0];
        assert!((1.0..5.0).contains(&period), "{name} roll period {period}");
        let net = load(name).params.net_buoyancy_n(&Water::default());
        eprintln!(
            "{name}: full ahead {surge:.2} m/s, full dive {heave:.2} m/s, turn {yaw_rate:.0} deg/s, roll period \
             {period:.1} s, net buoyancy {net:.1} N"
        );
        // An open-frame AUV with eight T200s: about 1 m/s, a fraction of that in heave, a turn in a few seconds
        assert!((0.8..2.0).contains(&surge), "{name} surge {surge}");
        assert!((0.5..1.5).contains(&heave), "{name} heave {heave}");
        assert!(yaw_rate < -30.0 && yaw_rate > -300.0, "{name} yaw {yaw_rate}");
        // Going straight ahead at any speed it rights the disturbance instead of pitching over
        for rpm in [1000.0, 2000.0, 3000.0, 4000.0] {
            let v = steady(name, |i, _| if horizontal.contains(&i) { rpm } else { 0.0 });
            let [roll, pitch, yaw] = v.state.attitude.euler().map(f64::to_degrees);
            assert!(
                roll.abs() < 1e-6 && pitch.abs() < 0.5 && yaw.abs() < 1e-6,
                "{name} at {rpm}: {roll} {pitch} {yaw}"
            );
        }
        // Slightly positive: it floats if it loses power
        assert!((0.5..5.0).contains(&net), "{name} net buoyancy {net}");
    }
}
