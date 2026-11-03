//! M4: IVCBoard2 in an acoustic environment. In pinger mode the firmware band-passes each capture around the
//! selected pinger frequency and reports the pinger band's amplitude; it must fall off with distance as the
//! environment's transmission loss predicts.

mod common;

use std::sync::Arc;

use sim_acoustics::Environment;
use sim_core::time::Time;
use sim_devices::can::CanBus;
use sim_host::board::Board;
use sim_host::mock_bridge::MockBridge;
use titan_link::{DeviceToHost, HostToDevice, ivc2};

const RX: [f64; 3] = [0.0, 0.0, 2.0];

fn pool_with_pinger(distance: f64) -> Arc<Environment> {
    // Open water, a continuous 30 kHz pinger (pulse = period) at 150 dB re 1 uPa at 1 m, some white noise
    let env = format!(
        r#"
kind = "open"
noise = {{ kind = "white", level_db_re_upa_per_hz = 40.0 }}
water = {{ temperature_c = 15.0 }}
[[source]]
kind = "pinger"
position_m = [{distance}, 0.0, 2.0]
frequency_hz = 30_000
pulse_ms = 100
period_ms = 100
source_level_db = 150
"#
    );
    Arc::new(Environment::parse(&env, std::path::Path::new(".")).unwrap())
}

/// Runs IVCBoard2 in pinger mode with the pinger `distance` metres away; returns the reported amplitudes.
fn pinger_amplitudes(elf: &std::path::Path, distance: f64) -> (Vec<f32>, Vec<i32>, f64) {
    let b = Board::load(&common::repo().join("boards/talos/ivc_board.toml")).unwrap();
    let mut s = common::boot(elf, None);
    b.apply(&mut s.machine).unwrap();
    let env = pool_with_pinger(distance);
    b.place_in(&mut s.machine, env.clone(), RX).unwrap();
    s.can_bus = Some(CanBus::new("can0", b.can_rates().unwrap()));
    let bridge = MockBridge::new(8, true, Time::ZERO);
    bridge.set_kill(Some(false));
    s.can_endpoints.push(Box::new(bridge.clone()));
    s.run_until(Time::from_ms(600));
    assert!(s.logged("ROS connected"));
    bridge.send(HostToDevice::Ivc2(ivc2::HostToDevice::PingerEnable(true)));
    s.run_until(Time::from_ms(3000));
    let st = bridge.state();
    let amps = st
        .received
        .iter()
        .filter_map(|(_, m)| match m {
            DeviceToHost::Ivc2(ivc2::DeviceToHost::PingerAmplitude(a)) => Some(*a),
            _ => None,
        })
        .collect();
    let khz = st
        .received
        .iter()
        .filter_map(|(_, m)| match m {
            DeviceToHost::Ivc2(ivc2::DeviceToHost::PingerFreqKhz(k)) => Some(*k),
            _ => None,
        })
        .collect();
    let loss = env.direct_loss_db(env.sources[0].position, RX, 30_000.0);
    (amps, khz, loss)
}

fn median(v: &[f32]) -> f64 {
    let mut v = v.to_vec();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v[v.len() / 2] as f64
}

#[test]
fn ivc2_pinger_amplitude_falls_off_with_distance() {
    let Some(elf) = common::titan_elf("ivc_board2") else { return };
    let mut results = Vec::new();
    for d in [4.0, 8.0, 16.0] {
        let (amps, khz, loss) = pinger_amplitudes(&elf, d);
        assert!(amps.len() >= 10, "{d} m: {} amplitudes", amps.len());
        assert!(khz.iter().all(|&k| k == 30) && !khz.is_empty(), "{khz:?}");
        results.push((d, median(&amps[2..]), loss));
    }
    eprintln!("{results:?}");
    // Amplitude ratios between distances match the environment's transmission loss (spherical spreading plus
    // absorption) within 10 %
    for w in results.windows(2) {
        let (d0, a0, l0) = w[0];
        let (d1, a1, l1) = w[1];
        let predicted = 10f64.powf((l1 - l0) / 20.0);
        let measured = a0 / a1;
        assert!((measured / predicted - 1.0).abs() < 0.1, "{d0} m -> {d1} m: {measured:.3} vs {predicted:.3}");
    }
}
