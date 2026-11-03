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

/// `file` from `environments/` with its pinger at 25 kHz and sounding continuously (pulse = period), so every
/// capture the firmware takes hears it: what is left is the receive chain, the paths and the firmware's own
/// processing, not where a 4 ms ping happens to fall in a capture.
fn continuous_25khz(file: &str) -> Arc<Environment> {
    let path = common::repo().join("environments").join(file);
    let text = std::fs::read_to_string(&path).unwrap();
    let text =
        text.replace("pulse_ms = 4\n", "pulse_ms = 2000\n").replace("frequency_hz = 30_000", "frequency_hz = 25_000");
    let env = Environment::parse(&text, path.parent().unwrap()).unwrap();
    assert!(env.sources.len() == 1 && (env.sources[0].nominal_hz() - 25_000.0).abs() < 1.0, "{file}");
    Arc::new(env)
}

/// The C IVCBoard2 in 25 kHz pinger mode (`test_programs/c_ivc_pinger`) `distance` metres from `env`'s pinger
/// (along -x), with the preamp at `gain_db`: the median amplitude it publishes, and the share of ADC samples the
/// receive chain clipped.
fn c_pinger(env: &Arc<Environment>, distance: f64, gain_db: f64) -> (f64, f64) {
    let elf = common::repo().join("test_programs/c_ivc_pinger/c_ivc_pinger.elf");
    let mut b = Board::load(&common::repo().join("boards/talos/ivc_board.toml")).unwrap();
    b.transducer.as_mut().unwrap().receive.as_mut().unwrap().chain.gain_db = gain_db;
    let mut s = common::boot(&elf, None);
    b.apply(&mut s.machine).unwrap();
    let src = env.sources[0].position;
    b.place_in(&mut s.machine, env.clone(), [src[0] - distance, src[1], src[2]]).unwrap();
    let amps = Arc::new(std::sync::Mutex::new(Vec::new()));
    let a = amps.clone();
    s.on_uart_line = Some(Box::new(move |t, _, line| {
        // After the second the C firmware waits in fft_setup, and a few captures more
        if let Some(v) = line.strip_prefix("ROS pinger_amp ")
            && t > Time::from_ms(1500)
        {
            a.lock().unwrap().push(v.parse::<f64>().unwrap());
        }
    }));
    s.run_until(Time::from_ms(3000));
    let (n, clipped) = sim_host::acoustic::hydrophones(&mut s.machine).next().unwrap().clipping();
    let mut v = amps.lock().unwrap().clone();
    assert!(v.len() >= 10, "{distance} m: {} amplitudes", v.len());
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    (v[v.len() / 2], clipped as f64 / n as f64)
}

/// What the pinger amplitude does with distance, and why (docs/acoustics.md, "What the pinger amplitude does with
/// distance and why"), for the C IVCBoard2 hearing a continuous 25 kHz pinger at 180 dB re 1 uPa. At the board
/// file's assumed 40 dB of preamp gain the receive chain sits at its rails most of the time at every distance the
/// pool allows (and in open water out to beyond 16 m), so the amplitude is flat: the ADC saturates. With the gain
/// 40 dB lower nothing clips: in open water the amplitude then follows the direct path's loss, in the pool it falls
/// by much less than that and not monotonically, as the walls' reflections add up with the direct path.
#[test]
fn c_ivc_board2_pinger_amplitude_against_distance() {
    let pool = continuous_25khz("pool.toml");
    let open = continuous_25khz("open_water.toml");
    let cases: Vec<(&str, &Arc<Environment>, f64, f64)> = [40.0, 0.0]
        .into_iter()
        .flat_map(|g| {
            let p = [2.0, 8.0, 16.0].map(|d| ("pool", &pool, g, d));
            let o = [2.0, 16.0, 45.0].map(|d| ("open water", &open, g, d));
            p.into_iter().chain(o)
        })
        .collect();
    let results: Vec<(f64, f64)> = std::thread::scope(|sc| {
        let hs: Vec<_> = cases.iter().map(|&(_, env, g, d)| sc.spawn(move || c_pinger(env, d, g))).collect();
        hs.into_iter().map(|h| h.join().unwrap()).collect()
    });
    let mut rows = Vec::new();
    for (&(name, env, g, d), &(amp, clipped)) in cases.iter().zip(&results) {
        let src = env.sources[0].position;
        let loss = env.direct_loss_db(src, [src[0] - d, src[1], src[2]], 25_000.0);
        eprintln!(
            "{name:10} gain {g:2.0} dB {d:4.0} m (direct path loss {loss:4.1} dB): amplitude {amp:8.1}, clipped {:4.1} %",
            100.0 * clipped
        );
        rows.push((name, g, d, amp, clipped, loss));
    }
    let row = |name: &str, g: f64, d: f64| *rows.iter().find(|r| r.0 == name && r.1 == g && r.2 == d).unwrap();
    let db = |a: f64, b: f64| 20.0 * (a / b).log10();
    // Saturated: in the pool at every distance, in open water close by
    for d in [2.0, 8.0, 16.0] {
        assert!(row("pool", 40.0, d).4 > 0.5, "{:?}", row("pool", 40.0, d));
    }
    let (near, far) = (row("pool", 40.0, 2.0), row("pool", 40.0, 16.0));
    assert!(db(near.3, far.3).abs() < 1.0, "flat in the pool at 40 dB: {near:?} {far:?}");
    assert!(row("open water", 40.0, 2.0).4 > 0.5 && row("open water", 40.0, 45.0).4 == 0.0);
    // Without clipping, open water follows the direct path within 2 dB from 2 to 16 m. (At 45 m the direct pulse is
    // under one step of the 8-bit samples the C firmware takes, and quantisation bends the curve: 3 dB more loss)
    let (a, b) = (row("open water", 0.0, 2.0), row("open water", 0.0, 16.0));
    assert!(a.4 == 0.0 && b.4 == 0.0);
    assert!((db(a.3, b.3) - (b.5 - a.5)).abs() < 2.0, "{a:?} {b:?}");
    assert!(row("open water", 0.0, 45.0).3 < b.3);
    // ...and the pool falls by at least 6 dB less than the direct path from 2 to 16 m: its reverberation
    let (a, b) = (row("pool", 0.0, 2.0), row("pool", 0.0, 16.0));
    assert!(a.4 == 0.0 && b.4 == 0.0);
    assert!(db(a.3, b.3) > 3.0 && db(a.3, b.3) < (b.5 - a.5) - 6.0, "{a:?} {b:?}");
}

/// The receive chain's two paths at the pinger frequencies and the modem's: GPIO 3 low selects the assumed
/// 40-60 kHz band-pass, which would cut a 25 kHz pinger by 15 dB against the 51 kHz modem tones; GPIO 3 high
/// selects the 60 kHz low-pass, flat over both. The C and Rust firmware both drive GPIO 3 high from boot on.
#[test]
fn ivc_board_receive_paths_at_the_pinger_and_modem_frequencies() {
    let b = Board::load(&common::repo().join("boards/talos/ivc_board.toml")).unwrap();
    let chain = sim_acoustics::chain::Chain::new(b.transducer.unwrap().receive.unwrap().chain, 500_000.0);
    let db = |hz: f64, pins: u32| 20.0 * chain.gain_at(hz, pins).log10();
    let rel = |hz: f64, pins: u32| db(hz, pins) - db(51_000.0, pins);
    eprintln!(
        "band-pass: 25 kHz {:.1} dB, 30 kHz {:.1} dB, 40 kHz {:.1} dB relative to 51 kHz; low-pass: {:.1}, {:.1}, {:.1} dB",
        rel(25e3, 0),
        rel(30e3, 0),
        rel(40e3, 0),
        rel(25e3, 1 << 3),
        rel(30e3, 1 << 3),
        rel(40e3, 1 << 3)
    );
    assert!((rel(25e3, 0) + 15.4).abs() < 0.5 && (rel(30e3, 0) + 9.2).abs() < 0.5);
    assert!(rel(25e3, 1 << 3).abs() < 1.5 && rel(30e3, 1 << 3).abs() < 1.5);
    assert!((db(25e3, 1 << 3) - 40.0).abs() < 0.2, "the 40 dB preamp, nothing taken off at 25 kHz");
}
