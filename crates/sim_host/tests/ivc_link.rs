//! M7: IVC boards talking acoustically. Each board's transducer drive pin feeds a projector in a shared
//! environment; the other board's hydrophone hears it through the water (spreading, absorption, multipath, noise)
//! and its firmware decodes the bytes, which reach that vehicle's (mock) bridge.

mod common;

use sim_core::time::Time;
use sim_host::mock_bridge::MockBridge;
use sim_host::robot::{Robot, RobotSpec};
use titan_link::{DeviceToHost, HostToDevice, ivc2};

fn ivc2_bytes(bridge: &MockBridge) -> Vec<(Time, u8)> {
    bridge
        .state()
        .received
        .iter()
        .filter_map(|(t, m)| match m {
            DeviceToHost::Ivc2(ivc2::DeviceToHost::RxNew(b)) => Some((*t, *b)),
            _ => None,
        })
        .collect()
}

/// Loads a robot file from `robots/` with two bridged IVC boards (client 8 on each bus).
fn robot(file: &str, edit: impl FnOnce(&mut RobotSpec)) -> (Robot, MockBridge, MockBridge) {
    let path = common::repo().join("robots").join(file);
    let mut spec: RobotSpec = toml::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    edit(&mut spec);
    let mut r = Robot::from_spec(&spec, path.parent().unwrap()).unwrap();
    let mut bridges = Vec::new();
    for b in r.buses.iter_mut() {
        let br = MockBridge::new(8, true, Time::ZERO);
        br.set_kill(Some(false));
        b.endpoints.push(Box::new(br.clone()));
        bridges.push(br);
    }
    (r, bridges.remove(0), bridges.remove(0))
}

/// Amplitude of the `hz` component in ADC samples `(time, volts)` taken in `[from, to)` seconds.
fn tone_v(rec: &[(Time, f32)], from: f64, to: f64, hz: f64) -> f64 {
    let xs: Vec<&(Time, f32)> = rec.iter().filter(|(t, _)| (from..to).contains(&t.as_secs_f64())).collect();
    let n = xs.len() as f64;
    let mean = xs.iter().map(|x| x.1 as f64).sum::<f64>() / n;
    let (mut re, mut im) = (0.0, 0.0);
    for (t, v) in &xs {
        let ph = 2.0 * std::f64::consts::PI * hz * t.as_secs_f64();
        re += (*v as f64 - mean) * ph.cos();
        im += (*v as f64 - mean) * ph.sin();
    }
    2.0 * (re * re + im * im).sqrt() / n
}

fn hydrophone(r: &mut Robot, board: usize) -> &mut sim_acoustics::Hydrophone {
    sim_host::acoustic::hydrophones(&mut r.boards[board].machine).next().unwrap()
}

#[test]
fn ivc2_boards_exchange_a_byte_across_the_pool() {
    if common::titan_elf("ivc_board2").is_none() {
        return;
    }
    let (mut r, talos, surface) = robot("ivc2_pool.toml", |_| {});
    // The receivers' noise floor settles over 50 captures (about 2.5 s) before they detect anything
    r.run_until(Time::from_ms(3500));
    assert!(r.boards[0].logged("ROS connected") && r.boards[1].logged("ROS connected"));
    hydrophone(&mut r, 1).recording = Some(Vec::new());
    talos.send(HostToDevice::Ivc2(ivc2::HostToDevice::EnqueueData(0xA5)));
    r.run_until(Time::from_ms(13_000));
    assert!(r.boards[0].logged("Transmitting 0xa5"));
    assert!(r.boards[1].logged("packet read complete with: 0xa5"), "{:?}", r.boards[1].logs);
    assert_eq!(ivc2_bytes(&surface).iter().map(|b| b.1).collect::<Vec<_>>(), [0xA5]);
    assert!(ivc2_bytes(&talos).is_empty(), "the sender doesn't hear itself");

    // What reached the other hydrophone: the packet's first slots are sync (50.5 kHz), then bit 0 (1: 51.5 kHz)
    let start = r.boards[0].logs.iter().find(|(_, l)| l.message.contains("Transmitting")).unwrap().0.as_secs_f64();
    let rec = hydrophone(&mut r, 1).recording.take().unwrap();
    let (a, b) = (start + 0.1, start + 0.4);
    let sync = tone_v(&rec, a, b, 50_500.0);
    assert!(sync > 0.05 && sync > 10.0 * tone_v(&rec, a, b, 51_500.0), "{sync}");
    let (a, b) = (start + 1.1, start + 1.4);
    let high = tone_v(&rec, a, b, 51_500.0);
    assert!(high > 0.05 && high > 10.0 * tone_v(&rec, a, b, 50_500.0), "{high}");
    // Talos's projector was driven for the whole packet (18 slots of 500 ms), the surface's never
    let m = r.medium.as_ref().unwrap().lock().unwrap();
    assert!(m.projectors[0].edge_count > 800_000, "{}", m.projectors[0].edge_count);
    assert!(m.projectors[1].edge_count < 10);
}

fn ivc_bytes(bridge: &MockBridge) -> Vec<(Time, u8)> {
    bridge
        .state()
        .received
        .iter()
        .filter_map(|(t, m)| match m {
            DeviceToHost::Ivc(titan_link::ivc::DeviceToHost::Rx(b)) => Some((*t, *b)),
            _ => None,
        })
        .collect()
}

#[test]
fn ivc_boards_exchange_bytes_across_the_pool() {
    if common::titan_elf("ivc_board").is_none() {
        return;
    }
    let (mut r, talos, surface) = robot("ivc_pool.toml", |_| {});
    r.run_until(Time::from_ms(600));
    assert!(r.boards[0].logged("This board is talos"));
    assert!(r.boards[1].logged("This board is not talos"));
    // Talos sends two bytes; each is acknowledged
    for b in [0x5A, 0xC3] {
        talos.send(HostToDevice::Ivc(titan_link::ivc::HostToDevice::Tx(b)));
    }
    r.run_until(Time::from_ms(4000));
    assert_eq!(ivc_bytes(&surface).iter().map(|b| b.1).collect::<Vec<_>>(), [0x5A, 0xC3]);
    let acks =
        |r: &Robot| r.boards[0].logs.iter().filter(|(_, l)| l.message == "Got response of true from receiver").count();
    // Both bytes acknowledged. (The heartbeat Talos sends at boot comes too early for the surface board, switched
    // on 37 ms later, and fails its CRC.)
    assert_eq!(acks(&r), 2);
    // The surface vehicle answers
    surface.send(HostToDevice::Ivc(titan_link::ivc::HostToDevice::Tx(0x17)));
    r.run_until(Time::from_ms(8000));
    let got = ivc_bytes(&talos);
    assert_eq!(got.iter().map(|b| b.1).collect::<Vec<_>>(), [0x17], "{:?}", r.boards[0].logs);
    // The reply goes out in the window after the last packet from Talos, collides with Talos's 5 s heartbeat and
    // gets through on a retry
    assert!(r.boards[1].logged("Got response of false from receiver"));
    assert!(r.boards[1].logged("Got response of true from receiver"));
    assert_eq!(ivc_bytes(&surface).len(), 2, "nothing heard twice");
}

/// The magnitudes `ivc_core::BandDetector` finds at 17 and 19 kHz (the single DFT bin in each band) in a capture
/// of 8-bit samples, as the firmware computes them.
fn band_magnitudes(capture: &[u8]) -> [f64; 2] {
    let mean = capture.iter().map(|&x| x as f64).sum::<f64>() / capture.len() as f64;
    [17, 19].map(|k| {
        let c = 2.0 * (2.0 * std::f64::consts::PI * k as f64 / capture.len() as f64).cos();
        let (mut s1, mut s2) = (0.0, 0.0);
        for &x in capture {
            let s0 = (x as f64 - mean) + c * s1 - s2;
            s2 = s1;
            s1 = s0;
        }
        (s1 * s1 + s2 * s2 - c * s1 * s2).max(0.0).sqrt()
    })
}

/// The IVCBoard's 1 ms captures in an ADC recording, as `(end time, 8-bit samples)`. Each `read_many` converts
/// back to back at 500 kS/s and the DMA takes the first 500 conversions; a few more convert before the firmware
/// stops the ADC, and a gap of about 14 µs separates one capture from the next.
fn captures(rec: &[(Time, f32)]) -> Vec<(f64, Vec<u8>)> {
    let mut runs = vec![0];
    runs.extend((1..rec.len()).filter(|&i| (rec[i].0 - rec[i - 1].0) > Time::from_ns(2500)));
    runs.windows(2)
        .filter(|w| w[1] - w[0] >= 500)
        .map(|w| {
            let samples = rec[w[0]..w[0] + 500].iter().map(|(_, v)| (v / 3.3 * 256.0).clamp(0.0, 255.0) as u8);
            (rec[w[0] + 499].0.as_secs_f64(), samples.collect())
        })
        .collect()
}

/// Firmware finding: the first-generation IVC receiver can lose a packet to the pool's own reverberation at full
/// signal strength, wherever it is, moving or not. `ivc_core::classify` takes a symbol as a 1 only if the 19 kHz
/// band is above 300 *and* the 17 kHz band below 300 (and the other way round for a 0). Mid-slot, 25 ms after the
/// tone switched, the previous symbol still arrives over the longer reflected paths (up to three bounces off the
/// walls, bottom and surface: 30 m and more of path), about 30 dB below the direct tone. With the assumed 40 dB
/// receive gain the direct tone reads 10000-20000, so that echo reads 100-450, around the fixed threshold: whether
/// a 0-to-1 symbol is decoded depends on how the echoes add up where the board is. Talos's board at 5.0 m (the
/// robot file) gets 0x5A through, at 5.6 m it does not; a board moving at 0.5 m/s goes through such spots during a
/// packet, which is why the drifting-vehicle test (`tests/vehicle.rs`) loses 0x5A at that speed and not at 0.2 m/s.
/// A threshold relative to the stronger band would not care.
#[test]
fn firmware_finding_ivc_symbols_lost_to_pool_reverberation() {
    if common::titan_elf("ivc_board").is_none() {
        return;
    }
    let (mut r, talos, surface) = robot("ivc_pool.toml", |spec| spec.board[0].position_m = Some([5.6, 6.0, 1.5]));
    hydrophone(&mut r, 1).recording = Some(Vec::new());
    r.run_until(Time::from_ms(600));
    talos.send(HostToDevice::Ivc(titan_link::ivc::HostToDevice::Tx(0x5A)));
    r.run_until(Time::from_ms(2400));
    assert!(ivc_bytes(&surface).is_empty());
    let bad = r.boards[1].logs.iter().rfind(|(_, l)| l.message.contains("Got CRC_BAD")).unwrap();
    assert!(bad.1.message.contains("expected 8"), "{}", bad.1.message);

    // What the receiver's detector saw at the middle of each symbol slot: it starts its ACK (and logs the CRC)
    // 725 ms after the start symbol's onset
    let onset = bad.0.as_secs_f64() - 0.725;
    let caps = captures(&hydrophone(&mut r, 1).recording.take().unwrap());
    let at = |t: f64| band_magnitudes(&caps[caps.partition_point(|c| c.0 <= t) - 1].1);
    let word = ivc_core_word(0x5A);
    let mut lost = 0;
    for (i, bit) in word.iter().enumerate() {
        let [low, high] = at(onset + 0.050 * (i + 1) as f64 + 0.025);
        let (wanted, other) = if *bit { (high, low) } else { (low, high) };
        // The wanted tone is far above the threshold every time ...
        assert!(wanted > 5000.0, "symbol {i}: {low} {high}");
        // ... the previous symbol's echo is only some 30 dB below it, and above the threshold on some 1s
        assert!(other < wanted / 20.0, "symbol {i}: {low} {high}");
        if other > 300.0 {
            lost += 1;
        }
    }
    assert!(lost >= 1, "no symbol lost");
}

/// The 13 symbols of a data packet after the start symbol (heartbeat flag, 8 data bits, 4 CRC bits), MSB first.
fn ivc_core_word(data: u8) -> Vec<bool> {
    let crc = (0..8).rev().fold(0u8, |crc, i| {
        let c = (crc << 1) | ((data >> i) & 1);
        if c & 0x10 != 0 { c ^ 0x1B } else { c }
    }) & 0xF;
    let word = (data as u16) << 4 | crc as u16;
    (0..13).rev().map(|b| (word >> b) & 1 != 0).collect()
}
