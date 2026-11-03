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
