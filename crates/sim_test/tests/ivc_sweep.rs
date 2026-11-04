//! M7: IVC delivery against range and noise. Two vehicles each with an IVC board, booted once; then every sweep point
//! starts from a snapshot of that, moves the second vehicle to the point's range and sets the ambient noise, and
//! counts the bytes that arrive. Each test prints its table (run with `--nocapture` to see it).

use sim_test::titan_link::{DeviceToHost, HostToDevice, ivc, ivc2};
use sim_test::{RobotBench, Time, Trajectory, sweep};

const THREADS: usize = 4;

/// What the IVCBoard sender logs for an acknowledged packet.
const ACK: &str = "Got response of true from receiver";

#[derive(Clone, Copy, Debug)]
struct Point {
    range_m: f64,
    /// White noise level, dB re 1 µPa²/Hz.
    noise_db: f64,
}

#[derive(Debug)]
struct Outcome {
    point: Point,
    delivered: usize,
    sent: usize,
    duplicates: usize,
    wrong: usize,
    /// Packets the sender saw acknowledged (IVCBoard only).
    acked: usize,
}

fn table(name: &str, rows: &[Outcome]) {
    eprintln!("{name}: range, noise (dB re 1 uPa^2/Hz) -> delivered / sent (duplicates, wrong bytes, ACKs seen)");
    for r in rows {
        eprintln!(
            "  {:>6.0} m  {:>5.0} dB  ->  {}/{}  ({} dup, {} wrong, {} acked)",
            r.point.range_m, r.point.noise_db, r.delivered, r.sent, r.duplicates, r.wrong, r.acked
        );
    }
}

fn outcome(point: Point, sent: &[u8], got: &[u8]) -> Outcome {
    let mut delivered = 0;
    let mut duplicates = 0;
    let mut seen = Vec::new();
    for b in got {
        if sent.contains(b) {
            if seen.contains(b) {
                duplicates += 1;
            } else {
                seen.push(*b);
                delivered += 1;
            }
        }
    }
    let wrong = got.iter().filter(|b| !sent.contains(b)).count();
    Outcome { point, delivered, sent: sent.len(), duplicates, wrong, acked: 0 }
}

/// The delivery rate must not grow along a sweep of increasing difficulty (allowing one byte of jitter: near a
/// cliff single points flip with the firmware build's timing and the noise), the easiest point must deliver
/// everything, once and correctly (and, with `acked`, see every byte acknowledged), and the hardest must lose bytes.
fn check_degrades(rows: &[Outcome], acked: bool) {
    let easiest = &rows[0];
    assert_eq!(easiest.delivered, easiest.sent, "the easiest point delivers everything: {rows:?}");
    assert_eq!((easiest.duplicates, easiest.wrong), (0, 0), "{rows:?}");
    if acked {
        assert!(easiest.acked >= easiest.sent, "and every byte is acknowledged: {rows:?}");
    }
    for w in rows.windows(2) {
        assert!(w[1].delivered <= w[0].delivered + 1, "{rows:?}");
    }
    let last = rows.last().unwrap();
    assert!(last.delivered < last.sent, "the hardest point loses bytes: {rows:?}");
}

/// First generation IVCBoard (17/19 kHz, ACKed and retransmitted, 650 ms packets) between Talos and the surface
/// vehicle in open water at 5 m depth.
#[test]
fn ivc_delivery_degrades_with_range_and_noise() {
    if sim_test::titan_rs("ivc_board").is_none() {
        return;
    }
    let mut base = RobotBench::load("ivc_pool.toml", &[("talos", 8), ("surface", 8)], |spec| {
        spec.environment.as_mut().unwrap().file = "../environments/open_water.toml".into();
        spec.board[0].position_m = Some([0.0, 0.0, 5.0]);
        spec.board[1].position_m = Some([25.0, 0.0, 5.0]);
    })
    .unwrap();
    // Until the link is established: the sender only retransmits once it has seen an ACK. The heartbeat Talos sends
    // at boot comes too early for the surface board (switched on 37 ms later); the next, 5 s later, is answered.
    let acks = |b: &RobotBench| b.board("talos_ivc").logs.iter().filter(|(_, l)| l.message == ACK).count();
    while acks(&base) == 0 {
        assert!(base.now() < Time::from_ms(7500), "no ACK: {:?}", base.board("talos_ivc").logs);
        base.run_for(Time::from_ms(100));
    }
    let bytes = [0x11u8, 0x22, 0x33, 0x44];
    let run = |mut b: RobotBench, p: &Point| {
        b.place("surface_ivc", Trajectory::Fixed([p.range_m, 0.0, 5.0]));
        b.robot.set_noise_db(p.noise_db);
        for &x in &bytes {
            b.bridge("talos").send(HostToDevice::Ivc(ivc::HostToDevice::Tx(x)));
        }
        let from = b.now();
        b.run_for(Time::from_secs(9));
        let acked = b.board("talos_ivc").logs.iter().filter(|(t, l)| *t > from && l.message == ACK).count();
        let got: Vec<u8> = b
            .picked("surface", |m| match m {
                DeviceToHost::Ivc(ivc::DeviceToHost::Rx(x)) => Some(*x),
                _ => None,
            })
            .into_iter()
            .map(|(_, x)| x)
            .collect();
        Outcome { acked, ..outcome(*p, &bytes, &got) }
    };

    let by_noise: Vec<Point> =
        [60.0, 74.0, 76.0, 77.0, 78.0, 85.0].map(|n| Point { range_m: 25.0, noise_db: n }).to_vec();
    let rows = sweep(&base, &by_noise, THREADS, run);
    table("IVCBoard, noise at 25 m", &rows);
    check_degrades(&rows, true);

    let by_range: Vec<Point> =
        [25.0, 50.0, 80.0, 86.0, 92.0, 150.0].map(|r| Point { range_m: r, noise_db: 70.0 }).to_vec();
    let rows = sweep(&base, &by_range, THREADS, run);
    table("IVCBoard, range at 70 dB", &rows);
    check_degrades(&rows, true);
}

/// Second generation IVCBoard2 (50.5/51/51.5 kHz, 9 s packets, no ACK) between the two vehicles.
#[test]
fn ivc2_delivery_degrades_with_range_and_noise() {
    if sim_test::titan_rs("ivc_board2").is_none() {
        return;
    }
    let mut base = RobotBench::load("ivc2_pool.toml", &[("talos", 8), ("surface", 8)], |spec| {
        spec.environment.as_mut().unwrap().file = "../environments/open_water.toml".into();
        spec.board[0].position_m = Some([0.0, 0.0, 5.0]);
        spec.board[1].position_m = Some([10.0, 0.0, 5.0]);
    })
    .unwrap();
    // The receivers' noise floor settles first
    base.run_until(Time::from_ms(3500));
    let byte = [0xA5u8];
    let run = |mut b: RobotBench, p: &Point| {
        b.place("surface_ivc", Trajectory::Fixed([p.range_m, 0.0, 5.0]));
        b.robot.set_noise_db(p.noise_db);
        // A second for the receiver's noise floor to follow the new noise level
        b.run_for(Time::from_secs(1));
        b.bridge("talos").send(HostToDevice::Ivc2(ivc2::HostToDevice::EnqueueData(byte[0])));
        b.run_for(Time::from_ms(9600));
        let got: Vec<u8> = b
            .picked("surface", |m| match m {
                DeviceToHost::Ivc2(ivc2::DeviceToHost::RxNew(x)) => Some(*x),
                _ => None,
            })
            .into_iter()
            .map(|(_, x)| x)
            .collect();
        outcome(*p, &byte, &got)
    };
    let by_noise: Vec<Point> = [50.0, 95.0, 105.0, 115.0].map(|n| Point { range_m: 10.0, noise_db: n }).to_vec();
    let rows = sweep(&base, &by_noise, THREADS, run);
    table("IVCBoard2, noise at 10 m", &rows);
    check_degrades(&rows, false);

    let by_range: Vec<Point> = [10.0, 300.0, 600.0, 1000.0].map(|r| Point { range_m: r, noise_db: 70.0 }).to_vec();
    let rows = sweep(&base, &by_range, THREADS, run);
    table("IVCBoard2, range at 70 dB", &rows);
    check_degrades(&rows, false);
}
