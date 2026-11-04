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

/// The typical amplitude of the `hz` component over `[from, to)` seconds: the median over the 5 ms windows that the
/// recording covers. The receiver's ADC records in bursts (IVCBoard2: about 8 ms every 43 ms, while its detector
/// runs in between), and the PWM tone is a little off the nominal frequency (51.5 kHz comes out about 37 Hz high
/// from the 8.4 divider), so a single DFT over hundreds of ms of bursts adds them with drifting phases and can come
/// out at any fraction of the amplitude, depending on where the bursts fall: on the firmware build.
fn tone_median(rec: &[(Time, f32)], from: f64, to: f64, hz: f64) -> f64 {
    let mut amps = Vec::new();
    let mut a = from;
    while a + 0.005 <= to {
        let n = rec.iter().filter(|(t, _)| (a..a + 0.005).contains(&t.as_secs_f64())).count();
        // Only windows the recording fills (2500 samples at 500 kS/s)
        if n >= 2000 {
            amps.push(tone_v(rec, a, a + 0.005, hz));
        }
        a += 0.005;
    }
    assert!(amps.len() >= 3, "{} recorded windows in {from}..{to}", amps.len());
    amps.sort_by(f64::total_cmp);
    amps[amps.len() / 2]
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
    let (sync, other) = (tone_median(&rec, a, b, 50_500.0), tone_median(&rec, a, b, 51_500.0));
    assert!(sync > 0.05 && sync > 10.0 * other, "{sync} {other}");
    let (a, b) = (start + 1.1, start + 1.4);
    let (high, other) = (tone_median(&rec, a, b, 51_500.0), tone_median(&rec, a, b, 50_500.0));
    assert!(high > 0.05 && high > 10.0 * other, "{high} {other}");
    // Talos's projector was driven for the whole packet (18 slots of 500 ms), the surface's never
    let m = r.medium.as_ref().unwrap().lock().unwrap();
    assert!(m.projectors[0].edge_count > 800_000, "{}", m.projectors[0].edge_count);
    assert!(m.projectors[1].edge_count < 10);
}

/// The C IVCBoard2 modem (titan_firmware blake/ivc's ivc/rx/tx/consensus/fft code, unmodified, with UART0 in place
/// of micro-ROS: `test_programs/c_ivc_modem`) and the Rust `ivc_board2` understand each other across the pool, both
/// ways. This is the C/Rust interop the emulator can check without the micro-ROS library (docs/acoustics.md).
#[test]
fn c_and_rust_ivc_board2_modems_exchange_bytes() {
    if common::titan_elf("ivc_board2").is_none() {
        return;
    }
    let c_modem = common::repo().join("test_programs/c_ivc_modem/c_ivc_modem.elf");
    let (mut r, talos, _) = robot("ivc2_pool.toml", |spec| {
        spec.board[1].name = "c_ivc".into();
        spec.board[1].elf = c_modem;
        spec.board[1].rom_boot = true;
    });
    // C -> Rust: a byte on the C board's UART is what its ivc/enqueue_data callback would enqueue
    r.run_until(Time::from_ms(3500));
    let now = r.now;
    r.boards[1].machine.sys.uart_inject(0, &[0xC3], now);
    // Rust -> C, once that packet (9 s) is over
    talos.send_at(Time::from_ms(13_500), HostToDevice::Ivc2(ivc2::HostToDevice::EnqueueData(0x5A)));
    r.run_until(Time::from_ms(22_500));
    let c_log = r.boards[1].uart_text[0].clone();
    assert!(c_log.contains("enqueueing 195") && c_log.contains("wrote out last sync"), "{c_log}");
    let got = ivc2_bytes(&talos);
    assert_eq!(got.iter().map(|b| b.1).collect::<Vec<_>>(), [0xC3], "{c_log}");
    assert!((12.0..13.5).contains(&got[0].0.as_secs_f64()), "{got:?}");
    assert!(r.boards[0].logged("Transmitting 0x5a"));
    assert!(c_log.contains("packet read complete with: 90") && c_log.contains("ROS ivc/rx_new 90"), "{c_log}");
    assert_eq!(c_log.matches("ROS ivc/rx_new").count(), 1, "{c_log}");
}

/// Firmware finding (C IVCBoard2, blake/ivc `tx.c`): `attempt_writing` dequeues the next byte on every main loop pass
/// even while a packet is going out, and only overwrites `data_to_write`; the packet in flight carries the bits of
/// the newest byte from then on, and the earlier byte is lost. Two bytes enqueued together arrive as one, the second.
/// (The Rust port waits for the packet to end; this is why the interop robot file says to send one byte at a time.)
#[test]
fn firmware_finding_c_ivc_board2_overwrites_the_packet_in_flight() {
    if common::titan_elf("ivc_board2").is_none() {
        return;
    }
    let c_modem = common::repo().join("test_programs/c_ivc_modem/c_ivc_modem.elf");
    let (mut r, talos, _) = robot("ivc2_pool.toml", |spec| {
        spec.board[1].elf = c_modem;
        spec.board[1].rom_boot = true;
    });
    r.run_until(Time::from_ms(3500));
    let now = r.now;
    r.boards[1].machine.sys.uart_inject(0, &[0xC3, 0x5A], now);
    r.run_until(Time::from_ms(14_000));
    let c_log = &r.boards[1].uart_text[0];
    assert!(c_log.contains("dequeeud: 195") && c_log.contains("dequeeud: 90"), "{c_log}");
    assert_eq!(c_log.matches("wrote out last sync").count(), 1, "one packet: {c_log}");
    assert_eq!(ivc2_bytes(&talos).iter().map(|b| b.1).collect::<Vec<_>>(), [0x5A]);
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

/// Log lines of `board` containing `text`, with their times.
fn log_times(r: &Robot, board: usize, text: &str) -> Vec<Time> {
    r.boards[board].logs.iter().filter(|(_, l)| l.message.contains(text)).map(|(t, _)| *t).collect()
}

const ACK: &str = "Got response of true from receiver";

/// Runs `r` in 10 ms steps until `done` or `limit`; whether it got there.
fn run_until(r: &mut Robot, limit: Time, done: impl Fn(&Robot) -> bool) -> bool {
    while !done(r) {
        if r.now >= limit {
            return false;
        }
        let next = r.now + Time::from_ms(10);
        r.run_until(next);
    }
    true
}

/// The pool link with the first-generation IVCBoard, booted until Talos's link is established: the sender only
/// retransmits once it has seen an ACK. The heartbeat Talos sends at boot comes too early for the surface board
/// (switched on 37 ms later); the next one, 5 s later, is answered.
fn ivc_pool_linked(edit: impl FnOnce(&mut RobotSpec)) -> (Robot, MockBridge, MockBridge, Time) {
    let (mut r, talos, surface) = robot("ivc_pool.toml", edit);
    r.run_until(Time::from_ms(600));
    assert!(r.boards[0].logged("This board is talos"));
    assert!(r.boards[1].logged("This board is not talos"));
    assert!(run_until(&mut r, Time::from_ms(7500), |r| !log_times(r, 0, ACK).is_empty()), "{:?}", r.boards[0].logs);
    let linked = r.now;
    (r, talos, surface, linked)
}

/// Talos sends two bytes across the pool: each arrives once, in order, and the last copy of each is acknowledged.
/// The pool's reverberation and its pinger (each ping starts a phantom packet that keeps a receiver busy for
/// 825 ms) cost single packets, so which attempt gets through depends on the firmware build's timing (one rustc
/// to the next); what holds is that retransmission gets every byte there within its attempts.
#[test]
fn ivc_boards_exchange_bytes_across_the_pool() {
    if common::titan_elf("ivc_board").is_none() {
        return;
    }
    let (mut r, talos, surface, linked) = ivc_pool_linked(|_| {});
    let sent = [0x5A, 0xC3];
    for b in sent {
        talos.send(HostToDevice::Ivc(titan_link::ivc::HostToDevice::Tx(b)));
    }
    // The bound: each byte gets up to 1 + MAX_MISSED_PACKETS (5) attempts, 1410 ms apart, and the 5 s heartbeats
    // go in between
    let first = |bytes: &[(Time, u8)]| {
        let mut seen = Vec::new();
        for (_, b) in bytes {
            if !seen.contains(b) {
                seen.push(*b);
            }
        }
        seen
    };
    let delivered = run_until(&mut r, linked + Time::from_secs(22), |_| first(&ivc_bytes(&surface)).len() == 2);
    let got = ivc_bytes(&surface);
    assert!(delivered, "{got:?} {:?}", r.boards[0].logs);
    assert_eq!(first(&got), sent, "in order: {got:?}");
    assert!(got.iter().all(|(_, b)| sent.contains(b)), "no wrong bytes: {got:?}");
    // The ACK is heard about 40-370 ms after the receiver passes the byte on
    r.run_until(r.now + Time::from_ms(600));
    let acks = log_times(&r, 0, ACK);
    for b in sent {
        let last = got.iter().rfind(|(_, x)| *x == b).unwrap().0;
        assert!(acks.iter().any(|&a| a > last && a < last + Time::from_ms(500)), "{b:#x} at {last}: {acks:?}");
    }
    let broken = log_times(&r, 0, "IVC link broken");
    assert!(broken.iter().all(|&t| t < linked), "the link held: {broken:?}");
    assert!(ivc_bytes(&talos).is_empty(), "the sender doesn't hear itself");
    let attempts = log_times(&r, 0, "Got response of").iter().filter(|&&t| t > linked).count();
    eprintln!("ivc pool: {got:?} after {attempts} packets from Talos (heartbeats included)");
}

/// Firmware finding (protocol, also in the C firmware's schedule): the surface vehicle's ("SSFB") answer to a
/// packet is lost when Talos has another packet queued. SSFB may only transmit in the 400 ms after its ACK, so its
/// answer starts about 830 ms after Talos's packet began, while Talos waits out its interframe space (it listens
/// again only at 1410 ms, when it sends its next packet). Talos never hears the answer; and SSFB, listening for
/// its ACK from 760 ms after its own start, hears Talos's next packet instead: a 1 symbol there counts as the ACK,
/// so SSFB takes the answer as delivered and never retransmits it. Talos's second packet, sent while SSFB was
/// transmitting, is lost too and gets through on a retry. Timing by protocol constants, the same with any build.
#[test]
fn firmware_finding_ivc_answer_lost_when_talos_sends_on() {
    if common::titan_elf("ivc_board").is_none() {
        return;
    }
    let (mut r, talos, surface, linked) = ivc_pool_linked(|_| {});
    for b in [0x5A, 0xC3] {
        talos.send(HostToDevice::Ivc(titan_link::ivc::HostToDevice::Tx(b)));
    }
    // The surface vehicle answers as soon as 0x5A arrives (retransmissions included)
    assert!(run_until(&mut r, linked + Time::from_secs(10), |_| !ivc_bytes(&surface).is_empty()));
    assert_eq!(ivc_bytes(&surface)[0].1, 0x5A);
    let answered = r.now;
    surface.send(HostToDevice::Ivc(titan_link::ivc::HostToDevice::Tx(0x17)));
    r.run_until(answered + Time::from_secs(12));
    // SSFB's ACK listener "hears" an ACK within its window: Talos's 0xC3 packet
    let acks = log_times(&r, 1, ACK);
    assert!(acks.first().is_some_and(|&a| a > answered + Time::from_ms(700) && a < answered + Time::from_ms(1300)));
    assert!(!r.boards[1].logged("Got response of false"), "so it never retransmits: {:?}", r.boards[1].logs);
    assert!(ivc_bytes(&talos).is_empty(), "the answer never arrives");
    // 0xC3 needed another attempt: its first one went out while SSFB was transmitting
    let got: Vec<u8> = ivc_bytes(&surface).iter().map(|b| b.1).collect();
    assert_eq!(got, [0x5A, 0xC3]);
    let c3 = ivc_bytes(&surface)[1].0;
    assert!(
        log_times(&r, 0, "Got response of false").iter().any(|&t| t > answered && t < c3),
        "{:?}",
        r.boards[0].logs
    );
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

/// Firmware finding: the first-generation IVC receiver loses packets to the pool's own reverberation at full
/// signal strength. `ivc_core::classify` takes a symbol as a 1 only if the 19 kHz band is above 300 *and* the
/// 17 kHz band below 300 (and the other way round for a 0). Mid-slot, 25 ms after the tone switched, the previous
/// symbol still arrives over the longer reflected paths (up to three bounces off the walls, bottom and surface: 30 m
/// and more of path), about 30 dB below the direct tone. With the assumed 40 dB receive gain the direct tone reads
/// 10000-20000, so that echo reads 100-450, around the fixed threshold: whether a symbol is decoded depends on how
/// the echoes add up where the board is, and on exactly when in the slot the receiver's capture falls (which moves
/// with the firmware build's timing). With Talos's board at 5.6 m most packets lose a symbol; a moving board goes
/// through such spots during a packet (the drifting-vehicle test, `tests/vehicle.rs`). Retransmission (titan_firmware_rs b278360) does not save a byte
/// here, since the echoes are the same on every attempt. A threshold relative to the stronger band would not care.
#[test]
fn firmware_finding_ivc_symbols_lost_to_pool_reverberation() {
    if common::titan_elf("ivc_board").is_none() {
        return;
    }
    let (mut r, talos, surface) = robot("ivc_pool.toml", |spec| spec.board[0].position_m = Some([5.6, 6.0, 1.5]));
    hydrophone(&mut r, 1).recording = Some(Vec::new());
    r.run_until(Time::from_ms(600));
    // Several packets, each at a different time and so a different phase of the receiver's captures
    let sent = [0x5A, 0xC3, 0x17, 0x3C, 0xA5, 0x66];
    for b in sent {
        talos.send(HostToDevice::Ivc(titan_link::ivc::HostToDevice::Tx(b)));
    }
    r.run_until(Time::from_ms(12_000));
    // The packets the surface board heard: every CRC line but the phantoms (a ping, or nothing, taken for a start
    // symbol: all 13 symbols missing)
    let heard: Vec<(Time, String)> = r.boards[1]
        .logs
        .iter()
        .filter(|(_, l)| l.message.starts_with("Got CRC_") && !l.message.contains("13 of 13"))
        .map(|(t, l)| (*t, l.message.clone()))
        .collect();
    let lost: Vec<&(Time, String)> = heard.iter().filter(|(_, m)| m.contains("symbols not detected")).collect();
    let delivered = ivc_bytes(&surface).len();
    eprintln!(
        "reverberation at 5.6 m: {} packets heard, {} lost symbols, {delivered} bytes delivered",
        heard.len(),
        lost.len()
    );
    assert!(heard.len() >= 5, "{heard:?}");
    assert!(2 * lost.len() > heard.len(), "most packets lose a symbol: {heard:?}");
    assert!(delivered < sent.len() / 2, "{:?}", ivc_bytes(&surface));

    // The physics, from every 1 ms capture of the surface board's hydrophone while a tone was sounding (Talos's
    // packets, and the surface board's own answers): the stronger band is the symbol, the weaker one its echo
    let caps = captures(&hydrophone(&mut r, 1).recording.take().unwrap());
    let mut tones: Vec<[f64; 2]> = caps
        .iter()
        .map(|(_, c)| {
            let [low, high] = band_magnitudes(c);
            [low.max(high), low.min(high)]
        })
        .filter(|[wanted, _]| *wanted > 2000.0)
        .collect();
    tones.sort_by(|a, b| a[0].total_cmp(&b[0]));
    let median = tones[tones.len() / 2][0];
    // An echo over the threshold although the symbol is more than 26 dB stronger (captures across a symbol change
    // hold both tones and are left out)
    let echo_over = tones.iter().filter(|[wanted, other]| *other > 300.0 && *other < wanted / 20.0).count();
    eprintln!("  {} captures with a tone: median {median:.0}; echo over 300 in {echo_over}", tones.len());
    // The wanted tone is far above the threshold ...
    assert!(median > 5000.0, "{median}");
    // ... and the previous symbol's echo, though some 30 dB below it, over the threshold in a good share of them
    assert!(echo_over * 20 >= tones.len(), "{echo_over} of {}", tones.len());
}
