//! Recording, replaying and comparing CAN traffic with real firmware: a board's run is logged in `candump -l`
//! format, the bridge's side of it is replayed into a fresh run (the board sees exactly what the bridge sent), the
//! two logs compare identical, and perturbed runs show up as the differences they are. Also a board reaching a
//! bridge on "another machine" through a cannelloni tunnel over loopback.

use std::io::Write;
use std::sync::{Arc, Mutex};

use sim_devices::can::CanBus;
use sim_host::candiff::{self, DiffOptions};
use sim_host::canlog::{self, CandumpWriter, Filter, LogFrame, Replay, ReplayOptions};
use sim_host::cannelloni::Cannelloni;
use sim_test::{Bench, MockBridge, Time};

#[derive(Clone, Default)]
struct Buf(Arc<Mutex<Vec<u8>>>);

impl Write for Buf {
    fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
        self.0.lock().unwrap().extend_from_slice(b);
        Ok(b.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

/// Runs `bench` to `until` with its bus logged; returns the log's text.
fn logged_run(mut bench: Bench, until: Time) -> (Bench, String) {
    let buf = Buf::default();
    let w = CandumpWriter::new(Box::new(buf.clone()));
    bench.session.on_can = Some(w.hook("can0", None));
    bench.run_until(until).unwrap();
    w.flush().unwrap();
    let text = String::from_utf8(buf.0.lock().unwrap().clone()).unwrap();
    (bench, text)
}

fn power_board(bridge: bool) -> Option<Bench> {
    let elf = sim_test::titan_rs("power_board")?;
    let b = Bench::builder(&elf).board("mk2/power_board").record_bus();
    Some(if bridge { b.bridge(1) } else { b }.boot().unwrap())
}

/// The bridge's frames of `log`, replayed into a fresh power board run that is logged too.
fn replayed(log: &[LogFrame], until: Time, edit: impl FnOnce(&Bench)) -> String {
    let mut b = power_board(false).unwrap();
    edit(&b);
    let rates = b.session.can_bus.as_ref().unwrap().rates;
    let opts = ReplayOptions {
        filters: vec![Filter::parse("sender=host").unwrap()],
        origin_ns: Some(0),
        rates: Some(rates),
        ..Default::default()
    };
    b.session.can_endpoints.push(Box::new(Replay::from_log(log, &opts)));
    logged_run(b, until).1
}

fn compare(a: &str, b: &str) -> candiff::Report {
    let (a, b) = (canlog::parse(a).unwrap(), canlog::parse(b).unwrap());
    candiff::diff(&candiff::decode(&a, false), &candiff::decode(&b, false), &DiffOptions::default())
}

#[test]
fn record_replay_and_diff_a_board() {
    let Some(bench) = power_board(true) else { return };
    let end = Time::from_ms(3000);
    let (bench, text_a) = logged_run(bench, end);
    // The log holds every acknowledged frame of the bus, as a listening candump would see them
    let log_a = canlog::parse(&text_a).unwrap();
    let acked = bench.frames().iter().filter(|r| r.acked).count();
    assert_eq!(log_a.len(), acked);
    assert!(log_a.iter().zip(bench.frames().iter().filter(|r| r.acked)).all(|(l, r)| l.frame == r.frame));
    assert_eq!(canlog::from_records(bench.frames(), "can0"), log_a, "microsecond timestamps of the frames' ends");
    assert_eq!(canlog::to_text(&log_a), text_a);
    assert!(bench.statuses().len() >= 2, "the bridge saw the board connect");

    // Replaying only what the bridge sent gives the board the same inputs: the runs are the same
    let text_b = replayed(&log_a, end, |_| {});
    let r = compare(&text_a, &text_b);
    eprintln!("{}", r.render(3));
    assert!(r.identical(), "{}", r.render(5));
    assert!(r.max_abs_skew_ns() <= 1_000, "frames within a microsecond: {}", r.max_abs_skew_ns());
    let pings = r.streams.iter().find(|s| s.name.ends_with("host->board Ping")).unwrap();
    assert!(pings.matched >= 27);
    let status = r.streams.iter().find(|s| s.name.ends_with("board->host FirmwareStatus")).unwrap();
    assert!((status.period_a.unwrap().mean_ms - 1000.0).abs() < 1.0);

    // A different battery voltage changes the readings and nothing else
    let text_c = replayed(&log_a, end, |b| b.set("port_battery", 13.0));
    let r = compare(&text_a, &text_c);
    let changed: Vec<&str> = r.streams.iter().filter(|s| s.differs()).map(|s| s.name.as_str()).collect();
    assert_eq!(changed, ["can0 client 1 board->host Power::Electrical::ElectricalReadings"], "{}", r.render(3));
    let (_, a, b) = &r.streams.iter().find(|s| s.differs()).unwrap().payload_diffs[0];
    let change = candiff::payload_change(a, b);
    assert!(change.starts_with("port_voltage: 16.") && change.contains(" -> 12.99"), "{change}");

    // The bridge going away at 1.5 s: its pings, then the board's link traffic, are missing after that
    let short: Vec<LogFrame> = log_a.iter().filter(|f| f.time_ns < 1_500_000_000).cloned().collect();
    let text_d = replayed(&short, end, |_| {});
    let r = compare(&text_a, &text_d);
    let missing = |suffix: &str| r.streams.iter().find(|s| s.name.ends_with(suffix)).unwrap().missing.clone();
    let pings = missing("host->board Ping");
    assert!(pings.len() >= 12 && pings[0].0 >= 1_500_000_000, "{pings:?}");
    let link = missing("board->host Heartbeat");
    assert!(!link.is_empty(), "{}", r.render(3));
    // The firmware counts ROS as lost 700 ms after the last ping and stops its link heartbeat
    let last_ping = log_a.iter().rfind(|f| f.time_ns < 1_500_000_000).unwrap().time_ns;
    assert!(link[0].0 as u64 > last_ping && (link[0].0 as u64) < last_ping + 800_000_000, "{link:?}");
    // CANmore heartbeats go on without ROS
    assert!(missing("board->host heartbeat").is_empty());
}

#[test]
fn bridge_on_the_far_side_of_a_cannelloni_tunnel() {
    let Some(mut board) = power_board(false) else { return };
    let a = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    let b = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    let (a_addr, b_addr) = (a.local_addr().unwrap(), b.local_addr().unwrap());
    drop((a, b));
    board.session.can_endpoints.push(Box::new(Cannelloni::bind(a_addr, b_addr).unwrap()));
    // The robot computer: its own bus with the bridge and the tunnel's other end
    let rates = board.session.can_bus.as_ref().unwrap().rates;
    let mut remote = CanBus::new("vcan0", rates);
    let mut bridge = MockBridge::new(1, true, Time::ZERO);
    bridge.set_kill(Some(false));
    let watch = bridge.clone();
    let mut far = Cannelloni::bind(b_addr, a_addr).unwrap();
    let mut t = Time::ZERO;
    while t < Time::from_ms(2500) {
        t += Time::from_ms(1);
        board.run_until(t).unwrap();
        remote.step(&mut [], &mut [&mut bridge, &mut far], t);
    }
    assert!(board.logged("ROS connected"), "{:?}", board.session.logs);
    let st = watch.statuses();
    assert!(!st.is_empty(), "the bridge got the board's status through the tunnel");
    assert_eq!(st[0].1.board_name.as_str(), "mk2/power_board");
    assert!(watch.link_heartbeats().len() >= 15);
    assert!(watch.state().errors.is_empty());
    assert_eq!(far.rx_gaps, 0);
}

/// candump's text output for a frame (`candump -t a -x can0`): `(1700000000.123456)  can0  RX B -  123  [02]  AA BB`.
fn candump_text(time_ns: u64, iface: &str, f: &sim_devices::can::CanFrame) -> String {
    let id = if f.extended { format!("{:08X}", f.id) } else { format!("{:03X}", f.id) };
    let len = if f.fd { format!("[{:02}]", f.data.len()) } else { format!("[{}]", f.data.len()) };
    let data: Vec<String> = f.data.iter().map(|b| format!("{b:02X}")).collect();
    let flags = if f.fd {
        format!("{} {}", if f.brs { 'B' } else { '-' }, if f.esi { 'E' } else { '-' })
    } else {
        "- -".into()
    };
    format!(
        "({}.{:06})  {iface}  RX {flags}  {id}  {len}  {}",
        time_ns / 1_000_000_000,
        time_ns % 1_000_000_000 / 1000,
        data.join(" ")
    )
}

/// The workflow against a hardware recording: a log in candump's text format, with Unix timestamps, the board's
/// frames a little early or late, on `can0`. Its robot computer side is replayed into the emulated board, and the
/// recording and the emulator's log compare identical once lined up (searched for), apart from that jitter.
#[test]
fn hardware_style_recording() {
    let Some(bench) = power_board(true) else { return };
    let (_, text) = logged_run(bench, Time::from_ms(3000));
    let sim = canlog::parse(&text).unwrap();
    const EPOCH: u64 = 1_700_000_000_000_000_000;
    let mut seed = 12345u64;
    let mut hw = String::new();
    for f in &sim {
        let board = canmore::id::Fields::decode(f.frame.id, f.frame.extended).direction
            == canmore::id::Direction::ClientToAgent;
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        // Up to ±150 µs, never reordering frames that were sent back to back
        let jitter = if board { (seed >> 33) % 300_000 } else { 150_000 };
        hw.push_str(&candump_text(EPOCH + f.time_ns + jitter - 150_000, "can0", &f.frame));
        hw.push('\n');
    }
    let hw_log = canlog::parse(&hw).unwrap();
    assert_eq!(hw_log.len(), sim.len());
    assert!(hw_log.iter().zip(&sim).all(|(a, b)| a.frame == b.frame), "the text format round-trips frames");

    // Replay what the robot computer sent, starting when it did relative to power-on
    let mut b = power_board(false).unwrap();
    let rates = b.session.can_bus.as_ref().unwrap().rates;
    let first = Time::from_ns(sim[0].time_ns);
    let opts = ReplayOptions {
        ifaces: vec!["can0".into()],
        filters: vec![Filter::parse("sender=host").unwrap()],
        offset: first,
        rates: Some(rates),
        ..Default::default()
    };
    b.session.can_endpoints.push(Box::new(Replay::from_log(&hw_log, &opts)));
    let (_, emulated) = logged_run(b, Time::from_ms(3000));
    let mut emu = canlog::parse(&emulated).unwrap();
    for f in &mut emu {
        f.iface = "can0".into();
    }
    let opts = DiffOptions { align: candiff::Align::Auto, ..Default::default() };
    let r = candiff::diff(&candiff::decode(&hw_log, false), &candiff::decode(&emu, false), &opts);
    eprintln!("{}", r.render(3));
    assert!(r.identical(), "{}", r.render(5));
    assert!((r.offset_ns + EPOCH as i64).abs() < 200_000, "offset {}", r.offset_ns);
    assert!(r.max_abs_skew_ns() < 400_000, "{}", r.max_abs_skew_ns());
    let hb = r.streams.iter().find(|s| s.name.ends_with("board->host Heartbeat")).unwrap();
    let jitter = hb.period_a.unwrap().std_ms;
    assert!(jitter > 0.05 && hb.period_b.unwrap().std_ms < 0.05, "the recording's jitter shows: {jitter}");
}
