//! Waveform export: VCD files of known waveforms, read back with the VCD parser.

mod common;

use sim_core::time::Time;
use sim_host::vcd::{Scope, Vcd, VcdOptions, parse};

fn temp(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("vcd-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

/// Durations between successive changes of a wire, with the level held during each.
fn segments(changes: &[(u64, String)]) -> Vec<(u64, String)> {
    changes.windows(2).map(|w| (w[1].0 - w[0].0, w[0].1.clone())).collect()
}

#[test]
fn blink_toggles_gpio25_in_the_waveform() {
    let path = temp("blink.vcd");
    let mut s = common::boot(&common::repo().join("test_programs/blink.elf"), None);
    let opts = VcdOptions { pins: vec![24, 25], events: false };
    let vcd = Vcd::create(&path, &[Scope::board("blink", &Default::default(), &opts)]).unwrap();
    vcd.attach(0, &mut s);
    s.run_for(Time::from_ms(20));
    assert_eq!(vcd.finish(s.machine.now).unwrap(), 0);
    let p = parse(&std::fs::read_to_string(&path).unwrap()).unwrap();
    assert_eq!(p.timescale, "1ns");
    let g25 = p.signal("blink", "gpio25").expect("gpio25 declared");
    // Unknown at 0, then the reset level, then a toggle every 3005 cycles of the ROSC
    assert_eq!(g25[0], (0, "x".to_string()));
    assert_eq!(g25[1].1, "0");
    let period_ns = s.machine.clk_sys().cycles(3005).as_ps() as f64 / 1000.0;
    let toggles = &g25[2..];
    assert!(toggles.len() >= 40, "{} toggles", toggles.len());
    for w in toggles.windows(2) {
        assert_ne!(w[0].1, w[1].1, "levels alternate");
        let dt = (w[1].0 - w[0].0) as f64;
        assert!((dt - period_ns).abs() <= 1.0, "toggle interval {dt} ns, expected {period_ns}");
    }
    // A pin nobody drives stays at its reset level
    assert_eq!(p.signal("blink", "gpio24").unwrap().len(), 2);
    // The file ends at the end of the run (the machine stops at the first instruction boundary after 20 ms)
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(text.trim_end().ends_with(&format!("#{}", s.machine.now.as_ns())), "{}", &text[text.len() - 100..]);
}

#[test]
fn several_boards_interleave_in_time_order() {
    // Two blinking boards in a robot run one after the other in slices; the file must still be in time order
    let path = temp("two.vcd");
    let mut r = sim_host::robot::Robot::default();
    for n in ["a", "b"] {
        r.add(n, common::boot(&common::repo().join("test_programs/blink.elf"), None), None);
    }
    r.power_on[1] = (Time::from_us(1234), false);
    let opts = VcdOptions { pins: vec![25], events: false };
    let scopes: Vec<Scope> = r.names.iter().map(|n| Scope::board(n, &Default::default(), &opts)).collect();
    let vcd = Vcd::create(&path, &scopes).unwrap();
    for (i, s) in r.boards.iter_mut().enumerate() {
        vcd.attach(i, s);
    }
    r.run_until(Time::from_ms(10));
    assert_eq!(vcd.finish(r.now).unwrap(), 0, "nothing arrived too late");
    let p = parse(&std::fs::read_to_string(&path).unwrap()).expect("time never goes backwards");
    let a = p.signal("a", "gpio25").unwrap();
    let b = p.signal("b", "gpio25").unwrap();
    assert!(a.len() > 20 && b.len() > 20);
    // Board b was switched on 1.234 ms later, so its first toggle comes that much later
    let first = |s: &[(u64, String)]| s.iter().find(|c| c.1 == "1").unwrap().0;
    assert_eq!(first(b) - first(a), 1_234_000);
}

/// Boots a titan_firmware_rs image on its board with a mock bridge for `client`.
fn on_board(elf: &std::path::Path, board: &sim_host::board::Board, client: u8) -> sim_host::Session {
    let mut s = common::boot(elf, None);
    board.apply(&mut s.machine).unwrap();
    s.can_bus = Some(sim_devices::can::CanBus::new("can0", board.can_rates().unwrap()));
    let bridge = sim_host::mock_bridge::MockBridge::new(client, true, Time::ZERO);
    bridge.set_kill(Some(false));
    s.can_endpoints.push(Box::new(bridge));
    s
}

/// The camera cage's WS2812 status strip, driven by PIO on GPIO 9: 800 kbit/s bits (1.25 µs), and the frames the
/// strip model decodes as device events.
#[test]
fn camera_cage_status_strip_waveform() {
    let Some(elf) = common::titan_elf("camera_cage_bb") else { return };
    let path = temp("cage.vcd");
    let board = sim_host::board::Board::load(&common::repo().join("boards/mk2/camera_cage_bb.toml")).unwrap();
    let mut s = on_board(&elf, &board, 4);
    let opts = VcdOptions { pins: vec![9], events: true };
    let vcd = Vcd::create(&path, &[Scope::board("cage", &board.pins, &opts)]).unwrap();
    vcd.attach(0, &mut s);
    s.run_for(Time::from_ms(300));
    vcd.finish(s.machine.now).unwrap();
    let p = parse(&std::fs::read_to_string(&path).unwrap()).unwrap();
    let strip = p.signal("cage", &sim_host::vcd::pin_labels(&board.pins)[9]).unwrap();
    // Within a frame each bit is a high pulse then a low gap, 1.25 µs together
    let seg = segments(strip);
    let bits = seg.windows(2).filter(|w| w[0].1 == "1" && (1200..=1300).contains(&(w[0].0 + w[1].0))).count();
    assert!(bits >= 24 * 12, "{bits} WS2812 bits");
    let device = p.signal("cage", "device").unwrap();
    assert!(device.iter().any(|(_, d)| d.contains("status_strip_(GPIO_9):_12_LEDs")), "{device:?}");
}

/// The ESC board's PIO DShot600 output on GPIO 6 (bidirectional, so inverted: a 0 bit is low for 625 ns, a 1 bit
/// for 1250 ns, every 1667 ns) and the ESC model's eRPM reply (GCR at 750 kbit/s, 1333 ns per bit), plus the
/// decoded logs and CAN frames.
#[test]
fn esc_board_dshot_waveform_and_events() {
    let Some(elf) = common::titan_elf("esc_board") else { return };
    let path = temp("esc.vcd");
    let board = sim_host::board::Board::load(&common::repo().join("boards/mk2/esc_board.toml")).unwrap();
    let mut s = on_board(&elf, &board, 2);
    let pins = sim_host::vcd::parse_pins("esc1_pwm,esc1_telem,status_ledg", &board.pins).unwrap();
    let opts = VcdOptions { pins, events: true };
    let vcd = Vcd::create(&path, &[Scope::board("esc", &board.pins, &opts)]).unwrap();
    vcd.attach(0, &mut s);
    s.run_for(Time::from_ms(400));
    vcd.finish(s.machine.now).unwrap();
    let p = parse(&std::fs::read_to_string(&path).unwrap()).unwrap();
    let dshot = p.signal("esc", "gpio6_esc1_pwm").expect("named after the board's pin");
    let seg = segments(dshot);
    let bits: Vec<u64> = seg
        .windows(2)
        .filter(|w| w[0].1 == "0" && w[1].1 == "1" && (1640..=1690).contains(&(w[0].0 + w[1].0)))
        .map(|w| w[0].0)
        .collect();
    assert!(bits.len() > 16 * 100, "{} DShot bits", bits.len());
    for b in &bits {
        assert!(b.abs_diff(625) <= 16 || b.abs_diff(1250) <= 16, "low time {b} ns is neither a 0 nor a 1 bit");
    }
    let replies = seg.iter().filter(|(d, _)| *d % 1333 <= 2 || 1333 - *d % 1333 <= 2).count();
    assert!(replies > 100, "{replies} eRPM reply bit cells");
    assert!(p.signal("esc", "gpio7_esc1_telem").is_some());
    let logs = p.signal("esc", "log").unwrap();
    assert!(logs.iter().any(|(_, l)| l.contains("CAN_controller_online")), "{logs:?}");
    let can = p.signal("esc", "can").unwrap();
    assert!(can.len() >= 5 && can.iter().all(|(_, f)| f.contains('#')), "{can:?}");
    // DShot commands decoded by the ESC model
    assert!(p.signal("esc", "device").is_some());
}
