//! M7: a smoke test for every titan_firmware_rs board image. Each boots on its board (with the devices of its board
//! file) next to a mock `titan_bridge`, and must connect, report its firmware status with the right identity, keep
//! its link clean and not panic or lock up. Faults raised by a board are listed with the reason they are expected.

use sim_test::{Bench, Time};

struct Case {
    elf: &'static str,
    board: &'static str,
    client: u8,
    /// The board name the firmware reports.
    name: &'static str,
    /// Faults expected to be raised (by bit), and why.
    faults: u32,
}

const CASES: &[Case] = &[
    Case { elf: "template", board: "mk2/power_board", client: 1, name: "mk2/power_board", faults: 0 },
    Case { elf: "power_board", board: "mk2/power_board", client: 1, name: "mk2/power_board", faults: 0 },
    Case { elf: "esc_board", board: "mk2/esc_board", client: 2, name: "mk2/esc_board", faults: 0 },
    Case { elf: "camera_cage_bb", board: "mk2/camera_cage_bb", client: 4, name: "mk2/camera_cage_bb", faults: 0 },
    Case { elf: "actuator_board", board: "mk2/actuator_board", client: 5, name: "mk2/actuator_board", faults: 0 },
    Case { elf: "led_board", board: "talos/led_board", client: 6, name: "talos/led_board", faults: 0 },
    Case { elf: "led_board2", board: "talos/led_board", client: 6, name: "talos/led_board", faults: 0 },
    Case { elf: "sbc_board", board: "talos/sbc_board", client: 7, name: "talos/sbc_board", faults: 0 },
    Case { elf: "ivc_board", board: "talos/ivc_board", client: 8, name: "talos/ivc_board", faults: 0 },
    Case { elf: "ivc_board2", board: "talos/ivc_board", client: 8, name: "talos/ivc_board", faults: 0 },
    Case { elf: "ivc_dac", board: "talos/ivc_board", client: 8, name: "talos/ivc_board", faults: 0 },
    Case { elf: "poac_board", board: "talos/poac_board", client: 1, name: "talos/poac_board", faults: 0 },
    Case { elf: "smart_battery", board: "mk2/sbh_mcu", client: 21, name: "mk2/sbh_mcu", faults: 0 },
    Case { elf: "hbridge_board", board: "lilboat/hbridge_board", client: 2, name: "lilboat/hbridge_board", faults: 0 },
    Case { elf: "po_half_board", board: "lilboat/po_half", client: 1, name: "lilboat/po_half", faults: 0 },
];

#[derive(Debug)]
struct Report {
    elf: &'static str,
    connected: Option<Time>,
    statuses: usize,
    name: String,
    faults: u32,
    link_errors: Vec<String>,
    panicked: bool,
    wall: std::time::Duration,
}

fn smoke(c: &Case) -> Option<Report> {
    let elf = sim_test::titan_rs(c.elf)?;
    let wall = std::time::Instant::now();
    let mut b = Bench::builder(&elf).board(c.board).bridge(c.client).boot().unwrap();
    if let Err(e) = b.run_until(Time::from_ms(2500)) {
        panic!("{} stopped at {}: {e:?}", c.elf, b.now());
    }
    let st = b.statuses();
    let last = st.last().map(|s| s.1.clone());
    Some(Report {
        elf: c.elf,
        connected: b.log_time("ROS connected"),
        statuses: st.len(),
        name: last.as_ref().map_or(String::new(), |s| s.board_name.to_string()),
        faults: last.map_or(u32::MAX, |s| s.faults),
        link_errors: b.link_errors(),
        panicked: b.logged("PANIC"),
        wall: wall.elapsed(),
    })
}

#[test]
fn every_board_boots_and_connects() {
    let reports: Vec<(&Case, Report)> = std::thread::scope(|s| {
        let handles: Vec<_> = CASES.iter().map(|c| (c, s.spawn(move || smoke(c)))).collect();
        handles.into_iter().filter_map(|(c, h)| h.join().unwrap().map(|r| (c, r))).collect()
    });
    eprintln!("board               connected   statuses  faults      wall");
    for (_, r) in &reports {
        eprintln!(
            "{:<18}  {:>9}  {:>9}  {:#010x}  {:>6.2?}  {}",
            r.elf,
            r.connected.map_or("never".to_string(), |t| t.to_string()),
            r.statuses,
            r.faults,
            r.wall,
            r.name
        );
    }
    for (c, r) in &reports {
        assert!(r.connected.is_some(), "{}: never connected", c.elf);
        assert!(r.statuses >= 2, "{}: {} firmware statuses", c.elf, r.statuses);
        assert_eq!(r.name, c.name, "{}", c.elf);
        assert_eq!(r.faults, c.faults, "{}: faults {:#x}", c.elf, r.faults);
        assert!(r.link_errors.is_empty(), "{}: {:?}", c.elf, r.link_errors);
        assert!(!r.panicked, "{} panicked", c.elf);
    }
}
