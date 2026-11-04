//! Replaying a "real" pool test: a log recorded from one emulator run (with a perturbation the replay does not
//! have: the port battery changed at 10 s and the CAN cable pulled from 15 s to 15.5 s), made to look like a
//! hardware recording (Unix timestamps from the robot computer's clock, which runs 500 ppm fast, started after
//! the boards booted, with a board that is not in the robot file, an identifier that is not CANmore, bus errors and
//! a damaged line), replayed into a fresh run of the same robot. The report must find exactly the perturbation.
//! (About 2 s: the power board runs far faster than real time.)

use std::path::PathBuf;

use canmore::{Direction, FrameType};
use sim_calib::pool::{self, LogFile, PoolOptions};
use sim_devices::can::CanFrame;
use sim_host::canlog::{self, LogFrame};
use sim_host::fault::Fault;
use sim_test::{RobotBench, Time};

const DRIFT: f64 = 500e-6;
const EPOCH_NS: u64 = 1_762_000_000_000_000_000;
/// When the robot computer started candump, in the "real" run's time.
const LOG_START_S: f64 = 0.8;

fn dir() -> PathBuf {
    let d = std::env::temp_dir().join(format!("pool-replay-{}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    d
}

fn robot_file(elf: &std::path::Path) -> PathBuf {
    let repo = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let p = dir().join("power.toml");
    std::fs::write(
        &p,
        format!(
            "[[bus]]\nname = \"internal\"\n\n[[board]]\nname = \"power\"\nboard = \"{}\"\nelf = \"{}\"\nbus = \"internal\"\n",
            repo.join("boards/mk2/power_board.toml").display(),
            elf.display()
        ),
    )
    .unwrap();
    p
}

/// The "real" run and its log as candump on the robot computer would have written it.
fn real_log(robot: &std::path::Path) -> String {
    let mut r = RobotBench::load(robot.to_str().unwrap(), &[("internal", 1)], |_| {}).unwrap();
    r.robot.buses[0].bus.record = Some(Vec::new());
    r.inject_at(Time::from_ms(15_000), "power", Fault::CanUnplugged(true));
    r.inject_at(Time::from_ms(15_500), "power", Fault::CanUnplugged(false));
    r.run_until(Time::from_secs(10));
    r.robot.inputs[0].set("port_battery", 13.0);
    r.run_until(Time::from_secs(30));
    let mut log = canlog::from_records(r.robot.buses[0].bus.record.as_deref().unwrap(), "can0");
    // A board the robot file does not have (client 9's heartbeats) and an identifier that is not CANmore
    for k in 0..200 {
        let t = 1.0 + k as f64 * 0.1;
        let hb =
            canmore::Heartbeat { cnt: (k % 4) as u8, error: false, mode: 1, term_valid: false, term_enabled: false };
        let id = canmore::id::std_id(9, FrameType::Util, Direction::ClientToAgent, canmore::CHAN_HEARTBEAT);
        log.push(LogFrame {
            time_ns: (t * 1e9) as u64,
            iface: "can0".into(),
            frame: CanFrame::classic(id, false, &[hb.to_byte()]),
        });
        if k % 10 == 0 {
            log.push(LogFrame {
                time_ns: (t * 1e9) as u64 + 3_000_000,
                iface: "can0".into(),
                frame: CanFrame::classic(0x7E0, false, &[1, 2]),
            });
        }
    }
    log.sort_by_key(|f| f.time_ns);
    log.retain(|f| f.time_ns as f64 / 1e9 >= LOG_START_S);
    // The robot computer's clock
    for f in &mut log {
        f.time_ns = EPOCH_NS + ((f.time_ns as f64 * (1.0 + DRIFT)) / 1000.0).round() as u64 * 1000;
    }
    let mut text = canlog::to_text(&log);
    let at = |s: f64| format!("{:.6}", (EPOCH_NS as f64 + s * 1e9 * (1.0 + DRIFT)) / 1e9);
    text += &format!("({}) can0 20000080#0000000000000000\n", at(15.1));
    text += &format!("({}) can0 20000004#0004000000000000\n", at(15.2));
    text += "(1762000020.1234\n";
    // A third interface the robot has no bus for
    text += &format!("({}) can7 123#00\n", at(5.0));
    text
}

#[test]
fn a_replayed_pool_test_finds_exactly_the_perturbation() {
    let Some(elf) = sim_test::titan_rs("power_board") else { return };
    let robot = robot_file(&elf);
    let path = dir().join("can0.log");
    std::fs::write(&path, real_log(&robot)).unwrap();
    let log = LogFile::read(&path).unwrap();
    assert_eq!(log.stats.error_frames.len(), 2);
    assert_eq!(log.stats.bad_lines.len(), 1);
    let spec = sim_host::bridges::read_spec(&robot).unwrap();
    // The real board had met the host long before the log started; the emulated one meets it at the log's start
    // and answers its first messages differently: compare from a second in
    let opts = PoolOptions { compare_after: Time::from_secs(1), ..Default::default() };
    let res = pool::replay(&spec, robot.parent().unwrap(), &[log], &opts).unwrap();
    let md = &res.markdown;
    // The clock: the log's first frame at 0.8 s, the drift found from the uptimes
    let c = &res.clocks[0];
    assert!((c.drift_ppm + DRIFT * 1e6 / (1.0 + DRIFT)).abs() < 30.0, "{c:?}\n{md}");
    let first = res.clock.to_virtual_ns(EPOCH_NS + (LOG_START_S * 1e9 * (1.0 + DRIFT)) as u64) / 1e9;
    assert!((first - LOG_START_S).abs() < 0.003, "first frame mapped to {first} s\n{md}");
    // Exactly the perturbation: the port voltage from 10 s on ...
    let mut found_payload = false;
    for s in &res.report.streams {
        for (t, a, b) in &s.payload_diffs {
            assert!(s.name.contains("ElectricalReadings"), "{}: {a} vs {b}\n{md}", s.name);
            assert!(*t as f64 / 1e9 > 10.0, "{} at {t}\n{md}", s.name);
            assert!(a.contains("port_voltage") && b.contains("port_voltage"));
            found_payload = true;
        }
        // ... and the pulled cable: what the emulated board sent while the real one was off the bus
        for (t, _) in s.extra.iter().chain(&s.missing) {
            let t = *t as f64 / 1e9;
            assert!((14.9..16.5).contains(&t), "{} at {t} s\n{md}", s.name);
        }
    }
    assert!(found_payload, "{md}");
    let (_, extra, payload) = res.report.counts();
    assert!(extra > 0 && payload >= 15, "{md}");
    assert!(!res.identical);
    assert!(res.fault_diffs.is_empty(), "{:?}", res.fault_diffs);
    // The rest of the log is accounted for
    assert!(md.contains("2 error frame(s)") && md.contains("1 unreadable line(s)"), "{md}");
    assert!(md.contains("internal client 9 (") && md.contains("internal client 31 ("), "{md}");
    assert!(md.contains("powered on 0.800 s before") || md.contains("powered on 0.80"), "{md}");
    assert!(md.contains("**DIFFERENT**"));
    assert!(md.contains("1 frame(s) on can7 are not replayed"), "{md}");

    // The same log replayed without the perturbation's window, and the run that made it, compared as recorded:
    // the emulator reproduces the real run where nothing was changed
    let opts =
        PoolOptions { max_time: Some(Time::from_secs(9)), compare_after: Time::from_secs(1), ..Default::default() };
    let log = LogFile::read(&path).unwrap();
    let res = pool::replay(&spec, robot.parent().unwrap(), &[log], &opts).unwrap();
    assert!(res.identical, "{}", res.markdown);
    assert!(res.report.streams.iter().any(|s| s.matched > 50), "{}", res.markdown);
}
