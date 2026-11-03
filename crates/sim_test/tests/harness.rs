//! M7: the test harness itself (snapshots, fault injection, board edits) and the boards that need more than the
//! smoke test: the H-bridge board's DRV8243 wake-up, and the Dynamixel bench test without a CAN bus.

use sim_devices::drv8243::{Drv8243, State};
use sim_test::{Bench, Time};

#[test]
fn a_snapshot_runs_exactly_like_the_original() {
    let Some(elf) = sim_test::titan_rs("power_board") else { return };
    let mut a = Bench::builder(&elf).board("mk2/power_board").bridge(1).record_bus().boot().unwrap();
    a.run_until(Time::from_ms(800)).unwrap();
    let mut b = a.snapshot();
    a.run_until(Time::from_ms(2500)).unwrap();
    b.run_until(Time::from_ms(2500)).unwrap();
    assert_eq!(a.now(), b.now());
    let frames = |x: &Bench| x.frames().iter().map(|r| (r.start, r.frame.id, r.frame.data.clone())).collect::<Vec<_>>();
    assert_eq!(frames(&a), frames(&b), "the same CAN traffic, frame for frame");
    let logs = |x: &Bench| x.session.logs.iter().map(|(t, l)| (*t, l.message.clone())).collect::<Vec<_>>();
    assert_eq!(logs(&a), logs(&b));
    assert_eq!(a.messages().len(), b.messages().len());
    assert_eq!(a.session.machine.cores[0].reg(15), b.session.machine.cores[0].reg(15));
    // And they are independent: a panic in one leaves the other running
    b.inject_panic().unwrap();
    b.run_for(Time::from_ms(500));
    a.run_for(Time::from_ms(500));
    assert!(b.logged("PANIC") && !a.logged("PANIC"));
}

fn hbridge(elf: &std::path::Path, fitted: [bool; 4]) -> Bench {
    let mut b = Bench::builder(elf)
        .board("lilboat/hbridge_board")
        .bridge(2)
        .edit_board(move |board| {
            for (br, f) in board.drv8243[0].bridges.iter_mut().zip(fitted) {
                br.fitted = f;
            }
        })
        .boot()
        .unwrap();
    b.run_until(Time::from_ms(1500)).unwrap();
    b
}

#[test]
fn hbridge_board_wakes_its_drv8243s() {
    let Some(elf) = sim_test::titan_rs("hbridge_board") else { return };
    let b = hbridge(&elf, [true; 4]);
    let drv = b.session.machine.sys.pin_device::<Drv8243>().unwrap();
    assert_eq!(drv.state, State::Active, "woken when the vehicle was enabled");
    assert!(drv.wakeups >= 1);
    assert!(!drv.outputs_off, "DRVOFF released");
    assert_eq!(b.statuses().last().unwrap().1.faults, 0);
    assert!(!b.logged("hbridges found"));

    // H3 (behind the mux) missing: FAULT_HBRIDGE_DISCONNECTED, and the firmware says how many it found
    let b = hbridge(&elf, [true, true, false, true]);
    assert_eq!(b.statuses().last().unwrap().1.faults, 1 << 5);
    assert!(b.logged("Only 3 hbridges found. Expected 4"), "{:?}", b.session.logs);
}

#[test]
fn dynamixel_bench_test_runs_without_a_bus() {
    let Some(elf) = sim_test::titan_rs("dynamixel_test") else { return };
    let mut b = Bench::builder(&elf).boot().unwrap();
    b.run_until(Time::from_ms(2000)).unwrap();
    assert!(!b.logged("PANIC"));
    assert!(!b.session.logs.is_empty(), "it logs over RTT");
    eprintln!("{:#?}", b.session.logs);
}
