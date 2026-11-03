//! The sampling profiler against programs whose hot spots are known: `test_programs/profile.S` (a fixed share of
//! cycles per function, a SysTick handler, a function without call frame information) and, when built, real
//! titan_firmware_rs images.

mod common;

use sim_core::cpu::Cpu;
use sim_core::time::Time;
use sim_host::profile::Profile;

fn profile_program(time: Time, interval: Time) -> Profile {
    let mut s = common::boot(&common::repo().join("test_programs/profile.elf"), None);
    s.start_profile(interval);
    assert_eq!(s.run_for(time), rp2040::StopReason::TimeLimit);
    s.profiler.as_ref().unwrap().report()
}

fn stack_samples(p: &Profile, core: usize, stack: &[&str]) -> u64 {
    p.cores[core].stacks.iter().filter(|(s, _)| s == stack).map(|(_, n)| n).sum()
}

#[test]
fn time_is_attributed_to_the_functions_that_spend_it() {
    let p = profile_program(Time::from_ms(300), Time::from_us(10));
    let c0 = &p.cores[0];
    assert!(p.unwind_info);
    // About 30000 samples at 10 µs over 300 ms
    assert!((28_000..32_000).contains(&c0.samples()), "{} samples", c0.samples());
    assert_eq!(c0.sleeping, 0);
    // Per 20000-cycle SysTick period the handler takes about 616 cycles (entry, loop, return); the rest is split
    // 9000 : 3000 : 1500 (+ a few cycles of calls) between hot, warm and nocfi
    let handler = 616.0 / 20000.0;
    let main = 1.0 - handler;
    let want = [
        ("hot", main * 9000.0 / 13530.0),
        ("warm", main * 3000.0 / 13530.0),
        ("nocfi", main * 1500.0 / 13530.0),
        ("systick", handler),
    ];
    for (f, share) in want {
        let got = c0.self_fraction(f);
        assert!((got - share).abs() < 0.02, "{f}: {got:.3} of the time, expected {share:.3}\n{}", p.flat_text("t", 20));
    }
    assert!(c0.function("hot").unwrap().self_samples > c0.functions[1].self_samples, "hot dominates");
    // Everything runs under reset and outer
    assert!(
        c0.total_fraction("reset") > 0.99,
        "{}: {:#?}",
        c0.total_fraction("reset"),
        &c0.stacks[..c0.stacks.len().min(8)]
    );
    assert!((c0.total_fraction("outer") - main).abs() < 0.02 + handler);
    // Stacks: the callers are found through CFI; the function without CFI through LR
    let total = c0.running as f64;
    let hot = stack_samples(&p, 0, &["reset", "outer", "hot"]) as f64 / total;
    assert!((hot - main * 9000.0 / 13530.0).abs() < 0.02, "{hot}");
    assert!(stack_samples(&p, 0, &["reset", "outer", "nocfi"]) > 0);
    // The handler is unwound into whatever it interrupted
    let in_handler: u64 = c0.stacks.iter().filter(|(s, _)| s.last().is_some_and(|f| f == "systick")).map(|x| x.1).sum();
    let interrupting_hot = stack_samples(&p, 0, &["reset", "outer", "hot", "systick"]);
    let interrupting_nocfi = stack_samples(&p, 0, &["reset", "outer", "nocfi", "systick"]);
    assert!(interrupting_hot > in_handler / 3, "{interrupting_hot} of {in_handler}: {:#?}", c0.stacks);
    assert!(interrupting_nocfi > 0, "{:#?}", c0.stacks);
    assert_eq!(c0.truncated, 0, "{:#?}", c0.stacks.iter().filter(|s| s.0[0] == "[unknown]").collect::<Vec<_>>());
    // Exception accounting: SysTick is exception 15
    let systick: u64 = c0.exceptions.iter().filter(|e| e.0 == 15).map(|e| e.1).sum();
    assert!((systick as f64 / total - handler).abs() < 0.01);
}

#[test]
fn a_parked_core_sleeps_in_the_bootrom() {
    let p = profile_program(Time::from_ms(20), Time::from_us(10));
    let c1 = &p.cores[1];
    assert!(c1.samples() > 1500);
    assert_eq!(c1.running, 0);
    assert_eq!(c1.sleeping, c1.samples());
    // Its stack ends in [sleep], inside the bootrom
    assert!(
        c1.stacks.iter().all(|(s, _)| s.last().unwrap() == "[sleep]" && s[0].starts_with("rom:")),
        "{:?}",
        c1.stacks
    );
    let text = p.flat_text("profile.elf", 10);
    assert!(text.contains("## core 1:") && text.contains("sleeping (WFI/WFE) 100.0%"), "{text}");
}

#[test]
fn reports_are_flat_tables_and_folded_stacks() {
    let p = profile_program(Time::from_ms(50), Time::from_us(20));
    let text = p.flat_text("profile.elf", 10);
    let hot = text.lines().find(|l| l.ends_with(" hot")).expect(&text);
    let cols: Vec<f64> = hot.split_whitespace().take(3).map(|c| c.parse().unwrap()).collect();
    assert!(cols[0] > 55.0 && cols[1] >= cols[0], "{hot}");
    assert!(text.contains("in exception handlers: SysTick"), "{text}");
    let folded = p.folded();
    let mut sum = 0u64;
    for line in folded.lines() {
        let (stack, n) = line.rsplit_once(' ').unwrap();
        assert!(stack.starts_with("core0;") || stack.starts_with("core1;"), "{line}");
        sum += n.parse::<u64>().unwrap();
    }
    assert_eq!(sum, p.cores[0].awake_or_sleeping() + p.cores[1].awake_or_sleeping());
    assert!(folded.lines().any(|l| l.starts_with("core0;reset;outer;hot ")), "{folded}");
    // Written to disk: the table and the folded stacks next to it
    let dir = std::env::temp_dir().join(format!("profile-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let folded_path = p.write(&dir.join("p.txt"), "profile.elf").unwrap();
    assert_eq!(folded_path, dir.join("p.folded"));
    assert_eq!(std::fs::read_to_string(&folded_path).unwrap(), folded);
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn profiling_does_not_change_what_a_single_core_program_does() {
    let path = common::repo().join("test_programs/profile.elf");
    let mut plain = common::boot(&path, None);
    plain.run_for(Time::from_ms(10));
    let mut profiled = common::boot(&path, None);
    profiled.start_profile(Time::from_us(3));
    profiled.run_for(Time::from_ms(10));
    assert_eq!(plain.machine.cores[0].reg(15), profiled.machine.cores[0].reg(15));
    assert_eq!(plain.machine.cores[0].cycles(), profiled.machine.cores[0].cycles());
}

/// Boots a titan_firmware_rs image on its board, on a CAN bus with a mock bridge (so it runs as on the robot).
fn on_board(elf: &std::path::Path, board: &str, client: u8) -> sim_host::Session {
    let b = sim_host::board::Board::load(&common::repo().join("boards").join(format!("{board}.toml"))).unwrap();
    let mut s = common::boot(elf, None);
    b.apply(&mut s.machine).unwrap();
    s.can_bus = Some(sim_devices::can::CanBus::new("can0", b.can_rates().unwrap()));
    let bridge = sim_host::mock_bridge::MockBridge::new(client, true, Time::ZERO);
    bridge.set_kill(Some(false));
    s.can_endpoints.push(Box::new(bridge));
    s
}

fn assert_walked(p: &Profile, core: usize, title: &str) {
    let c = &p.cores[core];
    assert!(
        (c.truncated as f64) < 0.01 * c.samples() as f64,
        "core {core}: {} of {} stacks truncated: {:#?}\n{}",
        c.truncated,
        c.samples(),
        c.stacks.iter().filter(|s| s.0[0] == "[unknown]").take(10).collect::<Vec<_>>(),
        p.flat_text(title, 30)
    );
}

/// Rust firmware: the embassy executor sleeps most of the time; every stack is walked to its root through the
/// image's CFI and Rust names come out demangled.
#[test]
fn template_firmware_stacks_reach_the_root() {
    let Some(elf) = common::titan_elf("template") else { return };
    let mut s = on_board(&elf, "mk2/power_board", 1);
    s.start_profile(Time::from_us(20));
    s.run_for(Time::from_ms(1500));
    let p = s.profiler.as_ref().unwrap().report();
    let c0 = &p.cores[0];
    assert!(p.unwind_info);
    assert!(c0.sleeping > 4 * c0.running, "an idle embassy board mostly sleeps: {}", p.flat_text("template", 20));
    assert_walked(&p, 0, "template");
    assert!(c0.functions.iter().all(|f| !f.name.starts_with("_R") && !f.name.starts_with("_ZN")), "{:?}", c0.functions);
    // It sleeps in the executor, called from main, called from the reset handler
    let sleeping: Vec<&Vec<String>> =
        c0.stacks.iter().filter(|s| s.0.last().unwrap() == "[sleep]").map(|s| &s.0).collect();
    assert!(sleeping.iter().all(|s| s[0] == "Reset"), "{sleeping:#?}");
    assert!(sleeping.iter().all(|s| s.iter().any(|f| f.contains("Executor>::run"))), "{sleeping:#?}");
}

/// The ESC board runs its control loop on core 1 (embassy multicore); both cores' stacks are walked.
#[test]
fn esc_board_profiles_both_cores() {
    let Some(elf) = common::titan_elf("esc_board") else { return };
    let mut s = on_board(&elf, "mk2/esc_board", 2);
    s.start_profile(Time::from_us(20));
    s.run_for(Time::from_ms(1500));
    let p = s.profiler.as_ref().unwrap().report();
    assert!(p.cores[1].running > 0, "core 1 runs firmware: {}", p.flat_text("esc_board", 30));
    assert_walked(&p, 0, "esc_board");
    assert_walked(&p, 1, "esc_board");
    assert!(p.cores[1].functions.iter().any(|f| f.name.contains("core1")), "{}", p.flat_text("esc_board", 30));
}

/// IVCBoard2 listens all the time: its receive task runs the spectrum analyzer over every block of ADC samples, in
/// soft float (the bootrom's `fadd`/`fmul` through embassy-rp's intrinsics). That is where the profile must put
/// the time.
#[test]
fn ivc_board2_time_is_in_the_receive_spectrum() {
    let Some(elf) = common::titan_elf("ivc_board2") else { return };
    let mut s = on_board(&elf, "talos/ivc_board", 8);
    s.start_profile(Time::from_us(20));
    s.run_for(Time::from_ms(1000));
    let p = s.profiler.as_ref().unwrap().report();
    let c0 = &p.cores[0];
    let text = p.flat_text("ivc_board2", 15);
    assert!(c0.running > 3 * c0.sleeping, "IVC2 keeps core 0 busy: {text}");
    assert_walked(&p, 0, "ivc_board2");
    let rx = c0.functions.iter().find(|f| f.name.contains("modem::__rx_task_task")).expect(&text);
    assert!(rx.total_samples as f64 > 0.7 * c0.running as f64, "{text}");
    assert!(c0.total_fraction("prepare") > 0.2, "{text}");
    let top3: Vec<&str> = c0.functions.iter().take(3).map(|f| f.name.as_str()).collect();
    assert!(top3.contains(&"rom:fadd"), "soft float addition dominates: {top3:?}");
}
