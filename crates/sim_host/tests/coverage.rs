//! Code coverage: a small program with known branches gives exact lcov lines and functions; a titan_firmware_rs
//! image gives plausible coverage (its panic handler unrun until a panic is injected); coverage on or off, the run
//! is the same.

mod common;

use sim_core::Time;
use sim_host::coverage::Coverage;

fn coverage_program() -> std::path::PathBuf {
    common::repo().join("test_programs/coverage.elf")
}

#[test]
fn branches_give_exact_lines_and_functions() {
    let mut s = common::boot(&coverage_program(), None);
    s.machine.enable_coverage();
    s.run_until(Time::from_ms(1));
    let mut cov = Coverage::new();
    cov.add_machine("coverage.elf", &s.machine);
    let r = cov.report();
    let (file, f) = r.files.iter().find(|(f, _)| f.ends_with("test_programs/coverage.S")).expect("the source file");
    let hit: Vec<u32> = f.lines.iter().filter(|(_, h)| **h).map(|(l, _)| *l).collect();
    let missed: Vec<u32> = f.lines.iter().filter(|(_, h)| !**h).map(|(l, _)| *l).collect();
    // reset: movs, cmp, beq; the taken branch, the call loop, wfi; helper
    assert_eq!(hit, [18, 19, 20, 25, 27, 28, 29, 31, 39, 40]);
    // not_taken, the `b done` after wfi (it sleeps for good), unused
    assert_eq!(missed, [22, 23, 32, 47, 48]);
    let funcs: Vec<(&str, u32, bool)> = f.functions.values().map(|x| (x.name.as_str(), x.line, x.hit)).collect();
    assert_eq!(funcs, [("helper", 39, true), ("reset", 18, true), ("unused", 47, false)]);
    let lcov = r.lcov("coverage");
    let expected = format!(
        "TN:coverage\nSF:{file}\nFN:39,helper\nFN:18,reset\nFN:47,unused\nFNDA:1,helper\nFNDA:1,reset\nFNDA:0,unused\n\
         FNF:3\nFNH:2\nDA:18,1\nDA:19,1\nDA:20,1\nDA:22,0\nDA:23,0\nDA:25,1\nDA:27,1\nDA:28,1\nDA:29,1\nDA:31,1\n\
         DA:32,0\nDA:39,1\nDA:40,1\nDA:47,0\nDA:48,0\nLF:15\nLH:10\nend_of_record\n"
    );
    assert_eq!(lcov, expected);
    assert!(r.summary().contains("test_programs"), "{}", r.summary());
}

/// Coverage on or off, the machine ends in the same state (it runs one instruction at a time, as with a breakpoint).
#[test]
fn coverage_changes_nothing() {
    let elf = common::repo().join("test_programs/pico/hello_multicore.elf");
    let run = |cov: bool| {
        let mut s = common::boot(&elf, None);
        if cov {
            s.machine.enable_coverage();
        }
        s.run_until(Time::from_ms(30));
        let m = &s.machine;
        let regs: Vec<Vec<u32>> =
            m.cores.iter().map(|c| (0..17).map(|r| sim_core::cpu::Cpu::read_reg(c, r)).collect()).collect();
        let cycles: Vec<u64> = m.cores.iter().map(sim_core::cpu::Cpu::cycles).collect();
        let hash = m.sys.sram.iter().fold(0u64, |h, &b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3));
        (regs, cycles, hash, s.uart_text.clone(), m.coverage.as_ref().map(|c| c.count()))
    };
    let (a, b) = (run(false), run(true));
    assert_eq!((&a.0, &a.1, a.2, &a.3), (&b.0, &b.1, b.2, &b.3));
    assert!(b.4.unwrap() > 500, "{:?}", b.4);
}

#[test]
fn a_titan_image_has_its_panic_handler_unrun_until_a_panic() {
    let Some(elf) = common::titan_elf("power_board") else { return };
    let run = |panic: bool| {
        let mut s = common::boot(&elf, None);
        s.machine.enable_coverage();
        s.run_until(Time::from_ms(300));
        if panic {
            sim_host::fault::FaultTarget { session: &mut s, bus: None, bridge: None, power: None }
                .apply(&sim_host::fault::Fault::Panic)
                .unwrap();
            s.run_until(Time::from_ms(400));
        }
        let mut cov = Coverage::new();
        cov.add_machine("power_board", &s.machine);
        cov.report()
    };
    let r = run(false);
    let (_, main) = r.function("power_board::__cortex_m_rt_main").expect("main is in the report");
    assert!(main.hit);
    let (file, handler) = r.function("__defmt_default_panic").expect("the panic handler");
    assert!(!handler.hit, "no panic, no panic handler");
    assert!(file.ends_with(".rs"), "{file}");
    let c = r.by_crate();
    let (hit, total) = c.values().fold((0, 0), |(h, t), x| (h + x[0], t + x[1]));
    assert!(total > 100 && hit > 30 && hit < total, "{hit}/{total}");
    eprintln!("{}", r.summary());
    assert!(c.get("power_board").is_some_and(|x| x[0] > 0), "{c:?}");
    assert!(r.lcov("power_board").contains("SF:"), "lcov has files");
    let p = run(true);
    assert!(p.function("__defmt_default_panic").unwrap().1.hit, "a panic runs the panic handler");
}
