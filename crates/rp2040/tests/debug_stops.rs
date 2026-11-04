//! A debugger stopping the machine must not change the run: a stop in the middle of a quantum (a breakpoint, a
//! watchpoint, a position reached with `Machine::stop_at`) is resumed exactly, so time-travel debugging can replay
//! from a snapshot to any position and find the same state as a run that never stopped. These tests run programs
//! uninterrupted and with many stops, on both cores, and compare everything observable.

use rp2040::machine::{WatchKind, Watchpoint};
use rp2040::{BOOTROM_B1, Machine, MachineConfig, StopReason};
use sim_core::cpu::Cpu;
use sim_core::image::Image;
use sim_core::time::Time;
use sim_core::trace::{Category, VecSink};

fn machine(elf: &str) -> (Machine, VecSink) {
    let path = format!("{}/../../test_programs/{elf}", env!("CARGO_MANIFEST_DIR"));
    let image = Image::parse(&std::fs::read(&path).unwrap()).unwrap();
    let mut m = Machine::new(MachineConfig { bootrom: Some(BOOTROM_B1.to_vec()), ..Default::default() });
    let sink = VecSink::new(&[Category::Gpio, Category::Unmodelled, Category::Device]);
    m.set_trace(Box::new(sink.clone()));
    m.load(&image).unwrap();
    m.direct_boot().unwrap();
    (m, sink)
}

/// Everything observable: time, both cores, SRAM, UART output, the trace.
fn state(m: &Machine, sink: &VecSink) -> Vec<String> {
    let mut out = vec![format!("now {}", m.now)];
    for (c, core) in m.cores.iter().enumerate() {
        let regs: Vec<u32> = (0..core.num_regs()).map(|r| core.read_reg(r)).collect();
        out.push(format!(
            "core{c} at {}: {} cycles, {} instructions, sleep {:?}, regs {regs:08x?}",
            m.core_time(c),
            core.cycles(),
            core.instructions(),
            core.sleep_state()
        ));
    }
    let hash = |b: &[u8]| b.iter().fold(0u64, |h, &b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3));
    out.push(format!("sram {:x}", hash(&m.sys.sram)));
    for u in &m.sys.uart {
        out.push(format!("uart {:?}", u.output));
    }
    out.extend(sink.take().into_iter().map(|(t, e)| format!("[{t}] {e}")));
    out
}

/// Runs to `end` in 100 µs slices (as a session does), with `stop` called before each slice to arm stops, and
/// every stop resumed. Returns the stops seen.
fn run_with_stops(m: &mut Machine, end: Time, mut arm: impl FnMut(&mut Machine, usize)) -> Vec<StopReason> {
    let mut stops = Vec::new();
    let mut slice = 0;
    while m.now < end {
        let t = (m.now + Time::from_us(100)).min(end);
        // A run stopped in the middle of a slice goes on to the same slice end
        arm(m, slice);
        loop {
            match m.run_until(t) {
                StopReason::TimeLimit => break,
                r => {
                    assert!(r.is_debug_stop(), "{r:?}");
                    stops.push(r);
                }
            }
        }
        slice += 1;
    }
    stops
}

#[test]
fn positions_reached_on_both_cores_change_nothing() {
    // Core 1 launched over the SIO FIFO, both cores printing to the UART: the cores' interleaving is visible
    let end = Time::from_ms(30);
    let (mut a, sa) = machine("pico/hello_multicore.elf");
    run_with_stops(&mut a, end, |_, _| {});
    let (mut b, sb) = machine("pico/hello_multicore.elf");
    let stops = run_with_stops(&mut b, end, |m, slice| {
        // A position a few µs into the slice, on each core in turn
        let core = slice % 2;
        if m.core_enabled[core] {
            m.stop_at[core] = Some(m.core_time(core) + Time::from_ns(1_000 + 370 * (slice as u64 % 7)));
        }
    });
    assert!(stops.len() > 100, "{}", stops.len());
    let reached = |c: u8| stops.iter().filter(|s| matches!(s, StopReason::Reached { core, .. } if *core == c)).count();
    assert!(reached(0) > 50 && reached(1) > 20, "{} {}", reached(0), reached(1));
    assert_eq!(state(&a, &sa), state(&b, &sb));
}

#[test]
fn a_position_is_the_first_step_at_or_after_its_time() {
    let (mut m, _) = machine("pico/hello_multicore.elf");
    m.run_until(Time::from_ms(5));
    let t = m.core_time(0) + Time::from_us(3);
    m.stop_at[0] = Some(t);
    let r = m.run_until(Time::from_ms(6));
    assert!(matches!(r, StopReason::Reached { core: 0, .. }), "{r:?}");
    let at = m.core_time(0);
    assert!(at >= t && at < t + Time::from_ns(200), "{at} for {t}");
    assert_eq!(m.stop_at, [None; 2], "cleared once reached");
    // From a snapshot, the same position gives the same state
    let mut copy = m.fork();
    let regs = |m: &Machine| (0..17).map(|r| m.cores[0].read_reg(r)).collect::<Vec<_>>();
    assert_eq!(regs(&copy), regs(&m));
    assert_eq!(m.run_until(Time::from_ms(6)), StopReason::TimeLimit);
    assert_eq!(copy.run_until(Time::from_ms(6)), StopReason::TimeLimit);
    assert_eq!(regs(&copy), regs(&m));
    assert_eq!(copy.cores[1].cycles(), m.cores[1].cycles());
}

#[test]
fn breakpoint_stops_change_nothing() {
    let end = Time::from_ms(30);
    let (mut a, sa) = machine("pico/hello_multicore.elf");
    // Stepped, like the run with breakpoints (batched runs equal stepped ones: tests/batching.rs)
    a.breakpoints.insert(0xFFFF_FFF1);
    run_with_stops(&mut a, end, |_, _| {});
    let (mut b, sb) = machine("pico/hello_multicore.elf");
    b.breakpoints.insert(0xFFFF_FFF1);
    // Somewhere both cores go often: the UART's putc loop is in flash; take a PC core 1 runs at 10 ms
    b.run_until(Time::ZERO);
    let stops = run_with_stops(&mut b, end, |m, slice| {
        if slice == 100 {
            let pc = m.cores[1].pc();
            m.breakpoints.insert(pc);
            m.breakpoints.insert(m.cores[0].pc());
        }
    });
    assert!(stops.iter().filter(|s| matches!(s, StopReason::Breakpoint { .. })).count() > 10, "{stops:?}");
    b.breakpoints.retain(|&p| p == 0xFFFF_FFF1);
    a.breakpoints.retain(|&p| p == 0xFFFF_FFF1);
    assert_eq!(state(&a, &sa), state(&b, &sb));
}

#[test]
fn watchpoints_stop_after_the_access_and_change_nothing() {
    let end = Time::from_ms(30);
    let (mut a, sa) = machine("pico/hello_multicore.elf");
    a.breakpoints.insert(0xFFFF_FFF1);
    run_with_stops(&mut a, end, |_, _| {});
    let (mut b, sb) = machine("pico/hello_multicore.elf");
    // A word of core 0's stack near its top: written by calls
    let sp = b.cores[0].reg(13);
    let addr = sp - 8;
    b.watchpoints.push(Watchpoint { addr, len: 4, kind: WatchKind::Write });
    let stops = run_with_stops(&mut b, end, |_, _| {});
    let hits: Vec<_> = stops.iter().filter(|s| matches!(s, StopReason::Watchpoint { .. })).collect();
    assert!(!hits.is_empty());
    for h in &hits {
        let StopReason::Watchpoint { addr: a, kind, .. } = h else { unreachable!() };
        assert_eq!((*a, *kind), (addr, WatchKind::Write));
    }
    b.watchpoints.clear();
    assert_eq!(state(&a, &sa), state(&b, &sb));

    // A read watchpoint on the same word stops after an instruction that read it
    let (mut c, _) = machine("pico/hello_multicore.elf");
    c.watchpoints.push(Watchpoint { addr, len: 4, kind: WatchKind::Read });
    let r = c.run_until(end);
    assert!(matches!(r, StopReason::Watchpoint { kind: WatchKind::Read, .. }), "{r:?}");
}
