//! Fast paths that can be switched off must change nothing a board does: board images run on their board files next
//! to a mock bridge with the fast path on and off, and everything observable is compared: log lines with their
//! times, every message the bridge received with its time, UART and RTT text, and the final CPU registers, cycle and
//! instruction counts, clocks and SRAM.
//!
//! * Quantum fusing (`Machine::fuse_quanta`, see `Machine::run_until`).
//! * PIO register reads that change nothing skip the second advance (`System::pio_fast_reads`).
//! * SIO accesses other than to the FIFOs leave the interrupt lines alone (`System::sio_fast_irqs`).

use rp2040::Machine;
use sim_core::cpu::Cpu;
use sim_test::{Bench, Time};

/// Runs `elf` with `setup` applied to the machine; returns what it did and how many quanta were fused.
fn run(elf: &str, board: &str, client: u8, setup: impl Fn(&mut Machine)) -> Option<(Vec<String>, u64)> {
    let Some(path) = sim_test::titan_rs(elf) else {
        eprintln!("skipping {elf}: image not built");
        return None;
    };
    let mut b = Bench::builder(&path).board(board).bridge(client).boot().unwrap();
    setup(&mut b.session.machine);
    b.run_until(Time::from_ms(300)).unwrap();
    let s = &b.session;
    let mut out: Vec<String> = s.logs.iter().map(|(t, l)| format!("[{t}] {l}")).collect();
    out.extend(b.messages().iter().map(|(t, m)| format!("[{t}] {m:?}")));
    out.push(format!("rtt {:?}", s.rtt_text));
    out.push(format!("uart {:?}", s.uart_text));
    let m = &s.machine;
    out.push(format!("now {}", m.now));
    for (c, core) in m.cores.iter().enumerate() {
        let regs: Vec<u32> = (0..core.num_regs()).map(|r| core.read_reg(r)).collect();
        out.push(format!(
            "core{c} at {}: {} cycles, {} instructions, regs {regs:08x?}",
            m.core_time(c),
            core.cycles(),
            core.instructions()
        ));
    }
    out.push(format!("sram {:x}", m.sys.sram.iter().fold(0u64, |h, &b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3))));
    Some((out, m.quanta_fused))
}

fn compare(elf: &str, a: &[String], b: &[String]) {
    assert_eq!(a.len(), b.len(), "{elf}");
    for (x, y) in a.iter().zip(b) {
        assert_eq!(x, y, "{elf}");
    }
}

fn same(elf: &str, board: &str, client: u8) {
    let Some((a, fused)) = run(elf, board, client, |_| {}) else { return };
    let (b, none) = run(elf, board, client, |m| m.fuse_quanta = false).unwrap();
    assert_eq!(none, 0);
    assert!(fused > 1000, "{elf}: {fused} quanta fused");
    compare(elf, &a, &b);
}

#[test]
fn esc_board_fused_as_unfused() {
    // DShot on PIO, the control loop's busy-waits (skipped), CAN over the MCP2517FD
    same("esc_board", "mk2/esc_board", 2);
}

#[test]
fn ivc_board2_fused_as_unfused() {
    // Mostly computation (the bootrom's soft float) between timer interrupts
    same("ivc_board2", "talos/ivc_board", 8);
}

#[test]
fn template_fused_as_unfused() {
    same("template", "mk2/power_board", 1);
}

fn same_pio(elf: &str, board: &str, client: u8) {
    let Some((a, _)) = run(elf, board, client, |_| {}) else { return };
    let (b, _) = run(elf, board, client, |m| m.sys.pio_fast_reads = false).unwrap();
    compare(elf, &a, &b);
}

#[test]
fn esc_board_pio_fast_reads() {
    // The control loop polls the DShot state machines' RX FIFOs (FSTAT) and pops eRPM replies; the telemetry task
    // reads the UART receivers' FIFOs
    same_pio("esc_board", "mk2/esc_board", 2);
}

#[test]
fn smart_battery_pio_fast_reads() {
    // SMBus on PIO
    same_pio("smart_battery", "mk2/sbh_mcu", 21);
}

#[test]
fn hbridge_board_pio_fast_reads() {
    // A multidrop UART on PIO
    same_pio("hbridge_board", "lilboat/hbridge_board", 2);
}

fn same_sio(elf: &str, board: &str, client: u8) {
    let Some((a, _)) = run(elf, board, client, |_| {}) else { return };
    let (b, _) = run(elf, board, client, |m| m.sys.sio_fast_irqs = false).unwrap();
    compare(elf, &a, &b);
}

#[test]
fn sio_fast_irqs() {
    // Critical sections take and release a spinlock; the ESC board runs both cores
    same_sio("esc_board", "mk2/esc_board", 2);
    same_sio("ivc_board2", "talos/ivc_board", 8);
}
