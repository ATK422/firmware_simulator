//! Fast paths that can be switched off must change nothing a board does: board images run on their board files next
//! to a mock bridge with the fast path on and off, and everything observable is compared: log lines with their
//! times, every message the bridge received with its time, UART and RTT text, and the final CPU registers, cycle and
//! instruction counts, clocks and SRAM.
//!
//! * Quantum fusing (`Machine::fuse_quanta`, see `Machine::run_until`).
//! * PIO register reads that change nothing skip the second advance (`System::pio_fast_reads`).
//! * SIO accesses other than to the FIFOs leave the interrupt lines alone (`System::sio_fast_irqs`).
//! * XOSC reads skip the general path and leave quanta quiet (`System::xosc_fast_reads`), so that embassy-rp's
//!   wait for the crystal is skipped as a busy-wait; and busy-wait skipping itself (`Machine::skip_spins`).
//! * Busy-waits on the timer are skipped across its ticks up to their deadline (`Machine::skip_timer_waits`).
//! * PIO state machines skip counted loops (`Pio::skip_counted`), keep looping through changes of pins they do not
//!   read (`Pio::loop_inputs`), and blocks remember their next ticks between runs (`Pio::fast_run`).

use rp2040::Machine;
use sim_core::cpu::Cpu;
use sim_test::{Bench, Time};

/// Runs `elf` with `setup` applied to the machine; returns what it did and how many quanta were fused.
fn run(elf: &str, board: &str, client: u8, setup: impl Fn(&mut Machine)) -> Option<(Vec<String>, u64)> {
    run_skipped(elf, board, client, setup).map(|(out, fused, _)| (out, fused))
}

/// [`run`], also returning how many instructions busy-wait skipping skipped.
fn run_skipped(elf: &str, board: &str, client: u8, setup: impl Fn(&mut Machine)) -> Option<(Vec<String>, u64, u64)> {
    let path = sim_test::titan_rs(elf)?;
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
    Some((out, m.quanta_fused, m.spin_skipped))
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

fn same_xosc(elf: &str, board: &str, client: u8) {
    let Some((a, _, skipped)) = run_skipped(elf, board, client, |_| {}) else { return };
    let (b, _, general) = run_skipped(elf, board, client, |m| m.sys.xosc_fast_reads = false).unwrap();
    // embassy-rp's `start_xosc` polls XOSC.STATUS for about 128 ms of the 300 (delay_multiplier 128), on the ring
    // oscillator: that is skipped only when those reads keep the quanta quiet
    assert!(skipped > general + 200_000, "{elf}: {skipped} instructions skipped, {general} without");
    compare(elf, &a, &b);
}

#[test]
fn xosc_fast_reads() {
    same_xosc("esc_board", "mk2/esc_board", 2);
    same_xosc("template", "mk2/power_board", 1);
}

fn same_unskipped(elf: &str, board: &str, client: u8) {
    let Some((a, _, skipped)) = run_skipped(elf, board, client, |_| {}) else { return };
    let (b, _, none) = run_skipped(elf, board, client, |m| m.skip_spins = false).unwrap();
    assert_eq!(none, 0);
    assert!(skipped > 200_000, "{elf}: {skipped} instructions skipped");
    compare(elf, &a, &b);
}

#[test]
fn skipped_as_unskipped() {
    // The XOSC wait (a loop over several quanta), the control loop's timer busy-waits
    same_unskipped("esc_board", "mk2/esc_board", 2);
    same_unskipped("template", "mk2/power_board", 1);
}

/// Runs `elf` with each of the PIO fast paths `set` switches off on both blocks, comparing with all on.
fn same_pio_paths(elf: &str, board: &str, client: u8) {
    let Some((a, _)) = run(elf, board, client, |_| {}) else { return };
    type Set = fn(&mut rp2040::periph::pio::Pio);
    let offs: [(&str, Set); 4] = [
        ("skip_counted", |p| p.skip_counted = false),
        ("loop_inputs", |p| p.loop_inputs = false),
        ("fast_run", |p| p.fast_run = false),
        ("all", |p| (p.skip_counted, p.loop_inputs, p.fast_run) = (false, false, false)),
    ];
    for (name, off) in offs {
        let (b, _) = run(elf, board, client, |m| m.sys.pio.iter_mut().for_each(off)).unwrap();
        compare(&format!("{elf} without {name}"), &a, &b);
    }
}

#[test]
fn esc_board_pio_paths() {
    // DShot: the receivers' clock recovery is a counted loop, and the four lines (and the telemetry UARTs) change
    // under each other's loops
    same_pio_paths("esc_board", "mk2/esc_board", 2);
}

#[test]
fn smart_battery_and_hbridge_pio_paths() {
    same_pio_paths("smart_battery", "mk2/sbh_mcu", 21);
    same_pio_paths("hbridge_board", "lilboat/hbridge_board", 2);
}

fn same_timer_waits(elf: &str, board: &str, client: u8) {
    let Some((a, _, skipped)) = run_skipped(elf, board, client, |_| {}) else { return };
    let (b, _, ticked) = run_skipped(elf, board, client, |m| m.skip_timer_waits = false).unwrap();
    compare(elf, &a, &b);
    assert!(skipped > ticked, "{elf}: {skipped} instructions skipped, {ticked} without");
}

#[test]
fn timer_waits_skipped_as_ticked() {
    // The control loop waits out the DShot frame time on core 1 (`while Instant::now() < min_frame_time {}`)
    same_timer_waits("esc_board", "mk2/esc_board", 2);
}
