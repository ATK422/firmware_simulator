//! Quantum fusing (`Machine::fuse_quanta`, see `Machine::run_until`) must change nothing a board does: board images
//! run on their board files next to a mock bridge with fusing on and off, and everything observable is compared:
//! log lines with their times, every message the bridge received with its time, UART and RTT text, and the final
//! CPU registers, cycle and instruction counts, clocks and SRAM.

use sim_core::cpu::Cpu;
use sim_test::{Bench, Time};

fn run(elf: &str, board: &str, client: u8, fuse: bool) -> Option<(Vec<String>, u64)> {
    let Some(path) = sim_test::titan_rs(elf) else {
        eprintln!("skipping {elf}: image not built");
        return None;
    };
    let mut b = Bench::builder(&path).board(board).bridge(client).boot().unwrap();
    b.session.machine.fuse_quanta = fuse;
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

fn same(elf: &str, board: &str, client: u8) {
    let Some((a, fused)) = run(elf, board, client, true) else { return };
    let (b, none) = run(elf, board, client, false).unwrap();
    assert_eq!(none, 0);
    assert!(fused > 1000, "{elf}: {fused} quanta fused");
    assert_eq!(a.len(), b.len(), "{elf}");
    for (x, y) in a.iter().zip(&b) {
        assert_eq!(x, y, "{elf}");
    }
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
