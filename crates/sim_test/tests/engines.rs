//! The CPU's engines run real firmware alike: board images run on two machines in lockstep, one with each engine,
//! and the machines are compared every half millisecond of virtual time: every core register, the exception, NVIC,
//! SysTick and SCB state, cycle and instruction counts, each core's clock, SRAM, and what the board logged and
//! printed. Engines: the interpreter in batches, decoded blocks in batches (the default), decoded blocks compiled by
//! the JIT, and single steps (a breakpoint that is never reached turns batching off, see
//! `crates/rp2040/tests/batching.rs`).
//!
//! The Rust IVCBoard2 (titan_firmware_rs) and, with `TITAN_FIRMWARE_DIR` set, titan_firmware's C IVCBoard2 booted
//! through the bootrom and its bootloader.

use rp2040::Machine;
use sim_test::{Bench, Time};

/// An engine: how to set up a machine to run with it.
type Engine = (&'static str, fn(&mut Machine));

const INTERPRETER: Engine = ("interpreter", |m| m.cores.iter_mut().for_each(|c| c.decode_blocks = false));
const BLOCKS: Engine = ("blocks", |m| m.cores.iter_mut().for_each(|c| c.decode_blocks = true));
const COMPILED: Engine = ("compiled", |m| {
    for c in &mut m.cores {
        // (Early, so that nearly every block runs compiled)
        (c.decode_blocks, c.jit, c.jit_threshold) = (true, true, 2);
    }
});
const STEPPED: Engine = ("stepped", |m| {
    // Never reached (instructions are halfword aligned), but makes the run loop step every instruction
    m.breakpoints.insert(0xFFFF_FFF1);
});

/// Everything about a board that the engine running it must not change.
fn state(b: &Bench) -> String {
    let s = &b.session;
    let m = &s.machine;
    let mut out = format!("now {} logs {} rtt {} uart {}", m.now, s.logs.len(), s.rtt_text.len(), s.uart_text.len());
    for (c, core) in m.cores.iter().enumerate() {
        out += &format!("\ncore{c} at {}: {}", m.core_time(c), core.comparable_state());
    }
    let hash =
        |mem: &[u8]| mem.iter().fold(0xcbf2_9ce4_8422_2325u64, |h, &b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3));
    out += &format!("\nsram {:016x} xip_sram {:016x}", hash(&m.sys.sram), hash(&m.sys.xip_sram));
    out
}

/// Runs two benches with engines `a` and `b` until `until`, comparing them every `every`.
fn lockstep(name: &str, make: &dyn Fn() -> Bench, a: Engine, b: Engine, until: Time, every: Time) {
    let (mut x, mut y) = (make(), make());
    (a.1)(&mut x.session.machine);
    (b.1)(&mut y.session.machine);
    let mut t = Time::ZERO;
    let mut checks = 0;
    while t < until {
        t = (t + every).min(until);
        x.run_until(t).unwrap();
        y.run_until(t).unwrap();
        let (sx, sy) = (state(&x), state(&y));
        if sx != sy {
            let line = sx.lines().zip(sy.lines()).find(|(p, q)| p != q).unwrap_or_default();
            panic!("{name}: {} and {} differ at {t}:\n{}\n  vs\n{}", a.0, b.0, line.0, line.1);
        }
        checks += 1;
    }
    let s = &x.session;
    assert_eq!(s.logs, y.session.logs, "{name}");
    assert_eq!((&s.rtt_text, &s.uart_text), (&y.session.rtt_text, &y.session.uart_text), "{name}");
    let instructions: u64 = s.machine.cores.iter().map(|c| c.instructions()).sum();
    eprintln!("{name}: {} and {} agree at {checks} points over {until} ({instructions} instructions)", a.0, b.0);
}

fn rust_ivc() -> Option<impl Fn() -> Bench> {
    let path = sim_test::titan_rs("ivc_board2")?;
    Some(move || Bench::builder(&path).board("talos/ivc_board").bridge(8).boot().unwrap())
}

fn c_ivc() -> Option<impl Fn() -> Bench> {
    let image = sim_test::titan_firmware("talos/IVCBoard2/build/IVCBoard2_with_bl.uf2")?;
    Some(move || Bench::builder(&image).board("talos/ivc_board").rom_boot().boot().unwrap())
}

const HALF_MS: Time = Time::from_us(500);

#[test]
fn rust_ivc_board2_runs_alike_interpreted_and_decoded() {
    let Some(make) = rust_ivc() else { return };
    lockstep("IVCBoard2", &make, INTERPRETER, BLOCKS, Time::from_ms(1000), HALF_MS);
}

#[test]
fn rust_ivc_board2_runs_alike_stepped_and_decoded() {
    let Some(make) = rust_ivc() else { return };
    lockstep("IVCBoard2", &make, STEPPED, BLOCKS, Time::from_ms(150), HALF_MS);
}

#[test]
fn c_ivc_board2_runs_alike_interpreted_and_decoded() {
    // Through the bootrom, the bootloader (from XIP SRAM) and into the application's sampling, past the second
    // of its FFT setup
    let Some(make) = c_ivc() else { return };
    lockstep("IVCBoard2 (C)", &make, INTERPRETER, BLOCKS, Time::from_ms(1300), HALF_MS);
}

#[test]
fn c_ivc_board2_runs_alike_stepped_and_decoded() {
    let Some(make) = c_ivc() else { return };
    lockstep("IVCBoard2 (C)", &make, STEPPED, BLOCKS, Time::from_ms(150), HALF_MS);
}

#[test]
fn rust_ivc_board2_runs_alike_decoded_and_compiled() {
    let Some(make) = rust_ivc() else { return };
    lockstep("IVCBoard2", &make, BLOCKS, COMPILED, Time::from_ms(1000), HALF_MS);
}

#[test]
fn c_ivc_board2_runs_alike_decoded_and_compiled() {
    let Some(make) = c_ivc() else { return };
    lockstep("IVCBoard2 (C)", &make, BLOCKS, COMPILED, Time::from_ms(1300), HALF_MS);
}

#[test]
fn rust_ivc_board2_runs_alike_stepped_and_compiled() {
    let Some(make) = rust_ivc() else { return };
    lockstep("IVCBoard2", &make, STEPPED, COMPILED, Time::from_ms(150), HALF_MS);
}
