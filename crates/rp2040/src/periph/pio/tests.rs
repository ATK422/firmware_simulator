//! Instruction-level tests of the PIO model, with instructions encoded by hand as in the datasheet's tables.

use super::*;

// ---- encoders (datasheet 3.4)
fn jmp(cond: u16, addr: u16) -> u16 {
    cond << 5 | addr
}
fn wait(pol: u16, src: u16, idx: u16) -> u16 {
    0x2000 | pol << 7 | src << 5 | idx
}
fn inp(src: u16, n: u16) -> u16 {
    0x4000 | src << 5 | (n & 31)
}
fn out(dst: u16, n: u16) -> u16 {
    0x6000 | dst << 5 | (n & 31)
}
fn push(if_full: bool, block: bool) -> u16 {
    0x8000 | (if_full as u16) << 6 | (block as u16) << 5
}
fn pull(if_empty: bool, block: bool) -> u16 {
    0x8080 | (if_empty as u16) << 6 | (block as u16) << 5
}
fn mov(dst: u16, op: u16, src: u16) -> u16 {
    0xA000 | dst << 5 | op << 3 | src
}
fn irq(clr: bool, wait: bool, idx: u16) -> u16 {
    0xC000 | (clr as u16) << 6 | (wait as u16) << 5 | idx
}
fn set(dst: u16, data: u16) -> u16 {
    0xE000 | dst << 5 | data
}
fn delay(i: u16, d: u16) -> u16 {
    i | d << 8
}

const NOP: u16 = 0xA042; // mov y, y

const SM0: u32 = SM0_CLKDIV;
fn reg(sm: usize, r: u32) -> u32 {
    SM0 + sm as u32 * SM_STRIDE + r
}
const EXECCTRL: u32 = 4;
const SHIFTCTRL: u32 = 8;
const ADDR: u32 = 0xC;
const SMINSTR: u32 = 0x10;
const PINCTRL: u32 = 0x14;

fn w(p: &mut Pio, off: u32, v: u32) {
    assert!(p.write(off, v, WriteMode::Normal, 0));
}

/// A block with `prog` at address 0, SM0 wrapping over the whole program.
fn pio(prog: &[u16]) -> Pio {
    let mut p = Pio::new(0);
    for (i, &ins) in prog.iter().enumerate() {
        w(&mut p, INSTR_MEM0 + 4 * i as u32, ins as u32);
    }
    w(&mut p, reg(0, EXECCTRL), (prog.len() as u32 - 1) << 12);
    p
}

fn start(p: &mut Pio, mask: u32) {
    w(p, CTRL, mask);
}

/// Runs to `cycle` and returns the cycles at which the outputs changed with the new pad_out values.
fn edges(p: &mut Pio, until: u64, inputs: u32) -> Vec<(u64, u32)> {
    let mut v = Vec::new();
    while p.run(until, inputs) == Stop::Outputs {
        v.push((p.cycle, p.pad_out));
    }
    v
}

#[test]
fn set_pins_with_delay_and_wrap_timing() {
    // set pins 1 [3] ; set pins 0 [5] : a 10-cycle period, high 4 cycles
    let mut p = pio(&[delay(set(0, 1), 3), delay(set(0, 0), 5)]);
    w(&mut p, reg(0, PINCTRL), 1 << 26 | 3 << 5); // SET_COUNT 1, SET_BASE 3
    start(&mut p, 1);
    let e = edges(&mut p, 35, 0);
    assert_eq!(e, vec![(1, 8), (5, 0), (11, 8), (15, 0), (21, 8), (25, 0), (31, 8), (35, 0)]);
    assert_eq!(p.read(reg(0, ADDR)), Some(0));
}

#[test]
fn fractional_clock_divider() {
    // SET X increments are not visible; use a 1-instruction loop toggling pins every tick and look at spacing
    let mut p = pio(&[set(0, 1), set(0, 0)]);
    w(&mut p, reg(0, PINCTRL), 1 << 26);
    w(&mut p, reg(0, 0), 2 << 16 | 0x80 << 8); // 2.5
    start(&mut p, 1);
    let e: Vec<u64> = edges(&mut p, 20, 0).iter().map(|e| e.0).collect();
    // floor(k * 2.5): 2, 5, 7, 10, 12, 15, 17, 20
    assert_eq!(e, vec![2, 5, 7, 10, 12, 15, 17, 20]);
}

#[test]
fn jmp_conditions() {
    // X--: count X from 3 to 0 with a loop, then Y--, !X, !Y, X!=Y, PIN, !OSRE
    let prog = [
        set(1, 3),            // 0: x = 3
        jmp(2, 1),            // 1: jmp x-- 1  (loops while x != 0 before decrement)
        set(2, 2),            // 2: y = 2
        jmp(4, 3),            // 3: jmp y-- 3
        jmp(1, 6),            // 4: jmp !x 6 (x is 0xFFFFFFFF after the last decrement: not taken)
        irq(false, false, 0), // 5: flag 0 marks "!x not taken"
        set(1, 0),            // 6: x = 0
        jmp(1, 9),            // 7: jmp !x 9 (taken)
        irq(false, false, 1), // 8: skipped
        jmp(5, 11),           // 9: jmp x!=y 11 (x 0, y 0xFFFFFFFF: taken)
        irq(false, false, 2), // 10: skipped
        jmp(6, 14),           // 11: jmp pin 14 (JMP_PIN 5 high: taken)
        irq(false, false, 3), // 12: skipped
        jmp(0, 12),           // 13
        jmp(7, 16),           // 14: jmp !osre (OSR empty after restart: not taken)
        irq(false, false, 4), // 15: flag 4 marks "!osre not taken"
        jmp(0, 16),           // 16: park
    ];
    let mut p = pio(&prog);
    w(&mut p, reg(0, EXECCTRL), 5 << 24 | 16 << 12);
    start(&mut p, 1);
    p.run(100, 1 << 5);
    assert_eq!(p.irq, 1 << 0 | 1 << 4, "{:#x}", p.irq);
    let (pc, x, y, ..) = p.sm_regs(0);
    assert_eq!((pc, x, y), (16, 0, u32::MAX));
}

#[test]
fn wait_gpio_pin_and_irq() {
    let prog = [wait(1, 0, 7), wait(0, 1, 2), wait(1, 2, 3), irq(false, false, 0), jmp(0, 4)];
    let mut p = pio(&prog);
    w(&mut p, reg(0, PINCTRL), 10 << 15); // IN_BASE 10: WAIT PIN 2 is GPIO 12
    start(&mut p, 1);
    p.run(10, 1 << 12);
    assert_eq!(p.sm_regs(0).0, 0, "waiting for GPIO 7 high");
    assert!(p.sm_stalled(0));
    assert!(!p.busy(), "stalled on something that has not changed");
    p.run(20, 1 << 7 | 1 << 12);
    assert_eq!(p.sm_regs(0).0, 1, "waiting for GPIO 12 low");
    p.run(30, 1 << 7);
    assert_eq!(p.sm_regs(0).0, 2, "waiting for IRQ 3");
    w(&mut p, IRQ_FORCE, 1 << 3);
    p.run(40, 0);
    assert_eq!(p.irq, 1, "WAIT 1 IRQ clears the flag; then IRQ 0 is set");
}

#[test]
fn irq_set_wait_clear_and_relative() {
    // SM1 runs "irq wait 0 rel": sets flag 1 (0 + SM 1) and waits for it to clear
    let mut p = Pio::new(0);
    w(&mut p, INSTR_MEM0, irq(false, true, 0x10) as u32);
    w(&mut p, INSTR_MEM0 + 4, jmp(0, 1) as u32);
    w(&mut p, reg(1, EXECCTRL), 1 << 12);
    start(&mut p, 2);
    p.run(10, 0);
    assert_eq!(p.irq, 1 << 1);
    assert_eq!(p.sm_regs(1).0, 0, "waiting for the flag to clear");
    assert!(!p.irq_line(0));
    w(&mut p, 0x12C, 1 << 9); // IRQ0_INTE: SM1 flag
    assert!(p.irq_line(0));
    w(&mut p, IRQ, 1 << 1); // write 1 to clear
    p.run(20, 0);
    assert_eq!(p.sm_regs(1).0, 1);
    assert!(!p.irq_line(0));
    // irq clear
    let mut p = pio(&[irq(true, false, 5), jmp(0, 1)]);
    w(&mut p, IRQ_FORCE, 0xFF);
    start(&mut p, 1);
    p.run(5, 0);
    assert_eq!(p.irq, !(1u8 << 5));
    assert_eq!(Pio::irq_index(0x13, 2), 1, "rel: (3 + 2) mod 4 = 1");
    assert_eq!(Pio::irq_index(0x16, 3), 5, "rel keeps bit 2");
}

#[test]
fn out_shifts_and_autopull() {
    // Autopull at 8 bits, shift right: OUT X, 4 twice per word
    let prog = [out(1, 4), mov(0, 0, 1)];
    let mut p = pio(&prog);
    w(&mut p, reg(0, SHIFTCTRL), 1 << 17 | 8 << 25 | 1 << 19 | 1 << 18);
    w(&mut p, reg(0, PINCTRL), 4 << 20); // OUT_COUNT 4 at base 0
    w(&mut p, TXF0, 0xA5);
    w(&mut p, TXF0, 0x3C);
    start(&mut p, 1);
    let e = edges(&mut p, 20, 0);
    let vals: Vec<u32> = e.iter().map(|e| e.1).collect();
    assert_eq!(vals, vec![0x5, 0xA, 0xC, 0x3], "LSB nibble first, refilled from the FIFO");
    assert!(p.sm_stalled(0), "stalled on an empty FIFO with autopull");
    assert_eq!(p.read(FDEBUG).unwrap() >> 24 & 1, 1, "TXSTALL");
    // Shift left: MSBs first
    let mut p = pio(&[out(1, 8), mov(0, 0, 1)]);
    w(&mut p, reg(0, SHIFTCTRL), 1 << 17 | 16 << 25);
    w(&mut p, reg(0, PINCTRL), 8 << 20);
    w(&mut p, TXF0, 0xABCD_0000);
    start(&mut p, 1);
    let vals: Vec<u32> = edges(&mut p, 10, 0).iter().map(|e| e.1).collect();
    assert_eq!(vals, vec![0xAB, 0xCD]);
}

#[test]
fn in_shifts_autopush_and_stall() {
    // IN PINS, 4 from IN_BASE 8, autopush at 8 bits, shift left
    let mut p = pio(&[inp(0, 4)]);
    w(&mut p, reg(0, SHIFTCTRL), 1 << 16 | 8 << 20);
    w(&mut p, reg(0, PINCTRL), 8 << 15);
    start(&mut p, 1);
    p.run(2, 0x5 << 8);
    assert_eq!(p.read(RXF0), Some(0x55));
    // Fill the FIFO (4 deep): the fifth push stalls with RXSTALL
    p.run(12, 0x9 << 8);
    assert_eq!(p.read(FLEVEL).unwrap() >> 4 & 0xF, 4);
    assert!(p.sm_stalled(0));
    assert_eq!(p.read(FDEBUG).unwrap() & 1, 1);
    assert_eq!(p.read(RXF0), Some(0x99));
    p.run(14, 0x9 << 8);
    assert_eq!(p.read(FLEVEL).unwrap() >> 4 & 0xF, 4, "the stalled push completed once there was room");
    // Shift right, IN X, 8; IN NULL, 24: X's low byte at the top
    let mut p = pio(&[set(1, 0x1F), inp(1, 8), inp(3, 24), push(false, true)]);
    w(&mut p, reg(0, SHIFTCTRL), 1 << 18);
    start(&mut p, 1);
    p.run(4, 0);
    assert_eq!(p.read(RXF0), Some(0x1F), "0x1F shifted in at the top, then moved down 24 places");
}

#[test]
fn push_pull_variants() {
    // pull noblock with an empty FIFO copies X; pull ifempty does nothing while OSR still has data
    let prog = [set(1, 7), pull(false, false), mov(2, 0, 7), pull(true, true), mov(1, 0, 7), jmp(0, 5)];
    let mut p = pio(&prog);
    w(&mut p, reg(0, SHIFTCTRL), 0); // PULL_THRESH 0 means 32
    start(&mut p, 1);
    p.run(10, 0);
    let (_, x, y, _, osr) = p.sm_regs(0);
    assert_eq!((y, osr, x), (7, 7, 7), "pull noblock -> X; ifempty no-op (OSR full)");
    // push iffull below threshold is a no-op; blocking push of a full FIFO stalls; nonblocking discards
    let prog = [set(1, 1), inp(1, 1), push(true, true), push(false, false), jmp(0, 4)];
    let mut p = pio(&prog);
    w(&mut p, reg(0, SHIFTCTRL), 8 << 20);
    start(&mut p, 1);
    p.run(10, 0);
    assert_eq!(p.read(FLEVEL).unwrap() >> 4 & 0xF, 1, "only the plain push pushed");
    assert_eq!(p.read(RXF0), Some(1));
    // Blocking pull stalls until the CPU writes
    let mut p = pio(&[pull(false, true), mov(1, 0, 7)]);
    start(&mut p, 1);
    p.run(10, 0);
    assert!(p.sm_stalled(0) && !p.busy());
    w(&mut p, TXF0, 42);
    p.run(13, 0);
    assert_eq!(p.sm_regs(0).1, 42);
}

#[test]
fn mov_operations_status_and_exec() {
    let prog = [
        set(1, 0b10110), // 0: x = 0b10110
        mov(2, 1, 1),    // 1: y = !x
        mov(6, 2, 1),    // 2: isr = reverse(x)
        mov(1, 0, 5),    // 3: x = STATUS (TX level < 1: all ones while empty)
        jmp(0, 4),       // 4: park
    ];
    let mut p = pio(&prog);
    w(&mut p, reg(0, EXECCTRL), 4 << 12 | 1);
    start(&mut p, 1);
    p.run(10, 0);
    let (_, x, y, isr, _) = p.sm_regs(0);
    assert_eq!((x, y, isr), (u32::MAX, !0b10110, 0b10110u32.reverse_bits()));
    // MOV EXEC and OUT EXEC run the instruction next; OUT PC and MOV PC jump
    let prog = [mov(4, 0, 1), jmp(0, 1), jmp(0, 2), out(7, 16), out(5, 5), jmp(0, 5), set(2, 9), jmp(0, 7)];
    let mut p = pio(&prog);
    w(&mut p, reg(0, EXECCTRL), 7 << 12);
    w(&mut p, reg(0, SHIFTCTRL), 1 << 19);
    // X holds "set y, 3"; the OSR holds "set y, 5" then the address 6
    p.force_for_test(0, set(1, 0));
    let mut p2 = p.clone();
    // Through MOV EXEC: x is "set x, 3" -> executing it sets x to 3
    p.sm[0].x = set(2, 3) as u32;
    start(&mut p, 1);
    p.run(3, 0);
    assert_eq!(p.sm_regs(0).2, 3, "MOV EXEC ran set y, 3");
    assert_eq!(p.sm_regs(0).0, 1);
    // OUT EXEC then OUT PC
    p2.sm[0].pc = 3;
    p2.sm[0].osr = (set(2, 5) as u32) | 6 << 16;
    p2.sm[0].osr_count = 0;
    start(&mut p2, 1);
    p2.run(6, 0);
    let (pc, _, y, ..) = p2.sm_regs(0);
    assert_eq!(y, 9, "OUT EXEC set y to 5, OUT PC jumped to 6 which set y to 9");
    assert_eq!(pc, 7);
}

#[test]
fn side_set_with_enable_bit_pindirs_and_priority() {
    // SIDESET_COUNT 2 with SIDE_EN: 1 enable bit + 1 side bit, 3 delay bits
    let side = |v: u16| (0b10 | v) << 11;
    let prog = [set(0, 0) | side(1), NOP | 2 << 8, set(0, 1) | side(0), jmp(0, 3)];
    let mut p = pio(&prog);
    w(&mut p, reg(0, EXECCTRL), 1 << 30 | 3 << 12);
    // SET_BASE 2 count 1, SIDESET_BASE 2: the same pin, side-set wins
    w(&mut p, reg(0, PINCTRL), 2 << 29 | 1 << 26 | 2 << 10 | 2 << 5);
    start(&mut p, 1);
    let e = edges(&mut p, 10, 0);
    assert_eq!(e, vec![(1, 1 << 2), (5, 0)], "side 1 beats SET 0; the nop's side-set is disabled, delay 2");
    // SIDE_PINDIR: side-set drives pin directions
    let mut p = pio(&[NOP | 1 << 12, jmp(0, 1)]);
    w(&mut p, reg(0, EXECCTRL), 1 << 29 | 1 << 12);
    w(&mut p, reg(0, PINCTRL), 1 << 29 | 7 << 10);
    start(&mut p, 1);
    p.run(3, 0);
    assert_eq!(p.pad_oe, 1 << 7);
}

#[test]
fn side_set_happens_once_even_while_stalled() {
    // pull block side 1 stalls; the side-set is applied at once
    let mut p = pio(&[pull(false, true) | 1 << 12, set(0, 0)]);
    w(&mut p, reg(0, PINCTRL), 1 << 29 | 1 << 26);
    start(&mut p, 1);
    let e = edges(&mut p, 10, 0);
    assert_eq!(e, vec![(1, 1)]);
}

#[test]
fn fifo_join_levels_status_and_errors() {
    let mut p = Pio::new(1);
    // Join TX: 8 deep
    w(&mut p, reg(2, SHIFTCTRL), 1 << 30);
    for i in 0..9 {
        w(&mut p, TXF0 + 8, i);
    }
    assert_eq!(p.read(FLEVEL).unwrap() >> 16 & 0xF, 8);
    assert_eq!(p.read(FDEBUG).unwrap() >> 16 & 0xF, 1 << 2, "TXOVER");
    assert_eq!(p.read(FSTAT).unwrap() >> 16 & 0xF, 1 << 2, "TXFULL");
    assert!(!p.tx_dreq(2) && p.tx_dreq(0));
    // Reading an empty RX FIFO sets RXUNDER
    assert_eq!(p.read(RXF0 + 4), Some(0));
    assert_eq!(p.read(FDEBUG).unwrap() >> 8 & 0xF, 1 << 1);
    p.write(FDEBUG, 0xFFFF_FFFF, WriteMode::Normal, 0);
    assert_eq!(p.read(FDEBUG), Some(0));
    // Changing the join flushes
    w(&mut p, reg(2, SHIFTCTRL), 0);
    assert_eq!(p.read(FLEVEL).unwrap() >> 16 & 0xF, 0);
    assert_eq!(p.read(0x044), Some(0x0020_0404));
    // INTR: TXNFULL for every empty TX FIFO
    assert_eq!(p.read(INTR).unwrap() >> 4 & 0xF, 0xF);
}

#[test]
fn instr_register_executes_on_a_disabled_machine() {
    let mut p = pio(&[jmp(0, 0), set(1, 5), jmp(0, 2)]);
    w(&mut p, reg(0, SMINSTR), jmp(0, 1) as u32);
    assert_eq!(p.read(reg(0, ADDR)), Some(1));
    w(&mut p, reg(0, SMINSTR), set(2, 17) as u32);
    assert_eq!(p.sm_regs(0).2, 17);
    // A stalling forced instruction stays pending (EXEC_STALLED) until it can complete
    w(&mut p, reg(0, SMINSTR), pull(false, true) as u32);
    assert_ne!(p.read(reg(0, EXECCTRL)).unwrap() & 1 << 31, 0);
    w(&mut p, TXF0, 99);
    start(&mut p, 1);
    p.run(5, 0);
    assert_eq!(p.sm_regs(0).4, 99);
    assert_eq!(p.sm_regs(0).1, 5, "then the program continues at 1");
}

#[test]
fn restart_and_clkdiv_restart_and_sticky() {
    let mut p = pio(&[out(0, 1), jmp(0, 1)]);
    w(&mut p, reg(0, EXECCTRL), 1 << 17 | 1 << 12);
    w(&mut p, reg(0, PINCTRL), 1 << 20 | 4);
    w(&mut p, reg(0, SHIFTCTRL), 1 << 19);
    p.sm[0].osr = 1;
    p.sm[0].osr_count = 0;
    start(&mut p, 1);
    let e = edges(&mut p, 10, 0);
    assert_eq!(e, vec![(1, 1 << 4)], "OUT sticky keeps re-asserting without further edges");
    w(&mut p, CTRL, 1 | 1 << 4);
    assert_eq!(p.sm[0].isr_count, 0);
    assert!(p.sm[0].sticky.is_none());
    // CLKDIV_RESTART re-phases the divider at the current cycle
    w(&mut p, CTRL, 1 | 1 << 8);
    assert_eq!(p.sm[0].anchor, p.cycle);
}

impl Pio {
    fn force_for_test(&mut self, sm: usize, instr: u16) {
        self.force(sm, instr, 0);
    }
}

// ---- pure loops: skipped, with exactly the results of ticking through them

/// Everything software or the system can see of a block.
fn observable(p: &Pio) -> String {
    let regs: Vec<Option<u32>> = (0..0x144).step_by(4).map(|off| p.peek(off)).collect();
    let sms: Vec<_> = (0..4).map(|i| (p.sm_regs(i), p.sm_stalled(i), p.tx_dreq(i), p.rx_dreq(i))).collect();
    format!(
        "cycle {} pads {:08x}/{:08x} irq {:02x} lines {:?} busy {} regs {regs:x?} sms {sms:?}",
        p.cycle,
        p.pad_out,
        p.pad_oe,
        p.irq,
        p.irq_lines(),
        p.busy()
    )
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

/// A random program of `len` instructions, mostly polling loops (JMP, WAIT on pins, MOV/SET to X and Y) with some
/// FIFO, pin and IRQ instructions in between.
fn random_program(r: &mut Rng, len: u16) -> Vec<u16> {
    (0..len)
        .map(|_| {
            let addr = r.below(len as u64) as u16;
            let i = match r.below(12) {
                0..=2 => jmp(r.below(8) as u16, addr),
                3 => wait(r.below(2) as u16, r.below(2) as u16, r.below(4) as u16),
                4 => mov(1 + r.below(2) as u16, r.below(3) as u16, [0, 1, 2, 3, 5][r.below(5) as usize]),
                5 => set(1 + r.below(2) as u16, r.below(32) as u16),
                6 => NOP,
                7 => set([0, 4][r.below(2) as usize], r.below(4) as u16),
                8 => inp([0, 1, 2][r.below(3) as usize], 1 + r.below(8) as u16),
                9 => out([1, 2, 3][r.below(3) as usize], 1 + r.below(8) as u16),
                10 => [push(r.below(2) == 0, r.below(2) == 0), pull(r.below(2) == 0, r.below(2) == 0)]
                    [r.below(2) as usize],
                _ => irq(r.below(2) == 0, false, r.below(4) as u16),
            };
            if r.below(3) == 0 { delay(i, r.below(4) as u16) } else { i }
        })
        .collect()
}

/// Runs the same random scenario on a block with loop skipping and one without, comparing what can be seen at every
/// step: run results, pins, FIFOs, registers.
fn compare_scenario(seed: u64, prog: Option<Vec<u16>>) -> u64 {
    compare_configs(seed, prog, SLOW, FAST).0
}

/// Every fast path off: `skip_loops`, `skip_counted`, `loop_inputs`, `fast_run`.
const SLOW: [bool; 4] = [false; 4];
/// Every fast path on.
const FAST: [bool; 4] = [true; 4];

fn configure(p: &mut Pio, [skip, counted, inputs, fast]: [bool; 4]) {
    (p.skip_loops, p.skip_counted, p.loop_inputs, p.fast_run) = (skip, counted, inputs, fast);
}

/// [`compare_scenario`] with `skip_loops`, `skip_counted`, `loop_inputs` and `fast_run` set as given on the two
/// blocks. Returns the ticks the second block skipped (of those the first ticked) and the counted loop iterations
/// it skipped.
fn compare_configs(seed: u64, prog: Option<Vec<u16>>, first: [bool; 4], second: [bool; 4]) -> (u64, u64) {
    let mut r = Rng(seed | 1);
    let len = 3 + r.below(10) as u16;
    let prog = prog.unwrap_or_else(|| random_program(&mut r, len));
    let mut a = pio(&prog);
    configure(&mut a, first);
    for sm in 0..4 {
        let wrap_top = (prog.len() as u32 - 1) << 12;
        let status = (r.below(2) as u32) << 4 | r.below(4) as u32; // STATUS_SEL, STATUS_N
        let jmp_pin = (r.below(4) as u32) << 24;
        w(&mut a, reg(sm, EXECCTRL), wrap_top | status | jmp_pin);
        w(&mut a, reg(sm, SHIFTCTRL), (r.below(4) as u32) << 16 | 0x000C_0000);
        w(&mut a, reg(sm, PINCTRL), 1 << 26 | 2 << 20 | (r.below(4) as u32) << 15 | (r.below(4) as u32) << 5);
        w(&mut a, reg(sm, 0), (1 + r.below(5) as u32) << 16 | (r.below(256) as u32) << 8);
        w(&mut a, reg(sm, SMINSTR), jmp(0, r.below(prog.len() as u64) as u16) as u32);
    }
    let mut b = a.clone();
    configure(&mut b, second);
    start(&mut a, 1 + r.below(15) as u32);
    start(&mut b, a.read(CTRL).unwrap());
    let mut inputs = r.next() as u32;
    let mut until = 0;
    for step in 0..400 {
        match r.below(10) {
            0 => inputs ^= 1 << r.below(4),
            1 => {
                let off = TXF0 + 4 * r.below(4) as u32;
                let v = r.next() as u32;
                w(&mut a, off, v);
                w(&mut b, off, v);
            }
            2 => {
                let off = RXF0 + 4 * r.below(4) as u32;
                assert_eq!(a.read(off), b.read(off), "seed {seed} step {step}: RX FIFO");
            }
            3 => {
                let off = reg(r.below(4) as usize, ADDR);
                assert_eq!(a.read(off), b.read(off), "seed {seed} step {step}: ADDR");
            }
            _ => {}
        }
        until += 1 + r.below(120);
        loop {
            let (sa, sb) = (a.run(until, inputs), b.run(until, inputs));
            assert_eq!((sa, a.cycle), (sb, b.cycle), "seed {seed} step {step}: run");
            assert_eq!(observable(&a), observable(&b), "seed {seed} step {step}");
            if sa == Stop::Reached {
                break;
            }
        }
    }
    (a.executed - b.executed, b.counted_skipped)
}

#[test]
fn skipped_loops_match_ticking_on_random_programs() {
    let mut skipped = 0;
    for seed in 0..300 {
        skipped += compare_scenario(0x5EED_0000 + seed, None);
    }
    assert!(skipped > 100_000, "loops were skipped: {skipped} ticks");
}

/// A random program built round counted loops: SET X or Y, a few pure instructions (JMP PIN out of the loop, a
/// delay, WAIT on a pin, MOV to the other register), `JMP X--`/`JMP Y--` back, then random instructions.
fn counted_program(r: &mut Rng) -> Vec<u16> {
    let y = r.below(2) as u16;
    let (reg, dec, other) = if y == 1 { (2, 4, 1) } else { (1, 2, 2) };
    let body = 1 + r.below(3) as u16;
    let len = 2 + body + 1 + r.below(4) as u16;
    let mut prog = vec![set(reg, r.below(32) as u16)];
    for _ in 0..body {
        let i = match r.below(5) {
            0 => jmp(6, (r.below(len as u64)) as u16),
            1 => wait(r.below(2) as u16, r.below(2) as u16, r.below(4) as u16),
            2 => mov(other, r.below(2) as u16, [0, 3, 5][r.below(3) as usize]),
            // Now and then something that reads or writes the counter: not a counted loop
            3 if r.below(4) == 0 => mov(reg, 0, 0),
            3 if r.below(3) == 0 => jmp([1, 3, 5][r.below(3) as usize], (r.below(len as u64)) as u16),
            _ => NOP,
        };
        prog.push(if r.below(2) == 0 { delay(i, r.below(8) as u16) } else { i });
    }
    prog.push(delay(jmp(dec, 1), r.below(3) as u16));
    while prog.len() < len as usize {
        let addr = r.below(len as u64) as u16;
        prog.push(match r.below(4) {
            0 => jmp(0, addr),
            1 => set(reg, r.below(32) as u16),
            2 => inp(0, 1 + r.below(4) as u16),
            _ => push(false, false),
        });
    }
    prog
}

#[test]
fn skipped_counted_loops_match_ticking() {
    let (mut ticks, mut counted) = (0, 0);
    for seed in 0..300 {
        let mut r = Rng(0xC0_0000 + seed);
        let prog = counted_program(&mut r);
        // Against ticking everything, and against skipping only pure loops that come back to the same registers
        let (t, c) = compare_configs(0xC1_0000 + seed, Some(prog.clone()), SLOW, FAST);
        ticks += t;
        counted += c;
        let (_, c2) = compare_configs(0xC2_0000 + seed, Some(prog), [true, false, true, true], FAST);
        counted += c2;
    }
    assert!(ticks > 100_000, "{ticks} ticks skipped");
    assert!(counted > 20_000, "{counted} counted iterations skipped");
}

#[test]
fn each_fast_path_alone_matches_ticking() {
    // Each of skip_counted, loop_inputs and fast_run on or off with the others on, and fast_run alone
    for seed in 0..150 {
        let mut r = Rng(0xE0_0000 + seed);
        let len = 3 + r.below(10) as u16;
        let prog = if seed % 2 == 0 { counted_program(&mut r) } else { random_program(&mut r, len) };
        for off in 1..4 {
            let mut cfg = FAST;
            cfg[off] = false;
            compare_configs(0xE1_0000 + seed * 4 + off as u64, Some(prog.clone()), cfg, FAST);
        }
        compare_configs(0xE2_0000 + seed, Some(prog), SLOW, [false, false, false, true]);
    }
}

#[test]
fn bidirectional_dshot_clock_recovery_is_skipped() {
    // The ESC board's bidir_dshot.pio receive section: sample, then wait up to 12 two-cycle iterations for an edge
    // (`JMP PIN` / `JMP Y--`) before sampling again
    let prog = vec![
        set(1, 20),          // 0: set x, 20
        delay(NOP, 13),      // 1: rx_from_edge: nop [13]
        inp(0, 1),           // 2: rx_immediate: in pins, 1
        jmp(2, 5),           // 3: jmp x-- clock_recovery
        jmp(0, 0),           // 4: (end of packet) jmp start
        jmp(6, 10),          // 5: clock_recovery: jmp pin clkrec_pin_high
        set(2, 11),          // 6: set y, 11
        jmp(6, 9),           // 7: clkrec_low_loop: jmp pin pin_low_edge_delay
        jmp(4, 7),           // 8: jmp y-- clkrec_low_loop
        delay(jmp(0, 2), 3), // 9: jmp rx_immediate [3] (the edge path is shortened here)
        set(2, 11),          // 10: clkrec_pin_high: set y, 11
        jmp(6, 13),          // 11: clkrec_high_loop: jmp pin clkrec_y_cmp
        jmp(0, 1),           // 12: jmp rx_from_edge
        jmp(4, 11),          // 13: clkrec_y_cmp: jmp y-- clkrec_high_loop
        delay(jmp(0, 2), 3), // 14: jmp rx_immediate [3]
    ];
    let mut counted = 0;
    for seed in 0..50 {
        counted += compare_configs(0xD6_0000 + seed, Some(prog.clone()), SLOW, FAST).1;
        counted += compare_configs(0xD7_0000 + seed, Some(prog.clone()), [true, false, true, true], FAST).1;
    }
    assert!(counted > 5_000, "{counted} counted iterations skipped");
}

#[test]
fn bidirectional_dshot_reply_wait_is_skipped() {
    // The ESC board's bidir_dshot.pio receive wait: poll the TX FIFO status and the pin until the reply starts
    let prog = vec![
        pull(false, true),
        set(0, 0),    // drive
        set(4, 0),    // release the pin
        mov(2, 0, 5), // wait_rx: mov y, status
        jmp(3, 0),    // jmp !y start
        jmp(6, 3),    // jmp pin wait_rx
        inp(0, 1),
        push(false, false),
        jmp(0, 0),
    ];
    let mut total = 0;
    for seed in 0..50 {
        total += compare_scenario(0xD5_0000 + seed, Some(prog.clone()));
    }
    assert!(total > 10_000, "{total} ticks skipped");
}

#[test]
fn pull_is_a_no_op_when_autopull_has_filled_the_osr() {
    // Datasheet 3.4.7.2: with autopull enabled, PULL is a no-op if the OSR is full (a fence against autopull).
    // OUT X, 32 autopulls A, shifts it out and refills the OSR with B; the PULL then must not discard B for C.
    let mut p = pio(&[out(1, 32), pull(false, true), mov(2, 0, 7), jmp(0, 3)]);
    w(&mut p, reg(0, SHIFTCTRL), 1 << 17); // autopull, threshold 32
    for v in [0xA, 0xB, 0xC] {
        w(&mut p, TXF0, v);
    }
    start(&mut p, 1);
    p.run(10, 0);
    let (_, x, y, _, _) = p.sm_regs(0);
    assert_eq!((x, y), (0xA, 0xB), "PULL kept the autopulled word");
    assert_eq!(p.read(FLEVEL).unwrap() & 0xF, 1, "C is still in the TX FIFO");
}
