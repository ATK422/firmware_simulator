//! PIO (datasheet 3): two blocks of four state machines sharing 32 instructions of memory, eight IRQ flags and the
//! GPIOs.
//!
//! State machines are cycle exact: each executes one instruction (or one delay or stall cycle) per tick of its
//! fractional clock divider, tick `k` falling on `clk_sys` cycle `anchor + floor(k * div / 256)`. The block is
//! advanced lazily ([`Pio::run`]) and skips ahead over delays and over state machines stalled on something that
//! has not changed since they stalled (FIFOs, IRQ flags, pins). [`Pio::run`] returns early when the block's pin
//! outputs change, so the system can resolve the pins and let devices react before the PIO goes on.
//!
//! Within one cycle state machines execute in order 0 to 3 and see each other's effects immediately (on hardware,
//! from the next cycle). Pin writes follow the hardware priority: a higher-numbered state machine wins, and
//! side-set wins over the same state machine's OUT/SET. The two-cycle input synchronisers are not modelled.

use std::collections::VecDeque;

use sim_core::bus::WriteMode;

pub const CTRL: u32 = 0x000;
pub const FSTAT: u32 = 0x004;
pub const FDEBUG: u32 = 0x008;
pub const FLEVEL: u32 = 0x00C;
pub const TXF0: u32 = 0x010;
pub const RXF0: u32 = 0x020;
pub const IRQ: u32 = 0x030;
pub const IRQ_FORCE: u32 = 0x034;
pub const INSTR_MEM0: u32 = 0x048;
pub const SM0_CLKDIV: u32 = 0x0C8;
pub const SM_STRIDE: u32 = 0x18;
pub const INTR: u32 = 0x128;

const NUM_SM: usize = 4;

/// What one tick of a state machine did.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Outcome {
    /// Finished; `jump` replaces the normal PC advance; `delay` ticks follow.
    Done { jump: Option<u8>, delay: u32 },
    /// Could not complete; retried on the next tick.
    Stall,
    /// Finished, and this instruction executes on the next tick (OUT EXEC, MOV EXEC). The own delay is ignored.
    Exec(u16),
}

#[derive(Clone, Debug, Default)]
struct Sm {
    clkdiv: u32,
    execctrl: u32,
    shiftctrl: u32,
    pinctrl: u32,
    pc: u8,
    x: u32,
    y: u32,
    isr: u32,
    osr: u32,
    isr_count: u32,
    osr_count: u32,
    tx: VecDeque<u32>,
    rx: VecDeque<u32>,
    /// An instruction to execute instead of the one at PC (EXEC, SMx_INSTR writes).
    exec: Option<u16>,
    /// The current instruction has issued its side-set (it stalled after issuing).
    issued: bool,
    /// IRQ ... WAIT: the flag being waited on after setting it.
    irq_wait: Option<u8>,
    stalled: bool,
    stall_epoch: u64,
    /// Clock: tick `k` is at cycle `anchor + floor(k * div / 256)`; `next_k` is the next tick that does anything.
    anchor: u64,
    next_k: u64,
    /// Values of the last OUT/SET to pins, re-asserted with OUT_STICKY.
    sticky: Option<(u32, u32, u32)>,
}

impl Sm {
    fn new() -> Self {
        Sm {
            clkdiv: 0x0001_0000,
            shiftctrl: 0x000C_0000,
            pinctrl: 0x1400_0000,
            execctrl: 0x1F << 12,
            ..Default::default()
        }
    }

    /// Clock divider in 1/256ths (INT 0 means 65536).
    fn div(&self) -> u64 {
        let int = (self.clkdiv >> 16) as u64;
        let int = if int == 0 { 65536 } else { int };
        (int << 8) | ((self.clkdiv >> 8) & 0xFF) as u64
    }

    #[inline]
    fn tick_cycle(&self, k: u64) -> u64 {
        let div = self.div();
        match k.checked_mul(div) {
            Some(p) => self.anchor + (p >> 8),
            None => self.anchor + ((k as u128 * div as u128) >> 8) as u64,
        }
    }

    /// Smallest tick index falling after `cycle`.
    fn first_tick_after(&self, cycle: u64) -> u64 {
        if cycle < self.anchor {
            return 1;
        }
        let d = self.div() as u128;
        let mut k = ((((cycle - self.anchor) as u128) << 8) / d) as u64;
        while self.tick_cycle(k) <= cycle {
            k += 1;
        }
        k.max(1)
    }

    fn tx_cap(&self) -> usize {
        if self.shiftctrl & (1 << 30) != 0 {
            8
        } else if self.shiftctrl & (1 << 31) != 0 {
            0
        } else {
            4
        }
    }

    fn rx_cap(&self) -> usize {
        if self.shiftctrl & (1 << 31) != 0 {
            8
        } else if self.shiftctrl & (1 << 30) != 0 {
            0
        } else {
            4
        }
    }

    fn push_thresh(&self) -> u32 {
        match (self.shiftctrl >> 20) & 0x1F {
            0 => 32,
            n => n,
        }
    }

    fn pull_thresh(&self) -> u32 {
        match (self.shiftctrl >> 25) & 0x1F {
            0 => 32,
            n => n,
        }
    }

    fn autopush(&self) -> bool {
        self.shiftctrl & (1 << 16) != 0
    }

    fn autopull(&self) -> bool {
        self.shiftctrl & (1 << 17) != 0
    }

    fn in_right(&self) -> bool {
        self.shiftctrl & (1 << 18) != 0
    }

    fn out_right(&self) -> bool {
        self.shiftctrl & (1 << 19) != 0
    }

    fn out_base(&self) -> u32 {
        self.pinctrl & 0x1F
    }
    fn set_base(&self) -> u32 {
        (self.pinctrl >> 5) & 0x1F
    }
    fn sideset_base(&self) -> u32 {
        (self.pinctrl >> 10) & 0x1F
    }
    fn in_base(&self) -> u32 {
        (self.pinctrl >> 15) & 0x1F
    }
    fn out_count(&self) -> u32 {
        (self.pinctrl >> 20) & 0x3F
    }
    fn set_count(&self) -> u32 {
        (self.pinctrl >> 26) & 0x7
    }
    fn sideset_count(&self) -> u32 {
        (self.pinctrl >> 29) & 0x7
    }

    fn wrap_top(&self) -> u8 {
        ((self.execctrl >> 12) & 0x1F) as u8
    }
    fn wrap_bottom(&self) -> u8 {
        ((self.execctrl >> 7) & 0x1F) as u8
    }

    /// Clears the state SM_RESTART clears: shift counters and registers' fill state, delays, stalls, pending EXEC.
    fn restart(&mut self) {
        self.isr = 0;
        self.isr_count = 0;
        self.osr = 0;
        self.osr_count = 32;
        self.exec = None;
        self.issued = false;
        self.irq_wait = None;
        self.stalled = false;
        self.sticky = None;
    }

    fn shift_out(&mut self, n: u32) -> u32 {
        let n = if n == 0 { 32 } else { n };
        let data = if n == 32 {
            let d = self.osr;
            self.osr = 0;
            d
        } else if self.out_right() {
            let d = self.osr & ((1 << n) - 1);
            self.osr >>= n;
            d
        } else {
            let d = self.osr >> (32 - n);
            self.osr <<= n;
            d
        };
        self.osr_count = (self.osr_count + n).min(32);
        data
    }

    fn shift_in(&mut self, data: u32, n: u32) {
        let n = if n == 0 { 32 } else { n };
        if n == 32 {
            self.isr = data;
        } else {
            let d = data & ((1 << n) - 1);
            self.isr = if self.in_right() { (self.isr >> n) | (d << (32 - n)) } else { (self.isr << n) | d };
        }
        self.isr_count = (self.isr_count + n).min(32);
    }
}

#[derive(Clone, Debug)]
pub struct Pio {
    pub index: u8,
    sm: [Sm; NUM_SM],
    pub instr: [u16; 32],
    enabled: u8,
    /// The eight IRQ flags.
    pub irq: u8,
    input_sync_bypass: u32,
    fdebug: u32,
    inte: [u32; 2],
    intf: [u32; 2],
    /// What the block drives onto the GPIOs (for pins whose function is this PIO).
    pub pad_out: u32,
    pub pad_oe: u32,
    /// The `clk_sys` cycle the block has run up to.
    pub cycle: u64,
    /// Incremented on anything that may unstall a state machine.
    epoch: u64,
    last_inputs: u32,
    outputs_changed: bool,
    /// Instructions executed (for statistics).
    pub executed: u64,
}

/// Why [`Pio::run`] returned.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Stop {
    /// Reached the requested cycle.
    Reached,
    /// The block's pin outputs changed at `self.cycle`.
    Outputs,
}

impl Pio {
    pub fn new(index: u8) -> Self {
        Pio {
            index,
            sm: std::array::from_fn(|_| {
                let mut s = Sm::new();
                s.osr_count = 32;
                s
            }),
            instr: [0; 32],
            enabled: 0,
            irq: 0,
            input_sync_bypass: 0,
            fdebug: 0,
            inte: [0; 2],
            intf: [0; 2],
            pad_out: 0,
            pad_oe: 0,
            cycle: 0,
            epoch: 0,
            last_inputs: 0,
            outputs_changed: false,
            executed: 0,
        }
    }

    /// Back to the reset state, keeping the cycle count.
    pub fn reset(&mut self) {
        let cycle = self.cycle;
        *self = Pio::new(self.index);
        self.cycle = cycle;
    }

    fn bump(&mut self) {
        self.epoch += 1;
    }

    /// Whether any state machine is enabled and able to make progress on its own.
    pub fn busy(&self) -> bool {
        (0..NUM_SM)
            .any(|i| self.enabled & (1 << i) != 0 && !(self.sm[i].stalled && self.sm[i].stall_epoch == self.epoch))
    }

    /// Cycle of the next tick of state machine `i` that can do anything, or `u64::MAX`.
    fn wake(&self, i: usize) -> u64 {
        if self.enabled & (1 << i) == 0 {
            return u64::MAX;
        }
        let s = &self.sm[i];
        if s.stalled && s.stall_epoch == self.epoch {
            return u64::MAX;
        }
        let next = s.tick_cycle(s.next_k);
        if s.stalled && next <= self.cycle {
            // Something changed: retry at the next tick after now (`first_tick_after(cycle).max(next_k)`, which is
            // `next_k` itself when that tick is still ahead)
            return s.tick_cycle(s.first_tick_after(self.cycle).max(s.next_k));
        }
        next
    }

    /// Runs the block until `until` (a `clk_sys` cycle) with the GPIOs reading `inputs`. Returns early when the
    /// block's pin outputs change.
    pub fn run(&mut self, until: u64, inputs: u32) -> Stop {
        if inputs != self.last_inputs {
            self.last_inputs = inputs;
            self.bump();
        }
        loop {
            let wakes: [u64; NUM_SM] = std::array::from_fn(|i| self.wake(i));
            let next = wakes.iter().copied().min().unwrap_or(u64::MAX);
            if next > until {
                self.cycle = self.cycle.max(until);
                return Stop::Reached;
            }
            self.cycle = next;
            for (i, &wake) in wakes.iter().enumerate() {
                if wake == next {
                    let k = {
                        let s = &self.sm[i];
                        if s.stalled && s.tick_cycle(s.next_k) < next {
                            s.first_tick_after(next - 1).max(s.next_k)
                        } else {
                            s.next_k
                        }
                    };
                    self.tick(i, k, inputs);
                }
            }
            if self.outputs_changed {
                self.outputs_changed = false;
                return Stop::Outputs;
            }
        }
    }

    /// The pins a state machine sees: GPIOs rotated so that `base` is bit 0.
    fn pins_from(inputs: u32, base: u32) -> u32 {
        inputs.rotate_right(base)
    }

    fn write_pins(&mut self, base: u32, count: u32, value: u32, dirs: bool) {
        for i in 0..count.min(32) {
            let pin = (base + i) % 32;
            let bit = 1 << pin;
            let v = value & (1 << i) != 0;
            let reg = if dirs { &mut self.pad_oe } else { &mut self.pad_out };
            let old = *reg;
            if v {
                *reg |= bit;
            } else {
                *reg &= !bit;
            }
            if *reg != old {
                self.outputs_changed = true;
            }
        }
    }

    fn irq_index(field: u32, sm: usize) -> u8 {
        let idx = field & 0x7;
        if field & 0x10 != 0 { ((idx & 0x4) | ((idx + sm as u32) & 0x3)) as u8 } else { idx as u8 }
    }

    /// One tick of state machine `i`, tick index `k`.
    fn tick(&mut self, i: usize, k: u64, inputs: u32) {
        let (instr, from_mem) = match self.sm[i].exec {
            Some(e) => (e, false),
            None => (self.instr[self.sm[i].pc as usize & 31], true),
        };
        let first_issue = !self.sm[i].issued;
        let outcome = self.execute(i, instr, inputs);
        let s = &self.sm[i];
        // Side-set: applied when the instruction issues, even if it then stalls
        let count = s.sideset_count();
        let side_en = s.execctrl & (1 << 30) != 0;
        let field = ((instr >> 8) & 0x1F) as u32;
        let delay_bits = 5 - count.min(5);
        let delay = field & ((1 << delay_bits) - 1);
        if first_issue && count > 0 {
            let side = field >> delay_bits;
            let (enabled, bits, value) = if side_en {
                (side >> (count - 1) & 1 != 0, count - 1, side & ((1 << (count - 1)) - 1))
            } else {
                (true, count, side)
            };
            if enabled && bits > 0 {
                let base = s.sideset_base();
                let dirs = s.execctrl & (1 << 29) != 0;
                self.write_pins(base, bits, value, dirs);
            }
        }
        self.executed += 1;
        let s = &mut self.sm[i];
        match outcome {
            Outcome::Stall => {
                s.stalled = true;
                s.issued = true;
                s.stall_epoch = self.epoch;
                s.next_k = k + 1;
            }
            Outcome::Done { jump, delay: extra } => {
                if s.stalled {
                    s.stalled = false;
                }
                s.issued = false;
                s.exec = None;
                match jump {
                    Some(target) => s.pc = target & 31,
                    None if from_mem => s.pc = if s.pc == s.wrap_top() { s.wrap_bottom() } else { (s.pc + 1) & 31 },
                    None => {}
                }
                s.next_k = k + 1 + delay as u64 + extra as u64;
                self.epoch += 1;
            }
            Outcome::Exec(next) => {
                s.stalled = false;
                s.issued = false;
                if from_mem {
                    s.pc = if s.pc == s.wrap_top() { s.wrap_bottom() } else { (s.pc + 1) & 31 };
                }
                s.exec = Some(next);
                s.next_k = k + 1;
                self.epoch += 1;
            }
        }
        // OUT_STICKY: keep asserting the last OUT/SET
        if self.sm[i].execctrl & (1 << 17) != 0
            && let Some((base, count, v)) = self.sm[i].sticky
        {
            self.write_pins(base, count, v, false);
        }
    }

    fn status(&self, i: usize) -> u32 {
        let s = &self.sm[i];
        let n = (s.execctrl & 0xF) as usize;
        let level = if s.execctrl & (1 << 4) != 0 { s.rx.len() } else { s.tx.len() };
        if level < n { u32::MAX } else { 0 }
    }

    fn mov_source(&self, i: usize, src: u32, inputs: u32) -> Option<u32> {
        let s = &self.sm[i];
        Some(match src {
            0 => Self::pins_from(inputs, s.in_base()),
            1 => s.x,
            2 => s.y,
            3 => 0,
            5 => self.status(i),
            6 => s.isr,
            7 => s.osr,
            _ => return None,
        })
    }

    /// Writes OUT/MOV data to pins: OUT_BASE/OUT_COUNT, or with INLINE_OUT_EN one data bit as the output enable.
    fn out_pins(&mut self, i: usize, data: u32) {
        let s = &self.sm[i];
        let (base, count) = (s.out_base(), s.out_count());
        if s.execctrl & (1 << 18) != 0 {
            let en_bit = (s.execctrl >> 19) & 0x1F;
            if data & (1 << en_bit) == 0 {
                return;
            }
        }
        self.sm[i].sticky = Some((base, count, data));
        self.write_pins(base, count, data, false);
    }

    fn execute(&mut self, i: usize, instr: u16, inputs: u32) -> Outcome {
        let op = instr >> 13;
        let arg1 = ((instr >> 5) & 7) as u32;
        let arg2 = (instr & 0x1F) as u32;
        let done = Outcome::Done { jump: None, delay: 0 };
        match op {
            // JMP
            0 => {
                let s = &mut self.sm[i];
                let take = match arg1 {
                    0 => true,
                    1 => s.x == 0,
                    2 => {
                        let t = s.x != 0;
                        s.x = s.x.wrapping_sub(1);
                        t
                    }
                    3 => s.y == 0,
                    4 => {
                        let t = s.y != 0;
                        s.y = s.y.wrapping_sub(1);
                        t
                    }
                    5 => s.x != s.y,
                    6 => inputs & (1 << ((s.execctrl >> 24) & 0x1F)) != 0,
                    _ => s.osr_count < s.pull_thresh(),
                };
                Outcome::Done { jump: take.then_some(arg2 as u8), delay: 0 }
            }
            // WAIT
            1 => {
                let pol = arg1 & 4 != 0;
                let ok = match arg1 & 3 {
                    0 => (inputs & (1 << arg2) != 0) == pol,
                    1 => {
                        let pin = (self.sm[i].in_base() + arg2) % 32;
                        (inputs & (1 << pin) != 0) == pol
                    }
                    2 => {
                        let f = Self::irq_index(arg2, i);
                        let set = self.irq & (1 << f) != 0;
                        if set == pol && pol {
                            self.irq &= !(1 << f);
                        }
                        set == pol
                    }
                    _ => true,
                };
                if ok { done } else { Outcome::Stall }
            }
            // IN
            2 => {
                let s = &self.sm[i];
                let n = if arg2 == 0 { 32 } else { arg2 };
                let data = match arg1 {
                    0 => Self::pins_from(inputs, s.in_base()),
                    1 => s.x,
                    2 => s.y,
                    6 => s.isr,
                    7 => s.osr,
                    _ => 0,
                };
                let push = s.autopush() && (s.isr_count + n).min(32) >= s.push_thresh();
                if push && s.rx.len() >= s.rx_cap() {
                    self.fdebug |= 1 << i;
                    return Outcome::Stall;
                }
                let s = &mut self.sm[i];
                s.shift_in(data, arg2);
                if push {
                    let v = s.isr;
                    s.rx.push_back(v);
                    s.isr = 0;
                    s.isr_count = 0;
                }
                done
            }
            // OUT
            3 => {
                let s = &mut self.sm[i];
                if s.autopull() && s.osr_count >= s.pull_thresh() {
                    match s.tx.pop_front() {
                        Some(v) => {
                            s.osr = v;
                            s.osr_count = 0;
                        }
                        None => {
                            self.fdebug |= 1 << (24 + i);
                            return Outcome::Stall;
                        }
                    }
                }
                let n = if arg2 == 0 { 32 } else { arg2 };
                let data = s.shift_out(arg2);
                let mut out = done;
                match arg1 {
                    0 => self.out_pins(i, data),
                    1 => self.sm[i].x = data,
                    2 => self.sm[i].y = data,
                    3 => {}
                    4 => {
                        let (base, count) = (self.sm[i].out_base(), self.sm[i].out_count());
                        self.write_pins(base, count, data, true);
                    }
                    5 => out = Outcome::Done { jump: Some((data & 31) as u8), delay: 0 },
                    6 => {
                        let s = &mut self.sm[i];
                        s.isr = data;
                        s.isr_count = n;
                    }
                    _ => out = Outcome::Exec(data as u16),
                }
                let s = &mut self.sm[i];
                if s.autopull() && s.osr_count >= s.pull_thresh() && !s.tx.is_empty() {
                    s.osr = s.tx.pop_front().unwrap();
                    s.osr_count = 0;
                }
                out
            }
            // PUSH / PULL
            4 => {
                let if_cond = instr & (1 << 6) != 0;
                let block = instr & (1 << 5) != 0;
                let s = &mut self.sm[i];
                if instr & (1 << 7) == 0 {
                    if if_cond && s.isr_count < s.push_thresh() {
                        return done;
                    }
                    if s.rx.len() >= s.rx_cap() {
                        if block {
                            self.fdebug |= 1 << i;
                            return Outcome::Stall;
                        }
                    } else {
                        let v = s.isr;
                        s.rx.push_back(v);
                    }
                    s.isr = 0;
                    s.isr_count = 0;
                } else {
                    if if_cond && s.osr_count < s.pull_thresh() {
                        return done;
                    }
                    match s.tx.pop_front() {
                        Some(v) => s.osr = v,
                        None if block => {
                            self.fdebug |= 1 << (24 + i);
                            return Outcome::Stall;
                        }
                        None => s.osr = s.x,
                    }
                    s.osr_count = 0;
                }
                done
            }
            // MOV
            5 => {
                let Some(mut v) = self.mov_source(i, arg2 & 7, inputs) else { return done };
                match (instr >> 3) & 3 {
                    1 => v = !v,
                    2 => v = v.reverse_bits(),
                    _ => {}
                }
                let s = &mut self.sm[i];
                match arg1 {
                    0 => self.out_pins(i, v),
                    1 => s.x = v,
                    2 => s.y = v,
                    4 => return Outcome::Exec(v as u16),
                    5 => return Outcome::Done { jump: Some((v & 31) as u8), delay: 0 },
                    6 => {
                        s.isr = v;
                        s.isr_count = 0;
                    }
                    7 => {
                        s.osr = v;
                        s.osr_count = 0;
                    }
                    _ => {}
                }
                done
            }
            // IRQ
            6 => {
                let f = Self::irq_index(arg2, i);
                let clear = instr & (1 << 6) != 0;
                let wait = instr & (1 << 5) != 0;
                if let Some(w) = self.sm[i].irq_wait {
                    if self.irq & (1 << w) != 0 {
                        return Outcome::Stall;
                    }
                    self.sm[i].irq_wait = None;
                    return done;
                }
                if clear {
                    self.irq &= !(1 << f);
                    return done;
                }
                self.irq |= 1 << f;
                if wait {
                    self.sm[i].irq_wait = Some(f);
                    return Outcome::Stall;
                }
                done
            }
            // SET
            _ => {
                let s = &mut self.sm[i];
                match arg1 {
                    0 => {
                        let (base, count) = (s.set_base(), s.set_count());
                        s.sticky = Some((base, count, arg2));
                        self.write_pins(base, count, arg2, false);
                    }
                    1 => s.x = arg2,
                    2 => s.y = arg2,
                    4 => {
                        let (base, count) = (s.set_base(), s.set_count());
                        self.write_pins(base, count, arg2, true);
                    }
                    _ => {}
                }
                done
            }
        }
    }

    // ------------------------------------------------------------------ system side

    pub fn tx_dreq(&self, sm: usize) -> bool {
        let s = &self.sm[sm];
        s.tx.len() < s.tx_cap()
    }

    pub fn rx_dreq(&self, sm: usize) -> bool {
        !self.sm[sm].rx.is_empty()
    }

    fn intr(&self) -> u32 {
        let mut v = ((self.irq & 0xF) as u32) << 8;
        for (i, s) in self.sm.iter().enumerate() {
            if !s.rx.is_empty() {
                v |= 1 << i;
            }
            if s.tx.len() < s.tx_cap() {
                v |= 1 << (4 + i);
            }
        }
        v
    }

    /// Interrupt line `n` (0 or 1) of the block.
    pub fn irq_line(&self, n: usize) -> bool {
        // Most blocks enable no interrupts: skip collecting the raw status
        self.inte[n] & 0xFFF != 0 && (self.intr() | self.intf[n]) & self.inte[n] & 0xFFF != 0
    }

    /// Executes an instruction written to SMx_INSTR: immediately (at the current cycle), whether or not the state
    /// machine is enabled. If it stalls it stays pending and is retried on the state machine's ticks.
    fn force(&mut self, i: usize, instr: u16, inputs: u32) {
        self.sm[i].exec = Some(instr);
        self.sm[i].issued = false;
        self.sm[i].stalled = false;
        let k = self.sm[i].next_k.saturating_sub(1);
        self.tick(i, k, inputs);
        // A forced instruction does not consume the state machine's own tick schedule
        let s = &mut self.sm[i];
        if self.enabled & (1 << i) == 0 {
            s.next_k = 1;
        }
    }

    pub fn read(&mut self, offset: u32) -> Option<u32> {
        Some(match offset {
            CTRL => self.enabled as u32,
            FSTAT => {
                let mut v = 0;
                for (i, s) in self.sm.iter().enumerate() {
                    if s.rx.len() >= s.rx_cap() {
                        v |= 1 << i;
                    }
                    if s.rx.is_empty() {
                        v |= 1 << (8 + i);
                    }
                    if s.tx.len() >= s.tx_cap() {
                        v |= 1 << (16 + i);
                    }
                    if s.tx.is_empty() {
                        v |= 1 << (24 + i);
                    }
                }
                v
            }
            FDEBUG => self.fdebug,
            FLEVEL => self.sm.iter().enumerate().fold(0, |v, (i, s)| {
                v | ((s.tx.len() as u32 & 0xF) << (8 * i)) | ((s.rx.len() as u32 & 0xF) << (8 * i + 4))
            }),
            0x010..=0x01C => 0,
            0x020..=0x02C => {
                let i = ((offset - RXF0) / 4) as usize;
                match self.sm[i].rx.pop_front() {
                    Some(v) => {
                        self.bump();
                        v
                    }
                    None => {
                        self.fdebug |= 1 << (8 + i);
                        0
                    }
                }
            }
            IRQ => self.irq as u32,
            IRQ_FORCE => 0,
            0x038 => self.input_sync_bypass,
            0x03C => self.pad_out,
            0x040 => self.pad_oe,
            0x044 => 0x0020_0404,
            0x048..=0x0C4 => 0,
            0x0C8..=0x124 => {
                let i = ((offset - SM0_CLKDIV) / SM_STRIDE) as usize;
                let s = &self.sm[i];
                match (offset - SM0_CLKDIV) % SM_STRIDE {
                    0x00 => s.clkdiv,
                    0x04 => s.execctrl | if s.stalled && s.exec.is_some() { 1 << 31 } else { 0 },
                    0x08 => s.shiftctrl,
                    0x0C => s.pc as u32,
                    0x10 => s.exec.unwrap_or(self.instr[s.pc as usize & 31]) as u32,
                    _ => s.pinctrl,
                }
            }
            INTR => self.intr(),
            0x12C => self.inte[0],
            0x130 => self.intf[0],
            0x134 => (self.intr() | self.intf[0]) & self.inte[0],
            0x138 => self.inte[1],
            0x13C => self.intf[1],
            0x140 => (self.intr() | self.intf[1]) & self.inte[1],
            _ => return None,
        })
    }

    pub fn peek(&self, offset: u32) -> Option<u32> {
        match offset {
            0x020..=0x02C => Some(self.sm[((offset - RXF0) / 4) as usize].rx.front().copied().unwrap_or(0)),
            _ => self.clone().read(offset),
        }
    }

    /// A register write at the block's current cycle (run the block up to the write's time first).
    pub fn write(&mut self, offset: u32, value: u32, mode: WriteMode, inputs: u32) -> bool {
        self.bump();
        match offset {
            CTRL => {
                let v = mode.apply(self.enabled as u32, value);
                let new = (v & 0xF) as u8;
                for i in 0..NUM_SM {
                    let bit = 1u32 << i;
                    if new as u32 & bit != 0 && self.enabled as u32 & bit == 0 {
                        // Enabled: the clock starts now
                        let s = &mut self.sm[i];
                        s.anchor = self.cycle;
                        s.next_k = 1;
                    }
                    if v & (bit << 4) != 0 {
                        let cycle = self.cycle;
                        let s = &mut self.sm[i];
                        s.restart();
                        s.next_k = s.first_tick_after(cycle);
                    }
                    if v & (bit << 8) != 0 {
                        let s = &mut self.sm[i];
                        s.anchor = self.cycle;
                        s.next_k = 1;
                    }
                }
                self.enabled = new;
            }
            FDEBUG => self.fdebug &= !mode.w1c_bits(value),
            0x010..=0x01C => {
                let i = ((offset - TXF0) / 4) as usize;
                let mut v = 0;
                mode.update(&mut v, value, u32::MAX);
                let s = &mut self.sm[i];
                if s.tx.len() >= s.tx_cap() {
                    self.fdebug |= 1 << (16 + i);
                } else {
                    s.tx.push_back(v);
                }
            }
            IRQ => self.irq &= !(mode.w1c_bits(value) as u8),
            IRQ_FORCE => {
                let mut v = 0;
                mode.update(&mut v, value, 0xFF);
                self.irq |= v as u8;
            }
            0x038 => mode.update(&mut self.input_sync_bypass, value, u32::MAX),
            0x048..=0x0C4 => {
                let i = ((offset - INSTR_MEM0) / 4) as usize;
                let mut v = self.instr[i] as u32;
                mode.update(&mut v, value, 0xFFFF);
                self.instr[i] = v as u16;
            }
            0x0C8..=0x124 => {
                let i = ((offset - SM0_CLKDIV) / SM_STRIDE) as usize;
                let cycle = self.cycle;
                let s = &mut self.sm[i];
                match (offset - SM0_CLKDIV) % SM_STRIDE {
                    0x00 => {
                        // Keep the phase: re-anchor at the next tick under the new divider
                        let k = s.next_k;
                        let next = s.tick_cycle(k).max(cycle);
                        mode.update(&mut s.clkdiv, value, 0xFFFF_FF00);
                        s.anchor = next.saturating_sub(s.div() >> 8);
                        s.next_k = 1;
                    }
                    0x04 => mode.update(&mut s.execctrl, value, 0x7FFF_FF9F),
                    0x08 => {
                        let old = s.shiftctrl;
                        mode.update(&mut s.shiftctrl, value, 0xFFFF_0000);
                        if (old ^ s.shiftctrl) & (3 << 30) != 0 {
                            s.tx.clear();
                            s.rx.clear();
                        }
                    }
                    0x0C => {}
                    0x10 => {
                        let mut v = 0;
                        mode.update(&mut v, value, 0xFFFF);
                        self.force(i, v as u16, inputs);
                    }
                    _ => mode.update(&mut s.pinctrl, value, u32::MAX),
                }
            }
            INTR | 0x134 | 0x140 => {}
            0x12C => mode.update(&mut self.inte[0], value, 0xFFF),
            0x130 => mode.update(&mut self.intf[0], value, 0xFFF),
            0x138 => mode.update(&mut self.inte[1], value, 0xFFF),
            0x13C => mode.update(&mut self.intf[1], value, 0xFFF),
            FSTAT | FLEVEL | 0x020..=0x02C | 0x03C..=0x044 => {}
            _ => return false,
        }
        true
    }

    /// Whether the last register write changed the pin outputs (SMx_INSTR executing SET PINS and the like).
    pub fn take_outputs_changed(&mut self) -> bool {
        std::mem::take(&mut self.outputs_changed)
    }

    /// State machine `i`'s program counter, X and Y, for tests and debugging.
    pub fn sm_regs(&self, i: usize) -> (u8, u32, u32, u32, u32) {
        let s = &self.sm[i];
        (s.pc, s.x, s.y, s.isr, s.osr)
    }

    pub fn sm_stalled(&self, i: usize) -> bool {
        self.sm[i].stalled
    }
}

#[cfg(test)]
mod tests;
