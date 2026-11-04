//! Busy-waits on a timer that last many of its ticks: [`CortexM0Plus::timer_wait`] works out how long a loop that
//! polls a free-running timer goes round the same way, so that the machine can skip it up to its deadline rather
//! than tick by tick.
//!
//! The loop is executed abstractly from the core's state: registers hold a known value, the low word of a timer
//! read (`Lo`), or some other value that depends on one (`Dead`); flags hold a known value or a predicate on the
//! low word read (`Lo >= k`, `Lo == k`...). The timer's high word reads as it is now, which holds while the low
//! word does not wrap. A conditional branch on a predicate goes the way it goes now as long as every read gives a
//! low word below the first value at which the predicate changes. When the abstract state at an address comes back
//! the same after one more time round, every further time round takes the same path and comes back to it again,
//! as long as the reads stay below that bound.
//!
//! The registers and flags whose value depends on the timer at that address (`Lo`, `Dead`, predicates) are left as
//! they were when the machine lands the core there: each is written, before it is read, before the first branch
//! that depends on the timer, so that the time round the core then runs (the machine leaves room for one before
//! anything can look at the core) gives them the values running every time round would have.
//!
//! Only a few kinds of instructions are understood (moves, additions, subtractions and comparisons, a few logic
//! operations, loads, branches); anything else, a store, or a timer value used any other way, and there is no such
//! loop.

use sim_core::bus::Size;

use crate::CortexM0Plus;
use crate::block::{K, Op, adc, decode16};

/// What reading an address gives a busy-wait loop, as far as [`CortexM0Plus::timer_wait`] needs to know.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaitRead {
    /// Memory that keeps this value while the loop runs (nothing but the loop runs meanwhile, and it stores
    /// nothing).
    Value(u32),
    /// The low word of a free-running timer that counts up by one per tick (a word read).
    TimerLow,
    /// The timer's high word (a word read): it reads as it does now while the low word does not wrap.
    TimerHigh,
    /// Anything else.
    Other,
}

/// A busy-wait loop found by [`CortexM0Plus::timer_wait`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TimerWait {
    /// The address the loop is landed at.
    pub head: u32,
    /// Instructions and cycles from the core's PC to the first arrival at `head`.
    pub prefix_steps: u32,
    pub prefix_cycles: u64,
    /// Instructions and cycles of one time round, from `head` back to it.
    pub steps: u32,
    pub cycles: u64,
    /// The start of the last timer read before the first arrival at `head` (cycles from now), if any.
    pub prefix_read: Option<u64>,
    /// The start of the last timer read in a time round (cycles from its start), if any.
    pub read: Option<u64>,
    /// The loop goes round the same way while every timer read gives a low word below this (at most 2^32: the
    /// high word reads as it does now).
    pub below: u64,
    /// The registers known at `head` (the others are left alone when landing).
    regs: [Option<u32>; 15],
    /// The flags (N, Z, C, V) known at `head`.
    flags: [Option<bool>; 4],
}

/// Instructions looked at, at most (a prefix and two times round a loop).
const MAX_STEPS: usize = 160;

/// An abstract register value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum V {
    Known(u32),
    /// The low word of a timer read.
    Lo,
    /// A value that depends on a timer read some other way.
    Dead,
}

/// An abstract flag: known, or a predicate on the low word of a timer read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum P {
    Known(bool),
    /// `Lo >= k` (k up to 2^32).
    Ge(u64),
    /// `Lo < k`.
    Lt(u64),
    Eq(u32),
    Ne(u32),
    Unknown,
}

const WRAP: u64 = 1 << 32;

impl P {
    fn not(self) -> P {
        match self {
            P::Known(b) => P::Known(!b),
            P::Ge(k) => P::Lt(k),
            P::Lt(k) => P::Ge(k),
            P::Eq(k) => P::Ne(k),
            P::Ne(k) => P::Eq(k),
            P::Unknown => P::Unknown,
        }
    }

    /// The predicate's value for a low word `lo`, and the first low word above `lo` at which it may change
    /// ([`WRAP`]: not before the low word wraps).
    fn at(self, lo: u32) -> Option<(bool, u64)> {
        let lo = lo as u64;
        Some(match self {
            P::Known(b) => (b, WRAP),
            P::Ge(k) if lo >= k => (true, WRAP),
            P::Ge(k) => (false, k),
            P::Lt(k) => P::Ge(k).at(lo as u32).map(|(b, until)| (!b, until))?,
            P::Eq(k) if lo == k as u64 => (true, k as u64 + 1),
            P::Eq(k) if lo < k as u64 => (false, k as u64),
            P::Eq(_) => (false, WRAP),
            P::Ne(k) => P::Eq(k).at(lo as u32).map(|(b, until)| (!b, until))?,
            P::Unknown => return None,
        })
    }

    fn known(self) -> Option<bool> {
        match self {
            P::Known(b) => Some(b),
            _ => None,
        }
    }
}

/// N, Z, C, V.
type Flags = [P; 4];

/// What one instruction did, for the checks on a loop.
#[derive(Clone, Copy, Default)]
struct Access {
    /// Registers read and written (bit per register).
    reads: u16,
    writes: u16,
    /// Flags read and written (N, Z, C, V: bits 0 to 3).
    flags_read: u8,
    flags_written: u8,
    /// A branch whose way depends on a timer read.
    decides: bool,
    /// A timer read.
    timer: bool,
}

/// The abstract state at an instruction.
#[derive(Clone, Copy, PartialEq, Eq)]
struct State {
    pc: u32,
    regs: [V; 15],
    flags: Flags,
}

/// One instruction looked at: the state before it, the cycles before it (from the start), and what it did.
struct Step {
    state: State,
    cycles: u64,
    access: Access,
}

/// Flags of `x - y` (`CMP`, `SUBS`) and the result.
fn sub(x: V, y: V) -> (V, Flags) {
    match (x, y) {
        (V::Known(a), V::Known(b)) => {
            let (r, c, v) = adc(a, !b, true);
            (V::Known(r), [P::Known(r >> 31 != 0), P::Known(r == 0), P::Known(c), P::Known(v)])
        }
        // Lo - b: no borrow when Lo >= b
        (V::Lo, V::Known(b)) => (V::Dead, [P::Unknown, P::Eq(b), P::Ge(b as u64), P::Unknown]),
        // a - Lo: no borrow when Lo <= a
        (V::Known(a), V::Lo) => (V::Dead, [P::Unknown, P::Eq(a), P::Lt(a as u64 + 1), P::Unknown]),
        _ => (V::Dead, [P::Unknown; 4]),
    }
}

/// `x + y + carry` (`ADCS`; `SBCS` with `y` inverted) with the carry in a predicate: known operands only.
fn add_carry(x: V, y: V, carry: P) -> (V, Flags) {
    let (V::Known(a), V::Known(b)) = (x, y) else { return (V::Dead, [P::Unknown; 4]) };
    if let P::Known(c) = carry {
        let (r, c, v) = adc(a, b, c);
        return (V::Known(r), [P::Known(r >> 31 != 0), P::Known(r == 0), P::Known(c), P::Known(v)]);
    }
    let flags = |r: u32, c: bool, v: bool| [r >> 31 != 0, r == 0, c, v];
    let (r1, c1, v1) = adc(a, b, true);
    let (r0, c0, v0) = adc(a, b, false);
    let (f1, f0) = (flags(r1, c1, v1), flags(r0, c0, v0));
    // Each flag is known, or follows the carry in (or its opposite)
    let flag = |i: usize| match (f1[i], f0[i]) {
        (a, b) if a == b => P::Known(a),
        (true, false) => carry,
        _ => carry.not(),
    };
    let r = if r1 == r0 { V::Known(r1) } else { V::Dead };
    (r, [flag(0), flag(1), flag(2), flag(3)])
}

impl CortexM0Plus {
    /// Looks for a busy-wait loop from the core's current state that polls a timer (see the module docs):
    /// `fetch` gives the code, `read` what a data read gives, `now` is the timer's value now. `None` if the code
    /// from here is not such a loop (or not one this understands).
    pub fn timer_wait(
        &self,
        mut fetch: impl FnMut(u32) -> Option<u16>,
        mut read: impl FnMut(u32, Size) -> WaitRead,
        now: u64,
    ) -> Option<TimerWait> {
        let mut regs = [V::Dead; 15];
        for (v, &r) in regs.iter_mut().zip(&self.r) {
            *v = V::Known(r);
        }
        let mut s = State {
            pc: self.r[15],
            regs,
            flags: [P::Known(self.n), P::Known(self.z), P::Known(self.c), P::Known(self.v)],
        };
        let mut steps: Vec<Step> = Vec::with_capacity(MAX_STEPS);
        let mut cycles = 0u64;
        let mut below = WRAP;
        while steps.len() < MAX_STEPS {
            // Back where the state was the same: a loop, if landing there is safe
            if let Some(i) = steps.iter().position(|p| p.state == s)
                && let Some(w) = self.landing(&steps, i, cycles, below)
            {
                return Some(w);
            }
            let ins = fetch(s.pc)? as u32;
            if ins >> 11 >= 0b11101 {
                return None;
            }
            let op = decode16(ins, s.pc);
            let before = s;
            let (access, taken, c) = step(&mut s, op, now, &mut read, &mut below)?;
            steps.push(Step { state: before, cycles, access });
            cycles += c as u64;
            if !taken {
                s.pc = before.pc.wrapping_add(2);
            }
        }
        None
    }

    /// The loop that the steps from `i` on go round, landed at its start, if that is safe (see the module docs).
    fn landing(&self, steps: &[Step], i: usize, cycles: u64, below: u64) -> Option<TimerWait> {
        let head = &steps[i].state;
        let round = &steps[i..];
        // Registers and flags the timer decides at the head must be written before they are read and before the
        // first branch the timer decides
        let first_decision = round.iter().position(|p| p.access.decides).unwrap_or(round.len());
        let unknown_regs: u16 =
            head.regs.iter().enumerate().filter(|(_, v)| !matches!(v, V::Known(_))).fold(0, |m, (r, _)| m | 1 << r);
        let unknown_flags: u8 =
            head.flags.iter().enumerate().filter(|(_, f)| f.known().is_none()).fold(0, |m, (f, _)| m | 1 << f);
        let (mut regs_left, mut flags_left) = (unknown_regs, unknown_flags);
        for p in &round[..first_decision] {
            if p.access.reads & regs_left != 0 || p.access.flags_read & flags_left != 0 {
                return None;
            }
            regs_left &= !p.access.writes;
            flags_left &= !p.access.flags_written;
        }
        if regs_left != 0 || flags_left != 0 {
            return None;
        }
        let last_read = |from: usize, to: usize| {
            steps[from..to].iter().rev().find(|p| p.access.timer).map(|p| p.cycles - steps[from].cycles)
        };
        let mut regs = [None; 15];
        for (r, v) in regs.iter_mut().zip(&head.regs) {
            if let V::Known(x) = v {
                *r = Some(*x);
            }
        }
        let flags = head.flags.map(P::known);
        Some(TimerWait {
            head: head.pc,
            prefix_steps: i as u32,
            prefix_cycles: steps[i].cycles,
            steps: round.len() as u32,
            cycles: cycles - steps[i].cycles,
            prefix_read: last_read(0, i),
            read: last_read(i, steps.len()),
            below,
            regs,
            flags,
        })
    }

    /// Lands the core at the start of a loop [`CortexM0Plus::timer_wait`] found (the machine accounts for the
    /// skipped instructions and cycles): the known registers and flags as they are there, the others as they are.
    pub fn land_timer_wait(&mut self, w: &TimerWait) {
        for (r, v) in self.r.iter_mut().zip(&w.regs) {
            if let Some(v) = v {
                *r = *v;
            }
        }
        for (f, v) in [&mut self.n, &mut self.z, &mut self.c, &mut self.v].into_iter().zip(w.flags) {
            if let Some(v) = v {
                *f = v;
            }
        }
        self.r[15] = w.head;
    }
}

/// Executes `op` abstractly. Returns what it accessed, whether it set the PC (a branch taken) and its cycles;
/// `None` for an instruction this does not understand. Narrows `below` to keep the branches' ways.
fn step(
    s: &mut State,
    op: Op,
    now: u64,
    read: &mut impl FnMut(u32, Size) -> WaitRead,
    below: &mut u64,
) -> Option<(Access, bool, u32)> {
    let (a, b, imm) = (op.a as usize & 15, op.b as usize & 15, op.imm);
    let m = imm as usize & 15;
    let lo = now as u32;
    let mut acc = Access::default();
    if a == 15 || b == 15 {
        return None;
    }
    let get = |s: &State, acc: &mut Access, r: usize| {
        acc.reads |= 1 << r;
        s.regs[r]
    };
    let set = |s: &mut State, acc: &mut Access, r: usize, v: V| {
        acc.writes |= 1 << r;
        s.regs[r] = v;
    };
    // N and Z of a result (for logic operations and moves)
    let nz = |v: V| match v {
        V::Known(x) => [P::Known(x >> 31 != 0), P::Known(x == 0)],
        V::Lo => [P::Ge(1 << 31), P::Eq(0)],
        V::Dead => [P::Unknown, P::Unknown],
    };
    let set_flags = |s: &mut State, acc: &mut Access, f: Flags, mask: u8| {
        for (i, flag) in f.iter().enumerate() {
            if mask & 1 << i != 0 {
                s.flags[i] = *flag;
            }
        }
        acc.flags_written |= mask;
    };
    let logic = |x: V, y: V, f: fn(u32, u32) -> u32| match (x, y) {
        (V::Known(x), V::Known(y)) => V::Known(f(x, y)),
        _ => V::Dead,
    };
    let mut cycles = 1;
    let mut taken = false;
    match op.kind {
        K::MovI => {
            set(s, &mut acc, a, V::Known(imm));
            let f = nz(V::Known(imm));
            set_flags(s, &mut acc, [f[0], f[1], P::Unknown, P::Unknown], 0b0011);
        }
        K::Movs => {
            let v = get(s, &mut acc, b);
            set(s, &mut acc, a, v);
            let f = nz(v);
            set_flags(s, &mut acc, [f[0], f[1], P::Unknown, P::Unknown], 0b0011);
        }
        K::MovHi => {
            if a == 13 {
                return None;
            }
            let v = get(s, &mut acc, b);
            set(s, &mut acc, a, v);
        }
        K::AddR | K::SubR | K::AddI3 | K::SubI3 | K::AddI8 | K::SubI8 | K::CmpI | K::CmpR | K::CmpHi => {
            let (x, y, dest) = match op.kind {
                K::AddR | K::SubR => (get(s, &mut acc, b), get(s, &mut acc, m), Some(a)),
                K::AddI3 | K::SubI3 => (get(s, &mut acc, b), V::Known(imm), Some(a)),
                K::AddI8 | K::SubI8 => (get(s, &mut acc, a), V::Known(imm), Some(a)),
                K::CmpI => (get(s, &mut acc, a), V::Known(imm), None),
                _ => (get(s, &mut acc, a), get(s, &mut acc, b), None),
            };
            let (r, f) = match op.kind {
                K::AddR | K::AddI3 | K::AddI8 => add_carry(x, y, P::Known(false)),
                _ => sub(x, y),
            };
            if let Some(d) = dest {
                set(s, &mut acc, d, r);
            }
            set_flags(s, &mut acc, f, 0b1111);
        }
        K::Adc | K::Sbc => {
            let x = get(s, &mut acc, a);
            let y = get(s, &mut acc, b);
            acc.flags_read |= 0b0100;
            let y = match (op.kind, y) {
                (K::Sbc, V::Known(y)) => V::Known(!y),
                (K::Sbc, _) => V::Dead,
                _ => y,
            };
            let (r, f) = add_carry(x, y, s.flags[2]);
            set(s, &mut acc, a, r);
            set_flags(s, &mut acc, f, 0b1111);
        }
        K::And | K::Eor | K::Orr | K::Bic | K::Tst => {
            let x = get(s, &mut acc, a);
            let y = get(s, &mut acc, b);
            let f: fn(u32, u32) -> u32 = match op.kind {
                K::And | K::Tst => |x, y| x & y,
                K::Eor => |x, y| x ^ y,
                K::Orr => |x, y| x | y,
                _ => |x, y| x & !y,
            };
            let r = logic(x, y, f);
            if op.kind != K::Tst {
                set(s, &mut acc, a, r);
            }
            let f = nz(r);
            set_flags(s, &mut acc, [f[0], f[1], P::Unknown, P::Unknown], 0b0011);
        }
        K::Mvn => {
            let r = logic(get(s, &mut acc, b), V::Known(0), |x, _| !x);
            set(s, &mut acc, a, r);
            let f = nz(r);
            set_flags(s, &mut acc, [f[0], f[1], P::Unknown, P::Unknown], 0b0011);
        }
        K::LslI | K::LsrI => {
            let x = get(s, &mut acc, b);
            let (r, c) = match x {
                V::Known(v) if op.kind == K::LslI => (V::Known(v << imm), P::Known((v >> (32 - imm)) & 1 != 0)),
                V::Known(v) => (V::Known(v >> imm), P::Known((v >> (imm - 1)) & 1 != 0)),
                _ => (V::Dead, P::Unknown),
            };
            set(s, &mut acc, a, r);
            let f = nz(r);
            set_flags(s, &mut acc, [f[0], f[1], c, P::Unknown], 0b0111);
        }
        K::Uxtb | K::Uxth => {
            let x = get(s, &mut acc, b);
            let mask = if op.kind == K::Uxtb { 0xFF } else { 0xFFFF };
            set(s, &mut acc, a, logic(x, V::Known(mask), |x, y| x & y));
        }
        K::Set => set(s, &mut acc, a, V::Known(imm)),
        K::AddSp => {
            let sp = get(s, &mut acc, 13);
            set(s, &mut acc, a, logic(sp, V::Known(imm), u32::wrapping_add));
        }
        K::Nop => {}
        K::Ldr | K::LdrI | K::LdrR | K::LdrbI | K::LdrbR | K::LdrhI | K::LdrhR => {
            let addr = match op.kind {
                K::Ldr => V::Known(imm),
                K::LdrI | K::LdrbI | K::LdrhI => logic(get(s, &mut acc, b), V::Known(imm), u32::wrapping_add),
                _ => logic(get(s, &mut acc, b), get(s, &mut acc, m), u32::wrapping_add),
            };
            let V::Known(addr) = addr else { return None };
            let size = match op.kind {
                K::LdrbI | K::LdrbR => Size::Byte,
                K::LdrhI | K::LdrhR => Size::Half,
                _ => Size::Word,
            };
            let v = match read(addr, size) {
                WaitRead::Value(x) => V::Known(x),
                WaitRead::TimerLow if size == Size::Word => {
                    acc.timer = true;
                    V::Lo
                }
                // While the low word does not wrap (`below` is at most that)
                WaitRead::TimerHigh if size == Size::Word => {
                    acc.timer = true;
                    V::Known((now >> 32) as u32)
                }
                _ => return None,
            };
            set(s, &mut acc, a, v);
            cycles = 2;
        }
        K::B => {
            s.pc = imm;
            taken = true;
            cycles = 2;
        }
        K::BCond => {
            let f = s.flags;
            let (uses, value): (u8, Option<(bool, u64)>) = match a >> 1 {
                0 => (0b0010, f[1].at(lo)),
                1 => (0b0100, f[2].at(lo)),
                2 => (0b0001, f[0].at(lo)),
                3 => (0b1000, f[3].at(lo)),
                4 => (0b0110, f[2].at(lo).zip(f[1].at(lo)).map(|((c, cu), (z, zu))| (c && !z, cu.min(zu)))),
                // GE, GT, LE: N and V must be known
                _ => {
                    let (n, v, z) = (f[0].known()?, f[3].known()?, f[1].at(lo)?);
                    let r = if a >> 1 == 5 { n == v } else { !z.0 && n == v };
                    (0b1011, Some((r, z.1)))
                }
            };
            acc.flags_read |= uses;
            let (mut pass, until) = value?;
            if a & 1 == 1 {
                pass = !pass;
            }
            if until < WRAP {
                acc.decides = true;
            }
            *below = (*below).min(until);
            if pass {
                s.pc = imm;
                taken = true;
                cycles = 2;
            }
        }
        _ => return None,
    }
    Some((acc, taken, cycles))
}
