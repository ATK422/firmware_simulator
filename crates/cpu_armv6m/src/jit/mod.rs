//! The JIT: decoded blocks ([`crate::block`]) compiled to native code with Cranelift, once they are hot.
//!
//! A compiled block does exactly what [`CortexM0Plus::run_blocks`] does for the block when it runs all of its ops
//! (from the first, with no step limit or budget able to end the batch before the last; `run_blocks` checks that, as
//! for the block executor): same results, flags, cycles, the same accesses in the same order stamped with the same
//! time, the same checks after them and the same exits. Register and flag operations, address arithmetic, the
//! condition of a conditional branch and the code-write check are emitted inline; everything that touches the bus,
//! the exception state or a multi-word transfer calls back into Rust through the thunks here, which run the very
//! code the block executor runs (`read_mem`, `write_mem`, `op_stm`, `op_bx`, ...). The instructions the block executor
//! leaves to the interpreter (`K::Step`) are run by the interpreter, through a thunk.
//!
//! The thunks are monomorphised for the batch's bus type: the batch hands the compiled code a table of them in its
//! [`Ctx`], so one compiled block serves any bus. A panic in a thunk is caught there, ends the block and is resumed
//! once the compiled code has returned (it must not unwind through it).
//!
//! Compiled code lives as long as the [`crate::BlockCache`] that compiled it, and an entry's code is dropped when the
//! entry is decoded again, which `BlockCache::lookup` does whenever memory no longer holds the code the block was
//! decoded from: invalidation is the block cache's code-change detection.

// (Without a code generator, some of the protocol is only spoken by one side)
#![cfg_attr(not(any(target_arch = "x86_64", target_arch = "aarch64")), allow(dead_code))]

#[cfg(any(target_arch = "x86_64", target_arch = "aarch64"))]
mod codegen;
#[cfg(any(target_arch = "x86_64", target_arch = "aarch64"))]
pub(crate) use codegen::Compiler;

#[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
mod unsupported;
#[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
pub(crate) use unsupported::Compiler;

use std::any::Any;
use std::panic::AssertUnwindSafe;

use sim_core::bus::{Bus, Size};

use crate::{CortexM0Plus, Fault};

/// A compiled block: takes a [`Ctx`] and returns `kind << 8 | ops run` (see [`EXIT_NEXT`]).
pub(crate) type JitFn = unsafe extern "C" fn(*mut Ctx) -> u32;

/// The block ran to its end, or stopped after an op as the block executor does (a slow access, a stall, a store
/// that may have changed its code, a taken conditional branch): the PC is at the next instruction.
pub(crate) const EXIT_NEXT: u32 = 0;
/// An access needs the machine's attention: the batch ends (the PC is at the next instruction).
pub(crate) const EXIT_BATCH: u32 = 1;
/// The last op run faulted ([`Ctx::fault`]), or a thunk panicked ([`Ctx::panic`]).
pub(crate) const EXIT_FAULT: u32 = 2;

/// What a thunk's result has set when the access faulted (or panicked).
pub(crate) const FAULT_BIT: u64 = 1 << 32;
/// [`post`]'s result: the op ends the block, and whether for the machine's attention.
pub(crate) const POST_EXIT: u64 = 1 << 32;
pub(crate) const POST_ATTENTION: u64 = 1 << 33;

/// Multi-word transfers ([`multi`]'s `kind`).
pub(crate) const STM: u32 = 0;
pub(crate) const LDM: u32 = 1;
pub(crate) const PUSH: u32 = 2;
pub(crate) const POP: u32 = 3;
/// Branches that may do more than branch ([`branch`]'s `kind`).
pub(crate) const BX: u32 = 0;
pub(crate) const POP_PC: u32 = 1;
pub(crate) const STEP: u32 = 2;

/// The thunks a compiled block calls, for one bus type.
#[repr(C)]
pub(crate) struct Thunks {
    /// `(ctx, addr, size in bytes, elapsed) -> value | FAULT_BIT`
    pub read: unsafe extern "C" fn(*mut Ctx, u32, u32, u64) -> u64,
    /// `(ctx, addr, size in bytes, value, elapsed) -> 0 | FAULT_BIT`
    pub write: unsafe extern "C" fn(*mut Ctx, u32, u32, u32, u64) -> u64,
    /// `(ctx, op cycles, leave) -> stall cycles | POST_EXIT | POST_ATTENTION`: the checks after an op
    pub post: unsafe extern "C" fn(*mut Ctx, u32, u32) -> u64,
    /// `(ctx, kind, rn, list, elapsed) -> cycles | FAULT_BIT`
    pub multi: unsafe extern "C" fn(*mut Ctx, u32, u32, u32, u64) -> u64,
    /// `(ctx, kind, target or list, the op's address, elapsed) -> cycles | FAULT_BIT`
    pub branch: unsafe extern "C" fn(*mut Ctx, u32, u32, u32, u64) -> u64,
}

impl Thunks {
    fn of<B: Bus + ?Sized>() -> Thunks {
        Thunks { read: read::<B>, write: write::<B>, post: post::<B>, multi: multi::<B>, branch: branch::<B> }
    }
}

/// What a compiled block runs with. The compiled code reads `cpu` (the core's registers and flags, at their
/// `offset_of!` offsets), `cycles` and `thunks`; only the thunks look at the rest.
#[repr(C)]
pub(crate) struct Ctx {
    pub cpu: *mut CortexM0Plus,
    /// The cycles the batch has taken, before the block and (written back) after it.
    pub cycles: u64,
    pub thunks: Thunks,
    /// Points at a `&mut B` for the bus type the thunks were made for.
    bus: *mut u8,
    pub fault: Option<Fault>,
    pub panic: Option<Box<dyn Any + Send>>,
}

impl Ctx {
    pub(crate) fn new<B: Bus + ?Sized>() -> Ctx {
        Ctx {
            cpu: std::ptr::null_mut(),
            cycles: 0,
            thunks: Thunks::of::<B>(),
            bus: std::ptr::null_mut(),
            fault: None,
            panic: None,
        }
    }
}

/// Runs the compiled block `f` on `cpu` and `bus`, `cycles` into the batch; returns the cycles then, the ops run
/// and the exit kind. `ctx` must have been made by `Ctx::new::<B>()` for this `B`.
#[inline(always)]
pub(crate) fn call<B: Bus + ?Sized>(
    cpu: &mut CortexM0Plus,
    bus: &mut B,
    ctx: &mut Ctx,
    f: JitFn,
    cycles: u64,
) -> (u64, u32, u32) {
    let mut bus: &mut B = bus;
    ctx.cpu = cpu;
    ctx.bus = (&raw mut bus).cast();
    ctx.cycles = cycles;
    // SAFETY: `f` was compiled by the `Compiler` of the block cache this block is in, which is alive (it is owned by
    // that cache, and only frees code once no entry holds it), from exactly this block's ops (an entry's code is
    // dropped when it is decoded again), with the `JitFn` signature. `ctx` is valid for the call: `cpu` and `bus`
    // point at the core and the bus reference, both exclusively borrowed here and not used by anything else until
    // the call returns, and its thunks are those for `B` (`Ctx::new::<B>`), which cast `bus` back to `&mut B`.
    // The compiled code only reads and writes the core's registers and flags (in bounds, at their field offsets)
    // and `ctx`'s `cycles`, and calls the thunks.
    let r = unsafe { f(ctx) };
    if let Some(p) = ctx.panic.take() {
        std::panic::resume_unwind(p);
    }
    (ctx.cycles, r & 0xFF, r >> 8)
}

/// Runs `body` on the core and bus `ctx` points at, for a thunk's result: what it returns, or [`FAULT_BIT`] with the
/// fault kept in the context. A panic (which must not unwind through compiled code) is caught and kept too: the
/// thunk reports a fault, and [`call`] resumes the panic.
///
/// # Safety
///
/// `ctx` is the [`Ctx`] a compiled block was called with by [`call`], for this `B`.
#[inline(always)]
unsafe fn with<B: Bus + ?Sized>(
    ctx: *mut Ctx,
    body: impl FnOnce(&mut CortexM0Plus, &mut B) -> Result<u64, Fault>,
) -> u64 {
    // SAFETY: as the function's contract says, `ctx` is the context `call` passed, whose `cpu` points at the core
    // and `bus` at a `&mut B`, both valid and not otherwise used while the compiled code runs (it does not touch
    // memory while a thunk runs).
    let (cpu, bus) = unsafe { (&mut *(*ctx).cpu, &mut **(*ctx).bus.cast::<&mut B>()) };
    let r = std::panic::catch_unwind(AssertUnwindSafe(|| body(cpu, bus)));
    // SAFETY: as above (the context is apart from the core and the bus); the references made for `body` are gone
    let ctx = unsafe { &mut *ctx };
    match r {
        Ok(Ok(v)) => v,
        Ok(Err(f)) => {
            ctx.fault = Some(f);
            FAULT_BIT
        }
        Err(p) => {
            ctx.panic = Some(p);
            FAULT_BIT
        }
    }
}

fn size_of(bytes: u32) -> Size {
    match bytes {
        1 => Size::Byte,
        2 => Size::Half,
        _ => Size::Word,
    }
}

// The thunks. Each does what `exec_op` does for its ops: the access at the op's start time (`timed!()`), through
// the same functions.

unsafe extern "C" fn read<B: Bus + ?Sized>(ctx: *mut Ctx, addr: u32, size: u32, elapsed: u64) -> u64 {
    // SAFETY: the compiled code passes the `ctx` it was called with
    unsafe {
        with::<B>(ctx, |cpu, bus| {
            bus.set_elapsed(elapsed);
            cpu.read_mem(bus, addr, size_of(size)).map(u64::from)
        })
    }
}

unsafe extern "C" fn write<B: Bus + ?Sized>(ctx: *mut Ctx, addr: u32, size: u32, value: u32, elapsed: u64) -> u64 {
    // SAFETY: the compiled code passes the `ctx` it was called with
    unsafe {
        with::<B>(ctx, |cpu, bus| {
            bus.set_elapsed(elapsed);
            cpu.write_mem(bus, addr, size_of(size), value).map(|()| 0)
        })
    }
}

/// What `run_blocks` checks after an op that may have touched the bus, in its order (the short circuits included):
/// stall cycles; SysTick's count of the op after a PPB access; attention (the bus's, then a batch break); then a
/// slow access, `leave` (a store that may have changed the block's code) or a stall end the block.
unsafe extern "C" fn post<B: Bus + ?Sized>(ctx: *mut Ctx, c: u32, leave: u32) -> u64 {
    // SAFETY: the compiled code passes the `ctx` it was called with
    unsafe {
        with::<B>(ctx, |cpu, bus| {
            let stall = bus.take_stall_cycles();
            if cpu.batch_break {
                cpu.tick(c + stall);
            }
            let attention = bus.take_attention() || std::mem::take(&mut cpu.batch_break);
            if attention {
                return Ok(stall as u64 | POST_EXIT | POST_ATTENTION);
            }
            let exit = bus.take_slow_access() || leave != 0 || stall != 0;
            Ok(stall as u64 | if exit { POST_EXIT } else { 0 })
        })
    }
}

unsafe extern "C" fn multi<B: Bus + ?Sized>(ctx: *mut Ctx, kind: u32, rn: u32, list: u32, elapsed: u64) -> u64 {
    // SAFETY: the compiled code passes the `ctx` it was called with
    unsafe {
        with::<B>(ctx, |cpu, bus| {
            bus.set_elapsed(elapsed);
            match kind {
                STM => cpu.op_stm(bus, rn, list),
                LDM => cpu.op_ldm(bus, rn, list),
                PUSH => cpu.op_push(bus, list),
                _ => cpu.op_pop(bus, list),
            }
            .map(u64::from)
        })
    }
}

unsafe extern "C" fn branch<B: Bus + ?Sized>(ctx: *mut Ctx, kind: u32, arg: u32, at: u32, elapsed: u64) -> u64 {
    // SAFETY: the compiled code passes the `ctx` it was called with
    unsafe {
        with::<B>(ctx, |cpu, bus| {
            bus.set_elapsed(elapsed);
            match kind {
                BX => cpu.op_bx(bus, arg, at),
                POP_PC => cpu.op_pop_pc(bus, arg, at),
                _ => cpu.op_step(bus, at),
            }
            .map(u64::from)
        })
    }
}
