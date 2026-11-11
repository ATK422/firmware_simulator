//! The JIT: decoded blocks ([`crate::block`]) compiled to native code with Cranelift, once they are hot, in regions
//! (a hot block and the block it goes on to, if that has run: `block::region`).
//!
//! A region is entered where [`CortexM0Plus::run_blocks`] would run its first block whole (from the first op, with
//! no step limit or budget able to end the batch before the last; `run_blocks` checks that, as for the block
//! executor), and does exactly what `run_blocks` does: same results, flags, cycles, the same accesses in the same
//! order stamped with the same time, the same checks after them and the same exits. Before each further block it
//! makes the checks `run_blocks` would make before running it whole, and compares the code in memory with what the
//! block was compiled from; where any fails it returns to the batch, which goes on there exactly as if it had run
//! everything so far itself. Branches out of the region go on to the region compiled there through a per-site cell
//! (filled by the `resolve` thunk, checked at each use) with a tail call, after the same checks.
//!
//! Register and flag operations, address arithmetic, the condition of a conditional branch, the code-write check,
//! and accesses into the bus's direct memory windows ([`sim_core::bus::Bus::direct_memory`]: SRAM, flash, ROM) are
//! emitted inline; every other access, the exception state and multi-word transfers outside those windows call back
//! into Rust through the thunks here, which run the very code the block executor runs (`read_mem`, `write_mem`,
//! `op_stm`, `op_bx`, ...). The instructions the block executor leaves to the interpreter (`K::Step`) are run by the
//! interpreter, through a thunk.
//!
//! The thunks are monomorphised for the batch's bus type: the batch hands the compiled code a table of them in its
//! [`Ctx`], so one compiled region serves any bus. A panic in a thunk is caught there, ends the region and is
//! resumed once the compiled code has returned (it must not unwind through it).
//!
//! Compiled code is the process's ([`service`]): never freed, and found again by any core whose block has the same
//! code and whose bus has windows of the same shape. A block cache entry drops its region when the entry is decoded
//! again, which `BlockCache::lookup` does whenever memory no longer holds the code the block was decoded from; a
//! region checks the code of every further block (and of a region it goes on to) itself.

// (Without a code generator, some of the protocol is only spoken by one side)
#![cfg_attr(not(all(feature = "jit", any(target_arch = "x86_64", target_arch = "aarch64"))), allow(dead_code))]

pub(crate) mod service;

#[cfg(all(feature = "jit", any(target_arch = "x86_64", target_arch = "aarch64")))]
mod codegen;
#[cfg(all(feature = "jit", any(target_arch = "x86_64", target_arch = "aarch64")))]
pub(crate) use codegen::{Compiler, RegionBlock};

#[cfg(not(all(feature = "jit", any(target_arch = "x86_64", target_arch = "aarch64"))))]
mod unsupported;
#[cfg(not(all(feature = "jit", any(target_arch = "x86_64", target_arch = "aarch64"))))]
pub(crate) use unsupported::{Compiler, RegionBlock};

use std::any::Any;
use std::panic::AssertUnwindSafe;
use std::sync::atomic::Ordering;

use sim_core::bus::{Bus, DirectMemory, Size};

use crate::{CortexM0Plus, Fault};

/// Enters compiled code: calls the region at the code pointer with the context (regions use Cranelift's `tail`
/// calling convention, so that they can go on to one another without growing the stack), and returns how it left
/// (see [`EXIT_NEXT`]); the steps run and the cycles are in the context.
pub(crate) type Trampoline = unsafe extern "C" fn(*mut Ctx, *const u8) -> u32;

/// A compiled region, as the code that goes on to it finds it (in a block cache's entry, or a chain site's cell):
/// its code, and what entering it requires of its first block.
#[repr(C)]
pub(crate) struct Desc {
    pub code: *const u8,
    /// The first block: its address, ops and the cycles before its last op (for the checks before it runs), the
    /// direct memory window its code is in (`u32::MAX`: none, it is not gone on to), and the code (masked words).
    pub pc: u32,
    pub ops: u32,
    pub lead: u32,
    pub window: u32,
    pub words: [u64; 4],
    pub masks: [u64; 4],
}

// SAFETY: a `Desc` is never written after it is made, and the code it points at is never freed (`service`)
unsafe impl Send for Desc {}
// SAFETY: as above
unsafe impl Sync for Desc {}

/// A compiled region.
pub(crate) type Region = &'static Desc;

/// The block ran to its end, or stopped after an op as the block executor does (a slow access, a stall, a store
/// that may have changed its code, a taken conditional branch): the PC is at the next instruction.
pub(crate) const EXIT_NEXT: u32 = 0;
/// An access needs the machine's attention: the batch ends (the PC is at the next instruction).
pub(crate) const EXIT_BATCH: u32 = 1;
/// The last op run faulted ([`Ctx::fault`]), or a thunk panicked ([`Ctx::panic`]).
pub(crate) const EXIT_FAULT: u32 = 2;

/// What a thunk's result has set: the op faulted (or panicked), the region must be left after it (the checks
/// `run_blocks` makes after an op said so), for the machine's attention. The low 32 bits are the loaded value or the
/// op's cycles; stall cycles are in [`Ctx::stall`].
pub(crate) const FAULT_BIT: u64 = 1 << 32;
pub(crate) const EXIT_BIT: u64 = 1 << 33;
pub(crate) const ATTENTION_BIT: u64 = 1 << 34;

/// [`access`]'s `how`: the size in bytes, and these.
pub(crate) const WRITE: u32 = 1 << 8;
pub(crate) const LEAVE: u32 = 1 << 9;

/// Multi-word transfers ([`multi`]'s `kind`).
pub(crate) const STM: u32 = 0;
pub(crate) const LDM: u32 = 1;
pub(crate) const PUSH: u32 = 2;
pub(crate) const POP: u32 = 3;
/// Branches that may do more than branch ([`branch`]'s `kind`).
pub(crate) const BX: u32 = 0;
pub(crate) const POP_PC: u32 = 1;
pub(crate) const STEP: u32 = 2;

/// The thunks compiled code calls, for one bus type. Each runs an op as `exec_op` does, then the checks
/// `run_blocks` makes after an op that may touch the bus.
#[repr(C)]
pub(crate) struct Thunks {
    /// `(ctx, addr, how, value, elapsed) -> value | bits`: a load or store
    pub access: unsafe extern "C" fn(*mut Ctx, u32, u32, u32, u64) -> u64,
    /// `(ctx, kind, rn, list, elapsed) -> cycles | bits`: always leaves after (if it does not fault)
    pub multi: unsafe extern "C" fn(*mut Ctx, u32, u32, u32, u64) -> u64,
    /// `(ctx, kind, target or list, the op's address, elapsed) -> cycles | bits`
    pub branch: unsafe extern "C" fn(*mut Ctx, u32, u32, u32, u64) -> u64,
    /// `(ctx, target, 0, 0, cell) -> desc or null`: the region to go on to at `target`, if any (and it is noted in the
    /// chain site's `cell`, an address)
    pub resolve: unsafe extern "C" fn(*mut Ctx, u32, u32, u32, u64) -> u64,
}

impl Thunks {
    fn of<B: Bus + ?Sized>() -> Thunks {
        Thunks { access: access::<B>, multi: multi::<B>, branch: branch::<B>, resolve: resolve::<B> }
    }
}

/// What a compiled region runs with. The compiled code reads and writes `cpu` (the core's registers and flags, at
/// their `offset_of!` offsets, and reads its IPSR), the limits and counts here, the direct memory windows `mem` points
/// at (and the memory they describe), and calls the thunks; only the thunks look at the rest.
#[repr(C)]
pub(crate) struct Ctx {
    pub cpu: *mut CortexM0Plus,
    /// The cycles the batch has taken, before the region and (written back) after it.
    pub cycles: u64,
    /// The batch's cycle budget, the steps it may still take, and the address below which the batch stops.
    pub budget: u64,
    pub steps_left: u32,
    pub trap_below: u32,
    /// The steps run (by the regions called one after another: written by them).
    pub steps: u32,
    /// The block cache the regions are found in, and when and how they are compiled (`CortexM0Plus::jit_threshold`,
    /// `jit_sync`).
    pub cache: *mut crate::BlockCache,
    pub threshold: u32,
    pub sync: u32,
    /// The bus's direct memory, or `none`.
    pub mem: *const DirectMemory,
    /// Where marks the bus does not keep go.
    pub sink: u64,
    /// The stall cycles the last thunk's op took.
    pub stall: u32,
    pub thunks: Thunks,
    none: DirectMemory,
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
            budget: 0,
            steps_left: 0,
            trap_below: 0,
            steps: 0,
            cache: std::ptr::null_mut(),
            threshold: 0,
            sync: 0,
            mem: std::ptr::null(),
            sink: 0,
            stall: 0,
            thunks: Thunks::of::<B>(),
            none: DirectMemory::NONE,
            bus: std::ptr::null_mut(),
            fault: None,
            panic: None,
        }
    }
}

/// The limits a region runs within: the batch's cycle budget, the steps it may still take and `trap_below`; and
/// the block cache with its JIT settings, for the regions it goes on to.
pub(crate) struct Limits<'a> {
    pub budget: u64,
    pub steps_left: u32,
    pub trap_below: u32,
    pub cache: &'a mut crate::BlockCache,
    pub jit: (u32, bool),
}

/// Runs the compiled region `f` on `cpu` and `bus`, `cycles` into the batch; returns the cycles then, the steps run
/// and the exit kind. `ctx` must have been made by `Ctx::new::<B>()` for this `B`.
#[inline(always)]
pub(crate) fn call<B: Bus + ?Sized>(
    cpu: &mut CortexM0Plus,
    bus: &mut B,
    ctx: &mut Ctx,
    f: Region,
    cycles: u64,
    limits: Limits,
) -> (u64, u32, u32) {
    let Some(enter) = service::service().trampoline() else { return (cycles, 0, EXIT_NEXT) };
    ctx.mem = match bus.direct_memory() {
        Some(m) => m,
        None => &ctx.none,
    };
    let mut bus: &mut B = bus;
    ctx.cpu = cpu;
    ctx.bus = (&raw mut bus).cast();
    ctx.cycles = cycles;
    ctx.steps = 0;
    (ctx.budget, ctx.steps_left, ctx.trap_below) = (limits.budget, limits.steps_left, limits.trap_below);
    ctx.cache = limits.cache;
    (ctx.threshold, ctx.sync) = (limits.jit.0, limits.jit.1 as u32);
    // SAFETY: `f` is a region the process's compiler made (`service`), whose code is never freed, entered through the
    // trampoline made for it. It was compiled from the ops of the block now at the PC (the cache entry holding it was
    // decoded from the same code: an entry's region is dropped when it is decoded again), and of other blocks whose
    // code it checks before running them; the regions it goes on to check their first block's. `cache` points at
    // the batch's block cache, exclusively borrowed here and used by nothing else until the call returns, but the
    // `resolve` thunk. `ctx` is valid for the call: `cpu` and `bus` point at the core and the bus reference,
    // both exclusively borrowed here and not used by anything else until the call returns, and its thunks are those
    // for `B` (`Ctx::new::<B>`), which cast `bus` back to `&mut B`. `mem` points at the bus's direct memory windows
    // (or none), whose memory the compiled code reads and writes only within the windows, as `Bus::direct_memory`
    // allows while the bus is not being called, and which it reads afresh at each call. Otherwise the compiled code
    // only reads and writes the core's registers and flags (at their field offsets) and the context's counts.
    let kind = unsafe { enter(ctx, f.code) };
    if let Some(p) = ctx.panic.take() {
        std::panic::resume_unwind(p);
    }
    (ctx.cycles, ctx.steps, kind)
}

/// Runs `body` on the core and bus `ctx` points at, then (if it did not fault) the checks after the op, for a
/// thunk's result: `body`'s value or the op's cycles and the bits, or [`FAULT_BIT`] with the fault kept in the
/// context. A panic (which must not unwind through compiled code) is caught and kept too: the thunk reports a fault,
/// and [`call`] resumes the panic.
///
/// # Safety
///
/// `ctx` is the [`Ctx`] a compiled region was called with by [`call`], for this `B`.
#[inline(always)]
unsafe fn with<B: Bus + ?Sized>(
    ctx: *mut Ctx,
    leave: bool,
    body: impl FnOnce(&mut CortexM0Plus, &mut B) -> Result<(u32, u32), Fault>,
) -> u64 {
    // SAFETY: as the function's contract says, `ctx` is the context `call` passed, whose `cpu` points at the core
    // and `bus` at a `&mut B`, both valid and not otherwise used while the compiled code runs (it does not touch
    // them while a thunk runs)
    let (cpu, bus) = unsafe { (&mut *(*ctx).cpu, &mut **(*ctx).bus.cast::<&mut B>()) };
    let r = std::panic::catch_unwind(AssertUnwindSafe(|| {
        // `body` gives the result's low bits and the op's cycles
        body(cpu, bus).map(|(low, c)| {
            // What `run_blocks` does after an op that may touch the bus, in its order (the short circuits
            // included): stall cycles; SysTick's count of the op after a PPB access; attention (the bus's, then a
            // batch break); then a slow access, `leave` (a store that may have changed the code) or a stall end
            // the block
            let stall = bus.take_stall_cycles();
            if cpu.batch_break {
                cpu.tick(c + stall);
            }
            let bits = if bus.take_attention() || std::mem::take(&mut cpu.batch_break) {
                EXIT_BIT | ATTENTION_BIT
            } else if bus.take_slow_access() || leave || stall != 0 {
                EXIT_BIT
            } else {
                0
            };
            (low as u64 | bits, stall)
        })
    }));
    // SAFETY: as above (the context is apart from the core and the bus); the references made for `body` are gone
    let ctx = unsafe { &mut *ctx };
    match r {
        Ok(Ok((v, stall))) => {
            ctx.stall = stall;
            v
        }
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

unsafe extern "C" fn access<B: Bus + ?Sized>(ctx: *mut Ctx, addr: u32, how: u32, value: u32, elapsed: u64) -> u64 {
    // SAFETY: the compiled code passes the `ctx` it was called with
    unsafe {
        with::<B>(ctx, how & LEAVE != 0, |cpu, bus| {
            bus.set_elapsed(elapsed);
            let size = size_of(how & 0xFF);
            if how & WRITE != 0 {
                cpu.write_mem(bus, addr, size, value).map(|()| (0, 2))
            } else {
                cpu.read_mem(bus, addr, size).map(|v| (v, 2))
            }
        })
    }
}

unsafe extern "C" fn multi<B: Bus + ?Sized>(ctx: *mut Ctx, kind: u32, rn: u32, list: u32, elapsed: u64) -> u64 {
    // SAFETY: the compiled code passes the `ctx` it was called with
    unsafe {
        // (Leaving after it whatever the checks say is allowed: the region goes on with the next instruction)
        with::<B>(ctx, true, |cpu, bus| {
            bus.set_elapsed(elapsed);
            match kind {
                STM => cpu.op_stm(bus, rn, list),
                LDM => cpu.op_ldm(bus, rn, list),
                PUSH => cpu.op_push(bus, list),
                _ => cpu.op_pop(bus, list),
            }
            .map(|c| (c, c))
        })
    }
}

unsafe extern "C" fn branch<B: Bus + ?Sized>(ctx: *mut Ctx, kind: u32, arg: u32, at: u32, elapsed: u64) -> u64 {
    // SAFETY: the compiled code passes the `ctx` it was called with
    unsafe {
        with::<B>(ctx, false, |cpu, bus| {
            bus.set_elapsed(elapsed);
            match kind {
                BX => cpu.op_bx(bus, arg, at),
                POP_PC => cpu.op_pop_pc(bus, arg, at),
                _ => cpu.op_step(bus, at),
            }
            .map(|c| (c, c))
        })
    }
}

/// Finds the region to go on to at `target` from a chain site (whose `cell` it notes it in): the region headed by
/// the block the cache has there, if it is to run whole and is compiled; null else (the region then leaves, and the
/// batch goes on as it would have).
unsafe extern "C" fn resolve<B: Bus + ?Sized>(ctx: *mut Ctx, target: u32, _: u32, _: u32, cell: u64) -> u64 {
    // SAFETY: the compiled code passes the `ctx` it was called with; `cache` points at the batch's block cache (see
    // `call`), and `cpu` and `bus` as for the other thunks
    let r = std::panic::catch_unwind(AssertUnwindSafe(|| unsafe {
        let c = &mut *ctx;
        let (cpu, bus, cache) = (&mut *c.cpu, &mut **c.bus.cast::<&mut B>(), &mut *c.cache);
        let (slot, first) = cache.lookup(bus, target)?;
        if first != 0 {
            return None;
        }
        let d = cache.hot(bus, &cpu.r, slot, (c.threshold, c.sync != 0))?;
        (*(cell as *const std::sync::atomic::AtomicPtr<Desc>)).store((d as *const Desc).cast_mut(), Ordering::Release);
        Some(d)
    }));
    match r {
        Ok(d) => d.map_or(0, |d| d as *const Desc as u64),
        Err(p) => {
            // SAFETY: as above
            unsafe { (*ctx).panic = Some(p) };
            0
        }
    }
}
