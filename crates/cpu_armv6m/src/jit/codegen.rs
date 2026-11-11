//! Cranelift code generation for regions of decoded blocks (see the module above).
//!
//! A region is a hot block and the blocks reachable from it by direct branches (taken and not), calls and the
//! returns from them, compiled into one function `fn(*mut Ctx) -> u32`. Each op becomes the IR for what `exec_op`
//! does; where `run_blocks` checks the bus after an op that went through it, a call of [`super::post`] and the exits
//! `run_blocks` takes. From one block to the next the function makes the checks `run_blocks` makes before it runs a
//! block whole (the step limit, the cycle budget, `trap_below`) and checks the block's code is still in memory, else
//! it returns to `run_blocks` there: leaving at an instruction boundary is always allowed, as `run_blocks` then goes
//! on from there exactly as the compiled code would have.
//!
//! Guest registers and flags are kept in SSA variables, loaded on entry and stored on exit (and around the thunks
//! that read or write them). Accesses to the bus's direct memory ([`sim_core::bus::DirectMemory`]: SRAM, flash, ROM
//! on the RP2040) are made inline, with the bus's marks; the rest go through the thunks.

use std::mem::offset_of;
use std::sync::atomic::{AtomicUsize, Ordering};

use cranelift_codegen::Context;
use cranelift_codegen::ir::condcodes::IntCC;
use cranelift_codegen::ir::types::{I8, I16, I32, I64};
use cranelift_codegen::ir::{AbiParam, Block, InstBuilder, MemFlagsData, SigRef, Signature, Type, UserFuncName, Value};
use cranelift_codegen::isa::{CallConv, TargetFrontendConfig};
use cranelift_codegen::settings::{self, Configurable};
use cranelift_frontend::{FunctionBuilder, FunctionBuilderContext, Variable};
use cranelift_jit::{JITBuilder, JITModule};
use cranelift_module::{FuncId, Module, default_libcall_names};
use sim_core::bus::{DIRECT_REGIONS, DirectMemory, DirectRegion};

use super::service::Shape;
use super::{
    ATTENTION_BIT, BX, Ctx, Desc, EXIT_BATCH, EXIT_BIT, EXIT_FAULT, EXIT_NEXT, FAULT_BIT, LDM, LEAVE, POP, POP_PC,
    PUSH, STEP, STM, Thunks, Trampoline, WRITE,
};
use crate::CortexM0Plus;
use crate::block::{Entry, K, Op};

/// Most memory compiled code may take, in the whole process (code is never freed: cores everywhere may hold it).
/// Nothing is compiled after ([`Compiler::full`]).
///
/// Code is made executable a batch of regions at a time (`Compiler::finish`), and memory made executable is never
/// written again, so each batch takes whole pages: regions compiled together share them, one compiled alone (as
/// tests compile, `jit_sync`) takes a page of its own, 16 KiB on Apple silicon. The limit leaves room for that: over
/// 130,000 regions compiled one at a time there.
const CODE_MEMORY: usize = 2 << 30;

/// The memory compiled code has taken in the process so far (whole pages).
static CODE_TAKEN: AtomicUsize = AtomicUsize::new(0);

/// [`CODE_TAKEN`].
pub(crate) fn code_taken() -> usize {
    CODE_TAKEN.load(Ordering::Relaxed)
}

/// Compiles regions ([`super::service`]): each with a module of its own.
pub(crate) struct Compiler {
    module: Option<JITModule>,
    ctx: Context,
    fctx: FunctionBuilderContext,
    page: usize,
    trampoline: Trampoline,
}

/// A region defined in a compiler's module, not yet executable: its function and its code's size.
pub(crate) struct Defined(FuncId, usize);

fn module() -> Option<JITModule> {
    let mut flags = settings::builder();
    // (Cranelift's optimiser costs more compile time than it saves here, and can take very long on some regions:
    // the IR emitted is close to what is wanted already)
    flags.set("opt_level", "none").ok()?;
    // As `JITBuilder::with_flags` sets them
    flags.set("use_colocated_libcalls", "false").ok()?;
    // (Tail calls between regions need frames, leaf regions included)
    flags.set("preserve_frame_pointers", "true").ok()?;
    flags.set("is_pic", if cfg!(target_arch = "x86_64") { "true" } else { "false" }).ok()?;
    flags.set("enable_verifier", if cfg!(debug_assertions) { "true" } else { "false" }).ok()?;
    // (No host: the core interprets)
    let isa = cranelift_native::builder().ok()?.finish(settings::Flags::new(flags)).ok()?;
    Some(JITModule::new(JITBuilder::with_isa(isa, default_libcall_names())))
}

/// A block of a region, with what compiling it needs to know.
#[derive(Clone, Copy)]
pub(crate) struct RegionBlock {
    pub entry: Entry,
    /// Whether its code is in memory the compiled code may write (stores there are checked).
    pub writable: bool,
    /// Whether it is where a call in the region returns (an indirect branch there stays in the region).
    pub return_site: bool,
}

impl Compiler {
    /// A compiler for this host, if Cranelift supports it.
    pub(crate) fn new() -> Option<Compiler> {
        let mut module = module()?;
        let mut ctx = module.make_context();
        let mut fctx = FunctionBuilderContext::new();
        // The trampoline: `(ctx, region) -> kind`, calling the region (in the `tail` convention) from Rust's
        let ptr = module.target_config().pointer_type();
        let mut sig = module.make_signature();
        sig.params.extend([AbiParam::new(ptr), AbiParam::new(ptr)]);
        sig.returns.push(AbiParam::new(I32));
        let id = module.declare_anonymous_function(&sig).ok()?;
        ctx.func.signature = sig;
        ctx.func.name = UserFuncName::user(0, id.as_u32());
        {
            let mut b = FunctionBuilder::new(&mut ctx.func, &mut fctx);
            let region = b.import_signature(region_signature(&module));
            let e = b.create_block();
            b.append_block_params_for_function_params(e);
            b.switch_to_block(e);
            b.seal_block(e);
            let (c, f) = (b.block_params(e)[0], b.block_params(e)[1]);
            let call = b.ins().call_indirect(region, f, &[c]);
            let k = b.inst_results(call)[0];
            b.ins().return_(&[k]);
            b.finalize(module.target_config());
        }
        module.define_function(id, &mut ctx).ok()?;
        module.clear_context(&mut ctx);
        module.finalize_definitions().ok()?;
        // SAFETY: the finalized code of the function just defined, `extern "C" fn(*mut Ctx, *const u8) -> u32` in
        // the module's default calling convention, the host's C ABI (two pointer-sized parameters, a 32-bit result)
        let trampoline = unsafe { std::mem::transmute::<*const u8, Trampoline>(module.get_finalized_function(id)) };
        Some(Compiler { module: Some(module), ctx, fctx, page: region_page(), trampoline })
    }

    /// How Rust calls a region.
    pub(crate) fn trampoline(&self) -> Trampoline {
        self.trampoline
    }

    /// Whether the process's compiled code has taken all the memory it may ([`CODE_MEMORY`]).
    pub(crate) fn full(&self) -> bool {
        CODE_TAKEN.load(Ordering::Relaxed) >= CODE_MEMORY
    }

    /// Compiles the region `blocks` (the first is where it is entered) into the module, to be made executable with
    /// others by [`Compiler::finish`]. `windows`: the shapes of the direct memory windows the bus had, and `regs`
    /// the core's registers: each access is tried inline in the window that has the address it would make now (a
    /// guess for later blocks: where it goes, if not there, is the thunk's).
    pub(crate) fn define(
        &mut self,
        blocks: &[RegionBlock],
        windows: &[Shape; DIRECT_REGIONS],
        regs: &[u32; 16],
    ) -> Option<Defined> {
        // (Whatever a compile that panicked left)
        self.ctx.clear();
        self.fctx = FunctionBuilderContext::new();
        let module = self.module.as_mut()?;
        let ptr = module.target_config().pointer_type();
        let sig = region_signature(module);
        let id = module.declare_anonymous_function(&sig).ok()?;
        self.ctx.func.signature = sig;
        self.ctx.func.name = UserFuncName::user(0, id.as_u32());
        // Every thunk: (ctx, u32, u32, u32, elapsed) -> u64
        let mut thunk = module.make_signature();
        thunk.params.push(AbiParam::new(ptr));
        thunk.params.extend([I32, I32, I32, I64].map(AbiParam::new));
        thunk.returns.push(AbiParam::new(I64));
        let config = module.target_config();
        let region = region_signature(module);
        {
            let b = FunctionBuilder::new(&mut self.ctx.func, &mut self.fctx);
            Emit::region(b, blocks, ptr, (thunk, region), config, (windows, regs));
        }
        let defined = module.define_function(id, &mut self.ctx);
        let size = self.ctx.compiled_code().map_or(0, |c| c.code_info().total_size as usize);
        module.clear_context(&mut self.ctx);
        defined.ok()?;
        Some(Defined(id, size))
    }

    /// Makes the regions `defined` since the last call executable, together: their code, to be called through the
    /// trampoline or by other regions (`None`: none can run).
    pub(crate) fn finish(&mut self, defined: &[Defined]) -> Option<Vec<*const u8>> {
        let module = self.module.as_mut()?;
        module.finalize_definitions().ok()?;
        // (They share pages: a batch takes the pages its code fills)
        let bytes: usize = defined.iter().map(|d| d.1).sum();
        CODE_TAKEN.fetch_add(bytes.div_ceil(self.page).max(1) * self.page, Ordering::Relaxed);
        Some(defined.iter().map(|d| module.get_finalized_function(d.0)).collect())
    }
}

/// A region's signature: `(ctx) -> kind`, in the `tail` convention so that regions can go on to one another.
fn region_signature(module: &JITModule) -> Signature {
    let ptr = module.target_config().pointer_type();
    let mut sig = module.make_signature();
    sig.call_conv = CallConv::Tail;
    sig.params.push(AbiParam::new(ptr));
    sig.returns.push(AbiParam::new(I32));
    sig
}

/// The host's page size (each batch of compiled functions takes at least one).
fn region_page() -> usize {
    if cfg!(all(target_arch = "aarch64", target_os = "macos")) { 16 << 10 } else { 4 << 10 }
}

// Field offsets the compiled code uses
const R: i32 = offset_of!(CortexM0Plus, r) as i32;
const N: i32 = offset_of!(CortexM0Plus, n) as i32;
const Z: i32 = offset_of!(CortexM0Plus, z) as i32;
const C: i32 = offset_of!(CortexM0Plus, c) as i32;
const V: i32 = offset_of!(CortexM0Plus, v) as i32;
const IPSR: i32 = offset_of!(CortexM0Plus, ipsr) as i32;
const FLAGS: [i32; 4] = [N, Z, C, V];
const CTX_CPU: i32 = offset_of!(Ctx, cpu) as i32;
const CTX_CYCLES: i32 = offset_of!(Ctx, cycles) as i32;
const CTX_BUDGET: i32 = offset_of!(Ctx, budget) as i32;
const CTX_STEPS: i32 = offset_of!(Ctx, steps) as i32;
const CTX_STEPS_LEFT: i32 = offset_of!(Ctx, steps_left) as i32;
const CTX_TRAP: i32 = offset_of!(Ctx, trap_below) as i32;
const CTX_MEM: i32 = offset_of!(Ctx, mem) as i32;
const CTX_SINK: i32 = offset_of!(Ctx, sink) as i32;
const THUNKS: i32 = offset_of!(Ctx, thunks) as i32;
const CTX_STALL: i32 = offset_of!(Ctx, stall) as i32;
const ACCESS: i32 = THUNKS + offset_of!(Thunks, access) as i32;
const MULTI: i32 = THUNKS + offset_of!(Thunks, multi) as i32;
const BRANCH: i32 = THUNKS + offset_of!(Thunks, branch) as i32;
const RESOLVE: i32 = THUNKS + offset_of!(Thunks, resolve) as i32;
const W_LEN: i32 = offset_of!(DirectRegion, len) as i32;
const D_CODE: i32 = offset_of!(Desc, code) as i32;
const D_PC: i32 = offset_of!(Desc, pc) as i32;
const D_OPS: i32 = offset_of!(Desc, ops) as i32;
const D_LEAD: i32 = offset_of!(Desc, lead) as i32;
const D_WINDOW: i32 = offset_of!(Desc, window) as i32;
const D_WORDS: i32 = offset_of!(Desc, words) as i32;
const D_MASKS: i32 = offset_of!(Desc, masks) as i32;
const WINDOW: i32 = size_of::<DirectRegion>() as i32;
const W_BASE: i32 = offset_of!(DirectRegion, base) as i32;
const W_SIZE: i32 = offset_of!(DirectRegion, size) as i32;
const W_MASK: i32 = offset_of!(DirectRegion, mask) as i32;
const W_WRITABLE: i32 = offset_of!(DirectRegion, writable) as i32;
const W_PTR: i32 = offset_of!(DirectRegion, ptr) as i32;
const W_READ_MARK: i32 = offset_of!(DirectRegion, read_mark) as i32;
const W_WRITE_MARK: i32 = offset_of!(DirectRegion, write_mark) as i32;
const W_READ_VALUE: i32 = offset_of!(DirectRegion, read_value) as i32;
const W_WRITE_VALUE: i32 = offset_of!(DirectRegion, write_value) as i32;
const _: () = assert!(offset_of!(DirectMemory, regions) == 0);
// (`post` picks the exit by adding attention to EXIT_NEXT)
const _: () = assert!(EXIT_BATCH == EXIT_NEXT + 1);

/// The core's fields and the context: always valid and aligned.
fn mem() -> MemFlagsData {
    MemFlagsData::trusted()
}

/// What does not change while the compiled code runs (the context's limits, the direct memory windows, which hold
/// until a slow access, after which the code returns).
fn fixed() -> MemFlagsData {
    MemFlagsData::trusted().with_readonly()
}

/// Guest memory: valid (the windows say so) but not necessarily aligned for the host.
fn guest() -> MemFlagsData {
    MemFlagsData::new().with_notrap()
}

/// What an access needs: its size in bytes, and for a load the register it loads (and the type it sign extends
/// from), for a store the value.
#[derive(Clone, Copy)]
enum Access {
    Load(u32, u8, Option<Type>),
    Store(u32, Value),
}

struct Emit<'a> {
    b: FunctionBuilder<'a>,
    ptr: Type,
    ctx: Value,
    cpu: Value,
    mem: Value,
    sink: Value,
    /// The guest registers and flags (N, Z, C, V): those the region reads or writes are loaded on entry, and writes
    /// go to the core at once (and to the variables).
    regs: [Variable; 16],
    flags: [Variable; 4],
    /// What each flag (N, Z, C, V) is at the point being emitted ([`Fl`]), and whether all four come from one
    /// subtraction `x - y` (a comparison), whose conditions are then comparisons of `x` and `y`.
    fl: [Fl; 4],
    sub: Option<(Value, Value)>,
    /// The cycles the batch has taken, the steps the region has run, the blocks whose code was found still in memory.
    cyc: Variable,
    /// Cycles of ops (that only touch registers) not yet added to `cyc`: added before an op with control flow of
    /// its own and at a block's end, so that one add counts a run of them.
    pend: u32,
    steps: Variable,
    valid: Variable,
    thunk: SigRef,
    /// The regions' own signature (for going on to another), and the chain site cells: each holds the region last
    /// gone on to from its site (or null).
    region: SigRef,
    cells: &'static [std::sync::atomic::AtomicPtr<Desc>],
    next_cell: usize,
    /// The region's one copy of what going on to another takes: `(target, cell)` (made when first needed).
    chain_block: Option<Block>,
    exit: Block,
    /// Where a thunk's op that faulted or must leave goes: `(bits, the op's address, the next PC, steps, cycles)`.
    slow_exit: Block,
    blocks: &'a [RegionBlock],
    /// Where each block's checks start (created on first use, when the block is to be emitted too: blocks nothing
    /// goes to are not, as unreachable loops would trouble the SSA construction).
    entries: Vec<Option<Block>>,
    /// Accesses tried inline that leave the region at their op where the address is not in the window (filled in
    /// at the end): the block, the steps at the start of the op's block, the ops there up to and with it, its PC, and
    /// the flags there.
    misses: Vec<(Block, Value, usize, u32, Flags)>,
    reached: Vec<usize>,
    /// Code a store may change: where blocks in writable memory are (`lo..hi`), if any.
    code: Option<(u32, u32)>,
    /// The windows when the region was compiled, and the registers then.
    windows: [Shape; DIRECT_REGIONS],
    regs_now: [u32; 16],
}

/// A flag as the compiled code has it at a point: in its variable (and the core), as a value computed already, or
/// to compute from the op that set it (lazily: most flags are set again before anything looks at them). Flags are
/// stored in the core wherever something may look: before a thunk that reads them, where the region is left, and
/// from one block to another (blocks start with their flags in the variables).
#[derive(Clone, Copy)]
enum Fl {
    Var,
    Val(Value),
    /// N or Z of a result.
    Neg(Value),
    Zero(Value),
    /// C or V of `x + y + c` (c an I8), V with the result.
    Carry(Value, Value, Value),
    Over(Value, Value, Value),
}

/// What [`Emit::fl`] and [`Emit::sub`] say at a point.
type Flags = ([Fl; 4], Option<(Value, Value)>);

/// Registers read or written by `op` (bit 16 + flag for flags) and written by it.
fn op_regs(op: Op) -> (u32, u32) {
    let (a, b, m) = (1u32 << (op.a & 15), 1u32 << (op.b & 15), 1u32 << (op.imm & 15));
    const NZ: u32 = 0b0011 << 16;
    const NZC: u32 = 0b0111 << 16;
    const ALL: u32 = 0b1111 << 16;
    let list8 = op.imm & 0xFF;
    match op.kind {
        K::LslI | K::LsrI | K::AsrI | K::Lsr32 | K::Asr32 => (a | b | NZC, a | NZC),
        K::Movs | K::Mvn => (a | b | NZ, a | NZ),
        K::AddR | K::SubR => (a | b | m | ALL, a | ALL),
        K::AddI3 | K::SubI3 => (a | b | ALL, a | ALL),
        K::MovI => (a | NZ, a | NZ),
        K::CmpI => (a | ALL, ALL),
        K::AddI8 | K::SubI8 => (a | ALL, a | ALL),
        K::And | K::Eor | K::Orr | K::Bic | K::Mul => (a | b | NZ, a | NZ),
        K::LslR | K::LsrR | K::AsrR | K::Ror => (a | b | NZC, a | NZC),
        K::Adc | K::Sbc => (a | b | ALL, a | ALL),
        K::Tst => (a | b | NZ, NZ),
        K::Rsb => (a | b | ALL, a | ALL),
        K::CmpR | K::Cmn | K::CmpHi => (a | b | ALL, ALL),
        K::AddHi | K::MovHi => (a | b, a),
        K::Set => (a, a),
        K::AddSp => (a | 1 << 13, a),
        K::SpAdd => (1 << 13, 1 << 13),
        K::Sxth | K::Sxtb | K::Uxth | K::Uxtb | K::Rev | K::Rev16 | K::Revsh => (a | b, a),
        K::Nop => (0, 0),
        K::Ldr => (a, a),
        K::LdrR | K::LdrhR | K::LdrbR | K::LdrsbR | K::LdrshR => (a | b | m, a),
        K::StrR | K::StrhR | K::StrbR => (a | b | m, 0),
        K::LdrI | K::LdrhI | K::LdrbI => (a | b, a),
        K::StrI | K::StrhI | K::StrbI => (a | b, 0),
        K::Stm => (a | list8, a),
        K::Ldm => (a | list8, a | list8),
        K::Push => (op.imm & 0x40FF | 1 << 13, 1 << 13),
        K::Pop => (list8 | 1 << 13, list8 | 1 << 13),
        K::BCond => (0xF << 16 | 1 << 15, 1 << 15),
        K::B => (1 << 15, 1 << 15),
        K::Bl => (1 << 14 | 1 << 15, 1 << 14 | 1 << 15),
        K::Bx | K::Blx => (a | 1 << 14 | 1 << 15, 1 << 14 | 1 << 15),
        K::PopPc => (list8 | 1 << 13 | 1 << 15, list8 | 1 << 13 | 1 << 15),
        K::Step => (0, 0),
    }
}

impl<'a> Emit<'a> {
    fn region(
        mut b: FunctionBuilder<'a>,
        blocks: &'a [RegionBlock],
        ptr: Type,
        (thunk, region): (Signature, Signature),
        config: TargetFrontendConfig,
        (windows, regs_now): (&[Shape; DIRECT_REGIONS], &[u32; 16]),
    ) {
        // Every register an op of the region touches (the PC always: exits set it)
        let mut used = 1u32 << 15;
        for blk in blocks {
            for &op in blk.entry.ops() {
                let (u, w) = op_regs(op);
                used |= u | w;
            }
        }
        let code = blocks
            .iter()
            .filter(|b| b.writable)
            .map(|b| (b.entry.start(), b.entry.start().wrapping_add(b.entry.len())))
            .reduce(|(lo, hi), (l, h)| (lo.min(l), hi.max(h)));

        let entry = b.create_block();
        b.append_block_params_for_function_params(entry);
        b.switch_to_block(entry);
        b.seal_block(entry);
        let ctx = b.block_params(entry)[0];
        let cpu = b.ins().load(ptr, mem(), ctx, CTX_CPU);
        let memp = b.ins().load(ptr, fixed(), ctx, CTX_MEM);
        let sink = b.ins().iadd_imm_u(ctx, CTX_SINK as i64);
        let regs = std::array::from_fn(|_| b.declare_var(I32));
        let flags = std::array::from_fn(|_| b.declare_var(I8));
        for (r, &var) in regs.iter().enumerate() {
            if used & 1 << r != 0 {
                let v = b.ins().load(I32, mem(), cpu, R + 4 * r as i32);
                b.def_var(var, v);
            }
        }
        for (f, &off) in FLAGS.iter().enumerate() {
            if used & 1 << (16 + f) != 0 {
                let v = b.ins().load(I8, mem(), cpu, off);
                b.def_var(flags[f], v);
            }
        }
        let (cyc, steps, valid) = (b.declare_var(I64), b.declare_var(I32), b.declare_var(I32));
        let c0 = b.ins().load(I64, mem(), ctx, CTX_CYCLES);
        b.def_var(cyc, c0);
        // (The steps the regions before this one ran, if it was gone on to)
        let s0 = b.ins().load(I32, mem(), ctx, CTX_STEPS);
        b.def_var(steps, s0);
        // The first block was found in memory by the batch
        let one = b.ins().iconst(I32, 1);
        b.def_var(valid, one);
        let thunk = b.import_signature(thunk);
        let region = b.import_signature(region);
        // (At most a site per block's end and conditional branch, and one per indirect branch: kept for ever, as
        // the code)
        let sites = blocks.iter().map(|b| 2 + b.entry.ops().iter().filter(|o| o.kind == K::BCond).count()).sum();
        let cells: &'static [std::sync::atomic::AtomicPtr<Desc>] = Box::leak(
            (0..sites).map(|_| std::sync::atomic::AtomicPtr::new(std::ptr::null_mut())).collect::<Box<[_]>>(),
        );
        let exit = b.create_block();
        b.append_block_param(exit, I32);
        let slow_exit = b.create_block();
        for t in [I64, I32, I32, I32, I32] {
            b.append_block_param(slow_exit, t);
        }
        let mut x = Emit {
            b,
            ptr,
            ctx,
            cpu,
            mem: memp,
            sink,
            regs,
            flags,
            fl: [Fl::Var; 4],
            sub: None,
            cyc,
            pend: 0,
            steps,
            valid,
            thunk,
            region,
            cells,
            next_cell: 0,
            chain_block: None,
            exit,
            slow_exit,
            blocks,
            entries: vec![None; blocks.len()],
            misses: Vec::new(),
            reached: vec![0],
            code,
            windows: *windows,
            regs_now: *regs_now,
        };
        let starts: Vec<Block> = blocks.iter().map(|_| x.b.create_block()).collect();
        x.b.ins().jump(starts[0], &[]);
        let mut emitted = vec![false; blocks.len()];
        while let Some(k) = x.reached.pop() {
            if std::mem::replace(&mut emitted[k], true) {
                continue;
            }
            x.b.switch_to_block(starts[k]);
            // (Every way into a block has its flags in the variables)
            (x.fl, x.sub) = ([Fl::Var; 4], None);
            x.block(&blocks[k].entry, &starts);
        }
        for j in 0..blocks.len() {
            if let Some(checks) = x.entries[j] {
                x.fill_checks(j, checks, &starts);
            }
        }
        for (miss, start, ran, pc, (fl, sub)) in std::mem::take(&mut x.misses) {
            x.enter(miss);
            (x.fl, x.sub, x.pend) = (fl, sub, 0);
            // (`ran` counts the op: it has not run)
            x.exit_at(EXIT_NEXT, start, ran - 1, pc);
        }
        x.fill_slow_exit();
        if let Some(c) = x.chain_block {
            x.fill_chain(c);
        }
        // The exit (the registers and flags are in the core already)
        x.b.switch_to_block(exit);
        let kind = x.b.block_params(exit)[0];
        let cyc = x.b.use_var(x.cyc);
        x.b.ins().store(mem(), cyc, x.ctx, CTX_CYCLES);
        let steps = x.b.use_var(x.steps);
        x.b.ins().store(mem(), steps, x.ctx, CTX_STEPS);
        x.b.ins().return_(&[kind]);
        let Emit { mut b, .. } = x;
        b.seal_all_blocks();
        b.finalize(config);
    }

    /// Goes on in `blk`, all of whose predecessors are emitted: sealing it now spares the SSA construction the
    /// block parameters it would otherwise make for every variable used there.
    fn enter(&mut self, blk: Block) {
        self.b.switch_to_block(blk);
        self.b.seal_block(blk);
    }

    // ---- state

    fn reg(&mut self, r: u8) -> Value {
        self.b.use_var(self.regs[(r & 15) as usize])
    }

    /// Writes a register: in the core at once (so that exits and thunks find it there), and for the ops after.
    fn set_reg(&mut self, r: u8, v: Value) {
        self.b.def_var(self.regs[(r & 15) as usize], v);
        self.b.ins().store(mem(), v, self.cpu, R + 4 * (r & 15) as i32);
    }

    fn set_reg_imm(&mut self, r: u8, v: u32) {
        let v = self.imm(v);
        self.set_reg(r, v);
    }

    /// A flag's value now (computed where it is lazy; it stays so).
    fn flag(&mut self, f: i32) -> Value {
        let i = FLAGS.iter().position(|&o| o == f).unwrap();
        self.flag_at(i)
    }

    fn flag_at(&mut self, i: usize) -> Value {
        match self.fl[i] {
            Fl::Var => self.b.use_var(self.flags[i]),
            Fl::Val(v) => v,
            Fl::Neg(r) => {
                let n = self.b.ins().ushr_imm_u(r, 31);
                self.b.ins().ireduce(I8, n)
            }
            Fl::Zero(r) => self.b.ins().icmp_imm_u(IntCC::Equal, r, 0),
            Fl::Carry(x, y, c) => {
                let b = &mut self.b;
                let (x64, y64, c64) = (b.ins().uextend(I64, x), b.ins().uextend(I64, y), b.ins().uextend(I64, c));
                let s = b.ins().iadd(x64, y64);
                let s = b.ins().iadd(s, c64);
                let c = b.ins().ushr_imm_u(s, 32);
                b.ins().ireduce(I8, c)
            }
            Fl::Over(x, y, r) => {
                let b = &mut self.b;
                let (xr, yr) = (b.ins().bxor(x, r), b.ins().bxor(y, r));
                let v = b.ins().band(xr, yr);
                let v = b.ins().ushr_imm_u(v, 31);
                b.ins().ireduce(I8, v)
            }
        }
    }

    fn set_flag(&mut self, f: i32, v: Value) {
        let i = FLAGS.iter().position(|&o| o == f).unwrap();
        self.fl[i] = Fl::Val(v);
        self.sub = None;
    }

    /// Puts the flags in their variables and the core, on the path being emitted (where the region may be left or a
    /// thunk may read them, or before another block). What is emitted after, on other paths, still has them as they
    /// were.
    fn store_flags(&mut self) {
        for (i, &off) in FLAGS.iter().enumerate() {
            if !matches!(self.fl[i], Fl::Var) {
                let v = self.flag_at(i);
                self.b.def_var(self.flags[i], v);
                self.b.ins().store(mem(), v, self.cpu, off);
            }
        }
    }

    fn imm(&mut self, v: u32) -> Value {
        self.b.ins().iconst(I32, v as i32 as i64)
    }

    fn add_cycles(&mut self, c: Value) {
        let c = self.b.ins().uextend(I64, c);
        let now = self.b.use_var(self.cyc);
        let sum = self.b.ins().iadd(now, c);
        self.b.def_var(self.cyc, sum);
    }

    /// Adds the pending cycles to `cyc`.
    fn flush(&mut self) {
        if self.pend != 0 {
            let c = std::mem::take(&mut self.pend);
            self.add_cycles_imm(c);
        }
    }

    fn add_cycles_imm(&mut self, c: u32) {
        let now = self.b.use_var(self.cyc);
        let sum = self.b.ins().iadd_imm_u(now, c as i64);
        self.b.def_var(self.cyc, sum);
    }

    /// N and Z from a result.
    fn nz(&mut self, r: Value) {
        (self.fl[0], self.fl[1]) = (Fl::Neg(r), Fl::Zero(r));
        self.sub = None;
    }

    /// `AddWithCarry` with N, Z, C and V set.
    fn adc(&mut self, x: Value, y: Value, carry: Value) -> Value {
        let s = self.b.ins().iadd(x, y);
        let c32 = self.b.ins().uextend(I32, carry);
        let r = self.b.ins().iadd(s, c32);
        self.nz(r);
        (self.fl[2], self.fl[3]) = (Fl::Carry(x, y, carry), Fl::Over(x, y, r));
        r
    }

    fn adc_const(&mut self, x: Value, y: Value, carry: bool) -> Value {
        let c = self.b.ins().iconst(I8, carry as i64);
        self.adc(x, y, c)
    }

    /// `x - y` (`AddWithCarry(x, NOT y, 1)`) with N, Z, C and V set, and noted as a comparison of `x` and `y`.
    fn sub(&mut self, x: Value, y: Value) -> Value {
        let ny = self.b.ins().bnot(y);
        let r = self.adc_const(x, ny, true);
        self.sub = Some((x, y));
        r
    }

    /// The bit 0 of `v` as a flag.
    fn bit0(&mut self, v: Value) -> Value {
        let v = self.b.ins().band_imm_u(v, 1);
        self.b.ins().ireduce(I8, v)
    }

    /// Whether condition `cond` (0-13) passes, as 0 or 1.
    fn condition(&mut self, cond: u8) -> Value {
        // After a comparison: a comparison of its operands
        if let Some((x, y)) = self.sub
            && cond >> 1 != 3
        {
            let cc = match cond {
                0 => IntCC::Equal,
                1 => IntCC::NotEqual,
                2 => IntCC::UnsignedGreaterThanOrEqual,
                3 => IntCC::UnsignedLessThan,
                8 => IntCC::UnsignedGreaterThan,
                9 => IntCC::UnsignedLessThanOrEqual,
                10 => IntCC::SignedGreaterThanOrEqual,
                11 => IntCC::SignedLessThan,
                12 => IntCC::SignedGreaterThan,
                13 => IntCC::SignedLessThanOrEqual,
                // MI, PL: the result's sign
                _ => {
                    let r = self.b.ins().isub(x, y);
                    let cc = if cond == 4 { IntCC::SignedLessThan } else { IntCC::SignedGreaterThanOrEqual };
                    return self.b.ins().icmp_imm_u(cc, r, 0);
                }
            };
            return self.b.ins().icmp(cc, x, y);
        }
        // Z and N of a result
        match (cond, self.fl[0], self.fl[1]) {
            (0 | 1, _, Fl::Zero(r)) => {
                let cc = if cond == 0 { IntCC::Equal } else { IntCC::NotEqual };
                return self.b.ins().icmp_imm_u(cc, r, 0);
            }
            (4 | 5, Fl::Neg(r), _) => {
                let cc = if cond == 4 { IntCC::SignedLessThan } else { IntCC::SignedGreaterThanOrEqual };
                return self.b.ins().icmp_imm_u(cc, r, 0);
            }
            _ => {}
        }
        let base = match cond >> 1 {
            0 => self.flag(Z),
            1 => self.flag(C),
            2 => self.flag(N),
            3 => self.flag(V),
            4 => {
                let (c, z) = (self.flag(C), self.flag(Z));
                let nz = self.b.ins().bxor_imm_u(z, 1);
                self.b.ins().band(c, nz)
            }
            5 => {
                let (n, v) = (self.flag(N), self.flag(V));
                self.b.ins().icmp(IntCC::Equal, n, v)
            }
            _ => {
                let (n, v, z) = (self.flag(N), self.flag(V), self.flag(Z));
                let ge = self.b.ins().icmp(IntCC::Equal, n, v);
                let nz = self.b.ins().bxor_imm_u(z, 1);
                self.b.ins().band(ge, nz)
            }
        };
        if cond & 1 == 1 { self.b.ins().bxor_imm_u(base, 1) } else { base }
    }

    // ---- control

    /// Leaves the region: `kind`, `ran` ops of the current block run (whose steps start at `start`), the PC set to
    /// `pc` if given. The cycles are the variable's.
    fn exit(&mut self, kind: u32, start: Value, ran: usize, pc: Option<Value>) {
        self.store_flags();
        if let Some(pc) = pc {
            self.set_reg(15, pc);
        }
        let s = self.b.ins().iadd_imm_u(start, ran as i64);
        self.b.def_var(self.steps, s);
        let k = self.b.ins().iconst(I32, kind as i64);
        self.b.ins().jump(self.exit, &[k.into()]);
    }

    fn exit_at(&mut self, kind: u32, start: Value, ran: usize, pc: u32) {
        let pc = self.imm(pc);
        self.exit(kind, start, ran, Some(pc));
    }

    /// Calls a thunk: `(ctx, a, b, c, elapsed) -> u64`, elapsed the cycles now.
    fn call(&mut self, slot: i32, a: Value, b: Value, c: Value) -> Value {
        let f = self.b.ins().load(self.ptr, fixed(), self.ctx, slot);
        let elapsed = self.b.use_var(self.cyc);
        let call = self.b.ins().call_indirect(self.thunk, f, &[self.ctx, a, b, c, elapsed]);
        self.b.inst_results(call)[0]
    }

    /// After a thunk (the `ran`th op of the block, at `pc`, `c` its cycles, `next` the PC after it): to the shared
    /// exit if it faulted or must leave, if `always` whatever it says; else goes on (and counts `c`).
    #[allow(clippy::too_many_arguments)]
    fn after_thunk(&mut self, r: Value, start: Value, ran: usize, pc: u32, next: Value, c: Value, always: bool) {
        let steps = self.b.ins().iadd_imm_u(start, ran as i64);
        let pc = self.imm(pc);
        let args = [r.into(), pc.into(), next.into(), steps.into(), c.into()];
        if always {
            self.store_flags();
            self.b.ins().jump(self.slow_exit, &args);
            return;
        }
        let (on, off) = (self.b.create_block(), self.b.create_block());
        let out = self.b.ins().band_imm_u(r, (FAULT_BIT | EXIT_BIT) as i64);
        self.b.ins().brif(out, off, &[], on, &[]);
        self.enter(off);
        self.store_flags();
        self.b.ins().jump(self.slow_exit, &args);
        self.enter(on);
        self.add_cycles(c);
    }

    /// The shared exit after a thunk: a fault leaves at the op with the cycles before it, anything else after it
    /// with its cycles and stall cycles, for the machine's attention or not.
    fn fill_slow_exit(&mut self) {
        let x = self.slow_exit;
        self.b.switch_to_block(x);
        let p = self.b.block_params(x).to_vec();
        let (r, pc, next, steps, c) = (p[0], p[1], p[2], p[3], p[4]);
        let fault = self.b.ins().band_imm_u(r, FAULT_BIT as i64);
        let fault = self.b.ins().icmp_imm_u(IntCC::NotEqual, fault, 0);
        let attn = self.b.ins().band_imm_u(r, ATTENTION_BIT as i64);
        let attn = self.b.ins().icmp_imm_u(IntCC::NotEqual, attn, 0);
        let (k_fault, k_batch, k_next) = (self.imm(EXIT_FAULT), self.imm(EXIT_BATCH), self.imm(EXIT_NEXT));
        let k = self.b.ins().select(attn, k_batch, k_next);
        let kind = self.b.ins().select(fault, k_fault, k);
        let r15 = self.b.ins().select(fault, pc, next);
        self.set_reg(15, r15);
        let stall = self.b.ins().load(I32, mem(), self.ctx, CTX_STALL);
        let add = self.b.ins().iadd(c, stall);
        let add = self.b.ins().uextend(I64, add);
        let now = self.b.use_var(self.cyc);
        let after = self.b.ins().iadd(now, add);
        let cyc = self.b.ins().select(fault, now, after);
        self.b.def_var(self.cyc, cyc);
        self.b.def_var(self.steps, steps);
        self.b.ins().jump(self.exit, &[kind.into()]);
    }

    /// 0 or 1: whether a store at `addr` may have reached code of the region (`margin` bytes before a block too).
    fn code_check(&mut self, addr: Value, margin: u32) -> Option<Value> {
        let (lo, hi) = self.code?;
        let d = self.b.ins().iadd_imm_u(addr, lo.wrapping_sub(margin).wrapping_neg() as i64);
        let hit = self.b.ins().icmp_imm_u(IntCC::UnsignedLessThan, d, hi.wrapping_sub(lo).wrapping_add(margin) as i64);
        Some(self.b.ins().uextend(I32, hit))
    }

    /// The window to try an access inline in: the one with `guess` (the address it would make with the registers
    /// as they were when the region was compiled), or for a guess outside every window, the first one it could use.
    fn window_for(&self, guess: u32, write: bool) -> Option<usize> {
        let usable = |w: &Shape| w.size != 0 && (!write || w.writable);
        let w = &self.windows;
        (0..DIRECT_REGIONS)
            .find(|&k| usable(&w[k]) && guess.wrapping_sub(w[k].base) < w[k].size)
            .or_else(|| (0..DIRECT_REGIONS).find(|&k| usable(&w[k])))
    }

    /// The host address of `bytes` bytes at `addr` (aligned to `align`) in direct memory window `k`, for reading or
    /// writing: continues in a new block with it, or jumps to `miss`. Sets the window's mark. (The window is read
    /// from the bus's table as it is now: the shape it had when the region was compiled only says what to try.)
    fn direct(&mut self, addr: Value, bytes: u32, align: u32, write: bool, k: Option<usize>, miss: Block) -> Value {
        let Some(k) = k else {
            self.b.ins().jump(miss, &[]);
            // (Unreachable: what follows is never run)
            let dead = self.b.create_block();
            self.enter(dead);
            return self.b.ins().iconst(self.ptr, 0);
        };
        let shape = self.windows[k];
        let w = WINDOW * k as i32;
        // Each condition a branch of its own (cheaper than combining the comparisons)
        let test = |x: &mut Self, bad: Value| {
            let next = x.b.create_block();
            x.b.ins().brif(bad, miss, &[], next, &[]);
            x.enter(next);
        };
        if align > 1 {
            let odd = self.b.ins().band_imm_u(addr, (align - 1) as i64);
            test(self, odd);
        }
        let base = self.b.ins().load(I32, fixed(), self.mem, w + W_BASE);
        let size = self.b.ins().load(I32, fixed(), self.mem, w + W_SIZE);
        let off = self.b.ins().isub(addr, base);
        // Within the window (sizes are multiples of four: an aligned access of up to a word that starts inside ends
        // inside), and the whole of a longer one
        let outside = self.b.ins().icmp(IntCC::UnsignedGreaterThanOrEqual, off, size);
        test(self, outside);
        if bytes > 4 {
            let rem = self.b.ins().isub(size, off);
            let short = self.b.ins().icmp_imm_u(IntCC::UnsignedLessThan, rem, bytes as i64);
            test(self, short);
        }
        if write {
            let wr = self.b.ins().load(I32, fixed(), self.mem, w + W_WRITABLE);
            let ro = self.b.ins().icmp_imm_u(IntCC::Equal, wr, 0);
            test(self, ro);
        }
        let host = self.b.ins().load(self.ptr, fixed(), self.mem, w + W_PTR);
        let off = if shape.masked {
            let mask = self.b.ins().load(I32, fixed(), self.mem, w + W_MASK);
            self.b.ins().band(off, mask)
        } else {
            off
        };
        let off = self.b.ins().uextend(self.ptr, off);
        let at = self.b.ins().iadd(host, off);
        let (has, mark, value) = if write {
            (shape.write_mark, W_WRITE_MARK, W_WRITE_VALUE)
        } else {
            (shape.read_mark, W_READ_MARK, W_READ_VALUE)
        };
        if has {
            // (A bus with the same shape but no mark now: none set)
            let m = self.b.ins().load(self.ptr, fixed(), self.mem, w + mark);
            let none = self.b.ins().icmp_imm_u(IntCC::Equal, m, 0);
            let m = self.b.ins().select(none, self.sink, m);
            let v = self.b.ins().load(I8, fixed(), self.mem, w + value);
            self.b.ins().store(guest(), v, m, 0);
        }
        at
    }

    /// The loaded value (zero extended) into the load's register, sign extended as the load says.
    fn loaded(&mut self, v: Value, rt: u8, signed: Option<Type>) {
        let v = match signed {
            Some(t) => {
                let narrow = self.b.ins().ireduce(t, v);
                self.b.ins().sextend(I32, narrow)
            }
            None => v,
        };
        self.set_reg(rt, v);
    }

    /// Loads or stores at `addr` for an op (the `ran`th of the block, at `pc`; `guess` the address it would make
    /// now): inline where the bus's direct memory has it, else through the thunk. Counts the op's two cycles; the
    /// block goes on after the op.
    fn access(&mut self, addr: Value, a: Access, start: Value, ran: usize, pc: u32, guess: u32) {
        let (bytes, write) = match a {
            Access::Load(n, ..) => (n, false),
            Access::Store(n, _) => (n, true),
        };
        let ty = match bytes {
            1 => I8,
            2 => I16,
            _ => I32,
        };
        // Inline where it went when the region was compiled was a window; leaving the region at the op if it is
        // not (the batch then makes the access, and will find a block starting at the op to compile)
        let usable = |w: &Shape| w.size != 0 && (!write || w.writable);
        let w = &self.windows;
        if let Some(k) = (0..DIRECT_REGIONS).find(|&k| usable(&w[k]) && guess.wrapping_sub(w[k].base) < w[k].size) {
            debug_assert_eq!(self.pend, 0);
            let miss = self.b.create_block();
            self.misses.push((miss, start, ran, pc, (self.fl, self.sub)));
            let at = self.direct(addr, bytes, bytes, write, Some(k), miss);
            match a {
                Access::Load(_, rt, signed) => {
                    let v = self.b.ins().load(ty, guest(), at, 0);
                    let v = if ty == I32 { v } else { self.b.ins().uextend(I32, v) };
                    self.loaded(v, rt, signed);
                }
                Access::Store(_, v) => {
                    let v = if ty == I32 { v } else { self.b.ins().ireduce(ty, v) };
                    self.b.ins().store(guest(), v, at, 0);
                }
            }
            self.add_cycles_imm(2);
            // (A store into the region's code leaves after it)
            let leave = if write { self.code_check(addr, 4) } else { None };
            self.leave_if(leave, start, ran, pc + 2);
            return;
        }
        // Else through the bus: the register loaded unless it faulted, then the checks, which may leave
        let mut how = self.imm(bytes | if write { WRITE } else { 0 });
        if let Some(l) = self.code_check(addr, 4) {
            let l = self.b.ins().ishl_imm_u(l, 9);
            how = self.b.ins().bor(how, l);
            debug_assert_eq!(LEAVE, 1 << 9);
        }
        let value = match a {
            Access::Store(_, v) => v,
            Access::Load(..) => self.imm(0),
        };
        let r = self.call(ACCESS, addr, how, value);
        if let Access::Load(_, rt, signed) = a {
            // (The register as it was is in the core: nothing lives across the call)
            let old = self.b.ins().load(I32, mem(), self.cpu, R + 4 * (rt & 15) as i32);
            let mut v = self.b.ins().ireduce(I32, r);
            if let Some(t) = signed {
                let narrow = self.b.ins().ireduce(t, v);
                v = self.b.ins().sextend(I32, narrow);
            }
            let f = self.b.ins().band_imm_u(r, FAULT_BIT as i64);
            let kept = self.b.ins().select(f, old, v);
            self.set_reg(rt, kept);
        }
        let (next, two) = (self.imm(pc + 2), self.imm(2));
        self.after_thunk(r, start, ran, pc, next, two, false);
    }

    /// The address a load or store with base `b` and offset `m` (a register) or `imm` would make with the registers
    /// as they were when the region was compiled.
    fn guess(&self, b: u8, m: u8, kind: K, imm: u32) -> u32 {
        let base = self.regs_now[b as usize];
        match kind {
            K::LdrI | K::LdrhI | K::LdrbI | K::StrI | K::StrhI | K::StrbI => base.wrapping_add(imm),
            _ => base.wrapping_add(self.regs_now[m as usize]),
        }
    }

    /// Leaves the region after the op (the PC at `next`) if `leave` (a store may have changed its code).
    fn leave_if(&mut self, leave: Option<Value>, start: Value, ran: usize, next: u32) {
        let Some(l) = leave else { return };
        let (out, on) = (self.b.create_block(), self.b.create_block());
        self.b.ins().brif(l, out, &[], on, &[]);
        self.enter(out);
        self.exit_at(EXIT_NEXT, start, ran, next);
        self.enter(on);
    }

    /// Goes on to the block at `target` (known now) if it is in the region, through its checks; else leaves there.
    fn goto(&mut self, target: u32, start: Value, ran: usize, starts: &[Block]) {
        self.store_flags();
        match self.blocks.iter().position(|b| b.entry.start() == target) {
            Some(j) => {
                let s = self.b.ins().iadd_imm_u(start, ran as i64);
                self.b.def_var(self.steps, s);
                self.set_reg_imm(15, target);
                let e = self.entry_checks(j, starts);
                self.b.ins().jump(e, &[]);
            }
            None => {
                let s = self.b.ins().iadd_imm_u(start, ran as i64);
                self.b.def_var(self.steps, s);
                self.set_reg_imm(15, target);
                let t = self.imm(target);
                self.chain(t);
            }
        }
    }

    /// Goes on to `target` (a value: an indirect branch's): to the region's block there if it is one calls return
    /// to, else leaves there.
    fn goto_value(&mut self, target: Value, start: Value, ran: usize, starts: &[Block]) {
        self.store_flags();
        let s = self.b.ins().iadd_imm_u(start, ran as i64);
        self.b.def_var(self.steps, s);
        self.set_reg(15, target);
        for j in 0..self.blocks.len() {
            if !self.blocks[j].return_site {
                continue;
            }
            let (there, next) = (self.b.create_block(), self.b.create_block());
            let is = self.b.ins().icmp_imm_u(IntCC::Equal, target, self.blocks[j].entry.start() as i64);
            self.b.ins().brif(is, there, &[], next, &[]);
            self.enter(there);
            let e = self.entry_checks(j, starts);
            self.b.ins().jump(e, &[]);
            self.enter(next);
        }
        self.chain(target);
    }

    /// Goes on to the region at `target` if it is compiled and the batch would run its first block whole there,
    /// making the checks `run_blocks` makes and that the block's code is still in memory; else leaves there. The
    /// steps, the PC (`target`) and the flags are as they are there. The region gone on to is found in a chain
    /// site's cell (filled by the `resolve` thunk).
    fn chain(&mut self, target: Value) {
        let k = self.next_cell.min(self.cells.len() - 1);
        self.next_cell += 1;
        let cell = self.b.ins().iconst(self.ptr, &self.cells[k] as *const _ as i64);
        let c = match self.chain_block {
            Some(c) => c,
            None => {
                let c = self.b.create_block();
                self.b.append_block_param(c, I32);
                self.b.append_block_param(c, self.ptr);
                self.chain_block = Some(c);
                c
            }
        };
        self.b.ins().jump(c, &[target.into(), cell.into()]);
    }

    /// The region's chain block (see [`Emit::chain`]).
    fn fill_chain(&mut self, c: Block) {
        self.b.switch_to_block(c);
        let p = self.b.block_params(c).to_vec();
        let (target, cell) = (p[0], p[1]);
        let (known, resolve, check, out) =
            (self.b.create_block(), self.b.create_block(), self.b.create_block(), self.b.create_block());
        self.b.append_block_param(check, self.ptr);
        // (No cycle among these blocks)
        let d0 = self.b.ins().atomic_load(self.ptr, MemFlagsData::trusted(), cell);
        self.b.ins().brif(d0, known, &[], resolve, &[]);
        self.enter(known);
        let pc = self.b.ins().load(I32, fixed(), d0, D_PC);
        let same = self.b.ins().icmp(IntCC::Equal, pc, target);
        self.b.ins().brif(same, check, &[d0.into()], resolve, &[]);
        // Not known yet (or another target): ask
        self.enter(resolve);
        let z = self.imm(0);
        let f = self.b.ins().load(self.ptr, fixed(), self.ctx, RESOLVE);
        let call = self.b.ins().call_indirect(self.thunk, f, &[self.ctx, target, z, z, cell]);
        let d1 = self.b.inst_results(call)[0];
        self.b.ins().brif(d1, check, &[d1.into()], out, &[]);
        self.enter(check);
        let d = self.b.block_params(check)[0];
        let fail = |x: &mut Self, bad: Value, to: Block| {
            let next = x.b.create_block();
            x.b.ins().brif(bad, to, &[], next, &[]);
            x.enter(next);
        };
        let window = self.b.ins().load(I32, fixed(), d, D_WINDOW);
        let none = self.b.ins().icmp_imm_u(IntCC::UnsignedGreaterThanOrEqual, window, DIRECT_REGIONS as i64);
        fail(self, none, out);
        // The checks before a block runs whole: the steps left, the budget, not a trapped address
        let ops = self.b.ins().load(I32, fixed(), d, D_OPS);
        let steps = self.b.use_var(self.steps);
        let need = self.b.ins().iadd(steps, ops);
        let left = self.b.ins().load(I32, fixed(), self.ctx, CTX_STEPS_LEFT);
        let short = self.b.ins().icmp(IntCC::UnsignedGreaterThan, need, left);
        fail(self, short, out);
        let lead = self.b.ins().load(I32, fixed(), d, D_LEAD);
        let lead = self.b.ins().uextend(I64, lead);
        let cyc = self.b.use_var(self.cyc);
        let at_last = self.b.ins().iadd(cyc, lead);
        let budget = self.b.ins().load(I64, fixed(), self.ctx, CTX_BUDGET);
        let late = self.b.ins().icmp(IntCC::UnsignedGreaterThanOrEqual, at_last, budget);
        fail(self, late, out);
        let trap = self.b.ins().load(I32, fixed(), self.ctx, CTX_TRAP);
        let trapped = self.b.ins().icmp(IntCC::UnsignedLessThan, target, trap);
        fail(self, trapped, out);
        // The code, read from its window: as fetching it would, that sets the window's read mark
        let woff = self.b.ins().imul_imm_u(window, WINDOW as i64);
        let woff = self.b.ins().uextend(self.ptr, woff);
        let w = self.b.ins().iadd(self.mem, woff);
        let base = self.b.ins().load(I32, fixed(), w, W_BASE);
        let size = self.b.ins().load(I32, fixed(), w, W_SIZE);
        let off = self.b.ins().isub(target, base);
        let outside = self.b.ins().icmp(IntCC::UnsignedGreaterThanOrEqual, off, size);
        fail(self, outside, out);
        let mask = self.b.ins().load(I32, fixed(), w, W_MASK);
        let off = self.b.ins().band(off, mask);
        let len = self.b.ins().load(I32, fixed(), w, W_LEN);
        let room = self.b.ins().isub(len, off);
        let short = self.b.ins().icmp_imm_u(IntCC::UnsignedLessThan, room, 32);
        let beyond = self.b.ins().icmp(IntCC::UnsignedGreaterThan, off, len);
        let short = self.b.ins().bor(short, beyond);
        fail(self, short, out);
        let host = self.b.ins().load(self.ptr, fixed(), w, W_PTR);
        let off = self.b.ins().uextend(self.ptr, off);
        let at = self.b.ins().iadd(host, off);
        let mut diff = self.b.ins().iconst(I64, 0);
        for k in 0..4 {
            let have = self.b.ins().load(I64, guest(), at, 8 * k);
            let want = self.b.ins().load(I64, fixed(), d, D_WORDS + 8 * k);
            let m = self.b.ins().load(I64, fixed(), d, D_MASKS + 8 * k);
            let x = self.b.ins().bxor(have, want);
            let x = self.b.ins().band(x, m);
            diff = self.b.ins().bor(diff, x);
        }
        fail(self, diff, out);
        let m = self.b.ins().load(self.ptr, fixed(), w, W_READ_MARK);
        let nomark = self.b.ins().icmp_imm_u(IntCC::Equal, m, 0);
        let m = self.b.ins().select(nomark, self.sink, m);
        let v = self.b.ins().load(I8, fixed(), w, W_READ_VALUE);
        self.b.ins().store(guest(), v, m, 0);
        // On: the counts in the context for it
        let cyc = self.b.use_var(self.cyc);
        self.b.ins().store(mem(), cyc, self.ctx, CTX_CYCLES);
        let steps = self.b.use_var(self.steps);
        self.b.ins().store(mem(), steps, self.ctx, CTX_STEPS);
        let code = self.b.ins().load(self.ptr, fixed(), d, D_CODE);
        self.b.ins().return_call_indirect(self.region, code, &[self.ctx]);
        // Else back to the batch, there
        self.enter(out);
        let k = self.b.ins().iconst(I32, EXIT_NEXT as i64);
        self.b.ins().jump(self.exit, &[k.into()]);
    }

    /// Where the checks before block `j` start (filled in by [`Emit::fill_checks`]).
    fn entry_checks(&mut self, j: usize, _starts: &[Block]) -> Block {
        if self.entries[j].is_none() {
            self.entries[j] = Some(self.b.create_block());
            self.reached.push(j);
        }
        self.entries[j].unwrap()
    }

    /// The checks before block `j` runs, as `run_blocks` makes them before running a block whole, and that its code
    /// is still in memory (once per call of the region); then the block. The steps, the cycles and the PC (at the
    /// block) are as they are there.
    fn fill_checks(&mut self, j: usize, checks: Block, starts: &[Block]) {
        self.b.switch_to_block(checks);
        let e = self.blocks[j].entry;
        let n = e.ops().len() as u32;
        let last = e.lead_cycles(e.ops().len() - 1);
        let out = self.b.create_block();
        // The steps left take the whole block, and the budget is not reached before its last op
        let steps = self.b.use_var(self.steps);
        let left = self.b.ins().load(I32, fixed(), self.ctx, CTX_STEPS_LEFT);
        let need = self.b.ins().iadd_imm_u(steps, n as i64);
        let ok1 = self.b.ins().icmp(IntCC::UnsignedLessThanOrEqual, need, left);
        let cyc = self.b.use_var(self.cyc);
        let budget = self.b.ins().load(I64, fixed(), self.ctx, CTX_BUDGET);
        let at_last = self.b.ins().iadd_imm_u(cyc, last as i64);
        let ok2 = self.b.ins().icmp(IntCC::UnsignedLessThan, at_last, budget);
        // Not a trapped address
        let trap = self.b.ins().load(I32, fixed(), self.ctx, CTX_TRAP);
        let ok3 = self.b.ins().icmp_imm_u(IntCC::UnsignedLessThanOrEqual, trap, e.start() as i64);
        let ok = self.b.ins().band(ok1, ok2);
        let ok = self.b.ins().band(ok, ok3);
        let (go, validate) = (self.b.create_block(), self.b.create_block());
        let fine = self.b.create_block();
        self.b.ins().brif(ok, go, &[], out, &[]);
        self.enter(go);
        if j == 0 {
            // (Found by the batch)
            self.b.ins().jump(starts[0], &[]);
        } else {
            let valid = self.b.use_var(self.valid);
            let seen = self.b.ins().band_imm_u(valid, 1i64 << j);
            self.b.ins().brif(seen, starts[j], &[], validate, &[]);
            // The code, compared halfword by halfword with what the block was compiled from; as a fetch would, the
            // fetch sets the window's read mark
            self.enter(validate);
            let bytes = e.len();
            let pc = self.imm(e.start());
            let k = self.window_for(e.start(), false);
            let at = self.direct(pc, bytes, 2, false, k, out);
            let code = e.code_bytes();
            let mut diff = self.imm(0);
            let mut k = 0;
            while k < bytes as usize {
                let (ty, want) = if bytes as usize - k >= 4 {
                    (I32, u32::from_le_bytes([code[k], code[k + 1], code[k + 2], code[k + 3]]))
                } else {
                    (I16, u16::from_le_bytes([code[k], code[k + 1]]) as u32)
                };
                let v = self.b.ins().load(ty, guest(), at, k as i32);
                let v = if ty == I32 { v } else { self.b.ins().uextend(I32, v) };
                let d = self.b.ins().bxor_imm_u(v, want as i64);
                diff = self.b.ins().bor(diff, d);
                k += if ty == I32 { 4 } else { 2 };
            }
            self.b.ins().brif(diff, out, &[], fine, &[]);
            self.enter(fine);
            let valid = self.b.use_var(self.valid);
            let valid = self.b.ins().bor_imm_u(valid, 1i64 << j);
            self.b.def_var(self.valid, valid);
            self.b.ins().jump(starts[j], &[]);
        }
        // Else back to the batch, at the block
        self.enter(out);
        let k = self.b.ins().iconst(I32, EXIT_NEXT as i64);
        self.b.ins().jump(self.exit, &[k.into()]);
    }

    /// Emits a block: its ops, then where it goes.
    fn block(&mut self, e: &Entry, starts: &[Block]) {
        let start = self.b.use_var(self.steps);
        let (lo, ops) = (e.start(), e.ops());
        for (i, &op) in ops.iter().enumerate() {
            if !self.op(op, i, lo, start, starts) {
                return;
            }
        }
        // Ran to its end: on to the next instruction
        self.flush();
        let last = ops[ops.len() - 1];
        let next = lo.wrapping_add(last.off as u32 + 2);
        if last.kind == K::BCond || !last.kind.sets_pc() {
            self.goto(next, start, ops.len(), starts);
        } else {
            // (The ops that leave by themselves have returned)
            unreachable!("{:?} ends a block", last.kind);
        }
    }

    /// A multi-word transfer (`kind`: STM, LDM, PUSH, POP) inline where its words are all in one direct memory
    /// window, else through the thunk (and then the region is left). `rn` is the base register (13 for PUSH and POP),
    /// `list` its registers.
    fn multi(&mut self, kind: u32, rn: u8, list: u32, start: Value, ran: usize, pc: u32) {
        let count = list.count_ones();
        let write = matches!(kind, STM | PUSH);
        let base = self.reg(rn);
        let first = if kind == PUSH { self.b.ins().iadd_imm_u(base, (4 * count).wrapping_neg() as i64) } else { base };
        let guess = self.regs_now[rn as usize].wrapping_sub(if kind == PUSH { 4 * count } else { 0 });
        let k = self.window_for(guess, write);
        let (slow, join) = (self.b.create_block(), self.b.create_block());
        let at = self.direct(first, 4 * count, 4, write, k, slow);
        // The registers in order, from the lowest address (as `op_stm`, `op_push` with the ram, `ldm`, `pop`)
        let regs: Vec<u8> = (0..15).filter(|&i| list & (1 << i) != 0).collect();
        if write {
            let vals: Vec<Value> = regs.iter().map(|&r| self.reg(r)).collect();
            for (k, v) in vals.into_iter().enumerate() {
                self.b.ins().store(guest(), v, at, 4 * k as i32);
            }
        } else {
            let vals: Vec<Value> = (0..regs.len()).map(|k| self.b.ins().load(I32, guest(), at, 4 * k as i32)).collect();
            for (&r, v) in regs.iter().zip(vals) {
                self.set_reg(r, v);
            }
        }
        let end = self.b.ins().iadd_imm_u(first, (4 * count) as i64);
        match kind {
            STM | POP => self.set_reg(rn, end),
            LDM if list & (1 << rn) == 0 => self.set_reg(rn, end),
            PUSH => self.set_reg(13, first),
            _ => {}
        }
        self.add_cycles_imm(1 + count);
        // Stores may reach any part of a block within the words stored: from the base (STM) or the SP pushed to
        if write {
            let leave = self.code_check(if kind == STM { base } else { first }, 64);
            self.leave_if(leave, start, ran, pc + 2);
        }
        self.b.ins().jump(join, &[]);
        // Through the bus, then out
        self.enter(slow);
        let (k, r, l) = (self.imm(kind), self.imm(rn as u32), self.imm(list));
        let res = self.call(MULTI, k, r, l);
        // (Then out: the thunk may have changed registers, in the core)
        let (next, c) = (self.imm(pc + 2), self.b.ins().ireduce(I32, res));
        self.after_thunk(res, start, ran, pc, next, c, true);
        self.enter(join);
    }

    /// `BX`/`BLX`/`POP {pc}` to `target` (`popped`: the POP's words were read inline): an interworking branch to an
    /// address with bit 0 set, not an exception return, branches here; anything else is the thunk's.
    fn interwork(&mut self, target: Value, cycles: u32, start: Value, ran: usize, starts: &[Block]) {
        self.add_cycles_imm(cycles);
        let t = self.b.ins().band_imm_u(target, 0xFFFF_FFFEu32 as i64);
        self.goto_value(t, start, ran, starts);
    }

    /// Whether a branch to `target` is a plain interworking branch (bit 0 set; not an exception return).
    fn plain_branch(&mut self, target: Value) -> Value {
        let thumb = self.b.ins().band_imm_u(target, 1);
        let ipsr = self.b.ins().load(I32, mem(), self.cpu, IPSR);
        let handler = self.b.ins().icmp_imm_u(IntCC::NotEqual, ipsr, 0);
        let magic = self.b.ins().icmp_imm_u(IntCC::UnsignedGreaterThanOrEqual, target, 0xF000_0000u32 as i64);
        let ret = self.b.ins().band(handler, magic);
        let thumb = self.b.ins().icmp_imm_u(IntCC::NotEqual, thumb, 0);
        let not_ret = self.b.ins().bxor_imm_u(ret, 1);
        self.b.ins().band(thumb, not_ret)
    }

    /// A branch thunk (`kind`: BX, POP_PC, STEP) for the last op of a block; then out (where the thunk left the PC).
    fn branch_thunk(&mut self, kind: u32, arg: Value, start: Value, ran: usize, pc: u32) {
        // (The thunk may read the flags, a `MRS APSR` or the xPSR stacked, and write them, an exception return: from
        // here on they are the core's)
        self.store_flags();
        let (fl, sub) = (std::mem::replace(&mut self.fl, [Fl::Var; 4]), self.sub.take());
        let (k, at) = (self.imm(kind), self.imm(pc));
        let r = self.call(BRANCH, k, arg, at);
        // (Then out, the PC where the thunk left it)
        let next = self.b.ins().load(I32, mem(), self.cpu, R + 4 * 15);
        let c = self.b.ins().ireduce(I32, r);
        self.after_thunk(r, start, ran, pc, next, c, true);
        (self.fl, self.sub) = (fl, sub);
    }

    /// Emits op `i` of the block at `lo` (whose steps start at `start`). Returns false if it left the block itself.
    fn op(&mut self, op: Op, i: usize, lo: u32, start: Value, starts: &[Block]) -> bool {
        let (a, bb, m, imm) = (op.a & 15, op.b & 15, (op.imm & 15) as u8, op.imm);
        let pc = lo.wrapping_add(op.off as u32);
        let ran = i + 1;
        if op.kind.uses_bus() {
            self.flush();
        }
        macro_rules! logic {
            ($r:expr) => {{
                let r = $r;
                self.nz(r);
                self.set_reg(a, r);
            }};
        }
        match op.kind {
            K::LslI => {
                let v = self.reg(bb);
                let c = self.b.ins().ushr_imm_u(v, (32 - imm) as i64);
                let c = self.bit0(c);
                self.set_flag(C, c);
                logic!(self.b.ins().ishl_imm_u(v, imm as i64));
            }
            K::LsrI | K::AsrI => {
                let v = self.reg(bb);
                let c = self.b.ins().ushr_imm_u(v, (imm - 1) as i64);
                let c = self.bit0(c);
                self.set_flag(C, c);
                let r = if op.kind == K::LsrI {
                    self.b.ins().ushr_imm_u(v, imm as i64)
                } else {
                    self.b.ins().sshr_imm_u(v, imm as i64)
                };
                logic!(r);
            }
            K::Movs => logic!(self.reg(bb)),
            K::Lsr32 | K::Asr32 => {
                let v = self.reg(bb);
                let c = self.b.ins().ushr_imm_u(v, 31);
                let c = self.b.ins().ireduce(I8, c);
                self.set_flag(C, c);
                let r = if op.kind == K::Lsr32 { self.imm(0) } else { self.b.ins().sshr_imm_u(v, 31) };
                logic!(r);
            }
            K::AddR | K::SubR => {
                let (x, y) = (self.reg(bb), self.reg(m));
                let r = if op.kind == K::AddR { self.adc_const(x, y, false) } else { self.sub(x, y) };
                self.set_reg(a, r);
            }
            K::AddI3 | K::SubI3 | K::AddI8 | K::SubI8 | K::CmpI => {
                let x = if matches!(op.kind, K::AddI3 | K::SubI3) { self.reg(bb) } else { self.reg(a) };
                let r = if matches!(op.kind, K::AddI3 | K::AddI8) {
                    let y = self.imm(imm);
                    self.adc_const(x, y, false)
                } else {
                    let y = self.imm(imm);
                    self.sub(x, y)
                };
                if op.kind != K::CmpI {
                    self.set_reg(a, r);
                }
            }
            K::MovI => {
                let v = self.imm(imm);
                logic!(v);
            }
            K::And | K::Eor | K::Orr | K::Bic | K::Mul | K::Tst => {
                let (x, y) = (self.reg(a), self.reg(bb));
                let r = match op.kind {
                    K::And | K::Tst => self.b.ins().band(x, y),
                    K::Eor => self.b.ins().bxor(x, y),
                    K::Orr => self.b.ins().bor(x, y),
                    K::Bic => self.b.ins().band_not(x, y),
                    _ => self.b.ins().imul(x, y),
                };
                if op.kind == K::Tst {
                    self.nz(r);
                } else {
                    logic!(r);
                }
            }
            K::Mvn => {
                let y = self.reg(bb);
                logic!(self.b.ins().bnot(y));
            }
            K::LslR | K::LsrR | K::AsrR | K::Ror => {
                let (dn, s) = (self.reg(a), self.reg(bb));
                let s = self.b.ins().band_imm_u(s, 0xFF);
                let c_old = self.flag(C);
                let zero = self.b.ins().icmp_imm_u(IntCC::Equal, s, 0);
                let below32 = self.b.ins().icmp_imm_u(IntCC::UnsignedLessThan, s, 32);
                let (r, cbit) = match op.kind {
                    K::LslR => {
                        let shifted = self.b.ins().ishl(dn, s);
                        let z = self.imm(0);
                        let r = self.b.ins().select(below32, shifted, z);
                        // Bit 32 - s, for s in 1..=32
                        let k32 = self.imm(32);
                        let k = self.b.ins().isub(k32, s);
                        let k = self.b.ins().band_imm_u(k, 31);
                        let bit = self.b.ins().ushr(dn, k);
                        let bit = self.bit0(bit);
                        let upto32 = self.b.ins().icmp_imm_u(IntCC::UnsignedLessThanOrEqual, s, 32);
                        let z8 = self.b.ins().iconst(I8, 0);
                        (r, self.b.ins().select(upto32, bit, z8))
                    }
                    K::LsrR => {
                        let shifted = self.b.ins().ushr(dn, s);
                        let z = self.imm(0);
                        let r = self.b.ins().select(below32, shifted, z);
                        // Bit s - 1, for s in 1..=32
                        let k = self.b.ins().iadd_imm_u(s, 0xFFFF_FFFFu32 as i64);
                        let k = self.b.ins().band_imm_u(k, 31);
                        let bit = self.b.ins().ushr(dn, k);
                        let bit = self.bit0(bit);
                        let upto32 = self.b.ins().icmp_imm_u(IntCC::UnsignedLessThanOrEqual, s, 32);
                        let z8 = self.b.ins().iconst(I8, 0);
                        (r, self.b.ins().select(upto32, bit, z8))
                    }
                    K::AsrR => {
                        let shifted = self.b.ins().sshr(dn, s);
                        let sign = self.b.ins().sshr_imm_u(dn, 31);
                        let r = self.b.ins().select(below32, shifted, sign);
                        // Bit s - 1 for s in 1..32, the sign beyond
                        let k = self.b.ins().iadd_imm_u(s, 0xFFFF_FFFFu32 as i64);
                        let k = self.b.ins().band_imm_u(k, 31);
                        let k31 = self.imm(31);
                        let k = self.b.ins().select(below32, k, k31);
                        let bit = self.b.ins().ushr(dn, k);
                        (r, self.bit0(bit))
                    }
                    _ => {
                        let r = self.b.ins().rotr(dn, s);
                        let c = self.b.ins().ushr_imm_u(r, 31);
                        let c = self.b.ins().ireduce(I8, c);
                        (self.b.ins().select(zero, dn, r), c)
                    }
                };
                let c = self.b.ins().select(zero, c_old, cbit);
                self.set_flag(C, c);
                logic!(r);
            }
            K::Adc | K::Sbc => {
                let (x, y, c) = (self.reg(a), self.reg(bb), self.flag(C));
                let y = if op.kind == K::Sbc { self.b.ins().bnot(y) } else { y };
                let r = self.adc(x, y, c);
                self.set_reg(a, r);
            }
            K::Rsb => {
                let y = self.reg(bb);
                let ny = self.b.ins().bnot(y);
                let z = self.imm(0);
                let r = self.adc_const(ny, z, true);
                self.set_reg(a, r);
            }
            K::CmpR | K::CmpHi => {
                let (x, y) = (self.reg(a), self.reg(bb));
                self.sub(x, y);
            }
            K::Cmn => {
                let (x, y) = (self.reg(a), self.reg(bb));
                self.adc_const(x, y, false);
            }
            K::AddHi | K::MovHi => {
                let mut r = self.reg(bb);
                if op.kind == K::AddHi {
                    let x = self.reg(a);
                    r = self.b.ins().iadd(x, r);
                }
                if a == 13 {
                    r = self.b.ins().band_imm_u(r, 0xFFFF_FFFCu32 as i64);
                }
                self.set_reg(a, r);
            }
            K::Set => self.set_reg_imm(a, imm),
            K::AddSp | K::SpAdd => {
                let sp = self.reg(13);
                let r = self.b.ins().iadd_imm_u(sp, imm as i64);
                self.set_reg(if op.kind == K::AddSp { a } else { 13 }, r);
            }
            K::Sxth | K::Sxtb | K::Uxth | K::Uxtb | K::Rev | K::Rev16 | K::Revsh => {
                let v = self.reg(bb);
                let r = match op.kind {
                    K::Sxth => {
                        let h = self.b.ins().ireduce(I16, v);
                        self.b.ins().sextend(I32, h)
                    }
                    K::Sxtb => {
                        let h = self.b.ins().ireduce(I8, v);
                        self.b.ins().sextend(I32, h)
                    }
                    K::Uxth => self.b.ins().band_imm_u(v, 0xFFFF),
                    K::Uxtb => self.b.ins().band_imm_u(v, 0xFF),
                    K::Rev => self.b.ins().bswap(v),
                    K::Rev16 => {
                        let lo8 = self.b.ins().band_imm_u(v, 0x00FF_00FF);
                        let lo8 = self.b.ins().ishl_imm_u(lo8, 8);
                        let hi8 = self.b.ins().band_imm_u(v, 0xFF00_FF00u32 as i64);
                        let hi8 = self.b.ins().ushr_imm_u(hi8, 8);
                        self.b.ins().bor(lo8, hi8)
                    }
                    _ => {
                        let h = self.b.ins().ireduce(I16, v);
                        let h = self.b.ins().bswap(h);
                        self.b.ins().sextend(I32, h)
                    }
                };
                self.set_reg(a, r);
            }
            K::Nop => {}
            // ---- through the bus
            K::Ldr => {
                let addr = self.imm(imm);
                self.access(addr, Access::Load(4, a, None), start, ran, pc, imm);
                return true;
            }
            K::LdrR | K::LdrhR | K::LdrbR | K::LdrsbR | K::LdrshR | K::LdrI | K::LdrhI | K::LdrbI => {
                let base = self.reg(bb);
                let addr = match op.kind {
                    K::LdrI | K::LdrhI | K::LdrbI => self.b.ins().iadd_imm_u(base, imm as i64),
                    _ => {
                        let off = self.reg(m);
                        self.b.ins().iadd(base, off)
                    }
                };
                let (size, signed) = match op.kind {
                    K::LdrR | K::LdrI => (4, None),
                    K::LdrhR | K::LdrhI => (2, None),
                    K::LdrbR | K::LdrbI => (1, None),
                    K::LdrsbR => (1, Some(I8)),
                    _ => (2, Some(I16)),
                };
                let guess = self.guess(bb, m, op.kind, imm);
                self.access(addr, Access::Load(size, a, signed), start, ran, pc, guess);
                return true;
            }
            K::StrR | K::StrhR | K::StrbR | K::StrI | K::StrhI | K::StrbI => {
                let base = self.reg(bb);
                let addr = match op.kind {
                    K::StrI | K::StrhI | K::StrbI => self.b.ins().iadd_imm_u(base, imm as i64),
                    _ => {
                        let off = self.reg(m);
                        self.b.ins().iadd(base, off)
                    }
                };
                let size = match op.kind {
                    K::StrR | K::StrI => 4,
                    K::StrhR | K::StrhI => 2,
                    _ => 1,
                };
                let v = self.reg(a);
                let guess = self.guess(bb, m, op.kind, imm);
                self.access(addr, Access::Store(size, v), start, ran, pc, guess);
                return true;
            }
            K::Stm => {
                self.multi(STM, a, imm & 0xFF, start, ran, pc);
                return true;
            }
            K::Ldm => {
                self.multi(LDM, a, imm & 0xFF, start, ran, pc);
                return true;
            }
            K::Push => {
                self.multi(PUSH, 13, imm, start, ran, pc);
                return true;
            }
            K::Pop => {
                self.multi(POP, 13, imm, start, ran, pc);
                return true;
            }
            K::BCond => {
                let taken = self.condition(a);
                let (yes, no) = (self.b.create_block(), self.b.create_block());
                self.b.ins().brif(taken, yes, &[], no, &[]);
                // Taken: to the target (a branch accesses nothing, so the bus has nothing for the checks after it: a
                // bus that serves code answers them for accesses only, see `Bus::code_at`)
                self.enter(yes);
                self.add_cycles_imm(2);
                self.goto(imm, start, ran, starts);
                self.enter(no);
                self.add_cycles_imm(1);
                return true;
            }
            K::B => {
                self.add_cycles_imm(2);
                self.goto(imm, start, ran, starts);
                return false;
            }
            K::Bl => {
                self.set_reg_imm(14, pc.wrapping_add(4) | 1);
                self.add_cycles_imm(3);
                self.goto(imm, start, ran, starts);
                return false;
            }
            K::Bx | K::Blx => {
                let target = self.reg(a);
                if op.kind == K::Blx {
                    self.set_reg_imm(14, pc.wrapping_add(2) | 1);
                }
                let plain = self.plain_branch(target);
                let (here, thunk) = (self.b.create_block(), self.b.create_block());
                self.b.ins().brif(plain, here, &[], thunk, &[]);
                self.enter(here);
                self.interwork(target, 2, start, ran, starts);
                self.enter(thunk);
                self.branch_thunk(BX, target, start, ran, pc);
                return false;
            }
            K::PopPc => {
                // The words straight from the stack where it is direct memory, and the branch here if it is plain
                let count = imm.count_ones();
                let sp = self.reg(13);
                let thunk = self.b.create_block();
                let k = self.window_for(self.regs_now[13], false);
                let at = self.direct(sp, 4 * count, 4, false, k, thunk);
                let regs: Vec<u8> = (0..8).filter(|&i| imm & (1 << i) != 0).collect();
                let vals: Vec<Value> =
                    (0..regs.len()).map(|k| self.b.ins().load(I32, guest(), at, 4 * k as i32)).collect();
                let target = self.b.ins().load(I32, guest(), at, 4 * regs.len() as i32);
                let plain = self.plain_branch(target);
                let here = self.b.create_block();
                self.b.ins().brif(plain, here, &[], thunk, &[]);
                self.enter(here);
                for (&r, v) in regs.iter().zip(vals) {
                    self.set_reg(r, v);
                }
                let sp = self.b.ins().iadd_imm_u(sp, (4 * count) as i64);
                self.set_reg(13, sp);
                self.interwork(target, 3 + count, start, ran, starts);
                self.enter(thunk);
                let list = self.imm(imm);
                self.branch_thunk(POP_PC, list, start, ran, pc);
                return false;
            }
            K::Step => {
                let zero = self.imm(0);
                self.branch_thunk(STEP, zero, start, ran, pc);
                return false;
            }
        }
        // The ops that only touch registers and flags take a cycle
        self.pend += 1;
        true
    }
}
