//! Cranelift code generation for decoded blocks (see the module above).
//!
//! A block compiles to one function `fn(*mut Ctx) -> u32`. Each op becomes the IR for what `exec_op` does, then,
//! where `run_blocks` checks the bus after the op (every op that may touch it), a call of [`super::post`] and the
//! exits `run_blocks` takes. Guest registers and flags are read and written in the core itself, at their field
//! offsets, so the thunks (which run the core's own code) always see them as they are. The batch's cycle count
//! is kept in a variable: a static count per op (as the block's `lead` cycles), plus what the thunks return.

use std::mem::offset_of;
use std::time::{Duration, Instant};

use cranelift_codegen::Context;
use cranelift_codegen::ir::condcodes::IntCC;
use cranelift_codegen::ir::types::{I8, I16, I32, I64};
use cranelift_codegen::ir::{AbiParam, InstBuilder, MemFlagsData, SigRef, Signature, Type, UserFuncName, Value};
use cranelift_codegen::isa::TargetFrontendConfig;
use cranelift_codegen::settings::{self, Configurable};
use cranelift_frontend::{FunctionBuilder, FunctionBuilderContext, Variable};
use cranelift_jit::{JITBuilder, JITModule};
use cranelift_module::{Module, default_libcall_names};

use super::{
    BX, Ctx, EXIT_BATCH, EXIT_FAULT, EXIT_NEXT, FAULT_BIT, JitFn, LDM, POP, POP_PC, POST_ATTENTION, POST_EXIT, PUSH,
    STEP, STM, Thunks,
};
use crate::CortexM0Plus;
use crate::block::{Entry, K, Op};

/// Functions compiled into one module before its memory is freed and a new one started (each is a few hundred bytes;
/// blocks decoded again leave theirs behind).
const MODULE_FUNCTIONS: usize = 20_000;

/// Compiles blocks. One per block cache.
pub(crate) struct Compiler {
    module: Option<JITModule>,
    ctx: Context,
    fctx: FunctionBuilderContext,
    /// Functions compiled into the current module.
    functions: usize,
    /// Blocks compiled, and the time it took.
    pub compiled: u64,
    pub time: Duration,
}

fn module() -> Option<JITModule> {
    let mut flags = settings::builder();
    flags.set("opt_level", "speed").ok()?;
    // As `JITBuilder::with_flags` sets them
    flags.set("use_colocated_libcalls", "false").ok()?;
    flags.set("is_pic", if cfg!(target_arch = "x86_64") { "true" } else { "false" }).ok()?;
    flags.set("enable_verifier", if cfg!(debug_assertions) { "true" } else { "false" }).ok()?;
    // (No host: the core interprets)
    let isa = cranelift_native::builder().ok()?.finish(settings::Flags::new(flags)).ok()?;
    Some(JITModule::new(JITBuilder::with_isa(isa, default_libcall_names())))
}

impl Compiler {
    /// A compiler for this host, if Cranelift supports it.
    pub(crate) fn new() -> Option<Compiler> {
        let module = module()?;
        let ctx = module.make_context();
        Some(Compiler {
            module: Some(module),
            ctx,
            fctx: FunctionBuilderContext::new(),
            functions: 0,
            compiled: 0,
            time: Duration::ZERO,
        })
    }

    /// Whether the module is full: the caller drops every compiled block's code, then calls [`Compiler::reset`].
    pub(crate) fn full(&self) -> bool {
        self.functions >= MODULE_FUNCTIONS
    }

    /// Frees the code compiled so far.
    ///
    /// # Safety
    ///
    /// No function this compiler returned may be called afterwards, and none may be running.
    pub(crate) unsafe fn reset(&mut self) {
        if let Some(m) = self.module.take() {
            // SAFETY: as the function's contract says
            unsafe { m.free_memory() };
        }
        self.module = module();
        self.functions = 0;
    }

    /// Compiles the block in `e` (all of its ops, from the first).
    pub(crate) fn compile(&mut self, e: &Entry) -> Option<JitFn> {
        let start = Instant::now();
        let module = self.module.as_mut()?;
        let ptr = module.target_config().pointer_type();
        let mut sig = module.make_signature();
        sig.params.push(AbiParam::new(ptr));
        sig.returns.push(AbiParam::new(I32));
        let id = module.declare_anonymous_function(&sig).ok()?;
        self.ctx.func.signature = sig;
        self.ctx.func.name = UserFuncName::user(0, id.as_u32());
        let thunk = |params: &[Type]| {
            let mut s = module.make_signature();
            s.params.push(AbiParam::new(ptr));
            s.params.extend(params.iter().map(|&t| AbiParam::new(t)));
            s.returns.push(AbiParam::new(I64));
            s
        };
        let sigs = [
            thunk(&[I32, I32, I64]),
            thunk(&[I32, I32, I32, I64]),
            thunk(&[I32, I32]),
            thunk(&[I32, I32, I32, I64]),
            thunk(&[I32, I32, I32, I64]),
        ];
        let config = module.target_config();
        {
            let b = FunctionBuilder::new(&mut self.ctx.func, &mut self.fctx);
            Emit::block(b, e, ptr, sigs, config);
        }
        let defined = module.define_function(id, &mut self.ctx);
        module.clear_context(&mut self.ctx);
        defined.ok()?;
        module.finalize_definitions().ok()?;
        let code = module.get_finalized_function(id);
        self.functions += 1;
        self.compiled += 1;
        self.time += start.elapsed();
        // SAFETY: `code` is the finalized (executable) code of the function just defined, whose signature is
        // `extern "C" fn(*mut Ctx) -> u32` in the module's default calling convention, the host's C ABI
        // (pointer-sized parameter, 32-bit result).
        Some(unsafe { std::mem::transmute::<*const u8, JitFn>(code) })
    }
}

/// Field offsets the compiled code uses.
const R: i32 = offset_of!(CortexM0Plus, r) as i32;
const N: i32 = offset_of!(CortexM0Plus, n) as i32;
const Z: i32 = offset_of!(CortexM0Plus, z) as i32;
const C: i32 = offset_of!(CortexM0Plus, c) as i32;
const V: i32 = offset_of!(CortexM0Plus, v) as i32;
const CTX_CPU: i32 = offset_of!(Ctx, cpu) as i32;
const CTX_CYCLES: i32 = offset_of!(Ctx, cycles) as i32;
const THUNKS: i32 = offset_of!(Ctx, thunks) as i32;
const READ: i32 = THUNKS + offset_of!(Thunks, read) as i32;
const WRITE: i32 = THUNKS + offset_of!(Thunks, write) as i32;
const POST: i32 = THUNKS + offset_of!(Thunks, post) as i32;
const MULTI: i32 = THUNKS + offset_of!(Thunks, multi) as i32;
const BRANCH: i32 = THUNKS + offset_of!(Thunks, branch) as i32;

/// The thunks' signatures: read, write, post, multi, branch.
const SIG_READ: usize = 0;
const SIG_WRITE: usize = 1;
const SIG_POST: usize = 2;
const SIG_MULTI: usize = 3;
const SIG_BRANCH: usize = 4;

/// Emits one block's function.
struct Emit<'a> {
    b: FunctionBuilder<'a>,
    ptr: Type,
    ctx: Value,
    cpu: Value,
    /// The batch's cycles so far.
    cyc: Variable,
    sigs: [SigRef; 5],
}

/// Memory the compiled code reads and writes: the core's fields and the context, always valid and aligned.
fn mem() -> MemFlagsData {
    MemFlagsData::trusted()
}

impl<'a> Emit<'a> {
    fn block(mut b: FunctionBuilder<'a>, e: &Entry, ptr: Type, sigs: [Signature; 5], config: TargetFrontendConfig) {
        let entry = b.create_block();
        b.append_block_params_for_function_params(entry);
        b.switch_to_block(entry);
        let ctx = b.block_params(entry)[0];
        let cpu = b.ins().load(ptr, mem(), ctx, CTX_CPU);
        let cyc = b.declare_var(I64);
        let c0 = b.ins().load(I64, mem(), ctx, CTX_CYCLES);
        b.def_var(cyc, c0);
        let sigs = sigs.map(|s| b.import_signature(s));
        let mut x = Emit { b, ptr, ctx, cpu, cyc, sigs };
        let (lo, len) = (e.start(), e.len());
        let ops = e.ops();
        let mut open = true;
        for (i, &op) in ops.iter().enumerate() {
            open = x.op(op, i, lo, len);
        }
        if open {
            // Ran to the end of the block
            let last = ops[ops.len() - 1];
            let pc = (!last.kind.sets_pc()).then(|| lo.wrapping_add(last.off as u32 + 2));
            x.exit(EXIT_NEXT, ops.len(), pc);
        }
        let Emit { mut b, .. } = x;
        b.seal_all_blocks();
        b.finalize(config);
    }

    // ---- state

    fn reg(&mut self, r: u8) -> Value {
        self.b.ins().load(I32, mem(), self.cpu, R + 4 * (r & 15) as i32)
    }

    fn set_reg(&mut self, r: u8, v: Value) {
        self.b.ins().store(mem(), v, self.cpu, R + 4 * (r & 15) as i32);
    }

    fn set_reg_imm(&mut self, r: u8, v: u32) {
        let v = self.imm(v);
        self.set_reg(r, v);
    }

    fn flag(&mut self, off: i32) -> Value {
        self.b.ins().load(I8, mem(), self.cpu, off)
    }

    fn set_flag(&mut self, off: i32, v: Value) {
        self.b.ins().store(mem(), v, self.cpu, off);
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

    fn add_cycles_imm(&mut self, c: u32) {
        let now = self.b.use_var(self.cyc);
        let sum = self.b.ins().iadd_imm_u(now, c as i64);
        self.b.def_var(self.cyc, sum);
    }

    /// N and Z from a result.
    fn nz(&mut self, r: Value) {
        let n = self.b.ins().ushr_imm_u(r, 31);
        let n = self.b.ins().ireduce(I8, n);
        let z = self.b.ins().icmp_imm_u(IntCC::Equal, r, 0);
        self.set_flag(N, n);
        self.set_flag(Z, z);
    }

    /// `AddWithCarry` with N, Z, C and V set.
    fn adc(&mut self, x: Value, y: Value, carry: Value) -> Value {
        let b = &mut self.b;
        let (x64, y64, c64) = (b.ins().uextend(I64, x), b.ins().uextend(I64, y), b.ins().uextend(I64, carry));
        let s = b.ins().iadd(x64, y64);
        let s = b.ins().iadd(s, c64);
        let r = b.ins().ireduce(I32, s);
        let c = b.ins().ushr_imm_u(s, 32);
        let c = b.ins().ireduce(I8, c);
        let (xr, yr) = (b.ins().bxor(x, r), b.ins().bxor(y, r));
        let v = b.ins().band(xr, yr);
        let v = b.ins().ushr_imm_u(v, 31);
        let v = b.ins().ireduce(I8, v);
        self.nz(r);
        self.set_flag(C, c);
        self.set_flag(V, v);
        r
    }

    fn adc_const(&mut self, x: Value, y: Value, carry: bool) -> Value {
        let c = self.b.ins().iconst(I8, carry as i64);
        self.adc(x, y, c)
    }

    /// The bit 0 of `v` as a flag.
    fn bit0(&mut self, v: Value) -> Value {
        let v = self.b.ins().band_imm_u(v, 1);
        self.b.ins().ireduce(I8, v)
    }

    /// Whether condition `cond` (0-13) passes, as 0 or 1.
    fn condition(&mut self, cond: u8) -> Value {
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

    /// Returns `kind` with `ran` ops run, the PC set to `pc` if given, and the cycles written back.
    fn exit(&mut self, kind: u32, ran: usize, pc: Option<u32>) {
        let cyc = self.b.use_var(self.cyc);
        self.b.ins().store(mem(), cyc, self.ctx, CTX_CYCLES);
        if let Some(pc) = pc {
            self.set_reg_imm(15, pc);
        }
        let r = self.b.ins().iconst(I32, (kind << 8 | ran as u32) as i64);
        self.b.ins().return_(&[r]);
    }

    /// Calls a thunk: `(ctx, args...) -> u64`.
    fn call(&mut self, slot: i32, sig: usize, args: &[Value]) -> Value {
        let f = self.b.ins().load(self.ptr, mem(), self.ctx, slot);
        let mut all = vec![self.ctx];
        all.extend_from_slice(args);
        let call = self.b.ins().call_indirect(self.sigs[sig], f, &all);
        self.b.inst_results(call)[0]
    }

    /// After a thunk that may fault: the op is the `ran`th; returns there with the fault, else goes on.
    fn check_fault(&mut self, res: Value, ran: usize) {
        let (fault, ok) = (self.b.create_block(), self.b.create_block());
        let f = self.b.ins().band_imm_u(res, FAULT_BIT as i64);
        self.b.ins().brif(f, fault, &[], ok, &[]);
        self.b.switch_to_block(fault);
        // (The cycles are those before the op; the batch puts the PC at it)
        self.exit(EXIT_FAULT, ran, None);
        self.b.switch_to_block(ok);
    }

    /// The checks `run_blocks` makes after an op that may touch the bus (`c` its cycles): the stall cycles count, and
    /// the block ends (the `ran`th op having run, the PC set to `pc` if the op did not set it) for the machine's
    /// attention, a slow access, `leave` or a stall.
    fn post(&mut self, c: Value, leave: Value, ran: usize, pc: Option<u32>) {
        let r = self.call(POST, SIG_POST, &[c, leave]);
        let stall = self.b.ins().band_imm_u(r, 0xFFFF_FFFF);
        let now = self.b.use_var(self.cyc);
        let sum = self.b.ins().iadd(now, stall);
        self.b.def_var(self.cyc, sum);
        let (out, on) = (self.b.create_block(), self.b.create_block());
        let e = self.b.ins().band_imm_u(r, POST_EXIT as i64);
        self.b.ins().brif(e, out, &[], on, &[]);
        self.b.switch_to_block(out);
        let (batch, next) = (self.b.create_block(), self.b.create_block());
        let a = self.b.ins().band_imm_u(r, POST_ATTENTION as i64);
        self.b.ins().brif(a, batch, &[], next, &[]);
        self.b.switch_to_block(batch);
        self.exit(EXIT_BATCH, ran, pc);
        self.b.switch_to_block(next);
        self.exit(EXIT_NEXT, ran, pc);
        self.b.switch_to_block(on);
    }

    fn post_imm(&mut self, c: u32, leave: Value, ran: usize, pc: Option<u32>) {
        let c = self.imm(c);
        self.post(c, leave, ran, pc);
    }

    /// Whether a store at `addr` may have reached the block's code (`len` bytes from `lo`; `margin` before it), as
    /// 0 or 1 (`exec_op`'s `code`).
    fn code_check(&mut self, addr: Value, lo: u32, len: u32, margin: u32) -> Value {
        let d = self.b.ins().iadd_imm_u(addr, lo.wrapping_sub(margin).wrapping_neg() as i64);
        let hit = self.b.ins().icmp_imm_u(IntCC::UnsignedLessThan, d, (len + margin) as i64);
        self.b.ins().uextend(I32, hit)
    }

    /// A load of `size` bytes from `addr` into `rt`, sign extended from `signed` bits.
    fn load(&mut self, rt: u8, addr: Value, size: u32, signed: Option<Type>, ran: usize, pc: u32) {
        let (sz, elapsed) = (self.imm(size), self.b.use_var(self.cyc));
        let r = self.call(READ, SIG_READ, &[addr, sz, elapsed]);
        self.check_fault(r, ran);
        let mut v = self.b.ins().ireduce(I32, r);
        if let Some(t) = signed {
            let narrow = self.b.ins().ireduce(t, v);
            v = self.b.ins().sextend(I32, narrow);
        }
        self.set_reg(rt, v);
        self.add_cycles_imm(2);
        let leave = self.imm(0);
        self.post_imm(2, leave, ran, Some(pc + 2));
    }

    fn store(&mut self, rt: u8, addr: Value, size: u32, ran: usize, (pc, lo, len): (u32, u32, u32)) {
        let v = self.reg(rt);
        let (sz, elapsed) = (self.imm(size), self.b.use_var(self.cyc));
        let r = self.call(WRITE, SIG_WRITE, &[addr, sz, v, elapsed]);
        self.check_fault(r, ran);
        let leave = self.code_check(addr, lo, len, 4);
        self.add_cycles_imm(2);
        self.post_imm(2, leave, ran, Some(pc + 2));
    }

    /// A thunk returning cycles (or a fault): checks it, counts them and makes the checks after the op.
    fn counted(&mut self, r: Value, leave: Option<Value>, ran: usize, pc: Option<u32>) {
        self.check_fault(r, ran);
        let c = self.b.ins().ireduce(I32, r);
        self.add_cycles(c);
        let leave = match leave {
            Some(l) => l,
            None => self.imm(0),
        };
        self.post(c, leave, ran, pc);
    }

    /// Emits op `i` (at `lo + op.off`). Returns false if every path out of it has returned.
    fn op(&mut self, op: Op, i: usize, lo: u32, len: u32) -> bool {
        let (a, bb, m, imm) = (op.a & 15, op.b & 15, (op.imm & 15) as u8, op.imm);
        let pc = lo.wrapping_add(op.off as u32);
        let ran = i + 1;
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
                let r = if op.kind == K::AddR {
                    self.adc_const(x, y, false)
                } else {
                    let ny = self.b.ins().bnot(y);
                    self.adc_const(x, ny, true)
                };
                self.set_reg(a, r);
            }
            K::AddI3 | K::SubI3 | K::AddI8 | K::SubI8 | K::CmpI => {
                let x = if matches!(op.kind, K::AddI3 | K::SubI3) { self.reg(bb) } else { self.reg(a) };
                let r = if matches!(op.kind, K::AddI3 | K::AddI8) {
                    let y = self.imm(imm);
                    self.adc_const(x, y, false)
                } else {
                    let y = self.imm(!imm);
                    self.adc_const(x, y, true)
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
                let ny = self.b.ins().bnot(y);
                self.adc_const(x, ny, true);
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
                self.load(a, addr, 4, None, ran, pc);
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
                self.load(a, addr, size, signed, ran, pc);
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
                self.store(a, addr, size, ran, (pc, lo, len));
                return true;
            }
            K::Stm | K::Ldm | K::Push | K::Pop => {
                let base = (op.kind == K::Stm).then(|| self.reg(a));
                let kind = match op.kind {
                    K::Stm => STM,
                    K::Ldm => LDM,
                    K::Push => PUSH,
                    _ => POP,
                };
                let (k, rn, list, elapsed) =
                    (self.imm(kind), self.imm(a as u32), self.imm(imm), self.b.use_var(self.cyc));
                let r = self.call(MULTI, SIG_MULTI, &[k, rn, list, elapsed]);
                // Stores may reach any part of the block within the words stored: from the base (STM) or the SP
                // pushed to
                let leave = match op.kind {
                    K::Stm => Some(self.code_check(base.unwrap(), lo, len, 64)),
                    K::Push => {
                        let sp = self.reg(13);
                        Some(self.code_check(sp, lo, len, 64))
                    }
                    _ => None,
                };
                self.counted(r, leave, ran, Some(pc + 2));
                return true;
            }
            K::BCond => {
                let taken = self.condition(a);
                let (yes, no) = (self.b.create_block(), self.b.create_block());
                self.b.ins().brif(taken, yes, &[], no, &[]);
                // Taken: the block ends (a branch accesses nothing, so the bus has nothing for the checks after it:
                // a bus that serves code answers them for accesses only, see `Bus::code_at`)
                self.b.switch_to_block(yes);
                self.add_cycles_imm(2);
                self.exit(EXIT_NEXT, ran, Some(imm));
                self.b.switch_to_block(no);
                self.add_cycles_imm(1);
                self.set_reg_imm(15, pc.wrapping_add(2));
                return true;
            }
            K::B => {
                self.add_cycles_imm(2);
                self.exit(EXIT_NEXT, ran, Some(imm));
                return false;
            }
            K::Bl => {
                self.set_reg_imm(14, pc.wrapping_add(4) | 1);
                self.add_cycles_imm(3);
                self.exit(EXIT_NEXT, ran, Some(imm));
                return false;
            }
            K::Bx | K::Blx | K::PopPc | K::Step => {
                let (kind, arg) = match op.kind {
                    K::Bx | K::Blx => {
                        let target = self.reg(a);
                        if op.kind == K::Blx {
                            self.set_reg_imm(14, pc.wrapping_add(2) | 1);
                        }
                        (BX, target)
                    }
                    K::PopPc => (POP_PC, self.imm(imm)),
                    _ => (STEP, self.imm(0)),
                };
                let (k, at, elapsed) = (self.imm(kind), self.imm(pc), self.b.use_var(self.cyc));
                let r = self.call(BRANCH, SIG_BRANCH, &[k, arg, at, elapsed]);
                self.counted(r, None, ran, None);
                // The last op: the block's end follows
                return true;
            }
        }
        // The ops that only touch registers and flags take a cycle
        self.add_cycles_imm(1);
        true
    }
}
