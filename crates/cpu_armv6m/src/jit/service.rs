//! The process's JIT compiler, shared by every core.
//!
//! Compiled regions live in one Cranelift module that is never freed, so any core may keep a pointer to one for as
//! long as it likes: copies of a core (snapshots, rolled-back boards of a robot running ahead) and other boards
//! running the same firmware find a region compiled once ([`Service::find`]) rather than compile it again. A region
//! is known by its first block (address and code) and the shape of the bus's direct memory windows it was compiled
//! for; whatever else it runs it checks before running it.
//!
//! Regions are compiled on worker threads (up to half the host's cores, at most [`WORKERS`], each with a compiler
//! and a module of its own), while the core that asked goes on interpreting: compiled code does exactly what
//! interpreting does, so when it starts being used changes nothing. A worker compiles the requests waiting for it
//! together (up to [`BATCH`]), and makes them executable at once, so that they share pages of memory. Tests compile
//! on the asking thread instead (`CortexM0Plus::jit_sync`), to run compiled code from the first moment it may run.
//!
//! Compiled code may take so much memory in the process (`codegen::CODE_MEMORY`); once it has, nothing more is
//! compiled, the code not compiled yet runs interpreted (exactly the same, more slowly), a warning says so once and
//! [`JitStats::refused`] counts the regions refused.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock, mpsc};
use std::time::{Duration, Instant};

use sim_core::bus::{DIRECT_REGIONS, DirectMemory};

use super::{Compiler, Defined, Desc, Region, RegionBlock, Trampoline};

/// What compiling depends on of a direct memory window: where it is, whether it can be written, whether it has a
/// mask other than none, and marks.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, Default)]
pub(crate) struct Shape {
    pub base: u32,
    pub size: u32,
    pub writable: bool,
    pub masked: bool,
    pub read_mark: bool,
    pub write_mark: bool,
}

/// The shapes of a bus's windows.
pub(crate) fn shapes(m: &DirectMemory) -> [Shape; DIRECT_REGIONS] {
    m.regions.map(|w| Shape {
        base: w.base,
        size: w.size,
        writable: w.size != 0 && w.writable != 0,
        masked: w.mask != u32::MAX,
        read_mark: !w.read_mark.is_null(),
        write_mark: !w.write_mark.is_null(),
    })
}

/// What a compiled region is known by.
#[derive(Clone, PartialEq, Eq, Hash)]
pub(crate) struct Key {
    pub pc: u32,
    pub len: u8,
    pub code: [u8; 32],
    pub shapes: [Shape; DIRECT_REGIONS],
}

/// A region being compiled, or compiled: the function once there is one.
pub(crate) struct Ticket(AtomicUsize);

const PENDING: usize = 0;
const FAILED: usize = 1;

/// What a ticket says.
pub(crate) enum State {
    Pending,
    Failed,
    Ready(Region),
}

impl Ticket {
    pub(crate) fn state(&self) -> State {
        match self.0.load(Ordering::Acquire) {
            PENDING => State::Pending,
            FAILED => State::Failed,
            // SAFETY: a value other than these was stored by `set` from a `Region`, which lives for ever
            d => State::Ready(unsafe { &*(d as *const Desc) }),
        }
    }

    fn set(&self, d: Option<Region>) {
        self.0.store(d.map_or(FAILED, |d| d as *const Desc as usize), Ordering::Release);
    }
}

struct Request {
    /// The window the first block's code is in, where its code can be read whole (`u32::MAX`: not).
    window: u32,
    blocks: Vec<RegionBlock>,
    shapes: [Shape; DIRECT_REGIONS],
    regs: [u32; 16],
    ticket: Arc<Ticket>,
}

/// Most worker threads compiling.
const WORKERS: usize = 3;
/// Most requests a worker compiles together.
const BATCH: usize = 64;

/// What the process's JIT has done ([`crate::CortexM0Plus::jit_stats`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct JitStats {
    /// Regions compiled, and the decoded blocks in them.
    pub regions: u64,
    pub blocks: u64,
    /// Time spent compiling (on every thread together).
    pub compile_time: Duration,
    /// Regions not compiled because compiled code had taken all the memory it may (they run interpreted).
    pub refused: u64,
    /// The memory compiled code has taken (whole pages: regions compiled together share them).
    pub code_bytes: u64,
}

pub(crate) struct Service {
    /// The compiler requests made `sync` use (`None` where Cranelift has no code generator for the host).
    compiler: Mutex<Option<Compiler>>,
    trampoline: Option<Trampoline>,
    tickets: Mutex<HashMap<Key, Arc<Ticket>>>,
    /// Where requests for the workers go (`None` until the first).
    workers: Mutex<Option<mpsc::Sender<Request>>>,
    regions: AtomicU64,
    blocks: AtomicU64,
    nanos: AtomicU64,
    refused: AtomicU64,
    warned: AtomicBool,
}

/// The process's service.
pub(crate) fn service() -> &'static Service {
    static SERVICE: OnceLock<Service> = OnceLock::new();
    SERVICE.get_or_init(|| {
        let compiler = Compiler::new();
        let trampoline = compiler.as_ref().map(|c| c.trampoline());
        Service {
            compiler: Mutex::new(compiler),
            trampoline,
            tickets: Mutex::new(HashMap::new()),
            workers: Mutex::new(None),
            regions: AtomicU64::new(0),
            blocks: AtomicU64::new(0),
            nanos: AtomicU64::new(0),
            refused: AtomicU64::new(0),
            warned: AtomicBool::new(false),
        }
    })
}

impl Service {
    /// How Rust enters compiled code (`None`: there is none).
    pub(crate) fn trampoline(&self) -> Option<Trampoline> {
        self.trampoline
    }

    /// The ticket for the region `key` names, if a core has asked for it.
    pub(crate) fn find(&self, key: &Key) -> Option<Arc<Ticket>> {
        self.tickets.lock().unwrap_or_else(|e| e.into_inner()).get(key).cloned()
    }

    /// Asks for the region `blocks` compiled (or finds it asked for already): now, on this thread, with `sync`.
    pub(crate) fn request(
        &self,
        key: Key,
        window: u32,
        blocks: Vec<RegionBlock>,
        shapes: [Shape; DIRECT_REGIONS],
        regs: [u32; 16],
        sync: bool,
    ) -> Arc<Ticket> {
        let ticket = {
            let mut tickets = self.tickets.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(t) = tickets.get(&key) {
                return t.clone();
            }
            let t = Arc::new(Ticket(AtomicUsize::new(PENDING)));
            tickets.insert(key, t.clone());
            t
        };
        let request = Request { window, blocks, shapes, regs, ticket: ticket.clone() };
        if sync {
            let mut compiler = self.compiler.lock().unwrap_or_else(|e| e.into_inner());
            self.compile_with(compiler.as_mut(), vec![request]);
        } else {
            let mut workers = self.workers.lock().unwrap_or_else(|e| e.into_inner());
            let tx = workers.get_or_insert_with(|| {
                let (tx, rx) = mpsc::channel::<Request>();
                let rx = Arc::new(Mutex::new(rx));
                let n = std::thread::available_parallelism().map_or(1, |n| (n.get() / 2).clamp(1, WORKERS));
                let mut spawned = 0;
                for k in 0..n {
                    let rx = rx.clone();
                    let worker = move || {
                        let mut compiler = Compiler::new();
                        loop {
                            // The next request, and those waiting behind it
                            let batch = {
                                let rx = rx.lock().unwrap_or_else(|e| e.into_inner());
                                let Ok(r) = rx.recv() else { return };
                                let mut batch = vec![r];
                                while batch.len() < BATCH
                                    && let Ok(r) = rx.try_recv()
                                {
                                    batch.push(r);
                                }
                                batch
                            };
                            service().compile_with(compiler.as_mut(), batch);
                        }
                    };
                    spawned += std::thread::Builder::new().name(format!("jit{k}")).spawn(worker).is_ok() as usize;
                }
                if spawned == 0 {
                    log::warn!("no thread for the JIT: blocks run interpreted");
                }
                tx
            });
            if let Err(mpsc::SendError(r)) = tx.send(request) {
                r.ticket.set(None);
            }
        }
        ticket
    }

    /// Compiles `batch`, made executable together, and settles each request's ticket.
    fn compile_with(&self, compiler: Option<&mut Compiler>, batch: Vec<Request>) {
        let start = Instant::now();
        let mut code = vec![None; batch.len()];
        if let Some(c) = compiler {
            let mut defined: Vec<(usize, Defined)> = Vec::new();
            for (k, r) in batch.iter().enumerate() {
                if c.full() {
                    self.refuse();
                    continue;
                }
                let define = || c.define(&r.blocks, &r.shapes, &r.regs);
                if let Ok(Some(d)) = std::panic::catch_unwind(std::panic::AssertUnwindSafe(define)) {
                    defined.push((k, d));
                }
            }
            if !defined.is_empty() {
                let (at, defined): (Vec<usize>, Vec<Defined>) = defined.into_iter().unzip();
                let finished = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| c.finish(&defined)));
                if let Ok(Some(f)) = finished {
                    for (k, f) in at.into_iter().zip(f) {
                        code[k] = Some(f);
                    }
                }
            }
        }
        for (r, code) in batch.iter().zip(code) {
            // The region as code going on to it finds it, made once and kept for ever (as its code is)
            let d = code.map(|code| {
                let e = &r.blocks[0].entry;
                let (words, masks) = e.code_words();
                let desc = Desc {
                    code,
                    pc: e.start(),
                    ops: e.ops().len() as u32,
                    lead: e.lead_cycles(e.ops().len() - 1) as u32,
                    window: r.window,
                    words,
                    masks,
                };
                &*Box::leak(Box::new(desc))
            });
            r.ticket.set(d);
            if d.is_some() {
                self.regions.fetch_add(1, Ordering::Relaxed);
                self.blocks.fetch_add(r.blocks.len() as u64, Ordering::Relaxed);
            }
        }
        self.nanos.fetch_add(start.elapsed().as_nanos() as u64, Ordering::Relaxed);
    }

    /// A region not compiled for want of memory: counted, and said once.
    fn refuse(&self) {
        self.refused.fetch_add(1, Ordering::Relaxed);
        if !self.warned.swap(true, Ordering::Relaxed) {
            log::warn!(
                "the JIT has compiled all the code it may keep ({} regions): code not compiled yet runs interpreted \
                 from now on (the same, more slowly)",
                self.regions.load(Ordering::Relaxed)
            );
        }
    }

    /// What the process's JIT has done so far.
    pub(crate) fn stats(&self) -> JitStats {
        JitStats {
            regions: self.regions.load(Ordering::Relaxed),
            blocks: self.blocks.load(Ordering::Relaxed),
            compile_time: Duration::from_nanos(self.nanos.load(Ordering::Relaxed)),
            refused: self.refused.load(Ordering::Relaxed),
            code_bytes: super::code_taken() as u64,
        }
    }
}
