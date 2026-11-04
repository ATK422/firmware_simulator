//! A sampling profiler for the firmware on each core.
//!
//! Every [`Profiler::interval`] of virtual time (jittered by ±50 % so periodic firmware activity does not alias
//! with the samples), the machine is stopped between instructions and each core's stack is walked
//! ([`crate::unwind`]): from the PC through the callers recorded in the ELF's call frame information, through
//! exception frames into the interrupted code. A core waiting in `WFI`/`WFE` counts as sleeping, with the stack it
//! sleeps in. Nothing runs per instruction: the profiler only cuts the run into pieces at the sample times, which
//! moves the cores' quantum boundaries (so dual-core interleaving, not program behaviour, can differ from an
//! unprofiled run).
//!
//! The result is a flat table per core (self and total time per function, sleeping shown separately, time per
//! exception handler) and folded stacks (`core0;main;foo;bar 42`) for flamegraph tools (`flamegraph.pl`,
//! `inferno-flamegraph`, speedscope).

use std::collections::{BTreeMap, HashMap};
use std::fmt::Write as _;

use rp2040::{Machine, StopReason};
use sim_core::bus::Size;
use sim_core::cpu::Cpu;
use sim_core::image::Image;
use sim_core::time::Time;

use crate::demangle::demangle;
use crate::unwind::{self, Memory, Registers, UnwindTable};

/// Default time between samples.
pub const DEFAULT_INTERVAL: Time = Time::from_us(10);

/// Pseudo-function at the top of a sleeping core's stack.
const SLEEP: u32 = u32::MAX;
/// Pseudo-function at the top of a stack the walk could not finish.
const TRUNCATED: u32 = u32::MAX - 1;

/// Names code addresses: the image's function symbols (demangled), and the bootrom's public functions by their
/// function and soft float/double table entries.
#[derive(Clone, Debug, Default)]
pub struct Symbolizer {
    /// Function symbols: (start, end, name), sorted.
    functions: Vec<(u32, u32, String)>,
    /// Bootrom entry points (start, name), sorted.
    rom: Vec<(u32, String)>,
}

/// ROM function table codes (RP2040 datasheet 2.8.3.1).
const ROM_FUNCTIONS: [(&[u8; 2], &str); 18] = [
    (b"P3", "popcount32"),
    (b"R3", "reverse32"),
    (b"L3", "clz32"),
    (b"T3", "ctz32"),
    (b"MS", "memset"),
    (b"S4", "memset4"),
    (b"MC", "memcpy"),
    (b"C4", "memcpy44"),
    (b"IF", "connect_internal_flash"),
    (b"EX", "flash_exit_xip"),
    (b"RE", "flash_range_erase"),
    (b"RP", "flash_range_program"),
    (b"FC", "flash_flush_cache"),
    (b"CX", "flash_enter_cmd_xip"),
    (b"UB", "reset_usb_boot"),
    (b"DT", "debug_trampoline"),
    (b"DE", "debug_trampoline_end"),
    (b"WV", "wait_for_vector"),
];

/// The soft float table (`SF_TABLE_*` in pico-sdk's `sf_table.h`), one entry per word. The soft double table has
/// the same layout with doubles.
const SF_TABLE: [&str; 32] = [
    "fadd",
    "fsub",
    "fmul",
    "fdiv",
    "fcmp_fast",
    "fcmp_fast_flags",
    "fsqrt",
    "float2int",
    "float2fix",
    "float2uint",
    "float2ufix",
    "int2float",
    "fix2float",
    "uint2float",
    "ufix2float",
    "fcos",
    "fsin",
    "ftan",
    "fsincos",
    "fexp",
    "fln",
    "fcmp_basic",
    "fatan2",
    "int642float",
    "fix642float",
    "uint642float",
    "ufix642float",
    "float2int64",
    "float2fix64",
    "float2uint64",
    "float2ufix64",
    "float2double",
];

fn double_name(f: &str) -> String {
    if f == "float2double" {
        return "double2float".into();
    }
    let f = f.replace("float", "double");
    match f.strip_prefix('f') {
        Some(rest) => format!("d{rest}"),
        None => f,
    }
}

impl Symbolizer {
    /// From an image's symbol table and the bootrom mapped at address 0 (empty if none).
    pub fn new(image: &Image, rom: &[u8]) -> Symbolizer {
        // Assembly labels often have no size: they run up to the next symbol (at most 4 KiB)
        let mut starts: Vec<u32> = image.symbols.iter().filter(|s| s.is_function).map(|s| s.addr & !1).collect();
        starts.sort_unstable();
        starts.dedup();
        let mut functions: Vec<(u32, u32, String)> = image
            .symbols
            .iter()
            .filter(|s| s.is_function)
            .map(|s| {
                let start = s.addr & !1;
                let end = if s.size > 0 {
                    start + s.size
                } else {
                    let i = starts.partition_point(|&a| a <= start);
                    starts.get(i).copied().filter(|&n| n - start <= 0x1000).unwrap_or(start + 2)
                };
                (start, end, demangle(&s.name))
            })
            .collect();
        functions.sort_by_key(|f| (f.0, std::cmp::Reverse(f.1)));
        functions.dedup_by_key(|f| f.0);
        Symbolizer { functions, rom: rom_entries(rom) }
    }

    /// The function an address belongs to, as its start address (the address itself if no function covers it).
    pub fn function(&self, addr: u32) -> u32 {
        let addr = addr & !1;
        let i = self.functions.partition_point(|f| f.0 <= addr);
        if let Some(f) = self.functions[..i].iter().rev().take(8).find(|f| addr < f.1) {
            return f.0;
        }
        if (addr as usize) < rp2040::memmap::ROM_SIZE {
            let i = self.rom.partition_point(|f| f.0 <= addr);
            if let Some(f) = self.rom[..i].last() {
                return f.0;
            }
        }
        addr
    }

    /// Whether a function (or bootrom entry point) starts at `addr`.
    pub fn is_function_start(&self, addr: u32) -> bool {
        self.functions.binary_search_by_key(&addr, |f| f.0).is_ok()
            || self.rom.binary_search_by_key(&addr, |f| f.0).is_ok()
    }

    /// The name of a function returned by [`Symbolizer::function`].
    pub fn name(&self, func: u32) -> String {
        match func {
            SLEEP => return "[sleep]".into(),
            TRUNCATED => return "[unknown]".into(),
            _ => {}
        }
        if let Ok(i) = self.functions.binary_search_by_key(&func, |f| f.0) {
            return self.functions[i].2.clone();
        }
        if (func as usize) < rp2040::memmap::ROM_SIZE {
            if let Ok(i) = self.rom.binary_search_by_key(&func, |f| f.0) {
                return format!("rom:{}", self.rom[i].1);
            }
            return format!("rom:{func:#06x}");
        }
        format!("{func:#010x}")
    }
}

/// The bootrom's public entry points: its function table and the soft float/double tables in its data table.
fn rom_entries(rom: &[u8]) -> Vec<(u32, String)> {
    let half = |a: usize| -> Option<u16> { Some(u16::from_le_bytes([*rom.get(a)?, *rom.get(a + 1)?])) };
    let word = |a: usize| -> Option<u32> { Some(u32::from_le_bytes(rom.get(a..a + 4)?.try_into().ok()?)) };
    let mut out = BTreeMap::new();
    // Function table (0x14) and data table (0x16): (code, halfword pointer) pairs ending in a zero code
    let table = |ptr: usize| -> Vec<([u8; 2], u16)> {
        let mut v = Vec::new();
        let Some(mut p) = half(ptr).map(|p| p as usize) else { return v };
        while let (Some(code), Some(addr)) = (half(p), half(p + 2)) {
            if code == 0 || v.len() > 64 {
                break;
            }
            v.push((code.to_le_bytes(), addr));
            p += 4;
        }
        v
    };
    // The header's lookup function (0x18) is not in its own table
    if let Some(f) = half(0x18).filter(|&f| f != 0 && (f as usize) < rom.len()) {
        out.insert(f as u32 & !1, "rom_table_lookup".to_string());
    }
    for (code, addr) in table(0x14) {
        if let Some((_, name)) = ROM_FUNCTIONS.iter().find(|(c, _)| **c == code) {
            out.insert(addr as u32 & !1, name.to_string());
        }
    }
    for (code, addr) in table(0x16) {
        let double = match &code {
            b"SF" => false,
            b"SD" => true,
            _ => continue,
        };
        for (k, f) in SF_TABLE.iter().enumerate() {
            let Some(entry) = word(addr as usize + 4 * k) else { break };
            let a = entry & !1;
            if entry & 1 == 0 || a as usize >= rom.len() || a == 0 {
                continue;
            }
            let name = if double { double_name(f) } else { f.to_string() };
            out.entry(a).or_insert(name);
        }
    }
    out.into_iter().collect()
}

/// The target as the unwinder sees it.
struct MachineMemory<'a> {
    m: &'a Machine,
    symbols: &'a Symbolizer,
}

impl Memory for MachineMemory<'_> {
    fn word(&self, addr: u32) -> Option<u32> {
        if addr & 3 != 0 || !is_memory(addr) {
            return None;
        }
        self.m.sys.peek(addr, Size::Word)
    }
    fn half(&self, addr: u32) -> Option<u16> {
        if addr & 1 != 0 || !is_memory(addr) {
            return None;
        }
        self.m.sys.peek(addr, Size::Half).map(|v| v as u16)
    }
    fn is_code(&self, addr: u32) -> bool {
        let flash = self.m.sys.flash.len() as u32;
        use rp2040::memmap::*;
        (addr as usize) < ROM_SIZE
            || (XIP_BASE..XIP_BASE + flash).contains(&addr)
            || (XIP_SRAM_BASE..XIP_SRAM_BASE + XIP_SRAM_SIZE as u32).contains(&addr)
            || (SRAM_BASE..SRAM_END).contains(&addr)
    }
    fn function_start(&self, addr: u32) -> Option<u32> {
        let f = self.symbols.function(addr);
        (f != addr & !1 || self.symbols.is_function_start(f)).then_some(f)
    }
}

/// Only memories are read (never peripheral registers, whose reads could have side effects).
fn is_memory(addr: u32) -> bool {
    use rp2040::memmap::*;
    (addr as usize) < ROM_SIZE
        || (XIP_BASE..XIP_CTRL_BASE).contains(&addr)
        || (XIP_SRAM_BASE..XIP_SRAM_BASE + XIP_SRAM_SIZE as u32).contains(&addr)
        || (SRAM_BASE..SRAM_END).contains(&addr)
}

/// Samples of one core.
#[derive(Clone, Debug, Default)]
struct CoreSamples {
    /// Stacks as functions, outermost first, with their sample counts.
    stacks: HashMap<Vec<u32>, u64>,
    running: u64,
    sleeping: u64,
    locked_up: u64,
    /// Samples while the core was held off (core 1 before launch without a bootrom, during a reset).
    off: u64,
    truncated: u64,
    /// Running samples by active exception number (0 = thread mode).
    exceptions: BTreeMap<u32, u64>,
}

/// The sampling profiler of one board.
#[derive(Clone, Debug)]
pub struct Profiler {
    /// Mean time between samples.
    pub interval: Time,
    next: Time,
    rng: u64,
    table: UnwindTable,
    pub symbols: Symbolizer,
    cores: [CoreSamples; 2],
    first: Option<Time>,
    last: Time,
    /// Samples taken but not counted yet ([`Profiler::hold`]), by core.
    held: Option<Vec<(usize, Taken)>>,
}

/// One core's sample.
#[derive(Clone, Debug)]
enum Taken {
    Off,
    LockedUp,
    Stack { funcs: Vec<u32>, truncated: bool, sleeping: bool, exception: u32 },
}

/// Where a profiler was ([`Profiler::mark`]), to go back to ([`Profiler::back_to`]).
#[derive(Clone, Copy, Debug)]
pub(crate) struct ProfileMark {
    next: Time,
    rng: u64,
    first: Option<Time>,
    last: Time,
    held: usize,
}

impl Profiler {
    /// A profiler for the image loaded in `m`, sampling every `interval` (on average) from now.
    pub fn new(m: &Machine, interval: Time) -> Profiler {
        let image = m.image.as_deref().cloned().unwrap_or_default();
        Profiler::for_image(&image, &m.sys.rom, interval, m.now)
    }

    /// A profiler for `image` with the bootrom `rom`, first sample at about `start + interval`.
    pub fn for_image(image: &Image, rom: &[u8], interval: Time, start: Time) -> Profiler {
        let table = image.elf.as_deref().map(UnwindTable::from_elf).unwrap_or_default();
        if table.is_empty() {
            log::warn!("profile: the image has no .debug_frame; stacks are only the PC and LR");
        }
        let interval = interval.max(Time::from_ns(100));
        let mut p = Profiler {
            interval,
            next: start,
            rng: 0x2545_F491_4F6C_DD1D,
            table,
            symbols: Symbolizer::new(image, rom),
            cores: Default::default(),
            first: None,
            last: start,
            held: None,
        };
        p.next = start + p.step();
        p
    }

    /// The next gap between samples: uniform in [0.5, 1.5) intervals.
    fn step(&mut self) -> Time {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        let u = (self.rng >> 11) as f64 / (1u64 << 53) as f64;
        Time(((self.interval.as_ps() as f64) * (0.5 + u)) as u64).max(Time(1))
    }

    /// Runs `m` until `end` like [`Machine::run_until`], taking the samples that fall in between.
    pub fn run_until(&mut self, m: &mut Machine, end: Time) -> StopReason {
        loop {
            if self.next <= m.now {
                // Time jumped (a power-on later, a brownout): start sampling again from here
                self.next = m.now + self.step();
            }
            if self.next > end {
                return m.run_until(end);
            }
            let reason = m.run_until(self.next);
            if reason != StopReason::TimeLimit {
                return reason;
            }
            self.sample(m);
            let step = self.step();
            self.next += step;
        }
    }

    /// Takes one sample of each core of `m` now.
    pub fn sample(&mut self, m: &Machine) {
        self.first.get_or_insert(m.now);
        self.last = m.now;
        let mem = MachineMemory { m, symbols: &self.symbols };
        let taken: [Taken; 2] = std::array::from_fn(|c| {
            let cpu = &m.cores[c];
            if !m.core_enabled[c] {
                Taken::Off
            } else if cpu.is_locked_up() {
                Taken::LockedUp
            } else {
                let regs = Registers { r: std::array::from_fn(|i| cpu.reg(i)), psp: cpu.psp() };
                let stack = unwind::unwind(&self.table, &mem, &regs);
                let mut funcs: Vec<u32> = Vec::with_capacity(stack.frames.len() + 2);
                if stack.truncated {
                    funcs.push(TRUNCATED);
                }
                funcs.extend(stack.frames.iter().rev().map(|&a| self.symbols.function(a)));
                let sleeping = cpu.is_sleeping();
                if sleeping {
                    funcs.push(SLEEP);
                }
                Taken::Stack { funcs, truncated: stack.truncated, sleeping, exception: cpu.ipsr() }
            }
        });
        for (c, taken) in taken.into_iter().enumerate() {
            match &mut self.held {
                Some(held) => held.push((c, taken)),
                None => self.count(c, taken),
            }
        }
    }

    /// Counts one core's sample.
    fn count(&mut self, c: usize, taken: Taken) {
        let s = &mut self.cores[c];
        match taken {
            Taken::Off => s.off += 1,
            Taken::LockedUp => s.locked_up += 1,
            Taken::Stack { funcs, truncated, sleeping, exception } => {
                s.truncated += truncated as u64;
                if sleeping {
                    s.sleeping += 1;
                } else {
                    s.running += 1;
                    *s.exceptions.entry(exception).or_default() += 1;
                }
                *s.stacks.entry(funcs).or_default() += 1;
            }
        }
    }

    /// Holds the samples it takes from now on until [`Profiler::commit`] counts them, so that a robot running the
    /// board ahead can take back those of a run it rolls back ([`Profiler::back_to`]).
    pub(crate) fn hold(&mut self) {
        self.held.get_or_insert_with(Vec::new);
    }

    /// Counts the samples held so far (they are final), and goes on holding.
    pub(crate) fn commit(&mut self) {
        if let Some(held) = self.held.as_mut().map(std::mem::take) {
            for (c, taken) in held {
                self.count(c, taken);
            }
        }
    }

    /// Counts the samples held, and stops holding them.
    pub(crate) fn release(&mut self) {
        self.commit();
        self.held = None;
    }

    /// Where it is now: when it samples next, and the samples held.
    pub(crate) fn mark(&self) -> ProfileMark {
        let held = self.held.as_ref().map_or(0, Vec::len);
        ProfileMark { next: self.next, rng: self.rng, first: self.first, last: self.last, held }
    }

    /// Back to `mark` (taken while holding, since the last commit): the samples taken since are dropped, and the
    /// next ones are taken at the same times again.
    pub(crate) fn back_to(&mut self, mark: &ProfileMark) {
        (self.next, self.rng, self.first, self.last) = (mark.next, mark.rng, mark.first, mark.last);
        if let Some(held) = &mut self.held {
            held.truncate(mark.held);
        }
    }

    /// Samples taken (per core, including sleeping ones).
    pub fn samples(&self, core: usize) -> u64 {
        let s = &self.cores[core];
        s.running + s.sleeping + s.locked_up + s.off
    }

    /// The results so far.
    pub fn report(&self) -> Profile {
        let cores = self
            .cores
            .iter()
            .map(|s| {
                let mut stacks: Vec<(Vec<String>, u64)> =
                    s.stacks.iter().map(|(k, &n)| (k.iter().map(|&f| self.symbols.name(f)).collect(), n)).collect();
                stacks.sort();
                let mut self_n: HashMap<u32, u64> = HashMap::new();
                let mut total_n: HashMap<u32, u64> = HashMap::new();
                for (k, &n) in &s.stacks {
                    if k.last() == Some(&SLEEP) {
                        continue;
                    }
                    if let Some(&leaf) = k.last() {
                        *self_n.entry(leaf).or_default() += n;
                    }
                    let mut seen: Vec<u32> = k.clone();
                    seen.sort_unstable();
                    seen.dedup();
                    for f in seen {
                        *total_n.entry(f).or_default() += n;
                    }
                }
                let mut functions: Vec<FunctionStat> = total_n
                    .iter()
                    .filter(|(f, _)| **f != TRUNCATED)
                    .map(|(&f, &total)| FunctionStat {
                        name: self.symbols.name(f),
                        address: f,
                        self_samples: self_n.get(&f).copied().unwrap_or(0),
                        total_samples: total,
                    })
                    .collect();
                functions.sort_by(|a, b| {
                    b.self_samples
                        .cmp(&a.self_samples)
                        .then(b.total_samples.cmp(&a.total_samples))
                        .then(a.name.cmp(&b.name))
                });
                CoreProfile {
                    running: s.running,
                    sleeping: s.sleeping,
                    locked_up: s.locked_up,
                    off: s.off,
                    truncated: s.truncated,
                    exceptions: s.exceptions.iter().map(|(&e, &n)| (e, n)).collect(),
                    functions,
                    stacks,
                }
            })
            .collect();
        Profile {
            interval: self.interval,
            span: self.first.map_or(Time::ZERO, |f| self.last - f),
            unwind_info: !self.table.is_empty(),
            cores,
        }
    }
}

/// Time spent in one function.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FunctionStat {
    pub name: String,
    /// Start address (or the address itself when no symbol covers it).
    pub address: u32,
    /// Samples with the function at the top of the stack (running, not sleeping).
    pub self_samples: u64,
    /// Samples with the function anywhere on the stack (running).
    pub total_samples: u64,
}

/// One core's profile.
#[derive(Clone, Debug, Default)]
pub struct CoreProfile {
    pub running: u64,
    pub sleeping: u64,
    pub locked_up: u64,
    pub off: u64,
    /// Samples whose stack walk ended early (the stack is incomplete at its root).
    pub truncated: u64,
    /// Running samples per active exception (0 is thread mode, 15 SysTick, 16 + n IRQ n).
    pub exceptions: Vec<(u32, u64)>,
    /// By self time, most first.
    pub functions: Vec<FunctionStat>,
    /// Folded stacks: function names outermost first (`[sleep]` on top while sleeping), with sample counts.
    pub stacks: Vec<(Vec<String>, u64)>,
}

impl CoreProfile {
    /// Samples while the core was powered and not locked up.
    pub fn awake_or_sleeping(&self) -> u64 {
        self.running + self.sleeping
    }

    /// All samples of this core.
    pub fn samples(&self) -> u64 {
        self.running + self.sleeping + self.locked_up + self.off
    }

    /// The function called `name` (exactly, or as a path suffix `::name`).
    pub fn function(&self, name: &str) -> Option<&FunctionStat> {
        self.functions.iter().find(|f| f.name == name).or_else(|| {
            let suffix = format!("::{name}");
            self.functions.iter().find(|f| f.name.ends_with(&suffix))
        })
    }

    /// Fraction of the running samples spent in `name` itself.
    pub fn self_fraction(&self, name: &str) -> f64 {
        self.function(name).map_or(0.0, |f| f.self_samples as f64 / self.running.max(1) as f64)
    }

    /// Fraction of the running samples with `name` on the stack.
    pub fn total_fraction(&self, name: &str) -> f64 {
        self.function(name).map_or(0.0, |f| f.total_samples as f64 / self.running.max(1) as f64)
    }
}

/// The result of profiling a board.
#[derive(Clone, Debug)]
pub struct Profile {
    pub interval: Time,
    /// Virtual time between the first and the last sample.
    pub span: Time,
    /// Whether the image had call frame information (full stacks) or only PC and LR were used.
    pub unwind_info: bool,
    pub cores: Vec<CoreProfile>,
}

/// The name of exception number `n` on the RP2040.
pub fn exception_name(n: u32) -> String {
    const IRQS: [&str; 26] = [
        "TIMER_IRQ_0",
        "TIMER_IRQ_1",
        "TIMER_IRQ_2",
        "TIMER_IRQ_3",
        "PWM_IRQ_WRAP",
        "USBCTRL_IRQ",
        "XIP_IRQ",
        "PIO0_IRQ_0",
        "PIO0_IRQ_1",
        "PIO1_IRQ_0",
        "PIO1_IRQ_1",
        "DMA_IRQ_0",
        "DMA_IRQ_1",
        "IO_IRQ_BANK0",
        "IO_IRQ_QSPI",
        "SIO_IRQ_PROC0",
        "SIO_IRQ_PROC1",
        "CLOCKS_IRQ",
        "SPI0_IRQ",
        "SPI1_IRQ",
        "UART0_IRQ",
        "UART1_IRQ",
        "ADC_IRQ_FIFO",
        "I2C0_IRQ",
        "I2C1_IRQ",
        "RTC_IRQ",
    ];
    match n {
        0 => "thread mode".into(),
        2 => "NMI".into(),
        3 => "HardFault".into(),
        11 => "SVCall".into(),
        14 => "PendSV".into(),
        15 => "SysTick".into(),
        n if n >= 16 && ((n - 16) as usize) < IRQS.len() => IRQS[(n - 16) as usize].into(),
        n if n >= 16 => format!("IRQ {}", n - 16),
        n => format!("exception {n}"),
    }
}

/// A duration for people: `2.000 s`, `12.500 ms`, `10.0 µs`.
pub fn human(t: Time) -> String {
    let s = t.as_secs_f64();
    if s >= 1.0 {
        format!("{s:.3} s")
    } else if s >= 1e-3 {
        format!("{:.3} ms", s * 1e3)
    } else {
        format!("{:.1} µs", s * 1e6)
    }
}

fn pct(n: u64, of: u64) -> f64 {
    100.0 * n as f64 / of.max(1) as f64
}

impl Profile {
    /// The flat table: per core, the share of running and sleeping time, time per exception handler, and the
    /// functions by self time (the first `limit`).
    pub fn flat_text(&self, title: &str, limit: usize) -> String {
        let mut o = String::new();
        let _ = writeln!(
            o,
            "# {title}: {} of virtual time, a sample every {} (jittered ±50%){}",
            human(self.span),
            human(self.interval),
            if self.unwind_info { "" } else { "; no .debug_frame in the image, so stacks are PC and LR only" }
        );
        for (c, p) in self.cores.iter().enumerate() {
            let n = p.samples();
            let _ = writeln!(o);
            if p.awake_or_sleeping() == 0 {
                let _ = writeln!(o, "## core {c}: not running ({n} samples while held off)");
                continue;
            }
            let _ = writeln!(
                o,
                "## core {c}: {n} samples: running {:.1}%, sleeping (WFI/WFE) {:.1}%{}{}",
                pct(p.running, n),
                pct(p.sleeping, n),
                if p.locked_up > 0 { format!(", locked up {:.1}%", pct(p.locked_up, n)) } else { String::new() },
                if p.off > 0 { format!(", held off {:.1}%", pct(p.off, n)) } else { String::new() },
            );
            if p.truncated > 0 {
                let _ = writeln!(o, "   {:.1}% of the stacks could not be walked to the root", pct(p.truncated, n));
            }
            if p.running == 0 {
                continue;
            }
            let handlers: Vec<String> = p
                .exceptions
                .iter()
                .filter(|(e, _)| *e != 0)
                .map(|(e, k)| format!("{} {:.1}%", exception_name(*e), pct(*k, n)))
                .collect();
            if !handlers.is_empty() {
                let _ = writeln!(o, "   in exception handlers: {}", handlers.join(", "));
            }
            let _ = writeln!(o);
            let _ = writeln!(o, "  self%  total%  of-run%  samples  function");
            for f in p.functions.iter().take(limit) {
                let _ = writeln!(
                    o,
                    "{:7.2} {:7.2} {:8.2} {:8}  {}",
                    pct(f.self_samples, n),
                    pct(f.total_samples, n),
                    pct(f.self_samples, p.running),
                    f.self_samples,
                    f.name
                );
            }
            if p.functions.len() > limit {
                let _ = writeln!(o, "  ... {} more functions", p.functions.len() - limit);
            }
        }
        o
    }

    /// Folded stacks, one line per distinct stack: `core0;outermost;...;innermost count`.
    pub fn folded(&self) -> String {
        let mut o = String::new();
        for (c, p) in self.cores.iter().enumerate() {
            for (stack, n) in &p.stacks {
                let _ = write!(o, "core{c}");
                for f in stack {
                    // `;` separates frames and the last space the count; keep both out of names
                    let _ = write!(o, ";{}", f.replace(';', ",").replace(' ', "_"));
                }
                let _ = writeln!(o, " {n}");
            }
        }
        o
    }

    /// Writes the flat table to `path` and the folded stacks next to it (`path` with the extension `.folded`).
    pub fn write(&self, path: &std::path::Path, title: &str) -> std::io::Result<std::path::PathBuf> {
        std::fs::write(path, self.flat_text(title, 60))?;
        let folded = path.with_extension("folded");
        let folded = if folded == path { path.with_extension("stacks.folded") } else { folded };
        std::fs::write(&folded, self.folded())?;
        Ok(folded)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rom_functions_and_float_tables_are_named() {
        let s = Symbolizer::new(&Image::default(), rp2040::BOOTROM_B1);
        let names: Vec<&str> = s.rom.iter().map(|(_, n)| n.as_str()).collect();
        for want in
            ["memcpy", "memset", "flash_range_erase", "fadd", "fmul", "dadd", "ddiv", "double2int", "rom_table_lookup"]
        {
            assert!(names.contains(&want), "{want} missing from {names:?}");
        }
        let fadd = s.rom.iter().find(|(_, n)| n == "fadd").unwrap().0;
        assert_eq!(s.name(s.function(fadd + 6)), "rom:fadd");
    }

    #[test]
    fn symbols_are_demangled_and_cover_their_size() {
        let image = Image {
            symbols: vec![
                sim_core::image::Symbol {
                    name: "_ZN3foo3bar17h0123456789abcdefE".into(),
                    addr: 0x1000_0101,
                    size: 0x20,
                    is_function: true,
                },
                sim_core::image::Symbol { name: "main".into(), addr: 0x1000_0201, size: 0x10, is_function: true },
            ],
            ..Default::default()
        };
        let s = Symbolizer::new(&image, &[]);
        assert_eq!(s.name(s.function(0x1000_0110)), "foo::bar");
        // A label without a size runs to the next symbol
        let label = sim_core::image::Symbol { name: "divmod".into(), addr: 0x1000_0181, size: 0, is_function: true };
        let mut with_label = image.clone();
        with_label.symbols.insert(1, label);
        let l = Symbolizer::new(&with_label, &[]);
        assert_eq!(l.name(l.function(0x1000_01f0)), "divmod");
        assert_eq!(l.name(l.function(0x1000_0204)), "main");
        assert_eq!(s.name(s.function(0x1000_0208)), "main");
        // Between functions: the address itself
        assert_eq!(s.name(s.function(0x1000_0180)), "0x10000180");
    }

    #[test]
    fn double_names_follow_the_float_ones() {
        assert_eq!(double_name("fadd"), "dadd");
        assert_eq!(double_name("float2int"), "double2int");
        assert_eq!(double_name("int642float"), "int642double");
        assert_eq!(double_name("float2double"), "double2float");
    }

    #[test]
    fn exception_names() {
        assert_eq!(exception_name(15), "SysTick");
        assert_eq!(exception_name(16), "TIMER_IRQ_0");
        assert_eq!(exception_name(16 + 13), "IO_IRQ_BANK0");
    }

    #[test]
    fn jittered_steps_stay_within_half_an_interval() {
        let mut p = Profiler::for_image(&Image::default(), &[], Time::from_us(10), Time::ZERO);
        let steps: Vec<Time> = (0..1000).map(|_| p.step()).collect();
        assert!(steps.iter().all(|&s| s >= Time::from_us(5) && s < Time::from_us(15)));
        let mean = steps.iter().map(|s| s.as_ps()).sum::<u64>() / 1000;
        assert!((9_500_000..10_500_000).contains(&mean), "mean step {mean} ps");
    }
}
