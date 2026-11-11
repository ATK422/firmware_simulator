//! The bus the differential tests run programs against: ROM, flash and SRAM served as code and plain memory the way
//! a machine's batch bus serves them, and a block of test registers that raise interrupts (now or at a chosen
//! time), read the time, stall, switch the flash's fast path off and ask for the machine's attention. Every data
//! access is logged with its time, so that two engines can be compared access by access.

use sim_core::bus::{Bus, BusFault, DirectMemory, DirectRegion, Size, mem};

pub(crate) const ROM_BASE: u32 = 0x0000_0000;
pub(crate) const ROM_SIZE: u32 = 0x4000;
pub(crate) const FLASH_BASE: u32 = 0x1000_0000;
pub(crate) const FLASH_SIZE: u32 = 0x8000;
pub(crate) const SRAM_BASE: u32 = 0x2000_0000;
pub(crate) const SRAM_SIZE: u32 = 0x1_0000;
pub(crate) const REGS_BASE: u32 = 0x4000_0000;
pub(crate) const REGS_SIZE: u32 = 0x100;

/// Test registers (offsets from [`REGS_BASE`]). All accesses to them are slow ([`Bus::take_slow_access`]).
pub(crate) mod reg {
    /// Write: raises the interrupt lines set in the value (attention if that changes them). Read: the lines.
    pub const IRQ_SET: u32 = 0x00;
    /// Write: lowers the lines set in the value (attention if that changes them).
    pub const IRQ_CLR: u32 = 0x04;
    /// Read: the time of the access (low word).
    pub const NOW: u32 = 0x08;
    /// Write: raises line `v & 31` at the time of the access plus `1 + (v >> 5 & 0x3FF)` (no attention: the run
    /// loop's limit is fixed for a quantum, as on the machine).
    pub const SCHED: u32 = 0x0C;
    /// Read: takes the stall cycles last written (default 2) on top of the instruction's.
    pub const STALL: u32 = 0x10;
    /// Write: bit 0 switches the flash's fast path (code, plain reads) on or off.
    pub const XIP: u32 = 0x14;
    /// Any access asks for the machine's attention.
    pub const ATTN: u32 = 0x18;
}

/// One data access.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) struct Access {
    pub write: bool,
    pub addr: u32,
    pub size: Size,
    pub value: u32,
    /// Cycles since reset at the start of the accessing instruction.
    pub at: u64,
}

impl std::fmt::Debug for Access {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let rw = if self.write { "W" } else { "R" };
        write!(f, "@{} {rw}{} {:#010x} = {:#x}", self.at, self.size.bytes() * 8, self.addr, self.value)
    }
}

#[derive(Clone)]
pub(crate) struct DiffBus {
    pub rom: Vec<u8>,
    pub flash: Vec<u8>,
    pub sram: Vec<u8>,
    pub regs: [u32; (REGS_SIZE / 4) as usize],
    /// The interrupt lines the driver hands the core.
    pub lines: u32,
    /// Lines to raise at a time, in the order they were scheduled.
    pub events: Vec<(u64, u32)>,
    pub log: Vec<Access>,
    /// The time the current step or batch started at, and the cycles the batch has taken before the current
    /// instruction ([`Bus::set_elapsed`]).
    base: u64,
    elapsed: u64,
    attention: bool,
    slow: bool,
    stall: u32,
    /// Whether the flash is served as code and plain memory (else every access is slow and stalls).
    pub xip: bool,
    /// SysTick's reference clock ticks every this many cycles (0: never), and the tick count SysTick has been fed
    /// up to while it counts it (as the machine's `systick_ref`).
    pub tick_period: u64,
    pub ref_seen: Option<u64>,
    /// SRAM handed out for writing ([`Bus::ram`]) and not yet logged: logged before the next access.
    pending: Option<(u32, u32, u64)>,
    /// Marks as a machine's bus keeps them (the RP2040's `noisy` and `xip_failed`): set by an SRAM write and by a
    /// flash read or fetch, cleared by any access to the test registers.
    pub noisy: u8,
    pub flash_seen: u8,
    /// Whether compiled code may access SRAM, ROM and flash directly ([`Bus::direct_memory`]), and the windows.
    pub direct: bool,
    windows: DirectMemory,
}

impl DiffBus {
    pub fn new() -> Self {
        DiffBus {
            rom: vec![0; ROM_SIZE as usize],
            flash: vec![0; FLASH_SIZE as usize],
            sram: vec![0; SRAM_SIZE as usize],
            regs: [0; (REGS_SIZE / 4) as usize],
            lines: 0,
            events: Vec::new(),
            log: Vec::new(),
            base: 0,
            elapsed: 0,
            attention: false,
            slow: false,
            stall: 0,
            xip: true,
            tick_period: 0,
            ref_seen: None,
            pending: None,
            noisy: 0,
            flash_seen: 0,
            direct: true,
            windows: DirectMemory::NONE,
        }
    }

    /// A step or batch starts at `now`.
    /// (What accesses left for the CPU to collect starts afresh, as each batch gets a new bus on the machine.)
    pub fn begin(&mut self, now: u64) {
        self.flush();
        self.base = now;
        self.elapsed = 0;
        self.attention = false;
        self.slow = false;
    }

    fn now(&self) -> u64 {
        self.base + self.elapsed
    }

    /// Reference clock ticks by `t`.
    pub fn ticks(&self, t: u64) -> u64 {
        t.checked_div(self.tick_period).unwrap_or(0)
    }

    /// When the earliest event is due.
    pub fn next_event(&self) -> Option<u64> {
        self.events.iter().map(|e| e.0).min()
    }

    /// Raises the lines of every event due by `now`.
    pub fn fire(&mut self, now: u64) {
        let lines = &mut self.lines;
        self.events.retain(|&(t, irq)| {
            if t <= now {
                *lines |= 1 << irq;
            }
            t > now
        });
    }

    /// Logs what was written through [`Bus::ram`].
    pub fn flush(&mut self) {
        if let Some((addr, len, at)) = self.pending.take() {
            for k in 0..len / 4 {
                let a = addr + 4 * k;
                let value = mem::read(&self.sram, (a - SRAM_BASE) as usize, Size::Word);
                self.log.push(Access { write: true, addr: a, size: Size::Word, value, at });
            }
        }
    }

    /// Everything about the bus but its memory and log, for comparing two runs.
    pub fn state(&self) -> String {
        format!(
            "lines {:#x} events {:?} regs {:x?} xip {} ref {:?} stall {} noisy {} flash {}",
            self.lines, self.events, self.regs, self.xip, self.ref_seen, self.stall, self.noisy, self.flash_seen
        )
    }

    fn region(&mut self, addr: u32) -> Option<(&mut [u8], usize)> {
        let in_ = |base: u32, size: u32| addr.wrapping_sub(base) < size;
        if in_(ROM_BASE, ROM_SIZE) {
            Some((&mut self.rom, (addr - ROM_BASE) as usize))
        } else if in_(FLASH_BASE, FLASH_SIZE) {
            Some((&mut self.flash, (addr - FLASH_BASE) as usize))
        } else if in_(SRAM_BASE, SRAM_SIZE) {
            Some((&mut self.sram, (addr - SRAM_BASE) as usize))
        } else {
            None
        }
    }

    /// Where code is fetched from without checks: ROM, SRAM, and flash while its fast path is on.
    fn code(&mut self, addr: u32) -> Option<&mut [u8]> {
        if addr.wrapping_sub(FLASH_BASE) < FLASH_SIZE {
            if !self.xip {
                return None;
            }
            self.flash_seen = 1;
        }
        self.region(addr).map(|(b, o)| &mut b[o..]).filter(|b| b.len() >= 2)
    }

    /// The halfword at `addr`, without side effects.
    pub fn peek16(&mut self, addr: u32) -> Option<u16> {
        self.region(addr).and_then(|(b, o)| b.get(o..o + 2)).map(|h| u16::from_le_bytes([h[0], h[1]]))
    }

    /// Writes a halfword anywhere in memory (loading programs).
    pub fn poke16(&mut self, addr: u32, v: u16) {
        let (b, o) = self.region(addr).expect("poke outside memory");
        mem::write(b, o, Size::Half, v as u32);
    }

    pub fn poke32(&mut self, addr: u32, v: u32) {
        let (b, o) = self.region(addr).expect("poke outside memory");
        mem::write(b, o, Size::Word, v);
    }

    fn reg_read(&mut self, addr: u32, size: Size) -> u32 {
        self.slow = true;
        (self.noisy, self.flash_seen) = (0, 0);
        let off = addr & (REGS_SIZE - 4);
        let word = match off {
            reg::IRQ_SET => self.lines,
            reg::NOW => self.now() as u32,
            reg::STALL => {
                self.stall += if self.regs[(off / 4) as usize] == 0 { 2 } else { self.regs[(off / 4) as usize] & 7 };
                self.regs[(off / 4) as usize]
            }
            reg::ATTN => {
                self.attention = true;
                0xA77E
            }
            _ => self.regs[(off / 4) as usize],
        };
        mem::lane(word, addr, size)
    }

    fn reg_write(&mut self, addr: u32, size: Size, value: u32) {
        self.slow = true;
        (self.noisy, self.flash_seen) = (0, 0);
        let off = addr & (REGS_SIZE - 4);
        let v = mem::replicate(value, size);
        match off {
            reg::IRQ_SET | reg::IRQ_CLR => {
                let old = self.lines;
                self.lines = if off == reg::IRQ_SET { old | v } else { old & !v };
                self.attention |= self.lines != old;
            }
            reg::SCHED => {
                let at = self.now() + 1 + ((v >> 5) & 0x3FF) as u64;
                self.events.push((at, v & 31));
            }
            reg::XIP => self.xip = v & 1 != 0,
            reg::ATTN => self.attention = true,
            _ => {}
        }
        self.regs[(off / 4) as usize] = v;
    }
}

impl Bus for DiffBus {
    fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
        self.flush();
        let at = self.now();
        let value = if addr.wrapping_sub(REGS_BASE) < REGS_SIZE {
            self.reg_read(addr, size)
        } else {
            if addr.wrapping_sub(FLASH_BASE) < FLASH_SIZE {
                self.flash_seen = 1;
                if !self.xip {
                    self.slow = true;
                    self.stall += 1;
                }
            }
            let (b, o) = self.region(addr).ok_or(BusFault::Unmapped(addr))?;
            mem::read(b, o, size)
        };
        self.log.push(Access { write: false, addr, size, value, at });
        Ok(value)
    }

    fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
        self.flush();
        let at = self.now();
        self.log.push(Access { write: true, addr, size, value: value & size.mask(), at });
        if addr.wrapping_sub(REGS_BASE) < REGS_SIZE {
            self.reg_write(addr, size, value);
            return Ok(());
        }
        if addr.wrapping_sub(SRAM_BASE) >= SRAM_SIZE {
            // ROM and flash reject writes
            return Err(if self.region(addr).is_some() { BusFault::Rejected(addr) } else { BusFault::Unmapped(addr) });
        }
        self.noisy = 1;
        let (b, o) = self.region(addr).ok_or(BusFault::Unmapped(addr))?;
        mem::write(b, o, size, value);
        Ok(())
    }

    fn fetch16(&mut self, addr: u32) -> Result<u16, BusFault> {
        self.flush();
        if addr.wrapping_sub(FLASH_BASE) < FLASH_SIZE {
            self.flash_seen = 1;
            if !self.xip {
                self.slow = true;
                self.stall += 1;
            }
        }
        let (b, o) = self.region(addr).ok_or(BusFault::Unmapped(addr))?;
        b.get(o..o + 2).map(|h| u16::from_le_bytes([h[0], h[1]])).ok_or(BusFault::Unmapped(addr))
    }

    fn fetch16_fast(&mut self, addr: u32) -> Option<u16> {
        self.flush();
        self.code(addr).map(|b| u16::from_le_bytes([b[0], b[1]]))
    }

    fn code_at(&mut self, addr: u32) -> Option<&[u8]> {
        self.flush();
        self.code(addr).map(|b| &*b)
    }

    fn take_slow_access(&mut self) -> bool {
        std::mem::take(&mut self.slow)
    }

    fn ram(&mut self, addr: u32, len: u32, write: bool) -> Option<&mut [u8]> {
        self.flush();
        let off = addr.wrapping_sub(SRAM_BASE);
        if off >= SRAM_SIZE || len > SRAM_SIZE - off {
            return None;
        }
        let at = self.now();
        if write {
            self.noisy = 1;
            self.pending = Some((addr, len, at));
        } else {
            for k in 0..len / 4 {
                let value = mem::read(&self.sram, (off + 4 * k) as usize, Size::Word);
                self.log.push(Access { write: false, addr: addr + 4 * k, size: Size::Word, value, at });
            }
        }
        Some(&mut self.sram[off as usize..(off + len) as usize])
    }

    fn take_stall_cycles(&mut self) -> u32 {
        std::mem::take(&mut self.stall)
    }

    fn set_elapsed(&mut self, cycles: u64) {
        self.elapsed = cycles;
    }

    fn take_attention(&mut self) -> bool {
        std::mem::take(&mut self.attention)
    }

    /// SRAM (writes set `noisy`), ROM, and flash while its fast path is on (reads set `flash_seen`), unless switched
    /// off. (Logging what compiled code reads and writes there is the CPU's business: see the tests' `Sim`, which
    /// turns them off where the access log is compared.)
    fn direct_memory(&mut self) -> Option<&DirectMemory> {
        if !self.direct {
            return None;
        }
        let window = |base: u32, size: u32, mem: &mut Vec<u8>| DirectRegion {
            base,
            size,
            mask: u32::MAX,
            len: mem.len() as u32,
            ptr: mem.as_mut_ptr(),
            ..DirectRegion::NONE
        };
        let sram = DirectRegion {
            writable: 1,
            write_mark: &raw mut self.noisy,
            write_value: 1,
            ..window(SRAM_BASE, SRAM_SIZE, &mut self.sram)
        };
        let rom = window(ROM_BASE, ROM_SIZE, &mut self.rom);
        let flash = if self.xip {
            DirectRegion {
                read_mark: &raw mut self.flash_seen,
                read_value: 1,
                ..window(FLASH_BASE, FLASH_SIZE, &mut self.flash)
            }
        } else {
            DirectRegion::NONE
        };
        self.windows = DirectMemory { regions: [sram, flash, rom, DirectRegion::NONE] };
        Some(&self.windows)
    }

    fn take_reference_ticks(&mut self) -> u64 {
        let Some(seen) = self.ref_seen else { return 0 };
        let ticks = self.ticks(self.now());
        self.ref_seen = Some(ticks);
        ticks.saturating_sub(seen)
    }
}
