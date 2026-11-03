//! The RP2040 bus fabric: memories, peripheral dispatch and the per-core view the CPUs execute against.
//!
//! Peripheral registers are accessed a word at a time: narrow writes are replicated across byte lanes (as the APB
//! and AHB-lite bridges do) and narrow reads take their lane from the word. The XOR/SET/CLR aliases at
//! `+0x1000/+0x2000/+0x3000` reach each model as a [`WriteMode`]. Registers no model claims fall through to the
//! [`Unmodelled`] fallback, which names them from the SVD.

use sim_core::bus::{Bus, BusFault, Size, WriteMode, mem};
use sim_core::sched::Scheduler;
use sim_core::time::{ClockPeriod, Time};
use sim_core::trace::{Category, TraceEvent, TraceSink};

use crate::irq;
use crate::memmap::*;
use crate::periph::clocks::{Clocks, Freqs, Pll};
use crate::periph::gpio::{Drive, Gpio, func};
use crate::periph::pwm::Pwm;
use crate::periph::sio::Sio;
use crate::periph::sysctl::{self, Psm, Resets, Vreg, bit, psm};
use crate::periph::timer::{Timer, Watchdog, WatchdogReset};
use crate::unmodelled::Unmodelled;

/// Scheduler keys. Each peripheral owns its keys; at most one event per key is pending.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Event {
    /// TIMER alarm n.
    TimerAlarm(u8),
    /// The watchdog countdown reaches zero.
    WatchdogTimeout,
    /// A PWM wrap interrupt or an observed output edge.
    Pwm,
    /// Wakes the run loop at a given time without doing anything (host input, test hooks).
    External(u32),
}

/// A reset the machine must carry out.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ResetRequest {
    /// The watchdog fired (or was triggered): reset per PSM WDSEL, keep scratch registers and RAM.
    Watchdog(WatchdogReset),
}

/// Default crystal on every UWRT board and the Pico.
pub const DEFAULT_CRYSTAL_HZ: u64 = 12_000_000;

/// Everything on the bus except the CPU cores. `Clone` gives a snapshot.
#[derive(Clone)]
pub struct System {
    pub rom: Vec<u8>,
    pub sram: Vec<u8>,
    pub flash: Vec<u8>,
    pub xip_sram: Vec<u8>,
    pub clocks: Clocks,
    /// Clock frequencies after the last clock tree change.
    pub freqs: Freqs,
    /// Current `clk_sys` period (the CPUs' cycle time).
    pub clk_sys: ClockPeriod,
    pub resets: Resets,
    pub psm: Psm,
    pub vreg: Vreg,
    pub timer: Timer,
    pub watchdog: Watchdog,
    pub gpio: Gpio,
    pub sio: Sio,
    pub pwm: Pwm,
    pub unmodelled: Unmodelled,
    pub sched: Scheduler<Event>,
    /// Set when an IRQ line may have changed; the machine recomputes the lines it feeds the NVICs.
    pub irq_dirty: bool,
    /// Set when `clk_sys` changed; the machine picks up the new period.
    pub clk_sys_changed: bool,
    /// A reset requested by a peripheral, for the machine to perform.
    pub reset_request: Option<ResetRequest>,
    /// PSM domains forced off (bits) and released since the machine last looked.
    pub psm_forced_off: u32,
    pub psm_released: u32,
    /// Chip revision reported by SYSINFO (1 = B1, 2 = B2).
    pub chip_revision: u32,
    /// Blocks already warned about being written while held in reset.
    warned_in_reset: u32,
    /// Virtual time used for debugger peeks (set by the machine).
    pub peek_time: Time,
}

impl System {
    pub fn new(flash_size: usize, bootrom: Option<&[u8]>) -> System {
        let mut rom = vec![0u8; ROM_SIZE];
        if let Some(b) = bootrom {
            let n = b.len().min(ROM_SIZE);
            rom[..n].copy_from_slice(&b[..n]);
        }
        // Bootrom version byte at 0x13: 1 = B0, 2 = B1, 3 = B2
        let chip_revision = match rom[0x13] {
            3 => 2,
            2 => 1,
            _ => 2,
        };
        let clocks = Clocks::new(DEFAULT_CRYSTAL_HZ, 0x5EED);
        let freqs = clocks.freqs(Time::ZERO);
        System {
            rom,
            sram: vec![0; SRAM_SIZE],
            flash: vec![0xFF; flash_size.max(4096)],
            xip_sram: vec![0; XIP_SRAM_SIZE],
            clk_sys: ClockPeriod::from_hz(freqs.clk_sys),
            freqs,
            clocks,
            resets: Resets::after_bootrom(),
            psm: Psm::default(),
            vreg: Vreg::default(),
            timer: Timer::new(),
            watchdog: Watchdog::new(freqs.clk_ref),
            gpio: Gpio::default(),
            sio: Sio::default(),
            pwm: Pwm::default(),
            unmodelled: Unmodelled::default(),
            sched: Scheduler::new(),
            irq_dirty: true,
            clk_sys_changed: true,
            reset_request: None,
            psm_forced_off: 0,
            psm_released: 0,
            chip_revision,
            warned_in_reset: 0,
            peek_time: Time::ZERO,
        }
    }

    /// Resets every block a watchdog reset reaches: clocks, PLLs, RESETS, the peripherals, SIO and PSM. RAM,
    /// flash, the watchdog's scratch registers, REASON and tick configuration, and VREG survive.
    pub fn watchdog_reset(&mut self, now: Time) {
        let crystal = self.clocks.xosc.crystal_hz;
        self.clocks = Clocks::new(crystal, 0x5EED ^ now.as_ps());
        self.resets = Resets::after_bootrom();
        self.psm = Psm::default();
        self.timer = Timer::new();
        self.timer.restart(&self.watchdog.tick, now);
        self.gpio.reset_io(false);
        self.sio = Sio::default();
        let observed = self.pwm.observed;
        self.pwm = Pwm { observed, ..Pwm::default() };
        self.unmodelled.reset();
        self.sched = Scheduler::new();
        self.update_clocks(now);
        self.irq_dirty = true;
        self.reset_request = None;
    }

    #[inline]
    fn flash_offset(&self, addr: u32) -> usize {
        (addr & 0x00FF_FFFF) as usize % self.flash.len()
    }

    /// IRQ lines seen by `core`'s NVIC.
    pub fn irq_lines(&self, core: u8) -> u32 {
        let c = core as usize & 1;
        let mut lines = self.timer.irq_lines();
        if self.pwm.irq_line() {
            lines |= 1 << irq::PWM_IRQ_WRAP;
        }
        if self.gpio.irq(c) {
            lines |= 1 << irq::IO_IRQ_BANK0;
        }
        if self.sio.fifo_irq(c) {
            lines |= 1 << (irq::SIO_IRQ_PROC0 + c as u32);
        }
        lines
    }

    // ------------------------------------------------------------------ cross-block effects

    /// Recomputes clock frequencies and propagates changes to everything clocked by them.
    pub fn update_clocks(&mut self, now: Time) {
        let new = self.clocks.freqs(now);
        let old = self.freqs;
        if new == old {
            return;
        }
        if new.clk_ref != old.clk_ref {
            self.watchdog.set_ref_hz(now, new.clk_ref);
            self.reschedule_timer(now, 0xF);
            self.reschedule_watchdog(now);
        }
        if new.clk_sys != old.clk_sys {
            if new.clk_sys == 0 {
                log::error!("[{now}] clk_sys configured to stop; keeping {} Hz", old.clk_sys);
            } else {
                self.pwm.rebase_all(&self.clk_sys, now);
                self.clk_sys = ClockPeriod::from_hz(new.clk_sys);
                self.clk_sys_changed = true;
                log::debug!("[{now}] clk_sys = {} Hz", new.clk_sys);
                self.reschedule_pwm(now);
            }
        }
        self.freqs = new;
    }

    /// Resets peripherals that just entered reset through RESETS.
    fn apply_resets(&mut self, now: Time, bits: u32) {
        let mut b = bits;
        while b != 0 {
            let n = b.trailing_zeros();
            b &= b - 1;
            match n {
                bit::IO_BANK0 => {
                    self.gpio.reset_io(true);
                }
                bit::PADS_BANK0 => self.gpio.pads = Gpio::default().pads,
                bit::PLL_SYS => self.clocks.pll_sys = Pll::default(),
                bit::PLL_USB => self.clocks.pll_usb = Pll::default(),
                bit::PWM => {
                    let observed = self.pwm.observed;
                    self.pwm = Pwm { observed, ..Pwm::default() };
                    self.sched.cancel(Event::Pwm);
                }
                bit::TIMER => {
                    self.timer = Timer::new();
                    self.timer.restart(&self.watchdog.tick, now);
                    for n in 0..4 {
                        self.sched.cancel(Event::TimerAlarm(n));
                    }
                }
                _ => {
                    if let Some(base) = reset_bit_base(n) {
                        self.unmodelled.reset_block(base);
                    }
                }
            }
        }
        if bits & (1 << bit::PLL_SYS | 1 << bit::PLL_USB) != 0 {
            self.update_clocks(now);
        }
        self.irq_dirty = true;
    }

    fn reschedule_timer(&mut self, now: Time, alarms: u8) {
        for n in 0..4u8 {
            if alarms & (1 << n) != 0 {
                match self.timer.alarm_due(&self.watchdog.tick, now, n as usize) {
                    Some(t) => self.sched.schedule(Event::TimerAlarm(n), t),
                    None => {
                        self.sched.cancel(Event::TimerAlarm(n));
                    }
                }
            }
        }
    }

    fn reschedule_watchdog(&mut self, now: Time) {
        match self.watchdog.timeout_due(now) {
            Some(t) => self.sched.schedule(Event::WatchdogTimeout, t),
            None => {
                self.sched.cancel(Event::WatchdogTimeout);
            }
        }
    }

    fn reschedule_pwm(&mut self, now: Time) {
        match self.pwm.next_event(&self.clk_sys, now) {
            Some(t) => self.sched.schedule(Event::Pwm, t.max(now + Time(1))),
            None => {
                self.sched.cancel(Event::Pwm);
            }
        }
    }

    /// Re-resolves every pin from the peripherals driving it, traces level changes and latches GPIO interrupts.
    pub fn update_pins(&mut self, trace: &mut dyn TraceSink, now: Time) {
        let sio_out = self.sio.gpio_out;
        let sio_oe = self.sio.gpio_oe;
        let pwm = &self.pwm;
        let clk = self.clk_sys;
        let changed = self.gpio.evaluate(|f, pin| match f {
            func::SIO => Drive { out: sio_out & (1 << pin) != 0, oe: sio_oe & (1 << pin) != 0 },
            func::PWM => Drive { out: pwm.pin_output(&clk, now, pin), oe: true },
            // GPOUT clock outputs on 21, 23, 24, 25 drive the pin (the waveform itself is not simulated)
            func::CLOCK if matches!(pin, 21 | 23 | 24 | 25) => Drive { out: false, oe: true },
            _ => Drive::default(),
        });
        if changed != 0 {
            self.irq_dirty = true;
            if trace.wants(Category::Gpio) {
                for pin in 0..NUM_GPIOS as u8 {
                    if changed & (1 << pin) != 0 {
                        trace.event(now, TraceEvent::Gpio { pin, level: self.gpio.level & (1 << pin) != 0 });
                    }
                }
            }
        }
    }

    /// Handles a scheduled event at its due time `t`.
    pub fn fire_event(&mut self, trace: &mut dyn TraceSink, t: Time, ev: Event) {
        match ev {
            Event::TimerAlarm(n) => {
                if self.timer.fire(&self.watchdog.tick, t, n as usize) {
                    self.irq_dirty = true;
                } else {
                    self.reschedule_timer(t, 1 << n);
                }
            }
            Event::WatchdogTimeout => {
                if self.watchdog.expired(t) {
                    log::warn!("[{t}] watchdog timeout");
                    self.reset_request = Some(ResetRequest::Watchdog(WatchdogReset::Timer));
                } else {
                    self.reschedule_watchdog(t);
                }
            }
            Event::Pwm => {
                self.pwm.sync(&self.clk_sys, t);
                self.irq_dirty = true;
                self.update_pins(trace, t);
                self.reschedule_pwm(t);
            }
            Event::External(_) => {}
        }
    }

    // ------------------------------------------------------------------ word-level peripheral access

    fn in_reset(&self, base: u32) -> bool {
        base_reset_bit(base).is_some_and(|b| self.resets.in_reset(b))
    }

    fn periph_read(&mut self, trace: &mut dyn TraceSink, core: u8, now: Time, addr: u32) -> u32 {
        if (SIO_BASE..SIO_BASE + 0x1000).contains(&addr) {
            let gpio_in = self.gpio.inputs();
            if let Some(v) = self.sio.read(core, addr - SIO_BASE, gpio_in, 0) {
                self.irq_dirty = true;
                return v;
            }
            return self.unmodelled.read(trace, now, core, addr);
        }
        let (base, off) = split(addr);
        let value = match base {
            SYSINFO_BASE => sysctl::sysinfo_read(off, self.chip_revision),
            CLOCKS_BASE => self.clocks.read(off),
            RESETS_BASE => self.resets.read(off),
            PSM_BASE => self.psm.read(off),
            IO_BANK0_BASE => self.gpio.read_io(off),
            PADS_BANK0_BASE => self.gpio.read_pads(off),
            XOSC_BASE => self.clocks.xosc.read(now, off),
            PLL_SYS_BASE => self.clocks.pll_sys.read(now, off),
            PLL_USB_BASE => self.clocks.pll_usb.read(now, off),
            PWM_BASE => {
                let v = self.pwm.read(&self.clk_sys, now, off);
                self.irq_dirty = true;
                v
            }
            TIMER_BASE => self.timer.read(&self.watchdog.tick, now, off),
            WATCHDOG_BASE => self.watchdog.read(now, off),
            ROSC_BASE => self.clocks.rosc.read(off),
            VREG_AND_CHIP_RESET_BASE => self.vreg.read(off),
            _ => None,
        };
        match value {
            Some(v) => v,
            None => self.unmodelled.read(trace, now, core, addr),
        }
    }

    fn periph_write(&mut self, trace: &mut dyn TraceSink, core: u8, now: Time, addr: u32, value: u32) {
        if (SIO_BASE..SIO_BASE + 0x1000).contains(&addr) {
            if self.sio.write(core, addr - SIO_BASE, value) {
                self.irq_dirty = true;
                if addr - SIO_BASE < 0x50 {
                    self.update_pins(trace, now);
                }
            } else {
                self.unmodelled.write(trace, now, core, addr, value);
            }
            return;
        }
        let (base, off) = split(addr);
        let mode = alias_mode(addr);
        if self.in_reset(base) {
            let b = base_reset_bit(base).unwrap();
            if self.warned_in_reset & (1 << b) == 0 {
                self.warned_in_reset |= 1 << b;
                log::warn!(
                    "[{now}] core{core} write to {} while the block is held in RESETS; ignored",
                    crate::svd::describe(addr)
                );
            }
            return;
        }
        let handled = match base {
            CLOCKS_BASE => self.clocks.write(now, off, value, mode).map(|changed| {
                if changed {
                    self.update_clocks(now);
                }
            }),
            RESETS_BASE => self.resets.write(off, value, mode).map(|entered| {
                if entered != 0 {
                    self.apply_resets(now, entered);
                }
            }),
            PSM_BASE => self.psm.write(off, value, mode).map(|e| {
                if e.forced_off & (1 << psm::SIO) != 0 {
                    self.sio.reset();
                    self.irq_dirty = true;
                }
                self.psm_forced_off |= e.forced_off;
                self.psm_released |= e.released;
            }),
            IO_BANK0_BASE => self.gpio.write_io(off, value, mode).map(|_| {
                self.update_pins(trace, now);
                self.irq_dirty = true;
            }),
            PADS_BANK0_BASE => self.gpio.write_pads(off, value, mode).map(|_| self.update_pins(trace, now)),
            XOSC_BASE => self.clocks.xosc.write(now, off, value, mode).then(|| self.update_clocks(now)),
            PLL_SYS_BASE => self.clocks.pll_sys.write(now, off, value, mode).then(|| self.update_clocks(now)),
            PLL_USB_BASE => self.clocks.pll_usb.write(now, off, value, mode).then(|| self.update_clocks(now)),
            ROSC_BASE => self.clocks.rosc.write(off, value, mode).then(|| self.update_clocks(now)),
            PWM_BASE => self.pwm.write(&self.clk_sys, now, off, value, mode).then(|| {
                self.irq_dirty = true;
                self.update_pins(trace, now);
                self.reschedule_pwm(now);
            }),
            TIMER_BASE => self.timer.write(&self.watchdog.tick, now, off, value, mode).map(|alarms| {
                self.irq_dirty = true;
                self.reschedule_timer(now, alarms);
            }),
            WATCHDOG_BASE => self.watchdog.write(now, off, value, mode).map(|force| {
                if force {
                    log::info!("[{now}] watchdog reset triggered by core{core}");
                    self.reset_request = Some(ResetRequest::Watchdog(WatchdogReset::Force));
                }
                // The tick configuration may have changed
                self.reschedule_timer(now, 0xF);
                self.reschedule_watchdog(now);
            }),
            VREG_AND_CHIP_RESET_BASE => self.vreg.write(off, value, mode).then_some(()),
            SYSINFO_BASE if off <= 0x40 => Some(()),
            _ => None,
        };
        if handled.is_none() {
            self.unmodelled.write(trace, now, core, addr, value);
        }
    }

    // ------------------------------------------------------------------ byte-level bus

    /// A data read by `core` at virtual time `now`.
    pub fn read(
        &mut self,
        trace: &mut dyn TraceSink,
        core: u8,
        now: Time,
        addr: u32,
        size: Size,
    ) -> Result<u32, BusFault> {
        match addr >> 28 {
            0x0 => {
                if (addr as usize) < ROM_SIZE {
                    Ok(mem::read(&self.rom, addr as usize, size))
                } else {
                    Err(BusFault::Unmapped(addr))
                }
            }
            0x1 => match addr >> 24 {
                0x10..=0x13 => Ok(mem::read(&self.flash, self.flash_offset(addr), size)),
                0x15 if ((addr & 0x00FF_FFFF) as usize) < XIP_SRAM_SIZE => {
                    Ok(mem::read(&self.xip_sram, (addr & 0x3FFF) as usize, size))
                }
                0x14 | 0x18 => {
                    let word = self.periph_read(trace, core, now, addr & !3);
                    Ok(mem::lane(word, addr, size))
                }
                _ => Err(BusFault::Unmapped(addr)),
            },
            0x2 => match sram_offset(addr) {
                Some(off) => Ok(mem::read(&self.sram, off, size)),
                None => Err(BusFault::Unmapped(addr)),
            },
            0x4 | 0x5 | 0xD => {
                let word = self.periph_read(trace, core, now, addr & !3);
                Ok(mem::lane(word, addr, size))
            }
            _ => Err(BusFault::Unmapped(addr)),
        }
    }

    /// A data write by `core` at virtual time `now`.
    pub fn write(
        &mut self,
        trace: &mut dyn TraceSink,
        core: u8,
        now: Time,
        addr: u32,
        size: Size,
        value: u32,
    ) -> Result<(), BusFault> {
        match addr >> 28 {
            0x0 => {
                log::debug!("core{core} write to ROM {addr:#010x} ignored");
                Ok(())
            }
            0x1 => match addr >> 24 {
                0x10..=0x13 => {
                    log::debug!("core{core} write to XIP {addr:#010x} ignored");
                    Ok(())
                }
                0x15 if ((addr & 0x00FF_FFFF) as usize) < XIP_SRAM_SIZE => {
                    mem::write(&mut self.xip_sram, (addr & 0x3FFF) as usize, size, value);
                    Ok(())
                }
                0x14 | 0x18 => {
                    self.periph_write(trace, core, now, addr & !3, mem::replicate(value, size));
                    Ok(())
                }
                _ => Err(BusFault::Unmapped(addr)),
            },
            0x2 => match sram_offset(addr) {
                Some(off) => {
                    mem::write(&mut self.sram, off, size, value);
                    Ok(())
                }
                None => Err(BusFault::Unmapped(addr)),
            },
            0x4 | 0x5 | 0xD => {
                self.periph_write(trace, core, now, addr & !3, mem::replicate(value, size));
                Ok(())
            }
            _ => Err(BusFault::Unmapped(addr)),
        }
    }

    /// Side-effect free read of memory (not peripherals), for debuggers and loaders.
    pub fn peek(&self, addr: u32, size: Size) -> Option<u32> {
        match addr >> 28 {
            0x0 if (addr as usize) < ROM_SIZE => Some(mem::read(&self.rom, addr as usize, size)),
            0x1 if (0x10..=0x13).contains(&(addr >> 24)) => Some(mem::read(&self.flash, self.flash_offset(addr), size)),
            0x1 if addr >> 24 == 0x15 && ((addr & 0x00FF_FFFF) as usize) < XIP_SRAM_SIZE => {
                Some(mem::read(&self.xip_sram, (addr & 0x3FFF) as usize, size))
            }
            0x2 => sram_offset(addr).map(|o| mem::read(&self.sram, o, size)),
            0x4 | 0x5 | 0xD | 0x1 => Some(mem::lane(self.peek_periph(0, addr & !3), addr, size)),
            _ => None,
        }
    }

    /// Side-effect free read of a peripheral register (FIFO pops, spinlock claims, latches do not happen).
    pub fn peek_periph(&self, core: u8, addr: u32) -> u32 {
        if (SIO_BASE..SIO_BASE + 0x1000).contains(&addr) {
            return self
                .sio
                .peek(core, addr - SIO_BASE, self.gpio.inputs())
                .unwrap_or_else(|| self.unmodelled.peek(addr));
        }
        let (base, off) = split(addr);
        let now = self.peek_time;
        let v = match base {
            SYSINFO_BASE => sysctl::sysinfo_read(off, self.chip_revision),
            CLOCKS_BASE => self.clocks.read(off),
            RESETS_BASE => self.resets.read(off),
            PSM_BASE => self.psm.read(off),
            IO_BANK0_BASE => self.gpio.read_io(off),
            PADS_BANK0_BASE => self.gpio.read_pads(off),
            XOSC_BASE => self.clocks.xosc.read(now, off),
            PLL_SYS_BASE => self.clocks.pll_sys.read(now, off),
            PLL_USB_BASE => self.clocks.pll_usb.read(now, off),
            TIMER_BASE => self.timer.peek(&self.watchdog.tick, now, off),
            WATCHDOG_BASE => self.watchdog.read(now, off),
            VREG_AND_CHIP_RESET_BASE => self.vreg.read(off),
            _ => None,
        };
        v.unwrap_or_else(|| self.unmodelled.peek(addr))
    }

    /// Writes memory directly (loaders, debuggers): flash, ROM and RAM are all writable this way.
    pub fn poke(&mut self, addr: u32, size: Size, value: u32) -> bool {
        match addr >> 28 {
            0x0 if (addr as usize) < ROM_SIZE => mem::write(&mut self.rom, addr as usize, size, value),
            0x1 if (0x10..=0x13).contains(&(addr >> 24)) => {
                let off = self.flash_offset(addr);
                mem::write(&mut self.flash, off, size, value)
            }
            0x1 if addr >> 24 == 0x15 && ((addr & 0x00FF_FFFF) as usize) < XIP_SRAM_SIZE => {
                mem::write(&mut self.xip_sram, (addr & 0x3FFF) as usize, size, value)
            }
            0x2 => match sram_offset(addr) {
                Some(o) => mem::write(&mut self.sram, o, size, value),
                None => return false,
            },
            _ => return false,
        }
        true
    }
}

/// Splits a peripheral address into its block base and register offset (alias bits removed).
fn split(addr: u32) -> (u32, u32) {
    if (APB_BASE..0x5000_0000).contains(&addr) {
        (addr & !0x3FFF, addr & 0xFFF)
    } else if (0x5000_0000..0x6000_0000).contains(&addr) {
        // AHB-lite blocks: DMA, USB, PIO0, PIO1, XIP_AUX at 1 MiB spacing (USB regs at +0x10000)
        let base = addr & !0xF_FFFF;
        if base == 0x5010_0000 && addr & 0x1_0000 != 0 {
            (USBCTRL_REGS_BASE, addr & 0xFFF)
        } else if base == 0x5010_0000 {
            (USBCTRL_DPRAM_BASE, addr & 0xFFFF)
        } else {
            (base, addr & 0xFFF)
        }
    } else {
        (addr & !0xFFF, addr & 0xFFF)
    }
}

/// The atomic alias an address falls in.
fn alias_mode(addr: u32) -> WriteMode {
    if !(APB_BASE..0x6000_0000).contains(&addr) || (USBCTRL_DPRAM_BASE..USBCTRL_REGS_BASE).contains(&addr) {
        return WriteMode::Normal;
    }
    match (addr >> 12) & 3 {
        1 => WriteMode::Xor,
        2 => WriteMode::Set,
        3 => WriteMode::Clear,
        _ => WriteMode::Normal,
    }
}

/// The RESETS bit holding a block in reset.
fn base_reset_bit(base: u32) -> Option<u32> {
    Some(match base {
        ADC_BASE => bit::ADC,
        BUSCTRL_BASE => bit::BUSCTRL,
        DMA_BASE => bit::DMA,
        I2C0_BASE => bit::I2C0,
        I2C1_BASE => bit::I2C1,
        IO_BANK0_BASE => bit::IO_BANK0,
        IO_QSPI_BASE => bit::IO_QSPI,
        PADS_BANK0_BASE => bit::PADS_BANK0,
        PADS_QSPI_BASE => bit::PADS_QSPI,
        PIO0_BASE => bit::PIO0,
        PIO1_BASE => bit::PIO1,
        PLL_SYS_BASE => bit::PLL_SYS,
        PLL_USB_BASE => bit::PLL_USB,
        PWM_BASE => bit::PWM,
        RTC_BASE => bit::RTC,
        SPI0_BASE => bit::SPI0,
        SPI1_BASE => bit::SPI1,
        SYSCFG_BASE => bit::SYSCFG,
        SYSINFO_BASE => bit::SYSINFO,
        TBMAN_BASE => bit::TBMAN,
        TIMER_BASE => bit::TIMER,
        UART0_BASE => bit::UART0,
        UART1_BASE => bit::UART1,
        USBCTRL_REGS_BASE | USBCTRL_DPRAM_BASE => bit::USBCTRL,
        _ => return None,
    })
}

fn reset_bit_base(b: u32) -> Option<u32> {
    [
        ADC_BASE,
        BUSCTRL_BASE,
        DMA_BASE,
        I2C0_BASE,
        I2C1_BASE,
        IO_BANK0_BASE,
        IO_QSPI_BASE,
        0,
        PADS_BANK0_BASE,
        PADS_QSPI_BASE,
        PIO0_BASE,
        PIO1_BASE,
        PLL_SYS_BASE,
        PLL_USB_BASE,
        PWM_BASE,
        RTC_BASE,
        SPI0_BASE,
        SPI1_BASE,
        SYSCFG_BASE,
        SYSINFO_BASE,
        TBMAN_BASE,
        TIMER_BASE,
        UART0_BASE,
        UART1_BASE,
        USBCTRL_REGS_BASE,
    ]
    .get(b as usize)
    .copied()
    .filter(|&a| a != 0)
}

/// One core's view of the system bus.
pub struct CoreBus<'a> {
    pub sys: &'a mut System,
    pub trace: &'a mut dyn TraceSink,
    pub core: u8,
    pub now: Time,
}

impl Bus for CoreBus<'_> {
    #[inline]
    fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
        self.sys.read(self.trace, self.core, self.now, addr, size)
    }

    #[inline]
    fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
        self.sys.write(self.trace, self.core, self.now, addr, size, value)
    }

    #[inline]
    fn fetch16(&mut self, addr: u32) -> Result<u16, BusFault> {
        // Fast paths for the places code runs from
        match addr >> 28 {
            0x1 if addr >> 24 <= 0x13 => {
                let off = self.sys.flash_offset(addr);
                Ok(u16::from_le_bytes([self.sys.flash[off], self.sys.flash[off + 1]]))
            }
            0x2 if addr < SRAM_END => {
                let off = (addr - SRAM_BASE) as usize;
                Ok(u16::from_le_bytes([self.sys.sram[off], self.sys.sram[off + 1]]))
            }
            0x0 if (addr as usize) < ROM_SIZE => {
                Ok(u16::from_le_bytes([self.sys.rom[addr as usize], self.sys.rom[addr as usize + 1]]))
            }
            _ => self.read(addr, Size::Half).map(|v| v as u16),
        }
    }

    fn peek(&mut self, addr: u32, size: Size) -> Option<u32> {
        self.sys.peek(addr, size)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::trace::{NullSink, VecSink};

    fn bus_rw(sys: &mut System, addr: u32, size: Size, v: u32) -> u32 {
        let mut t = NullSink;
        sys.write(&mut t, 0, Time::ZERO, addr, size, v).unwrap();
        sys.read(&mut t, 0, Time::ZERO, addr, size).unwrap()
    }

    fn w(sys: &mut System, now: Time, addr: u32, v: u32) {
        sys.write(&mut NullSink, 0, now, addr, Size::Word, v).unwrap();
    }

    fn r(sys: &mut System, now: Time, addr: u32) -> u32 {
        sys.read(&mut NullSink, 0, now, addr, Size::Word).unwrap()
    }

    fn quiet() -> System {
        let mut sys = System::new(4096, None);
        sys.unmodelled.log_mode = crate::unmodelled::LogMode::Off;
        sys
    }

    #[test]
    fn memories() {
        let mut sys = System::new(2 * 1024 * 1024, Some(&[1, 2, 3, 4]));
        let mut t = NullSink;
        assert_eq!(sys.read(&mut t, 0, Time::ZERO, 0, Size::Word), Ok(0x0403_0201));
        assert_eq!(bus_rw(&mut sys, SRAM_BASE + 8, Size::Word, 0xAABB_CCDD), 0xAABB_CCDD);
        assert_eq!(sys.read(&mut t, 0, Time::ZERO, SRAM_BASE + 9, Size::Byte), Ok(0xCC));
        assert_eq!(bus_rw(&mut sys, SRAM_END - 2, Size::Half, 0x1234), 0x1234);
        assert_eq!(sys.read(&mut t, 0, Time::ZERO, SRAM_END, Size::Byte), Err(BusFault::Unmapped(SRAM_END)));
        // Flash reads as erased, writes to XIP are ignored, every alias reads the same flash
        assert_eq!(sys.read(&mut t, 0, Time::ZERO, XIP_BASE, Size::Word), Ok(0xFFFF_FFFF));
        sys.poke(XIP_BASE + 4, Size::Word, 0x1122_3344);
        for base in [XIP_BASE, XIP_NOALLOC_BASE, XIP_NOCACHE_BASE, XIP_NOCACHE_NOALLOC_BASE] {
            assert_eq!(sys.read(&mut t, 0, Time::ZERO, base + 4, Size::Word), Ok(0x1122_3344));
        }
        sys.write(&mut t, 0, Time::ZERO, XIP_BASE + 4, Size::Word, 0).unwrap();
        assert_eq!(sys.peek(XIP_BASE + 4, Size::Word), Some(0x1122_3344));
        assert_eq!(bus_rw(&mut sys, XIP_SRAM_BASE + 0x100, Size::Word, 99), 99);
        // Non-striped alias sees the same storage
        assert_eq!(
            sys.read(&mut t, 0, Time::ZERO, 0x2102_0000, Size::Word),
            Ok(0xAABB_CCDD),
            "striped word 2 is bank 2 word 0"
        );
        assert!(sys.read(&mut t, 0, Time::ZERO, 0x3000_0000, Size::Word).is_err());
    }

    #[test]
    fn narrow_peripheral_writes_replicate() {
        let mut sys = quiet();
        let mut t = NullSink;
        w(&mut sys, Time::ZERO, RESETS_BASE + 0x3000, 1 << bit::UART0);
        // UART0.UARTIBRD: byte write at offset +1 lands replicated in all lanes
        sys.write(&mut t, 0, Time::ZERO, UART0_BASE + 0x25, Size::Byte, 0xAB).unwrap();
        assert_eq!(sys.unmodelled.peek(UART0_BASE + 0x24), 0xABAB_ABAB);
        assert_eq!(sys.read(&mut t, 0, Time::ZERO, UART0_BASE + 0x26, Size::Half), Ok(0xABAB));
    }

    #[test]
    fn blocks_in_reset_ignore_writes() {
        let mut sys = quiet();
        w(&mut sys, Time::ZERO, IO_BANK0_BASE + 25 * 8 + 4, 5);
        assert_eq!(r(&mut sys, Time::ZERO, IO_BANK0_BASE + 25 * 8 + 4), 0x1F, "IO_BANK0 still in reset");
        w(&mut sys, Time::ZERO, RESETS_BASE + 0x3000, 1 << bit::IO_BANK0);
        assert_eq!(r(&mut sys, Time::ZERO, RESETS_BASE + 8) & (1 << bit::IO_BANK0), 1 << bit::IO_BANK0);
        w(&mut sys, Time::ZERO, IO_BANK0_BASE + 25 * 8 + 4, 5);
        assert_eq!(r(&mut sys, Time::ZERO, IO_BANK0_BASE + 25 * 8 + 4), 5);
        // Re-entering reset restores reset values
        w(&mut sys, Time::ZERO, RESETS_BASE + 0x2000, 1 << bit::IO_BANK0);
        assert_eq!(r(&mut sys, Time::ZERO, IO_BANK0_BASE + 25 * 8 + 4), 0x1F);
    }

    #[test]
    fn sio_gpio_through_io_bank0_traces_levels() {
        let mut sys = quiet();
        let sink = VecSink::new(&[Category::Gpio]);
        let mut t = sink.clone();
        let mut wr = |sys: &mut System, us: u64, addr: u32, v: u32| {
            sys.write(&mut t, 0, Time::from_us(us), addr, Size::Word, v).unwrap();
        };
        wr(&mut sys, 0, RESETS_BASE + 0x3000, 1 << bit::IO_BANK0 | 1 << bit::PADS_BANK0);
        wr(&mut sys, 0, IO_BANK0_BASE + 25 * 8 + 4, func::SIO);
        wr(&mut sys, 0, SIO_BASE + 0x24, 1 << 25); // OE_SET
        wr(&mut sys, 1, SIO_BASE + 0x14, 1 << 25); // OUT_SET
        wr(&mut sys, 2, SIO_BASE + 0x14, 1 << 25); // no change
        wr(&mut sys, 3, SIO_BASE + 0x1C, 1 << 25); // OUT_XOR
        let ev: Vec<String> = sink.take().iter().map(|(t, e)| format!("{t} {e}")).collect();
        assert_eq!(ev, vec!["0.000001s GPIO25 = 1", "0.000003s GPIO25 = 0"]);
        assert_eq!(r(&mut sys, Time::ZERO, SIO_BASE + 4) & (1 << 25), 0, "GPIO_IN follows the pin");
        let mut t = NullSink;
        assert_eq!(sys.read(&mut t, 1, Time::ZERO, SIO_BASE, Size::Word), Ok(1), "CPUID of core 1");
    }

    #[test]
    fn embassy_clock_bring_up_sets_clk_sys_and_tick() {
        let mut sys = quiet();
        let t0 = Time::ZERO;
        w(&mut sys, t0, XOSC_BASE + 0x0C, 47);
        w(&mut sys, t0, XOSC_BASE, 0xFAB << 12 | 0xAA0);
        let t1 = Time::from_ms(2);
        assert!(r(&mut sys, t1, XOSC_BASE + 4) & (1 << 31) != 0, "XOSC stable");
        w(&mut sys, t1, RESETS_BASE + 0x3000, 1 << bit::PLL_SYS);
        w(&mut sys, t1, PLL_SYS_BASE, 1);
        w(&mut sys, t1, PLL_SYS_BASE + 8, 125);
        w(&mut sys, t1, PLL_SYS_BASE + 4, 1 << 3);
        let t2 = Time::from_ms(3);
        assert!(r(&mut sys, t2, PLL_SYS_BASE) & (1 << 31) != 0, "PLL locked");
        w(&mut sys, t2, PLL_SYS_BASE + 0xC, 6 << 16 | 2 << 12);
        w(&mut sys, t2, PLL_SYS_BASE + 0x3004, 1 << 3);
        w(&mut sys, t2, CLOCKS_BASE + 4 * 0xC, 2); // clk_ref = XOSC
        w(&mut sys, t2, CLOCKS_BASE + 5 * 0xC, 1); // clk_sys = aux (PLL_SYS)
        assert_eq!(r(&mut sys, t2, CLOCKS_BASE + 5 * 0xC + 8), 2, "CLK_SYS_SELECTED");
        assert_eq!(sys.freqs.clk_sys, 125_000_000);
        assert!(sys.clk_sys_changed);
        assert_eq!(sys.clk_sys.hz(), 125_000_000);
        // Tick: 12 clk_ref cycles = 1 us
        w(&mut sys, t2, RESETS_BASE + 0x3000, 1 << bit::TIMER);
        w(&mut sys, t2, WATCHDOG_BASE + 0x2C, 1 << 9 | 12);
        let a = r(&mut sys, t2, TIMER_BASE + 0x28);
        let b = r(&mut sys, t2 + Time::from_us(250), TIMER_BASE + 0x28);
        assert_eq!(b - a, 250);
    }

    #[test]
    fn timer_alarm_event_raises_irq() {
        let mut sys = quiet();
        sys.update_clocks(Time::ZERO);
        w(&mut sys, Time::ZERO, RESETS_BASE + 0x3000, 1 << bit::TIMER);
        // clk_ref is ROSC (6.5 MHz) at reset; 65 cycles per tick = 10 us
        w(&mut sys, Time::ZERO, WATCHDOG_BASE + 0x2C, 1 << 9 | 65);
        w(&mut sys, Time::ZERO, TIMER_BASE + 0x38, 1);
        w(&mut sys, Time::ZERO, TIMER_BASE + 0x10, 5);
        let due = sys.sched.due(Event::TimerAlarm(0)).unwrap();
        assert_eq!(due.as_us(), 50);
        assert_eq!(sys.irq_lines(0) & 1, 0);
        sys.fire_event(&mut NullSink, due, Event::TimerAlarm(0));
        assert_eq!(sys.irq_lines(0) & 1, 1);
        assert_eq!(sys.irq_lines(1) & 1, 1, "timer IRQs reach both cores");
    }

    #[test]
    fn watchdog_force_requests_reset_and_keeps_scratch() {
        let mut sys = quiet();
        w(&mut sys, Time::ZERO, WATCHDOG_BASE + 0x0C + 12, 0x9A41_C0DE);
        w(&mut sys, Time::ZERO, SRAM_BASE, 0x1234);
        w(&mut sys, Time::ZERO, WATCHDOG_BASE, 1 << 31);
        assert_eq!(sys.reset_request, Some(ResetRequest::Watchdog(WatchdogReset::Force)));
        sys.watchdog.record_reset(WatchdogReset::Force);
        sys.watchdog_reset(Time::from_ms(1));
        assert_eq!(r(&mut sys, Time::from_ms(1), WATCHDOG_BASE + 8), 2);
        assert_eq!(r(&mut sys, Time::from_ms(1), WATCHDOG_BASE + 0x18), 0x9A41_C0DE);
        assert_eq!(r(&mut sys, Time::from_ms(1), SRAM_BASE), 0x1234);
        assert_eq!(sys.resets.reset, Resets::after_bootrom().reset);
    }

    #[test]
    fn sio_fifo_irq_per_core_and_psm_reset() {
        let mut sys = quiet();
        w(&mut sys, Time::ZERO, SIO_BASE + 0x54, 7); // core 0 pushes to core 1
        assert_eq!(sys.irq_lines(1) & (1 << irq::SIO_IRQ_PROC1), 1 << irq::SIO_IRQ_PROC1);
        assert_eq!(sys.irq_lines(0) & (3 << irq::SIO_IRQ_PROC0), 0);
        sys.read(&mut NullSink, 0, Time::ZERO, SIO_BASE + 0x17C, Size::Word).unwrap(); // claim spinlock 31
        w(&mut sys, Time::ZERO, PSM_BASE + 4, 1 << psm::SIO | 1 << psm::PROC1);
        assert_eq!(sys.sio.spinlocks, 0, "PSM force-off resets SIO");
        assert_eq!(sys.psm_forced_off, 1 << psm::SIO | 1 << psm::PROC1);
    }
}
