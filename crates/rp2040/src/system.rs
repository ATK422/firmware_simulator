//! The RP2040 bus fabric: memories, peripheral dispatch and the per-core view the CPUs execute against.

use sim_core::bus::{Bus, BusFault, Size, mem};
use sim_core::sched::Scheduler;
use sim_core::time::Time;
use sim_core::trace::{Category, TraceEvent, TraceSink};

use crate::memmap::*;
use crate::periph::sio::Sio;
use crate::unmodelled::Unmodelled;

/// Scheduler keys. Each peripheral owns its keys; at most one event per key is pending.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Event {
    /// Wakes the run loop at a given time without doing anything (host input, test hooks).
    External(u32),
}

/// Everything on the bus except the CPU cores. `Clone` gives a snapshot.
#[derive(Clone)]
pub struct System {
    pub rom: Vec<u8>,
    pub sram: Vec<u8>,
    pub flash: Vec<u8>,
    pub xip_sram: Vec<u8>,
    pub sio: Sio,
    pub unmodelled: Unmodelled,
    pub sched: Scheduler<Event>,
    /// Set when an IRQ line may have changed; the machine recomputes the lines it feeds the NVICs.
    pub irq_dirty: bool,
    /// Last traced SIO output state, for change detection.
    traced_gpio: (u32, u32),
}

impl System {
    pub fn new(flash_size: usize, bootrom: Option<&[u8]>) -> System {
        let mut rom = vec![0u8; ROM_SIZE];
        if let Some(b) = bootrom {
            let n = b.len().min(ROM_SIZE);
            rom[..n].copy_from_slice(&b[..n]);
        }
        System {
            rom,
            sram: vec![0; SRAM_SIZE],
            flash: vec![0xFF; flash_size.max(4096)],
            xip_sram: vec![0; XIP_SRAM_SIZE],
            sio: Sio::default(),
            unmodelled: Unmodelled::default(),
            sched: Scheduler::new(),
            irq_dirty: true,
            traced_gpio: (0, 0),
        }
    }

    #[inline]
    fn flash_offset(&self, addr: u32) -> usize {
        (addr & 0x00FF_FFFF) as usize % self.flash.len()
    }

    /// IRQ lines seen by `core`'s NVIC.
    pub fn irq_lines(&self, _core: u8) -> u32 {
        0
    }

    /// Input levels of bank 0. Until IO_BANK0 is modelled, a pin reads back what SIO drives on it.
    pub fn gpio_in(&self) -> u32 {
        self.sio.gpio_out & self.sio.gpio_oe
    }

    fn trace_gpio(&mut self, trace: &mut dyn TraceSink, now: Time) {
        let state = (self.sio.gpio_out, self.sio.gpio_oe);
        if state == self.traced_gpio {
            return;
        }
        if trace.wants(Category::Gpio) {
            let changed = (state.0 ^ self.traced_gpio.0) | (state.1 ^ self.traced_gpio.1);
            for pin in 0..NUM_GPIOS as u8 {
                if changed & (1 << pin) != 0 {
                    trace.event(
                        now,
                        TraceEvent::GpioOutput {
                            pin,
                            value: state.0 & (1 << pin) != 0,
                            enabled: state.1 & (1 << pin) != 0,
                        },
                    );
                }
            }
        }
        self.traced_gpio = state;
    }

    // ------------------------------------------------------------------ word-level peripheral access

    fn periph_read(&mut self, trace: &mut dyn TraceSink, core: u8, now: Time, addr: u32) -> u32 {
        if (SIO_BASE..SIO_BASE + 0x1000).contains(&addr) {
            let gpio_in = self.gpio_in();
            if let Some(v) = self.sio.read(core, addr - SIO_BASE, gpio_in, 0) {
                return v;
            }
        }
        self.unmodelled.read(trace, now, core, addr)
    }

    fn periph_write(&mut self, trace: &mut dyn TraceSink, core: u8, now: Time, addr: u32, value: u32) {
        if (SIO_BASE..SIO_BASE + 0x1000).contains(&addr) && self.sio.write(addr - SIO_BASE, value) {
            self.trace_gpio(trace, now);
            return;
        }
        self.unmodelled.write(trace, now, core, addr, value);
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
            0x4 | 0x5 | 0xD => Some(mem::lane(self.unmodelled.peek(addr & !3), addr, size)),
            _ => None,
        }
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
        let mut sys = System::new(4096, None);
        sys.unmodelled.log_mode = crate::unmodelled::LogMode::Off;
        let mut t = NullSink;
        // UART0.UARTIBRD: byte write at offset +1 lands replicated in all lanes
        sys.write(&mut t, 0, Time::ZERO, UART0_BASE + 0x25, Size::Byte, 0xAB).unwrap();
        assert_eq!(sys.unmodelled.peek(UART0_BASE + 0x24), 0xABAB_ABAB);
        assert_eq!(sys.read(&mut t, 0, Time::ZERO, UART0_BASE + 0x26, Size::Half), Ok(0xABAB));
    }

    #[test]
    fn sio_gpio_trace() {
        let mut sys = System::new(4096, None);
        let sink = VecSink::new(&[Category::Gpio]);
        let mut t = sink.clone();
        sys.write(&mut t, 0, Time::ZERO, SIO_BASE + 0x24, Size::Word, 1 << 25).unwrap(); // OE_SET
        sys.write(&mut t, 0, Time::from_us(1), SIO_BASE + 0x14, Size::Word, 1 << 25).unwrap(); // OUT_SET
        sys.write(&mut t, 0, Time::from_us(2), SIO_BASE + 0x14, Size::Word, 1 << 25).unwrap(); // no change
        sys.write(&mut t, 0, Time::from_us(3), SIO_BASE + 0x1C, Size::Word, 1 << 25).unwrap(); // OUT_XOR
        let ev: Vec<String> = sink.take().iter().map(|(t, e)| format!("{t} {e}")).collect();
        assert_eq!(ev, vec!["0.000000s GPIO25 out = 0", "0.000001s GPIO25 out = 1", "0.000003s GPIO25 out = 0"]);
        assert_eq!(sys.read(&mut t, 1, Time::ZERO, SIO_BASE, Size::Word), Ok(1), "CPUID of core 1");
    }
}
