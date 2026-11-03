//! The RP2040 bus fabric: memories, peripheral dispatch and the per-core view the CPUs execute against.
//!
//! Peripheral registers are accessed a word at a time: narrow writes are replicated across byte lanes (as the APB
//! and AHB-lite bridges do) and narrow reads take their lane from the word. The XOR/SET/CLR aliases at
//! `+0x1000/+0x2000/+0x3000` reach each model as a [`WriteMode`]. Registers no model claims fall through to the
//! [`Unmodelled`] fallback, which names them from the SVD.

use sim_core::analog::{AnalogSource, Signal};
use sim_core::bus::{Bus, BusFault, Size, WriteMode, mem};
use sim_core::sched::Scheduler;
use sim_core::time::{ClockPeriod, Time};
use sim_core::trace::{Category, TraceEvent, TraceSink};
use sim_core::wire::External;
use sim_devices::DeviceCtx;
use sim_devices::i2c::I2cDevice;
use sim_devices::pin::PinDevice;
use sim_devices::spi::SpiDevice;
use sim_devices::uart::UartDevice;
use sim_devices::usb::host::HostController;

use crate::irq;
use crate::memmap::*;
use crate::periph::adc::{self, Adc};
use crate::periph::clocks::{Clocks, Freqs, Pll};
use crate::periph::dma::{self, Dma, DmaBus};
use crate::periph::gpio::{Drive, Gpio, func};
use crate::periph::i2c::{I2c, Step, StepResult};
use crate::periph::pio::{self, Pio};
use crate::periph::pwm::Pwm;
use crate::periph::sio::Sio;
use crate::periph::spi::Spi;
use crate::periph::ssi::{IoQspi, Ssi};
use crate::periph::sysctl::{self, Psm, Resets, Vreg, bit, psm};
use crate::periph::timer::{Timer, Watchdog, WatchdogReset};
use crate::periph::uart::Uart;
use crate::periph::usb::{self, Usb};
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
    /// SPI n finished a frame or its receive timeout is due.
    SpiFrame(u8),
    /// UART n finished shifting out a byte.
    UartTx(u8),
    /// UART n received a byte from the host, or its receive timeout expired.
    UartRx(u8),
    /// I2C n finished a bus step.
    I2cStep(u8),
    /// Advance the PIO blocks (while a state machine is running).
    Pio,
    /// A device drives a pin, or asked to be woken.
    Wire,
    /// An ADC conversion ends or starts.
    Adc,
    /// The USB host controller's next step (a transaction, SOF or port state change).
    Usb,
    /// Core 1 comes out of reset after its PSM force-off was released (handled by the machine).
    Core1Boot,
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
    pub uart: [Uart; 2],
    pub ssi: Ssi,
    pub io_qspi: IoQspi,
    /// Cached [`Ssi::xip_config`] and chip busy time, checked on every flash access.
    xip_config: Result<(), &'static str>,
    xip_busy_until: Time,
    /// An XIP failure was reported since the last time XIP worked.
    xip_failed: bool,
    pub spi: [Spi; 2],
    pub dma: Dma,
    /// Devices on the SPI buses.
    pub spi_devices: Vec<SpiAttachment>,
    /// Devices on the UARTs' TX/RX lines, and how much of each UART's output they have seen.
    pub uart_devices: [Option<Box<dyn UartDevice>>; 2],
    uart_device_seen: [usize; 2],
    pub i2c: [I2c; 2],
    /// Devices on the I2C buses.
    pub i2c_devices: Vec<I2cAttachment>,
    /// The device addressed by the transfer in progress on each I2C bus (index into `i2c_devices`).
    i2c_target: [Option<usize>; 2],
    pub adc: Adc,
    /// The USB controller and its DPRAM.
    pub usb: Usb,
    /// The USB host the chip's USB port is plugged into, if any (see [`System::attach_usb_host`]).
    pub usb_host: Option<Box<HostController>>,
    pub pio: [Pio; 2],
    /// Time base of the PIO cycle count: `clk_sys` cycle `.1` was at time `.0`.
    pio_base: (Time, u64),
    in_pio: bool,
    /// PIO IRQ lines after the last advance.
    pio_lines: u8,
    /// Pin drives and wake-ups devices scheduled for later.
    pub wire: WireQueue,
    /// Devices watching and driving pins.
    pub pin_devices: Vec<Box<dyn PinDevice>>,
    /// PWM slices whose output pins a pin device watches (their edges must be delivered).
    pub pwm_device_slices: u32,
    /// The Ethernet cable of an attached Ethernet controller, for the host to carry frames.
    pub ethernet: Option<sim_devices::w5500::EthWire>,
    /// What drives ADC inputs 0-3 (GPIO 26-29). Unconnected inputs read 0 V.
    pub analog_inputs: [Option<Box<dyn AnalogSource>>; 4],
    /// Die temperature in °C, for the temperature sensor (ADC input 4).
    pub chip_temp_c: Signal,
    /// ADC reference (the boards' 3.3 V rail).
    pub adc_vref: f64,
    clk_adc: ClockPeriod,
    /// Pulse-type DMA requests not yet consumed (PWM wraps), by DREQ number.
    pub dreq_pulses: [u32; 64],
    in_dma: bool,
    /// Current `clk_peri` period.
    pub clk_peri: ClockPeriod,
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
            uart: [Uart::new(0), Uart::new(1)],
            ssi: Ssi::default(),
            io_qspi: IoQspi::default(),
            xip_config: Err("the SSI is disabled"),
            xip_busy_until: Time::ZERO,
            xip_failed: false,
            spi: [Spi::new(0), Spi::new(1)],
            dma: Dma::default(),
            spi_devices: Vec::new(),
            uart_devices: [None, None],
            uart_device_seen: [0; 2],
            i2c: [I2c::new(0), I2c::new(1)],
            i2c_devices: Vec::new(),
            i2c_target: [None; 2],
            adc: Adc::default(),
            usb: Usb::default(),
            usb_host: None,
            pio: [Pio::new(0), Pio::new(1)],
            pio_base: (Time::ZERO, 0),
            in_pio: false,
            pio_lines: 0,
            wire: WireQueue::default(),
            pin_devices: Vec::new(),
            pwm_device_slices: 0,
            ethernet: None,
            analog_inputs: [None, None, None, None],
            chip_temp_c: Signal::Const(27.0),
            adc_vref: 3.3,
            clk_adc: ClockPeriod::from_hz(freqs.clk_adc),
            dreq_pulses: [0; 64],
            in_dma: false,
            clk_peri: ClockPeriod::from_hz(freqs.clk_peri),
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
        // The SSI and IO_QSPI reset; the flash chip is outside the RP2040 and keeps its state
        let chip = std::mem::take(&mut self.ssi.chip);
        self.ssi = Ssi::default();
        self.ssi.chip = chip;
        // BOOTSEL is a button on the board: it stays pressed through the reset
        let bootsel = self.io_qspi.bootsel_pressed;
        self.io_qspi = IoQspi::default();
        self.io_qspi.bootsel_pressed = bootsel;
        self.refresh_xip();
        self.sio = Sio::default();
        let observed = self.pwm.observed;
        self.pwm = Pwm { observed, ..Pwm::default() };
        for u in &mut self.uart {
            u.reset();
        }
        for s in &mut self.spi {
            s.reset();
        }
        for i in &mut self.i2c {
            i.reset();
        }
        self.i2c_target = [None; 2];
        self.adc = Adc::default();
        for p in &mut self.pio {
            p.reset();
        }
        self.dma = Dma::default();
        self.dreq_pulses = [0; 64];
        self.usb.reset();
        self.unmodelled.reset();
        self.sched = Scheduler::new();
        self.update_clocks(now);
        // The host notices the pull-up going away
        self.usb_kicked(now);
        self.irq_dirty = true;
        self.reset_request = None;
    }

    /// Recomputes the cached XIP state after the SSI, IO_QSPI or the flash chip changed.
    pub fn refresh_xip(&mut self) {
        self.xip_config = self.ssi.xip_config();
        self.xip_busy_until = self.ssi.chip.busy_until();
    }

    /// Leaves the SSI and flash as boot2 does, for direct boot.
    pub fn preset_xip(&mut self) {
        self.ssi.preset_boot2_xip();
        self.refresh_xip();
    }

    /// Whether a memory-mapped flash access at `now` works (see [`Ssi::xip_ready`]); reports the first failure.
    #[inline]
    fn xip_check(&mut self, now: Time, addr: u32) -> Result<(), BusFault> {
        if self.xip_config.is_ok() && now >= self.xip_busy_until {
            self.xip_failed = false;
            return Ok(());
        }
        // XIP_NOCACHE_NOALLOC and friends hit the same flash
        let reason = self.ssi.xip_ready(now).err().unwrap_or("");
        if reason.is_empty() {
            return Ok(());
        }
        if !self.xip_failed {
            self.xip_failed = true;
            log::warn!("[{now}] XIP access to {addr:#010x} while {reason}: bus fault");
        }
        Err(BusFault::Unmapped(addr))
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
        if self.uart[0].irq() {
            lines |= 1 << irq::UART0_IRQ;
        }
        if self.spi[0].irq() {
            lines |= 1 << irq::SPI0_IRQ;
        }
        if self.spi[1].irq() {
            lines |= 1 << irq::SPI1_IRQ;
        }
        if self.dma.irq_line(0) {
            lines |= 1 << irq::DMA_IRQ_0;
        }
        if self.dma.irq_line(1) {
            lines |= 1 << irq::DMA_IRQ_1;
        }
        if self.uart[1].irq() {
            lines |= 1 << irq::UART1_IRQ;
        }
        if self.adc.irq() {
            lines |= 1 << irq::ADC_IRQ_FIFO;
        }
        if self.usb.irq() {
            lines |= 1 << irq::USBCTRL_IRQ;
        }
        if self.i2c[0].irq() {
            lines |= 1 << irq::I2C0_IRQ;
        }
        if self.i2c[1].irq() {
            lines |= 1 << irq::I2C1_IRQ;
        }
        for (b, base) in [(0, irq::PIO0_IRQ_0), (1, irq::PIO1_IRQ_0)] {
            for n in 0..2 {
                if self.pio[b].irq_line(n) {
                    lines |= 1 << (base + n as u32);
                }
            }
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
                self.pio_advance(&mut sim_core::trace::NullSink, now);
                self.pio_base = (now, self.pio_cycle(now));
                self.pwm.rebase_all(&self.clk_sys, now);
                self.clk_sys = ClockPeriod::from_hz(new.clk_sys);
                self.clk_sys_changed = true;
                log::debug!("[{now}] clk_sys = {} Hz", new.clk_sys);
                self.reschedule_pwm(now);
            }
        }
        if new.clk_adc != old.clk_adc {
            self.adc_advance(&mut sim_core::trace::NullSink, now);
            self.clk_adc = ClockPeriod::from_hz(new.clk_adc);
            self.adc_advance(&mut sim_core::trace::NullSink, now);
        }
        if new.clk_peri != old.clk_peri {
            for n in 0..2 {
                self.uart_advance(n, now);
            }
            self.clk_peri = ClockPeriod::from_hz(new.clk_peri);
        }
        if new.clk_usb != old.clk_usb {
            self.usb.clk_ok = usb::clk_usb_ok(new.clk_usb);
            if new.clk_usb != 0 && !self.usb.clk_ok {
                log::warn!("[{now}] clk_usb = {} Hz: USB full speed needs 48 MHz", new.clk_usb);
            }
        }
        self.freqs = new;
    }

    /// Advances UART `n` to `now` and schedules its next byte boundary.
    fn uart_advance(&mut self, n: usize, now: Time) {
        let s = self.uart[n].advance(&self.clk_peri, now);
        if self.uart_devices[n].is_some() {
            self.uart_feed_device(n, now);
        }
        match s.tx_done {
            Some(t) => self.sched.schedule(Event::UartTx(n as u8), t),
            None => {
                self.sched.cancel(Event::UartTx(n as u8));
            }
        }
        match s.rx_next {
            Some(t) => self.sched.schedule(Event::UartRx(n as u8), t),
            None => {
                self.sched.cancel(Event::UartRx(n as u8));
            }
        }
        self.irq_dirty = true;
    }

    /// Hands bytes UART `n` transmitted since the last call to its device and queues the device's answers.
    fn uart_feed_device(&mut self, n: usize, now: Time) {
        let out = &self.uart[n].output;
        if out.len() < self.uart_device_seen[n] {
            // The host drained the output buffer
            self.uart_device_seen[n] = 0;
        }
        let new: Vec<u8> = out[self.uart_device_seen[n]..].iter().map(|(_, b)| *b).collect();
        self.uart_device_seen[n] = out.len();
        if new.is_empty() {
            return;
        }
        let mut replies = Vec::new();
        if let Some(dev) = &mut self.uart_devices[n] {
            let mut ctx =
                DevPins { now, gpio: &mut self.gpio, freqs: &self.freqs, changed: false, wire: &mut self.wire };
            for b in new {
                replies.extend(dev.receive(&mut ctx, b));
            }
        }
        if !replies.is_empty() {
            self.uart[n].inject(&replies);
            let s = self.uart[n].advance(&self.clk_peri, now);
            if let Some(t) = s.rx_next {
                self.sched.schedule(Event::UartRx(n as u8), t);
            }
        }
    }

    /// Attaches a device to UART `n`'s TX and RX lines.
    pub fn attach_uart(&mut self, n: usize, dev: Box<dyn UartDevice>) {
        self.uart_devices[n] = Some(dev);
        self.uart_device_seen[n] = self.uart[n].output.len();
    }

    /// Queues bytes for UART `n` to receive (from a host pty, a test or a device).
    pub fn uart_inject(&mut self, n: usize, bytes: &[u8], now: Time) {
        self.uart[n].inject(bytes);
        self.uart_advance(n, now);
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
                bit::SPI0 | bit::SPI1 => {
                    let i = (n - bit::SPI0) as usize;
                    self.spi[i].reset();
                    self.sched.cancel(Event::SpiFrame(i as u8));
                }
                bit::DMA => self.dma = Dma::default(),
                bit::I2C0 | bit::I2C1 => {
                    let i = (n - bit::I2C0) as usize;
                    self.i2c[i].reset();
                    self.i2c_target[i] = None;
                    self.sched.cancel(Event::I2cStep(i as u8));
                }
                bit::ADC => {
                    self.adc = Adc::default();
                    self.sched.cancel(Event::Adc);
                }
                bit::PIO0 | bit::PIO1 => {
                    let i = (n - bit::PIO0) as usize;
                    self.pio[i].reset();
                }
                bit::UART0 | bit::UART1 => {
                    let i = (n - bit::UART0) as usize;
                    self.uart[i].reset();
                    self.sched.cancel(Event::UartTx(i as u8));
                    self.sched.cancel(Event::UartRx(i as u8));
                }
                bit::USBCTRL => {
                    self.usb.reset();
                    self.usb_kicked(now);
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
        let spis = &self.spi;
        let pios = &self.pio;
        let clk = self.clk_sys;
        let changed = self.gpio.evaluate(|f, pin| match f {
            func::SIO => Drive { out: sio_out & (1 << pin) != 0, oe: sio_oe & (1 << pin) != 0 },
            func::PWM if pwm.drives_pin(pin) => Drive { out: pwm.pin_output(&clk, now, pin), oe: true },
            func::PIO0 | func::PIO1 => {
                let p = &pios[(f == func::PIO1) as usize];
                Drive { out: p.pad_out & (1 << pin) != 0, oe: p.pad_oe & (1 << pin) != 0 }
            }
            // SPI: SCK idles at CPOL, TX low, the hardware chip select is low while a frame is on the wire
            func::SPI => {
                let spi = &spis[(pin >> 3) & 1];
                match pin & 3 {
                    1 => Drive { out: !spi.busy(), oe: true },
                    2 | 3 => Drive { out: false, oe: true },
                    _ => Drive::default(),
                }
            }
            // UART TX pins idle high (the bit stream itself is not put on the pin)
            func::UART if pin % 4 == 0 => Drive { out: true, oe: true },
            // GPOUT clock outputs on 21, 23, 24, 25 drive the pin (the waveform itself is not simulated)
            func::CLOCK if matches!(pin, 21 | 23 | 24 | 25) => Drive { out: false, oe: true },
            _ => Drive::default(),
        });
        if changed != 0 {
            self.irq_dirty = true;
            if self.pwm.input_slices() != 0 {
                self.update_pwm_inputs(now);
            }
            if !self.pin_devices.is_empty() {
                self.notify_pin_devices(trace, changed, now);
            }
            if trace.wants(Category::Gpio) {
                for pin in 0..NUM_GPIOS as u8 {
                    if changed & (1 << pin) != 0 {
                        trace.event(now, TraceEvent::Gpio { pin, level: self.gpio.level & (1 << pin) != 0 });
                    }
                }
            }
            self.update_chip_selects(trace, now);
        }
    }

    /// Tells pin-level devices about level changes on the pins they watch.
    fn notify_pin_devices(&mut self, trace: &mut dyn TraceSink, changed: u32, now: Time) {
        let level = self.gpio.level;
        let mut immediate = false;
        for dev in &mut self.pin_devices {
            for pin in 0..NUM_GPIOS as u8 {
                if changed & (1 << pin) != 0 && dev.watches(pin) {
                    let mut ctx =
                        DevPins { now, gpio: &mut self.gpio, freqs: &self.freqs, changed: false, wire: &mut self.wire };
                    dev.pin_changed(&mut ctx, pin, level & (1 << pin) != 0);
                    immediate |= ctx.changed;
                }
            }
        }
        self.schedule_wire();
        self.flush_reports(trace);
        if immediate {
            self.update_pins(trace, now);
        }
    }

    /// Sends device reports to the trace (and the log).
    fn flush_reports(&mut self, trace: &mut dyn TraceSink) {
        for (t, m) in self.wire.reports.drain(..) {
            log::debug!("[{t}] {m}");
            if trace.wants(Category::Device) {
                trace.event(t, TraceEvent::Device { source: "pin".into(), message: m });
            }
        }
    }

    fn schedule_wire(&mut self) {
        match self.wire.next_time() {
            Some(t) => self.sched.schedule(Event::Wire, t),
            None => {
                self.sched.cancel(Event::Wire);
            }
        }
    }

    /// Applies the drives and wake-ups due by `now`.
    fn apply_wire(&mut self, trace: &mut dyn TraceSink, now: Time) {
        while let Some((t, item)) = self.wire.pop_due(now) {
            match item {
                WireItem::Drive(pin, value) => {
                    self.gpio.external[pin as usize] = value;
                    self.update_pins(trace, t);
                }
                WireItem::Wake => {
                    let mut immediate = false;
                    for dev in &mut self.pin_devices {
                        let mut ctx = DevPins {
                            now: t,
                            gpio: &mut self.gpio,
                            freqs: &self.freqs,
                            changed: false,
                            wire: &mut self.wire,
                        };
                        dev.update(&mut ctx);
                        immediate |= ctx.changed;
                    }
                    if immediate {
                        self.update_pins(trace, t);
                    }
                    self.flush_reports(trace);
                }
            }
        }
        self.schedule_wire();
    }

    /// Attaches a pin-level device.
    pub fn attach_pin_device(&mut self, dev: Box<dyn PinDevice>) {
        for pin in 0..30u8 {
            if dev.watches(pin) {
                self.pwm_device_slices |= 1 << ((pin as u32 / 2) % 8);
            }
        }
        self.pwm.observed |= self.pwm_device_slices;
        self.pin_devices.push(dev);
    }

    /// The first pin device model of type `T`.
    pub fn pin_device<T: 'static>(&self) -> Option<&T> {
        self.pin_devices.iter().find_map(|d| d.as_any().downcast_ref::<T>())
    }

    /// The `clk_sys` cycle at `t` on the PIO time base.
    fn pio_cycle(&self, t: Time) -> u64 {
        if t <= self.pio_base.0 {
            return self.pio_base.1;
        }
        self.pio_base.1 + self.clk_sys.cycles_in(t - self.pio_base.0)
    }

    fn pio_time(&self, c: u64) -> Time {
        self.pio_base.0 + self.clk_sys.cycles(c.saturating_sub(self.pio_base.1))
    }

    /// Runs both PIO blocks up to `now`: pin output changes are resolved (and reach pin devices) at the cycle they
    /// happen, and device drives due meanwhile are applied on time.
    pub fn pio_advance(&mut self, trace: &mut dyn TraceSink, now: Time) {
        if self.in_pio {
            return;
        }
        self.in_pio = true;
        let target = self.pio_cycle(now);
        // When both blocks run, neither gets more than this many cycles ahead of the other
        const MAX_SKEW: u64 = 32;
        loop {
            let wire_t = self.wire.next_time().filter(|&t| t <= now);
            let seg_end = wire_t.map_or(target, |t| self.pio_cycle(t).min(target));
            let both = self.pio[0].busy() && self.pio[1].busy();
            let b = (self.pio[1].cycle < self.pio[0].cycle) as usize;
            if self.pio[b].cycle < seg_end {
                let limit = if both { seg_end.min(self.pio[1 - b].cycle + MAX_SKEW) } else { seg_end };
                let inputs = self.gpio.inputs();
                if self.pio[b].run(limit, inputs) == pio::Stop::Outputs {
                    let t = self.pio_time(self.pio[b].cycle);
                    self.update_pins(trace, t);
                }
                continue;
            }
            if self.pio[1 - b].cycle < seg_end {
                let inputs = self.gpio.inputs();
                if self.pio[1 - b].run(seg_end, inputs) == pio::Stop::Outputs {
                    let t = self.pio_time(self.pio[1 - b].cycle);
                    self.update_pins(trace, t);
                }
                continue;
            }
            match wire_t {
                Some(t) if self.pio_cycle(t) <= seg_end => self.apply_wire(trace, t),
                _ => break,
            }
        }
        self.in_pio = false;
        let lines = self.pio.iter().flat_map(|p| [p.irq_line(0), p.irq_line(1)]).fold(0u8, |a, l| a << 1 | l as u8);
        if lines != self.pio_lines {
            self.pio_lines = lines;
            self.irq_dirty = true;
        }
        if self.pio.iter().any(|p| p.busy()) {
            self.sched.schedule(Event::Pio, now + Time::from_us(1));
        } else {
            self.sched.cancel(Event::Pio);
        }
    }

    /// Reports B pin levels to PWM slices counting on them.
    fn update_pwm_inputs(&mut self, now: Time) {
        let slices = self.pwm.input_slices();
        if slices == 0 {
            return;
        }
        let inputs = self.gpio.inputs();
        for slice in 0..8 {
            if slices & (1 << slice) == 0 {
                continue;
            }
            // B of slice n is on GPIO 2n+1 and 2n+17
            let pin = [2 * slice + 1, 2 * slice + 17]
                .into_iter()
                .find(|&p| p < NUM_GPIOS && self.gpio.funcsel(p) == func::PWM);
            if let Some(p) = pin {
                let clk = self.clk_sys;
                self.pwm.set_input(&clk, now, slice, inputs & (1 << p) != 0);
            }
        }
        self.reschedule_pwm(now);
    }

    /// Tells SPI devices about chip select changes; devices may drive pins in response.
    fn update_chip_selects(&mut self, trace: &mut dyn TraceSink, now: Time) {
        let mut pins_changed = false;
        for att in &mut self.spi_devices {
            let level = self.gpio.level & (1 << att.cs_pin) != 0;
            let selected = level == att.cs_active_high;
            if selected != att.selected {
                att.selected = selected;
                let mut ctx =
                    DevPins { now, gpio: &mut self.gpio, freqs: &self.freqs, changed: false, wire: &mut self.wire };
                att.dev.select(&mut ctx, selected);
                pins_changed |= ctx.changed;
            }
        }
        if pins_changed {
            self.update_pins(trace, now);
        }
    }

    /// Lets every device update its outputs (after time passed or a CAN frame reached it).
    pub fn update_devices(&mut self, trace: &mut dyn TraceSink, now: Time) {
        let mut pins_changed = false;
        for att in &mut self.spi_devices {
            let mut ctx =
                DevPins { now, gpio: &mut self.gpio, freqs: &self.freqs, changed: false, wire: &mut self.wire };
            att.dev.update(&mut ctx);
            pins_changed |= ctx.changed;
        }
        for att in &mut self.i2c_devices {
            let mut ctx =
                DevPins { now, gpio: &mut self.gpio, freqs: &self.freqs, changed: false, wire: &mut self.wire };
            att.dev.update(&mut ctx);
            pins_changed |= ctx.changed;
        }
        if pins_changed {
            self.update_pins(trace, now);
        }
    }

    /// Attaches an SPI device to bus `bus` with its chip select on GPIO `cs_pin` (active low).
    pub fn attach_spi(&mut self, bus: usize, cs_pin: u8, dev: Box<dyn SpiDevice>) {
        self.spi_devices.push(SpiAttachment { bus, cs_pin, cs_active_high: false, selected: false, dev });
    }

    /// Attaches an I2C device to bus `bus` (I2C0 or I2C1).
    pub fn attach_i2c(&mut self, bus: usize, dev: Box<dyn I2cDevice>) {
        self.i2c_devices.push(I2cAttachment { bus, dev });
    }

    /// The first I2C device model of type `T`.
    pub fn i2c_device<T: 'static>(&self) -> Option<&T> {
        self.i2c_devices.iter().find_map(|a| a.dev.as_any().downcast_ref::<T>())
    }

    /// Connects ADC input `input` (0-3, GPIO 26-29) to an analog source.
    pub fn attach_analog(&mut self, input: usize, source: Box<dyn AnalogSource>) {
        self.analog_inputs[input] = Some(source);
    }

    /// Whether I2C `n` has both SDA and SCL routed to pins.
    fn i2c_pins_connected(&self, n: usize) -> bool {
        let mut sda = false;
        let mut scl = false;
        for p in 0..NUM_GPIOS {
            if self.gpio.funcsel(p) == func::I2C && (p >> 1) & 1 == n {
                if p % 2 == 0 {
                    sda = true;
                } else {
                    scl = true;
                }
            }
        }
        sda && scl
    }

    /// Runs I2C `n` up to `now`: finishes the step on the wire, puts the next commands on the bus (performing them
    /// on the devices) and schedules the next step end.
    fn i2c_advance(&mut self, trace: &mut dyn TraceSink, n: usize, now: Time) {
        for _ in 0..64 {
            let mut progressed = false;
            if self.i2c[n].finish(now) {
                progressed = true;
            }
            if !self.i2c[n].busy()
                && let Some(step) = self.i2c[n].next_step()
            {
                let at = self.i2c[n].next_event().unwrap_or(now).max(now);
                let result = self.i2c_perform(trace, n, step, at);
                let clk = self.clk_sys;
                self.i2c[n].begin(step, result, &clk, at);
                progressed = true;
            }
            if self.service_dma(trace, now) {
                progressed = true;
            }
            if !progressed || self.i2c[n].next_event().is_some_and(|t| t > now) {
                break;
            }
        }
        match self.i2c[n].next_event() {
            Some(t) => self.sched.schedule(Event::I2cStep(n as u8), t.max(now)),
            None => {
                self.sched.cancel(Event::I2cStep(n as u8));
            }
        }
        self.irq_dirty = true;
    }

    /// Performs one bus step on the devices of I2C `n`.
    fn i2c_perform(&mut self, trace: &mut dyn TraceSink, n: usize, step: Step, at: Time) -> StepResult {
        let mut r = StepResult { addr_ack: true, data_ack: true, read: 0xFF, stretch: Time::ZERO };
        let connected = self.i2c_pins_connected(n);
        let mut ctx =
            DevPins { now: at, gpio: &mut self.gpio, freqs: &self.freqs, changed: false, wire: &mut self.wire };
        if let Some((addr, read)) = step.start {
            let target = if connected {
                self.i2c_devices.iter().position(|a| a.bus == n && a.dev.responds_to(addr))
            } else {
                None
            };
            self.i2c_target[n] = target;
            r.addr_ack = match target {
                Some(i) => self.i2c_devices[i].dev.start(&mut ctx, addr, read),
                None => false,
            };
            if trace.wants(Category::Device) {
                trace.event(
                    at,
                    TraceEvent::Device {
                        source: format!("i2c{n}"),
                        message: format!(
                            "{} {addr:#04x} {}",
                            if step.restart { "Sr" } else { "S" },
                            if read { "R" } else { "W" }
                        ) + if r.addr_ack { "" } else { " NAK" },
                    },
                );
            }
            if !r.addr_ack {
                if let Some(i) = target {
                    self.i2c_devices[i].dev.stop(&mut ctx);
                }
                self.i2c_target[n] = None;
                let changed = ctx.changed;
                if changed {
                    self.update_pins(trace, at);
                }
                return r;
            }
        }
        match self.i2c_target[n] {
            Some(i) => {
                let dev = &mut self.i2c_devices[i].dev;
                r.stretch = dev.stretch(&mut ctx);
                match step.write {
                    Some(b) => r.data_ack = dev.write(&mut ctx, b),
                    None => r.read = dev.read(&mut ctx, step.ack),
                }
                if step.stop || (step.write.is_some() && !r.data_ack) {
                    dev.stop(&mut ctx);
                    self.i2c_target[n] = None;
                }
            }
            None => r.data_ack = false,
        }
        let changed = ctx.changed;
        if changed {
            self.update_pins(trace, at);
        }
        r
    }

    /// Voltage on ADC input `input` at `t`.
    fn analog_voltage(&mut self, input: u8, t: Time) -> f64 {
        match input {
            0..=3 => {
                let pins = self.gpio.level;
                self.analog_inputs[input as usize].as_mut().map_or(0.0, |s| s.voltage(t, pins))
            }
            4 if self.adc.temp_sensor_enabled() => adc::temp_sensor_volts(self.chip_temp_c.at(t)),
            _ => 0.0,
        }
    }

    /// Runs the ADC up to `now`: finishes conversions (sampling their inputs), starts new ones, lets the DMA drain
    /// the FIFO, and schedules the next conversion.
    fn adc_advance(&mut self, trace: &mut dyn TraceSink, now: Time) {
        let clk = self.clk_adc;
        for _ in 0..10_000 {
            self.adc.advance(&clk, now);
            let Some((input, at)) = self.adc.take_due(now) else { break };
            let v = self.analog_voltage(input, at);
            let code = adc::code_for(v, self.adc_vref);
            self.adc.complete(code, &clk, at);
            if self.adc.dreq() {
                self.service_dma(trace, at);
            }
        }
        match self.adc.next_event().filter(|_| !clk.is_stopped()) {
            Some(t) => self.sched.schedule(Event::Adc, t.max(now)),
            None => {
                self.sched.cancel(Event::Adc);
            }
        }
        self.irq_dirty = true;
    }

    /// Runs SPI `n` up to `now`: completes frames (exchanging them with the selected devices), starts the next
    /// ones, lets the DMA move data, and schedules the next frame end.
    fn spi_advance(&mut self, trace: &mut dyn TraceSink, n: usize, now: Time) {
        let was_busy = self.spi[n].busy();
        for _ in 0..64 {
            let mut progressed = false;
            while let Some((frame, at)) = self.spi[n].take_completed(now) {
                let reply = if self.spi[n].loopback() { frame } else { self.spi_exchange(trace, n, frame, at) };
                let clk = self.clk_peri;
                self.spi[n].complete(reply, at, &clk);
                self.spi[n].start(&clk, at);
                progressed = true;
            }
            self.spi[n].check_timeout(now);
            let clk = self.clk_peri;
            self.spi[n].start(&clk, now);
            if self.service_dma(trace, now) {
                progressed = true;
            }
            if !progressed {
                break;
            }
        }
        match self.spi[n].next_event() {
            Some(t) => self.sched.schedule(Event::SpiFrame(n as u8), t.max(now)),
            None => {
                self.sched.cancel(Event::SpiFrame(n as u8));
            }
        }
        self.irq_dirty = true;
        if was_busy != self.spi[n].busy() && self.uses_hardware_cs(n) {
            self.update_pins(trace, now);
        }
    }

    fn uses_hardware_cs(&self, n: usize) -> bool {
        (0..NUM_GPIOS).any(|p| self.gpio.funcsel(p) == func::SPI && p & 3 == 1 && (p >> 3) & 1 == n)
    }

    /// Exchanges one frame with the devices selected on bus `n`. MISO reads 0 (pulled down) with nobody selected.
    fn spi_exchange(&mut self, trace: &mut dyn TraceSink, n: usize, frame: u16, at: Time) -> u16 {
        let bits = self.spi[n].bits();
        let mut reply = 0;
        let mut pins_changed = false;
        for att in self.spi_devices.iter_mut().filter(|a| a.bus == n && a.selected) {
            let mut ctx =
                DevPins { now: at, gpio: &mut self.gpio, freqs: &self.freqs, changed: false, wire: &mut self.wire };
            reply = att.dev.transfer(&mut ctx, frame, bits);
            pins_changed |= ctx.changed;
        }
        if pins_changed {
            self.update_pins(trace, at);
        }
        reply
    }

    /// Lets the DMA move whatever it can. Returns whether anything moved.
    pub fn service_dma(&mut self, trace: &mut dyn TraceSink, now: Time) -> bool {
        if self.in_dma || !self.dma.any_busy() {
            return false;
        }
        self.in_dma = true;
        let mut dma = std::mem::take(&mut self.dma);
        let moved = dma.service(&mut DmaPort { sys: self, trace, now });
        self.dma = dma;
        self.in_dma = false;
        if moved {
            self.irq_dirty = true;
            // DMA writes may have filled FIFOs that now need to start shifting
            for n in 0..2 {
                if self.spi[n].busy() && !self.sched.is_pending(Event::SpiFrame(n as u8)) {
                    let clk = self.clk_peri;
                    if let Some(t) = self.spi[n].start(&clk, now) {
                        self.sched.schedule(Event::SpiFrame(n as u8), t);
                    }
                }
            }
        }
        moved
    }

    fn dreq(&self, n: u32) -> bool {
        match n {
            dma::dreq::SPI0_TX => self.spi[0].tx_dreq(),
            dma::dreq::SPI0_RX => self.spi[0].rx_dreq(),
            dma::dreq::SPI1_TX => self.spi[1].tx_dreq(),
            dma::dreq::SPI1_RX => self.spi[1].rx_dreq(),
            dma::dreq::UART0_TX => self.uart[0].tx_dreq(),
            dma::dreq::UART0_RX => self.uart[0].rx_dreq(),
            dma::dreq::UART1_TX => self.uart[1].tx_dreq(),
            dma::dreq::UART1_RX => self.uart[1].rx_dreq(),
            dma::dreq::I2C0_TX => self.i2c[0].tx_dreq(),
            dma::dreq::I2C0_RX => self.i2c[0].rx_dreq(),
            dma::dreq::I2C1_TX => self.i2c[1].tx_dreq(),
            dma::dreq::I2C1_RX => self.i2c[1].rx_dreq(),
            dma::dreq::ADC => self.adc.dreq(),
            0..=3 => self.pio[0].tx_dreq(n as usize),
            4..=7 => self.pio[0].rx_dreq(n as usize - 4),
            8..=11 => self.pio[1].tx_dreq(n as usize - 8),
            12..=15 => self.pio[1].rx_dreq(n as usize - 12),
            24..=31 => self.dreq_pulses[n as usize] > 0,
            _ => false,
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
            Event::UartTx(n) | Event::UartRx(n) => {
                self.uart_advance(n as usize, t);
                self.service_dma(trace, t);
            }
            Event::SpiFrame(n) => self.spi_advance(trace, n as usize, t),
            Event::I2cStep(n) => self.i2c_advance(trace, n as usize, t),
            Event::Pio => {
                self.pio_advance(trace, t);
                self.service_dma(trace, t);
            }
            Event::Wire => {
                self.pio_advance(trace, t);
                self.apply_wire(trace, t);
            }
            Event::Adc => self.adc_advance(trace, t),
            Event::Usb => self.usb_poll(trace, t),
            Event::Core1Boot | Event::External(_) => {}
        }
    }

    // ------------------------------------------------------------------ USB

    /// Plugs the chip's USB port into a host (VBUS comes up; the host enumerates the device once its pull-up
    /// appears).
    pub fn attach_usb_host(&mut self, host: HostController, now: Time) {
        self.usb.set_vbus(true);
        self.usb_host = Some(Box::new(host));
        self.usb.kick = true;
        self.usb_kicked(now);
    }

    /// Pulls the USB cable: VBUS drops, the device is disconnected, and the host (returned) is gone.
    pub fn detach_usb_host(&mut self) -> Option<HostController> {
        self.usb.set_vbus(false);
        self.sched.cancel(Event::Usb);
        self.irq_dirty = true;
        self.usb_host.take().map(|h| *h)
    }

    /// The attached USB host, for queueing transfers. Call [`System::usb_reschedule`] after changing it.
    pub fn usb_host_mut(&mut self) -> Option<&mut HostController> {
        self.usb_host.as_deref_mut()
    }

    /// Runs `f` on the attached USB host and lets it pick up whatever `f` queued.
    pub fn with_usb_host<R>(&mut self, now: Time, f: impl FnOnce(&mut HostController) -> R) -> Option<R> {
        let r = f(self.usb_host.as_deref_mut()?);
        self.usb_reschedule(now);
        Some(r)
    }

    /// Tells the host that something changed (transfers queued from outside, firmware armed a buffer).
    pub fn usb_reschedule(&mut self, now: Time) {
        self.usb.kick = true;
        self.usb_kicked(now);
    }

    fn usb_line_state(&self) -> u32 {
        self.usb.line_state(self.gpio.inputs(), |p| self.gpio.funcsel(p))
    }

    /// The controller asked for the host's attention: let it retry NAKed pipes and see the pull-up.
    fn usb_kicked(&mut self, now: Time) {
        if !std::mem::take(&mut self.usb.kick) {
            return;
        }
        let Some(host) = self.usb_host.as_deref_mut() else { return };
        if let Some(t) = host.kick(now, &mut self.usb) {
            let t = t.max(now);
            if self.sched.due(Event::Usb).is_none_or(|d| d > t) {
                self.sched.schedule(Event::Usb, t);
            }
        }
        self.irq_dirty = true;
    }

    /// The host's scheduled step.
    fn usb_poll(&mut self, trace: &mut dyn TraceSink, now: Time) {
        let Some(host) = self.usb_host.as_deref_mut() else { return };
        let next = host.poll(now, &mut self.usb);
        self.usb.kick = false;
        match next {
            Some(t) => self.sched.schedule(Event::Usb, t.max(now)),
            None => {
                self.sched.cancel(Event::Usb);
            }
        }
        self.irq_dirty = true;
        let mut reports = host.take_reports();
        reports.append(&mut self.usb.reports);
        for (t, m) in reports {
            if trace.wants(Category::Device) {
                trace.event(t, TraceEvent::Device { source: "usb".into(), message: m });
            }
        }
    }

    // ------------------------------------------------------------------ word-level peripheral access

    fn in_reset(&self, base: u32) -> bool {
        base_reset_bit(base).is_some_and(|b| self.resets.in_reset(b))
    }

    fn periph_read(&mut self, trace: &mut dyn TraceSink, core: u8, now: Time, addr: u32) -> u32 {
        if (SIO_BASE..SIO_BASE + 0x1000).contains(&addr) {
            if addr - SIO_BASE < 0x50 {
                self.pio_advance(trace, now);
            }
            let gpio_in = self.gpio.inputs();
            if let Some(v) = self.sio.read(core, addr - SIO_BASE, gpio_in, self.io_qspi.levels()) {
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
            XIP_SSI_BASE => {
                let v = self.ssi.read(off, now, &mut self.flash);
                self.refresh_xip();
                v
            }
            IO_QSPI_BASE => self.io_qspi.read(off),
            SPI0_BASE | SPI1_BASE => {
                let n = (base == SPI1_BASE) as usize;
                let v = self.spi[n].read(off);
                self.spi_advance(trace, n, now);
                v
            }
            DMA_BASE => self.dma.read(off),
            PIO0_BASE | PIO1_BASE => {
                self.pio_advance(trace, now);
                let v = self.pio[(base == PIO1_BASE) as usize].read(off);
                self.irq_dirty = true;
                self.pio_advance(trace, now);
                v
            }
            I2C0_BASE | I2C1_BASE => {
                let n = (base == I2C1_BASE) as usize;
                self.i2c_advance(trace, n, now);
                let v = self.i2c[n].read(off);
                self.i2c_advance(trace, n, now);
                v
            }
            ADC_BASE => {
                self.adc_advance(trace, now);
                let v = self.adc.read(off);
                self.irq_dirty = true;
                v
            }
            UART0_BASE | UART1_BASE => {
                let n = (base == UART1_BASE) as usize;
                let v = self.uart[n].read(&self.clk_peri, now, off);
                self.uart_advance(n, now);
                v
            }
            USBCTRL_REGS_BASE => {
                let ls = self.usb_line_state();
                let v = self.usb.read(off, ls);
                self.irq_dirty = true;
                v
            }
            _ => None,
        };
        match value {
            Some(v) => v,
            None => self.unmodelled.read(trace, now, core, addr),
        }
    }

    fn periph_write(&mut self, trace: &mut dyn TraceSink, core: u8, now: Time, addr: u32, value: u32) {
        if (SIO_BASE..SIO_BASE + 0x1000).contains(&addr) {
            if addr - SIO_BASE < 0x50 {
                self.pio_advance(trace, now);
            }
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
            if self.warned_in_reset & (1 << b) == 0 && b != bit::SYSCFG {
                // (The bootrom writes SYSCFG's NMI mask before releasing it; that one is expected)
                self.warned_in_reset |= 1 << b;
                log::warn!(
                    "[{now}] core{core} write to {} while the block is held in RESETS; ignored",
                    crate::svd::describe(addr)
                );
            }
            return;
        }
        if matches!(base, IO_BANK0_BASE | PADS_BANK0_BASE) {
            self.pio_advance(trace, now);
        }
        let handled = match base {
            PIO0_BASE | PIO1_BASE => {
                self.pio_advance(trace, now);
                let b = (base == PIO1_BASE) as usize;
                let inputs = self.gpio.inputs();
                let ok = self.pio[b].write(off, value, mode, inputs);
                self.irq_dirty = true;
                if self.pio[b].take_outputs_changed() {
                    self.update_pins(trace, now);
                }
                // The write may have unstalled a state machine: run it (and schedule the next advance)
                self.pio_advance(trace, now);
                self.service_dma(trace, now);
                ok.then_some(())
            }
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
                self.update_pwm_inputs(now);
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
            XIP_SSI_BASE => {
                let ok = self.ssi.write(off, value, WriteMode::Normal, now, &mut self.flash);
                self.refresh_xip();
                ok.then_some(())
            }
            IO_QSPI_BASE => {
                let ok = self.io_qspi.write(off, value, mode);
                let o = self.io_qspi.cs_override();
                self.ssi.set_cs_override(o, now, &mut self.flash);
                self.refresh_xip();
                ok.then_some(())
            }
            SPI0_BASE | SPI1_BASE => {
                let n = (base == SPI1_BASE) as usize;
                let ok = self.spi[n].write(off, value, mode);
                self.spi_advance(trace, n, now);
                if off == crate::periph::spi::SSPCR1 {
                    self.update_pins(trace, now);
                }
                ok.then_some(())
            }
            DMA_BASE => {
                let ok = self.dma.write(off, value, mode);
                self.irq_dirty = true;
                self.service_dma(trace, now);
                ok.then_some(())
            }
            I2C0_BASE | I2C1_BASE => {
                let n = (base == I2C1_BASE) as usize;
                self.i2c_advance(trace, n, now);
                let ok = self.i2c[n].write(off, value, mode);
                if off == crate::periph::i2c::IC_ENABLE && !self.i2c[n].holds_bus() {
                    self.i2c_target[n] = None;
                }
                self.i2c_advance(trace, n, now);
                ok.then_some(())
            }
            ADC_BASE => {
                self.adc_advance(trace, now);
                let clk = self.clk_adc;
                let ok = self.adc.write(&clk, now, off, value, mode);
                self.adc_advance(trace, now);
                ok.then_some(())
            }
            UART0_BASE | UART1_BASE => {
                let n = (base == UART1_BASE) as usize;
                let ok = self.uart[n].write(&self.clk_peri, now, off, value, mode);
                self.uart_advance(n, now);
                ok.then_some(())
            }
            USBCTRL_REGS_BASE => {
                let ok = self.usb.write(off, value, mode);
                self.irq_dirty = true;
                self.usb_kicked(now);
                ok.then_some(())
            }
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
                0x10..=0x13 => {
                    self.xip_check(now, addr)?;
                    Ok(mem::read(&self.flash, self.flash_offset(addr), size))
                }
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
            // The USB DPRAM is RAM on the AHB-lite bus: any access width, no atomic aliases
            0x5 if addr >> 16 == USBCTRL_DPRAM_BASE >> 16 => Ok(self.usb.dpram_read((addr & 0xFFF) as usize, size)),
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
            0x5 if addr >> 16 == USBCTRL_DPRAM_BASE >> 16 => {
                self.usb.dpram_write((addr & 0xFFF) as usize, size, value);
                if self.usb.kick {
                    self.usb_kicked(now);
                }
                Ok(())
            }
            0x4 | 0x5 | 0xD => {
                self.periph_write(trace, core, now, addr & !3, mem::replicate(value, size));
                Ok(())
            }
            _ => Err(BusFault::Unmapped(addr)),
        }
    }

    /// Side-effect free read (memory and peripheral registers), as core 0 sees it.
    pub fn peek(&self, addr: u32, size: Size) -> Option<u32> {
        self.peek_core(0, addr, size)
    }

    /// Side-effect free read as `core` sees it (SIO is per core).
    pub fn peek_core(&self, core: u8, addr: u32, size: Size) -> Option<u32> {
        match addr >> 28 {
            0x0 if (addr as usize) < ROM_SIZE => Some(mem::read(&self.rom, addr as usize, size)),
            0x1 if (0x10..=0x13).contains(&(addr >> 24)) => Some(mem::read(&self.flash, self.flash_offset(addr), size)),
            0x1 if addr >> 24 == 0x15 && ((addr & 0x00FF_FFFF) as usize) < XIP_SRAM_SIZE => {
                Some(mem::read(&self.xip_sram, (addr & 0x3FFF) as usize, size))
            }
            0x2 => sram_offset(addr).map(|o| mem::read(&self.sram, o, size)),
            0x5 if addr >> 16 == USBCTRL_DPRAM_BASE >> 16 => Some(self.usb.dpram_read((addr & 0xFFF) as usize, size)),
            0x4 | 0x5 | 0xD | 0x1 => Some(mem::lane(self.peek_periph(core, addr & !3), addr, size)),
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
            SPI0_BASE | SPI1_BASE => self.spi[(base == SPI1_BASE) as usize].peek(off),
            I2C0_BASE | I2C1_BASE => self.i2c[(base == I2C1_BASE) as usize].peek(off),
            ADC_BASE => self.adc.peek(off),
            PIO0_BASE | PIO1_BASE => self.pio[(base == PIO1_BASE) as usize].peek(off),
            DMA_BASE => self.dma.read(off),
            WATCHDOG_BASE => self.watchdog.read(now, off),
            VREG_AND_CHIP_RESET_BASE => self.vreg.read(off),
            USBCTRL_REGS_BASE => self.usb.peek(off, self.usb_line_state()),
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
            0x5 if addr >> 16 == USBCTRL_DPRAM_BASE >> 16 => {
                mem::write(&mut self.usb.dpram, (addr & 0xFFF) as usize, size, value)
            }
            _ => return false,
        }
        true
    }
}

/// An SPI device and how it is wired.
#[derive(Clone)]
pub struct SpiAttachment {
    pub bus: usize,
    pub cs_pin: u8,
    /// Chip select polarity (the UWRT parts are all active low).
    pub cs_active_high: bool,
    pub selected: bool,
    pub dev: Box<dyn SpiDevice>,
}

/// An I2C device and the bus it is on.
#[derive(Clone)]
pub struct I2cAttachment {
    pub bus: usize,
    pub dev: Box<dyn I2cDevice>,
}

/// Something scheduled on the wires: a device driving a pin, or a device wake-up.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WireItem {
    Drive(u8, External),
    Wake,
}

/// Pin drives and wake-ups in time order (equal times in the order they were scheduled).
#[derive(Clone, Debug, Default)]
pub struct WireQueue {
    items: std::collections::VecDeque<(Time, WireItem)>,
    /// Device reports waiting to go to the trace.
    pub reports: Vec<(Time, String)>,
}

impl WireQueue {
    pub fn push(&mut self, t: Time, item: WireItem) {
        let i = self.items.partition_point(|(x, _)| *x <= t);
        self.items.insert(i, (t, item));
    }

    pub fn next_time(&self) -> Option<Time> {
        self.items.front().map(|i| i.0)
    }

    pub fn pop_due(&mut self, now: Time) -> Option<(Time, WireItem)> {
        if self.items.front().is_some_and(|i| i.0 <= now) { self.items.pop_front() } else { None }
    }

    pub fn len(&self) -> usize {
        self.items.len()
    }

    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
}

/// What a device sees of the board.
struct DevPins<'a> {
    now: Time,
    gpio: &'a mut Gpio,
    freqs: &'a Freqs,
    changed: bool,
    wire: &'a mut WireQueue,
}

impl DeviceCtx for DevPins<'_> {
    fn now(&self) -> Time {
        self.now
    }

    fn set_pin(&mut self, pin: u8, value: External) {
        if let Some(p) = self.gpio.external.get_mut(pin as usize)
            && *p != value
        {
            *p = value;
            self.changed = true;
        }
    }

    fn pin_level(&self, pin: u8) -> bool {
        self.gpio.level & (1 << pin) != 0
    }

    fn drive_at(&mut self, at: Time, pin: u8, value: External) {
        if (pin as usize) < NUM_GPIOS {
            self.wire.push(at.max(self.now), WireItem::Drive(pin, value));
        }
    }

    fn wake_at(&mut self, at: Time) {
        self.wire.push(at.max(self.now), WireItem::Wake);
    }

    fn report(&mut self, message: String) {
        self.wire.reports.push((self.now, message));
    }

    fn pin_clock_hz(&self, pin: u8) -> u64 {
        if (pin as usize) >= NUM_GPIOS || self.gpio.funcsel(pin as usize) != func::CLOCK {
            return 0;
        }
        match pin {
            21 => self.freqs.gpout[0],
            23 => self.freqs.gpout[1],
            24 => self.freqs.gpout[2],
            25 => self.freqs.gpout[3],
            _ => 0,
        }
    }
}

/// The system as a DMA bus master.
struct DmaPort<'a> {
    sys: &'a mut System,
    trace: &'a mut dyn TraceSink,
    now: Time,
}

impl DmaBus for DmaPort<'_> {
    fn dreq(&mut self, n: u32) -> bool {
        self.sys.dreq(n)
    }

    fn ack_dreq(&mut self, n: u32) {
        if (24..=31).contains(&n) {
            self.sys.dreq_pulses[n as usize] = self.sys.dreq_pulses[n as usize].saturating_sub(1);
        }
    }

    fn dma_read(&mut self, addr: u32, size: Size) -> Option<u32> {
        // SIO and the PPB are private to the cores
        if addr >= SIO_BASE {
            return None;
        }
        self.sys.read(self.trace, 2, self.now, addr, size).ok()
    }

    fn dma_write(&mut self, addr: u32, size: Size, value: u32) -> bool {
        if addr >= SIO_BASE {
            return false;
        }
        self.sys.write(self.trace, 2, self.now, addr, size, value).is_ok()
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
                self.sys.xip_check(self.now, addr)?;
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
        self.sys.peek_core(self.core, addr, size)
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
        // XIP faults until the SSI is set up (by boot2, here preset)
        assert_eq!(sys.read(&mut t, 0, Time::ZERO, XIP_BASE, Size::Word), Err(BusFault::Unmapped(XIP_BASE)));
        sys.preset_xip();
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
        w(&mut sys, Time::ZERO, RESETS_BASE + 0x3000, 1 << bit::BUSCTRL);
        // BUSCTRL.PERFSEL0 (unmodelled): a byte write at offset +1 lands replicated in all lanes
        sys.write(&mut t, 0, Time::ZERO, BUSCTRL_BASE + 0x0D, Size::Byte, 0xAB).unwrap();
        assert_eq!(sys.unmodelled.peek(BUSCTRL_BASE + 0x0C), 0xABAB_ABAB);
        assert_eq!(sys.read(&mut t, 0, Time::ZERO, BUSCTRL_BASE + 0x0E, Size::Half), Ok(0xABAB));
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

    /// Fires every event due up to `until`.
    fn run_events(sys: &mut System, until: Time) {
        while let Some((t, ev)) = sys.sched.pop_due(until) {
            sys.fire_event(&mut NullSink, t, ev);
        }
    }

    /// Clocks running and every block out of reset, like after `embassy_rp::init`.
    fn running() -> System {
        let mut sys = quiet();
        sys.update_clocks(Time::ZERO);
        w(&mut sys, Time::ZERO, RESETS_BASE, 0);
        sys
    }

    #[test]
    fn i2c_transfer_reaches_the_device_through_registers() {
        use sim_devices::mcp342x::Mcp342x;
        let mut sys = running();
        sys.attach_i2c(0, Box::new(Mcp342x::mcp3426(1.0, 0.0)));
        const I2C: u32 = I2C0_BASE;
        // No pins routed to I2C0 yet: the address is NAKed
        w(&mut sys, Time::ZERO, I2C + 0x04, 0x68);
        w(&mut sys, Time::ZERO, I2C + 0x6C, 1);
        w(&mut sys, Time::ZERO, I2C + 0x10, 0x18 | 1 << 9);
        run_events(&mut sys, Time::from_ms(1));
        assert_eq!(r(&mut sys, Time::from_ms(1), I2C + 0x80) & 1, 1, "ABRT_7B_ADDR_NOACK");
        r(&mut sys, Time::from_ms(1), I2C + 0x54);
        // GPIO 4/5 to I2C0, then configure continuous 16-bit and read it back 70 ms later
        w(&mut sys, Time::from_ms(1), IO_BANK0_BASE + 4 * 8 + 4, 3);
        w(&mut sys, Time::from_ms(1), IO_BANK0_BASE + 5 * 8 + 4, 3);
        w(&mut sys, Time::from_ms(1), I2C + 0x10, 0x18 | 1 << 9);
        run_events(&mut sys, Time::from_ms(2));
        assert_eq!(r(&mut sys, Time::from_ms(2), I2C + 0x80), 0);
        assert_ne!(r(&mut sys, Time::from_ms(2), I2C + 0x34) & 1 << 9, 0, "STOP_DET");
        let t = Time::from_ms(80);
        for i in 0..3 {
            w(&mut sys, t, I2C + 0x10, 1 << 8 | if i == 2 { 1 << 9 } else { 0 } | if i == 0 { 1 << 10 } else { 0 });
        }
        run_events(&mut sys, t + Time::from_ms(1));
        let now = t + Time::from_ms(1);
        assert_eq!(r(&mut sys, now, I2C + 0x78), 3);
        let b: Vec<u32> = (0..3).map(|_| r(&mut sys, now, I2C + 0x10) & 0xFF).collect();
        assert_eq!(((b[0] << 8) | b[1], b[2]), (16000, 0x18));
        assert_eq!(sys.i2c_device::<Mcp342x>().unwrap().config_writes, 1, "the NAKed write never reached it");
    }

    #[test]
    fn adc_round_robin_into_dma() {
        use sim_core::analog::Signal;
        let mut sys = running();
        sys.attach_analog(0, Box::new(Signal::Const(1.65)));
        sys.attach_analog(2, Box::new(Signal::ramp(&[(Time::ZERO, 0.0), (Time::from_us(100), 3.3)])));
        sys.chip_temp_c = Signal::Const(27.0);
        // With clk_adc stopped nothing converts (and nothing is scheduled)
        w(&mut sys, Time::ZERO, ADC_BASE, 1 | 1 << 3);
        assert!(!sys.sched.is_pending(Event::Adc));
        w(&mut sys, Time::ZERO, ADC_BASE, 0);
        sys.clk_adc = ClockPeriod::from_hz(48_000_000);
        // FIFO with DREQ, 8-bit samples; DMA channel 0 reads 8 bytes from the FIFO paced by DREQ_ADC
        w(&mut sys, Time::ZERO, ADC_BASE + 0x08, 1 | 1 << 1 | 1 << 3 | 1 << 24);
        w(&mut sys, Time::ZERO, DMA_BASE, ADC_BASE + 0x0C);
        w(&mut sys, Time::ZERO, DMA_BASE + 4, SRAM_BASE + 0x100);
        w(&mut sys, Time::ZERO, DMA_BASE + 8, 8);
        w(&mut sys, Time::ZERO, DMA_BASE + 0xC, 1 | 1 << 5 | dma::dreq::ADC << 15);
        // Inputs 0, 2 and the temperature sensor (4) round robin
        w(&mut sys, Time::ZERO, ADC_BASE, 1 | 1 << 1 | 0b10101 << 16 | 1 << 3);
        run_events(&mut sys, Time::from_us(30));
        let got: Vec<u8> = (0..8).map(|i| sys.sram[0x100 + i]).collect();
        // 1.65 V -> 0x80; the ramp at 4 us (0.13 V) and 10 us (0.33 V); the temperature sensor's 0.706 V -> 0x36
        assert_eq!(got[0], 0x80);
        assert_eq!(got[2], 0x36);
        assert_eq!(got[3], 0x80);
        assert!(got[1] < got[4] && got[4] < got[7], "{got:?}");
        assert!(!sys.dma.any_busy(), "DMA finished after 8 samples");
    }

    #[test]
    fn uart_device_answers() {
        use sim_devices::uart::Scripted;
        let mut sys = running();
        sys.attach_uart(1, Box::new(Scripted::lines(&[("AT", "OK\n")])));
        // 115200 8N1 at a 48 MHz clk_peri: IBRD/FBRD, then enable TX/RX
        sys.clk_peri = ClockPeriod::from_hz(48_000_000);
        let clk = 48_000_000u64;
        let div = (clk * 4 / 115_200) as u32;
        w(&mut sys, Time::ZERO, UART1_BASE + 0x24, div >> 6);
        w(&mut sys, Time::ZERO, UART1_BASE + 0x28, div & 0x3F);
        w(&mut sys, Time::ZERO, UART1_BASE + 0x2C, 3 << 5 | 1 << 4);
        w(&mut sys, Time::ZERO, UART1_BASE + 0x30, 1 | 1 << 8 | 1 << 9);
        for &b in b"AT\n" {
            w(&mut sys, Time::ZERO, UART1_BASE, b as u32);
        }
        run_events(&mut sys, Time::from_ms(2));
        let mut rx = Vec::new();
        while r(&mut sys, Time::from_ms(2), UART1_BASE + 0x18) & 1 << 4 == 0 {
            rx.push(r(&mut sys, Time::from_ms(2), UART1_BASE) as u8);
        }
        assert_eq!(rx, b"OK\n");
    }

    #[test]
    fn pwm_counts_edges_of_an_external_signal() {
        let mut sys = running();
        // GPIO 1 (PWM 0B) to the PWM function, slice 0 counting rising edges
        w(&mut sys, Time::ZERO, IO_BANK0_BASE + 8 + 4, 4);
        w(&mut sys, Time::ZERO, PWM_BASE, 1 | 2 << 4);
        for i in 0..5u64 {
            sys.gpio.external[1] = External::Driven(sim_core::wire::Level::High);
            sys.update_pins(&mut NullSink, Time::from_us(10 * i + 5));
            sys.gpio.external[1] = External::Driven(sim_core::wire::Level::Low);
            sys.update_pins(&mut NullSink, Time::from_us(10 * i + 10));
        }
        assert_eq!(r(&mut sys, Time::from_us(60), PWM_BASE + 8), 5);
    }
}
