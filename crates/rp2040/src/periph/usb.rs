//! The USB controller in device mode: USBCTRL_REGS and the 4 KiB USBCTRL_DPRAM (datasheet 4.1).
//!
//! The DPRAM is ordinary memory to the CPU (byte, halfword and word accesses, no atomic aliases). The controller
//! reads and writes it as the host's transactions arrive through [`UsbDevicePort`]: the setup packet lands at
//! offset 0, endpoint control registers at `0x08..0x80` say where each endpoint's buffers are and how it
//! interrupts, and buffer control registers at `0x80..0x100` hand buffers between CPU and controller (4.1.2.5):
//!
//! * An IN token finds the endpoint's selected buffer AVAILABLE (and FULL): its LEN bytes go out with the PID in
//!   the buffer control register, and once the host ACKs the controller clears AVAILABLE and FULL. Otherwise it
//!   NAKs, or STALLs if the endpoint's STALL bit is set (on EP0 only while armed in EP_STALL_ARM, 4.1.2.5.3).
//! * An OUT packet goes into the selected AVAILABLE buffer if its PID matches the expected one (otherwise
//!   DATA_SEQ_ERROR and the packet is dropped, still ACKed); the controller writes LEN, sets FULL and clears
//!   AVAILABLE.
//! * Completed buffers set the endpoint's BUFF_STATUS bit (per buffer or per two buffers, by the endpoint control
//!   register or SIE_CTRL for EP0) and BUFF_CPU_SHOULD_HANDLE says which of a double-buffered pair it was. Double
//!   buffered endpoints alternate buffers; the RESET bit in buffer 0's control word puts the selector back on 0.
//! * A SETUP is always ACKed: it is written to DPRAM, sets SIE_STATUS.SETUP_REC and disarms EP_STALL_ARM.
//!
//! The controller only answers while it is enabled (MAIN_CTRL.CONTROLLER_EN), muxed to the PHY, clocked (clk_usb
//! within the 0.25% full speed tolerance of 48 MHz), and addressed (ADDR_ENDP.ADDRESS). Its D+ pull-up is
//! SIE_CTRL.PULLUP_EN or, when overridden, USBPHY_DIRECT.DP_PULLUP_EN (the pico-sdk's RP2040-E5 workaround).
//!
//! Interrupt sources (INTR) follow the datasheet's "source" notes: most are SIE_STATUS bits (write 1 to clear),
//! BUFF_STATUS, ABORT_DONE and EP_STATUS_STALL_NAK are "any bit set", DEV_SOF clears when SOF_RD is read and
//! DEV_CONN_DIS latches connection changes until SIE_STATUS.CONNECTED is written. Write-1-to-clear registers clear
//! the written bits through any alias: the bootrom and TinyUSB both clear them with the CLR alias
//! (`usb_hw_clear->sie_status = ...`).
//!
//! Suspend (3 ms of idle bus) latches SIE_STATUS.SUSPENDED, the host's resume SIE_STATUS.RESUME, and
//! SIE_CTRL.RESUME while suspended is a remote wakeup the host answers.
//!
//! Not modelled: host mode (MAIN_CTRL.HOST_NDEVICE), isochronous endpoints, the direct PHY
//! drive (USBPHY_DIRECT TX bits) and the RP2040-E5 requirement for 800 µs of J after a reset (CONNECTED is set as
//! soon as the reset ends).

use sim_core::bus::WriteMode;
use sim_core::time::Time;
use sim_devices::usb::{Handshake, InReply, Pid, UsbDevicePort};

pub const DPRAM_SIZE: usize = 4096;

// Register offsets (USBCTRL_REGS)
pub const ADDR_ENDP: u32 = 0x00;
pub const MAIN_CTRL: u32 = 0x40;
pub const SOF_WR: u32 = 0x44;
pub const SOF_RD: u32 = 0x48;
pub const SIE_CTRL: u32 = 0x4C;
pub const SIE_STATUS: u32 = 0x50;
pub const INT_EP_CTRL: u32 = 0x54;
pub const BUFF_STATUS: u32 = 0x58;
pub const BUFF_CPU_SHOULD_HANDLE: u32 = 0x5C;
pub const EP_ABORT: u32 = 0x60;
pub const EP_ABORT_DONE: u32 = 0x64;
pub const EP_STALL_ARM: u32 = 0x68;
pub const NAK_POLL: u32 = 0x6C;
pub const EP_STATUS_STALL_NAK: u32 = 0x70;
pub const USB_MUXING: u32 = 0x74;
pub const USB_PWR: u32 = 0x78;
pub const USBPHY_DIRECT: u32 = 0x7C;
pub const USBPHY_DIRECT_OVERRIDE: u32 = 0x80;
pub const USBPHY_TRIM: u32 = 0x84;
pub const INTR: u32 = 0x8C;
pub const INTE: u32 = 0x90;
pub const INTF: u32 = 0x94;
pub const INTS: u32 = 0x98;

pub mod main_ctrl {
    pub const CONTROLLER_EN: u32 = 1 << 0;
    pub const HOST_NDEVICE: u32 = 1 << 1;
}

pub mod sie_ctrl {
    pub const PULLUP_EN: u32 = 1 << 16;
    pub const RESUME: u32 = 1 << 12;
    pub const EP0_INT_NAK: u32 = 1 << 27;
    pub const EP0_INT_2BUF: u32 = 1 << 28;
    pub const EP0_INT_1BUF: u32 = 1 << 29;
    pub const EP0_DOUBLE_BUF: u32 = 1 << 30;
    pub const EP0_INT_STALL: u32 = 1 << 31;
    /// Host-mode bits that clear themselves (START_TRANS, STOP_TRANS, RESET_BUS, RESUME).
    pub const SELF_CLEARING: u32 = 1 << 0 | 1 << 4 | 1 << 13 | 1 << 12;
}

pub mod sie_status {
    pub const VBUS_DETECTED: u32 = 1 << 0;
    pub const LINE_STATE_LSB: u32 = 2;
    pub const SUSPENDED: u32 = 1 << 4;
    pub const RESUME: u32 = 1 << 11;
    pub const CONNECTED: u32 = 1 << 16;
    pub const SETUP_REC: u32 = 1 << 17;
    pub const TRANS_COMPLETE: u32 = 1 << 18;
    pub const BUS_RESET: u32 = 1 << 19;
    pub const CRC_ERROR: u32 = 1 << 24;
    pub const BIT_STUFF_ERROR: u32 = 1 << 25;
    pub const RX_OVERFLOW: u32 = 1 << 26;
    pub const RX_TIMEOUT: u32 = 1 << 27;
    pub const NAK_REC: u32 = 1 << 28;
    pub const STALL_REC: u32 = 1 << 29;
    pub const ACK_REC: u32 = 1 << 30;
    pub const DATA_SEQ_ERROR: u32 = 1 << 31;
    /// The bits that latch events and clear when written with 1.
    pub const W1C: u32 = SUSPENDED
        | RESUME
        | 3 << 8
        | SETUP_REC
        | TRANS_COMPLETE
        | BUS_RESET
        | CRC_ERROR
        | BIT_STUFF_ERROR
        | RX_OVERFLOW
        | RX_TIMEOUT
        | NAK_REC
        | STALL_REC
        | ACK_REC
        | DATA_SEQ_ERROR;
}

pub mod intr {
    pub const TRANS_COMPLETE: u32 = 1 << 3;
    pub const BUFF_STATUS: u32 = 1 << 4;
    pub const ERROR_DATA_SEQ: u32 = 1 << 5;
    pub const ERROR_RX_TIMEOUT: u32 = 1 << 6;
    pub const ERROR_RX_OVERFLOW: u32 = 1 << 7;
    pub const ERROR_BIT_STUFF: u32 = 1 << 8;
    pub const ERROR_CRC: u32 = 1 << 9;
    pub const STALL: u32 = 1 << 10;
    pub const VBUS_DETECT: u32 = 1 << 11;
    pub const BUS_RESET: u32 = 1 << 12;
    pub const DEV_CONN_DIS: u32 = 1 << 13;
    pub const DEV_SUSPEND: u32 = 1 << 14;
    pub const DEV_RESUME_FROM_HOST: u32 = 1 << 15;
    pub const SETUP_REQ: u32 = 1 << 16;
    pub const DEV_SOF: u32 = 1 << 17;
    pub const ABORT_DONE: u32 = 1 << 18;
    pub const EP_STALL_NAK: u32 = 1 << 19;
    pub const ALL: u32 = 0xF_FFFF;
}

pub mod muxing {
    pub const TO_PHY: u32 = 1 << 0;
    pub const TO_DIGITAL_PAD: u32 = 1 << 2;
}

pub mod pwr {
    pub const VBUS_DETECT: u32 = 1 << 2;
    pub const VBUS_DETECT_OVERRIDE_EN: u32 = 1 << 3;
}

/// DPRAM endpoint control register bits (4.1.2.5.1).
pub mod ep_ctrl {
    pub const ENABLE: u32 = 1 << 31;
    pub const DOUBLE_BUFFERED: u32 = 1 << 30;
    pub const INT_PER_BUFF: u32 = 1 << 29;
    pub const INT_PER_DOUBLE_BUFF: u32 = 1 << 28;
    pub const INT_ON_STALL: u32 = 1 << 17;
    pub const INT_ON_NAK: u32 = 1 << 16;
    pub const BUFFER_ADDRESS: u32 = 0xFFC0;
}

/// DPRAM buffer control register bits, per 16-bit half (4.1.2.5.2).
pub mod buf_ctrl {
    pub const FULL: u32 = 1 << 15;
    pub const LAST: u32 = 1 << 14;
    pub const DATA1: u32 = 1 << 13;
    /// Buffer 0 only: reset the buffer selector to buffer 0.
    pub const RESET: u32 = 1 << 12;
    /// Buffer 0 only: send STALL.
    pub const STALL: u32 = 1 << 11;
    pub const AVAILABLE: u32 = 1 << 10;
    pub const LEN: u32 = 0x3FF;
}

/// Full speed needs the bit clock within 0.25% (USB 2.0 7.1.11): clk_usb must be 48 MHz.
pub fn clk_usb_ok(hz: u64) -> bool {
    hz.abs_diff(48_000_000) <= 48_000_000 / 400
}

/// How an endpoint behaves, from its endpoint control register (or SIE_CTRL for EP0).
#[derive(Clone, Copy, Debug)]
struct EpConfig {
    enabled: bool,
    double: bool,
    int_per_buf: bool,
    int_per_double: bool,
    int_nak: bool,
    int_stall: bool,
    /// DPRAM offset of buffer 0.
    buffer: usize,
}

#[derive(Clone, Debug)]
pub struct Usb {
    pub dpram: Vec<u8>,
    addr_endp: [u32; 16],
    main_ctrl: u32,
    sie_ctrl: u32,
    /// Latched (write-1-to-clear) SIE_STATUS bits.
    sie_status: u32,
    int_ep_ctrl: u32,
    buff_status: u32,
    buff_cpu_should_handle: u32,
    ep_abort: u32,
    ep_abort_done: u32,
    ep_stall_arm: u32,
    nak_poll: u32,
    ep_status_stall_nak: u32,
    muxing: u32,
    pwr: u32,
    phy_direct: u32,
    phy_direct_override: u32,
    phy_trim: u32,
    inte: u32,
    intf: u32,
    /// Next buffer (0/1) of each endpoint direction, indexed by BUFF_STATUS bit (`2 * ep + out`).
    buf_sel: [u8; 32],
    /// VBUS is present (a host is plugged in).
    pub vbus: bool,
    /// The host drives SE0 (bus reset).
    bus_in_reset: bool,
    /// The device saw the end of a reset while presenting its pull-up.
    connected: bool,
    conn_changed: bool,
    sof_latched: bool,
    frame: u16,
    /// The bus is suspended (the device saw 3 ms of idle), and the firmware asked for a remote wakeup.
    suspended: bool,
    wakeup: bool,
    /// clk_usb runs at 48 MHz.
    pub clk_ok: bool,
    /// Set when the host should look again (a buffer was armed, the pull-up or a register changed).
    pub kick: bool,
    /// Messages about unusual traffic, for the trace.
    pub reports: Vec<(Time, String)>,
    warned: u32,
}

impl Default for Usb {
    fn default() -> Self {
        Usb {
            dpram: vec![0; DPRAM_SIZE],
            addr_endp: [0; 16],
            main_ctrl: 0,
            sie_ctrl: 0,
            sie_status: 0,
            int_ep_ctrl: 0,
            buff_status: 0,
            buff_cpu_should_handle: 0,
            ep_abort: 0,
            ep_abort_done: 0,
            ep_stall_arm: 0,
            nak_poll: 0x0010_0010,
            ep_status_stall_nak: 0,
            muxing: 0,
            pwr: 0,
            phy_direct: 0,
            phy_direct_override: 0,
            phy_trim: 0x1F1F,
            inte: 0,
            intf: 0,
            buf_sel: [0; 32],
            vbus: false,
            bus_in_reset: false,
            connected: false,
            conn_changed: false,
            sof_latched: false,
            frame: 0,
            suspended: false,
            wakeup: false,
            clk_ok: false,
            kick: false,
            reports: Vec::new(),
            warned: 0,
        }
    }
}

impl Usb {
    /// Resets the registers (RESETS.USBCTRL); the DPRAM keeps its contents and VBUS stays as it is.
    pub fn reset(&mut self) {
        let dpram = std::mem::take(&mut self.dpram);
        let vbus = self.vbus;
        let clk_ok = self.clk_ok;
        let bus_in_reset = self.bus_in_reset;
        *self = Usb { dpram, vbus, clk_ok, bus_in_reset, kick: true, ..Usb::default() };
    }

    /// VBUS from the cable: a host plugged in (`true`) or the cable pulled. Without VBUS the bus is dead: the
    /// device is disconnected (unless firmware overrides VBUS detection, it also sees VBUS_DETECTED drop).
    pub fn set_vbus(&mut self, present: bool) {
        self.vbus = present;
        if !present {
            self.bus_in_reset = false;
            self.suspended = false;
            self.set_connected(false);
        }
    }

    /// Bit of an endpoint direction in BUFF_STATUS and friends.
    fn bit(ep: u8, dir_in: bool) -> u32 {
        1 << (2 * ep as u32 + !dir_in as u32)
    }

    fn word(&self, off: usize) -> u32 {
        u32::from_le_bytes(self.dpram[off..off + 4].try_into().unwrap())
    }

    fn set_word(&mut self, off: usize, v: u32) {
        self.dpram[off..off + 4].copy_from_slice(&v.to_le_bytes());
    }

    // ------------------------------------------------------------------------------------------ CPU side

    /// A CPU (or DMA) write to the DPRAM. Arming a buffer makes the host look again.
    pub fn dpram_write(&mut self, off: usize, size: sim_core::bus::Size, value: u32) {
        let off = off % DPRAM_SIZE;
        sim_core::bus::mem::write(&mut self.dpram, off, size, value);
        if (0x80..0x100).contains(&off) {
            self.kick = true;
        }
    }

    pub fn dpram_read(&self, off: usize, size: sim_core::bus::Size) -> u32 {
        sim_core::bus::mem::read(&self.dpram, off % DPRAM_SIZE, size)
    }

    fn live(&self) -> bool {
        self.main_ctrl & main_ctrl::CONTROLLER_EN != 0
            && self.main_ctrl & main_ctrl::HOST_NDEVICE == 0
            && self.muxing & muxing::TO_PHY != 0
            && self.clk_ok
    }

    fn vbus_detected(&self) -> bool {
        if self.pwr & pwr::VBUS_DETECT_OVERRIDE_EN != 0 { self.pwr & pwr::VBUS_DETECT != 0 } else { self.vbus }
    }

    /// The bus line state: SE0 during a reset or without a pull-up, J (D+ high) while idle. Through the digital
    /// pads (USB_MUXING.TO_DIGITAL_PAD) D+ and D- come from GPIO 15 and 16 when they select function 8, as the
    /// RP2040-E5 workaround uses to force J; `gpio_in` holds the pins' input levels.
    pub fn line_state(&self, gpio_in: u32, funcsel: impl Fn(usize) -> u32) -> u32 {
        if self.muxing & muxing::TO_DIGITAL_PAD != 0 {
            let dp = funcsel(15) == 8 && gpio_in & (1 << 15) != 0;
            let dm = funcsel(16) == 8 && gpio_in & (1 << 16) != 0;
            return dp as u32 | (dm as u32) << 1;
        }
        if self.bus_in_reset || !self.pullup_on() { 0 } else { 1 }
    }

    fn pullup_on(&self) -> bool {
        // USBPHY_DIRECT_OVERRIDE.DP_PULLUP_EN_OVERRIDE_EN hands the pull-up to USBPHY_DIRECT.DP_PULLUP_EN
        if self.phy_direct_override & (1 << 2) != 0 {
            self.phy_direct & (1 << 1) != 0
        } else {
            self.sie_ctrl & sie_ctrl::PULLUP_EN != 0
        }
    }

    fn sie_status_value(&self, line_state: u32) -> u32 {
        let mut v = self.sie_status | line_state << sie_status::LINE_STATE_LSB;
        if self.vbus_detected() {
            v |= sie_status::VBUS_DETECTED;
        }
        if self.connected {
            v |= sie_status::CONNECTED;
        }
        v
    }

    /// Raw interrupt status (INTR).
    pub fn intr(&self) -> u32 {
        use intr::*;
        let s = self.sie_status;
        let mut v = 0;
        let map = [
            (sie_status::TRANS_COMPLETE, TRANS_COMPLETE),
            (sie_status::DATA_SEQ_ERROR, ERROR_DATA_SEQ),
            (sie_status::RX_TIMEOUT, ERROR_RX_TIMEOUT),
            (sie_status::RX_OVERFLOW, ERROR_RX_OVERFLOW),
            (sie_status::BIT_STUFF_ERROR, ERROR_BIT_STUFF),
            (sie_status::CRC_ERROR, ERROR_CRC),
            (sie_status::STALL_REC, STALL),
            (sie_status::BUS_RESET, BUS_RESET),
            (sie_status::SUSPENDED, DEV_SUSPEND),
            (sie_status::RESUME, DEV_RESUME_FROM_HOST),
            (sie_status::SETUP_REC, SETUP_REQ),
        ];
        for (from, to) in map {
            if s & from != 0 {
                v |= to;
            }
        }
        if self.buff_status != 0 {
            v |= BUFF_STATUS;
        }
        if self.vbus_detected() {
            v |= VBUS_DETECT;
        }
        if self.conn_changed {
            v |= DEV_CONN_DIS;
        }
        if self.sof_latched {
            v |= DEV_SOF;
        }
        if self.ep_abort_done != 0 {
            v |= ABORT_DONE;
        }
        if self.ep_status_stall_nak != 0 {
            v |= EP_STALL_NAK;
        }
        v
    }

    pub fn ints(&self) -> u32 {
        (self.intr() | self.intf) & self.inte
    }

    pub fn irq(&self) -> bool {
        // (Most firmware enables none: spare collecting the raw status)
        self.inte != 0 && self.ints() != 0
    }

    /// A register read. SOF_RD clears the SOF interrupt; `line_state` comes from [`Usb::line_state`].
    pub fn read(&mut self, off: u32, line_state: u32) -> Option<u32> {
        let v = self.peek(off, line_state)?;
        if off == SOF_RD {
            self.sof_latched = false;
        }
        Some(v)
    }

    /// A register read without side effects.
    pub fn peek(&self, off: u32, line_state: u32) -> Option<u32> {
        Some(match off {
            0x00..=0x3C => self.addr_endp[(off / 4) as usize],
            MAIN_CTRL => self.main_ctrl,
            SOF_WR => 0,
            SOF_RD => self.frame as u32,
            SIE_CTRL => self.sie_ctrl,
            SIE_STATUS => self.sie_status_value(line_state),
            INT_EP_CTRL => self.int_ep_ctrl,
            BUFF_STATUS => self.buff_status,
            BUFF_CPU_SHOULD_HANDLE => self.buff_cpu_should_handle,
            EP_ABORT => self.ep_abort,
            EP_ABORT_DONE => self.ep_abort_done,
            EP_STALL_ARM => self.ep_stall_arm,
            NAK_POLL => self.nak_poll,
            EP_STATUS_STALL_NAK => self.ep_status_stall_nak,
            USB_MUXING => self.muxing,
            USB_PWR => self.pwr,
            USBPHY_DIRECT => {
                // RX_DP / RX_DM / RX_DD follow the line state
                let ls = line_state & 3;
                let mut v = self.phy_direct;
                if ls & 1 != 0 {
                    v |= 1 << 17 | 1 << 16;
                }
                if ls & 2 != 0 {
                    v |= 1 << 18;
                }
                v
            }
            USBPHY_DIRECT_OVERRIDE => self.phy_direct_override,
            USBPHY_TRIM => self.phy_trim,
            INTR => self.intr(),
            INTE => self.inte,
            INTF => self.intf,
            INTS => self.ints(),
            // A hole between USBPHY_TRIM and INTR that TinyUSB's `memset(usb_hw, 0, ...)` writes: reads as 0
            0x88 => 0,
            _ => return None,
        })
    }

    /// A register write; returns false for offsets that are not registers.
    pub fn write(&mut self, off: u32, value: u32, mode: WriteMode) -> bool {
        // Write-1-to-clear registers clear the written bits whichever alias was used (see the module docs)
        let w1c = value;
        match off {
            0x00 => mode.update(&mut self.addr_endp[0], value, 0x000F_007F),
            0x04..=0x3C => mode.update(&mut self.addr_endp[(off / 4) as usize], value, 0x060F_007F),
            MAIN_CTRL => {
                mode.update(&mut self.main_ctrl, value, 0x8000_0003);
                if self.main_ctrl & main_ctrl::HOST_NDEVICE != 0 && self.warned & 1 == 0 {
                    self.warned |= 1;
                    log::warn!("USB host mode (MAIN_CTRL.HOST_NDEVICE) is not modelled");
                }
            }
            SOF_WR => {}
            SIE_CTRL => {
                mode.update(&mut self.sie_ctrl, value, 0xFF07_BF5F);
                if self.sie_ctrl & sie_ctrl::RESUME != 0 && self.suspended {
                    // Remote wakeup: the device drives K and the host takes over the resume
                    self.wakeup = true;
                }
                self.sie_ctrl &= !sie_ctrl::SELF_CLEARING;
                if !self.pullup_on() {
                    self.set_connected(false);
                }
            }
            SIE_STATUS => {
                self.sie_status &= !(w1c & sie_status::W1C);
                if w1c & sie_status::CONNECTED != 0 {
                    self.conn_changed = false;
                }
            }
            INT_EP_CTRL => mode.update(&mut self.int_ep_ctrl, value, 0xFFFE),
            BUFF_STATUS => self.buff_status &= !w1c,
            EP_ABORT => {
                let before = self.ep_abort;
                mode.update(&mut self.ep_abort, value, 0xFFFF_FFFF);
                // No transaction is ever in flight between two bus accesses, so the abort completes at once
                self.ep_abort_done |= self.ep_abort & !before;
            }
            EP_ABORT_DONE => self.ep_abort_done &= !w1c,
            EP_STALL_ARM => mode.update(&mut self.ep_stall_arm, value, 3),
            NAK_POLL => mode.update(&mut self.nak_poll, value, 0x03FF_03FF),
            EP_STATUS_STALL_NAK => self.ep_status_stall_nak &= !w1c,
            USB_MUXING => mode.update(&mut self.muxing, value, 0xF),
            USB_PWR => mode.update(&mut self.pwr, value, 0x3F),
            USBPHY_DIRECT => mode.update(&mut self.phy_direct, value, 0xFF77),
            USBPHY_DIRECT_OVERRIDE => mode.update(&mut self.phy_direct_override, value, 0x9FFF),
            USBPHY_TRIM => mode.update(&mut self.phy_trim, value, 0x1F1F),
            INTE => mode.update(&mut self.inte, value, intr::ALL),
            INTF => mode.update(&mut self.intf, value, intr::ALL),
            SOF_RD | BUFF_CPU_SHOULD_HANDLE | INTR | INTS | 0x88 => {}
            _ => return false,
        }
        self.kick = true;
        true
    }

    fn set_connected(&mut self, c: bool) {
        if self.connected != c {
            self.connected = c;
            self.conn_changed = true;
        }
    }

    // ------------------------------------------------------------------------------------------ bus side

    fn report(&mut self, now: Time, msg: String) {
        log::debug!("[{now}] usb: {msg}");
        if self.reports.len() < 1000 {
            self.reports.push((now, msg));
        }
    }

    fn ep_config(&self, ep: u8, dir_in: bool) -> EpConfig {
        if ep == 0 {
            let s = self.sie_ctrl;
            return EpConfig {
                enabled: true,
                double: s & sie_ctrl::EP0_DOUBLE_BUF != 0,
                int_per_buf: s & sie_ctrl::EP0_INT_1BUF != 0,
                int_per_double: s & sie_ctrl::EP0_INT_2BUF != 0,
                int_nak: s & sie_ctrl::EP0_INT_NAK != 0,
                int_stall: s & sie_ctrl::EP0_INT_STALL != 0,
                buffer: 0x100,
            };
        }
        let c = self.word(8 * ep as usize + if dir_in { 0 } else { 4 });
        EpConfig {
            enabled: c & ep_ctrl::ENABLE != 0,
            double: c & ep_ctrl::DOUBLE_BUFFERED != 0,
            int_per_buf: c & ep_ctrl::INT_PER_BUFF != 0,
            int_per_double: c & ep_ctrl::INT_PER_DOUBLE_BUFF != 0,
            int_nak: c & ep_ctrl::INT_ON_NAK != 0,
            int_stall: c & ep_ctrl::INT_ON_STALL != 0,
            buffer: (c & ep_ctrl::BUFFER_ADDRESS) as usize,
        }
    }

    fn buf_ctrl_off(ep: u8, dir_in: bool) -> usize {
        0x80 + 8 * ep as usize + if dir_in { 0 } else { 4 }
    }

    /// The selected buffer of an endpoint, or how the endpoint refuses the transaction. Applies a pending RESET of
    /// the buffer selector.
    fn select(&mut self, now: Time, ep: u8, dir_in: bool, cfg: &EpConfig) -> Result<u8, Refusal> {
        let bit = Self::bit(ep, dir_in);
        if self.ep_abort & bit != 0 {
            return Err(Refusal::Nak);
        }
        let off = Self::buf_ctrl_off(ep, dir_in);
        let mut bc = self.word(off);
        if bc & buf_ctrl::RESET != 0 {
            bc &= !buf_ctrl::RESET;
            self.set_word(off, bc);
            self.buf_sel[bit.trailing_zeros() as usize] = 0;
        }
        let armed = ep != 0 || self.ep_stall_arm & (if dir_in { 1 } else { 2 }) != 0;
        if bc & buf_ctrl::STALL != 0 && armed {
            if cfg.int_stall {
                self.ep_status_stall_nak |= bit;
            }
            return Err(Refusal::Stall);
        }
        let b = if cfg.double { self.buf_sel[bit.trailing_zeros() as usize] } else { 0 };
        let half = (bc >> (16 * b as u32)) & 0xFFFF;
        if half & buf_ctrl::AVAILABLE == 0 {
            if cfg.int_nak {
                self.ep_status_stall_nak |= bit;
            }
            return Err(Refusal::Nak);
        }
        if dir_in && half & buf_ctrl::FULL == 0 {
            self.report(now, format!("EP{ep} IN buffer {b} available but not full: sending it anyway"));
        }
        Ok(b)
    }

    fn half(&self, ep: u8, dir_in: bool, b: u8) -> u32 {
        (self.word(Self::buf_ctrl_off(ep, dir_in)) >> (16 * b as u32)) & 0xFFFF
    }

    fn set_half(&mut self, ep: u8, dir_in: bool, b: u8, half: u32) {
        let off = Self::buf_ctrl_off(ep, dir_in);
        let shift = 16 * b as u32;
        let w = (self.word(off) & !(0xFFFF << shift)) | (half & 0xFFFF) << shift;
        self.set_word(off, w);
    }

    /// DPRAM offset of buffer `b`. BUFFER_ADDRESS has more bits than the 4 KiB DPRAM decodes: they wrap. (The
    /// offset is 64 byte aligned, so a 64 byte packet never runs past the end.)
    fn buffer_offset(cfg: &EpConfig, b: u8) -> usize {
        (cfg.buffer + 64 * b as usize) % DPRAM_SIZE
    }

    /// A buffer was handed back to the CPU: advance the selector, raise BUFF_STATUS as configured.
    fn buffer_done(&mut self, ep: u8, dir_in: bool, b: u8, cfg: &EpConfig, last: bool) {
        let bit = Self::bit(ep, dir_in);
        if cfg.double {
            self.buf_sel[bit.trailing_zeros() as usize] = b ^ 1;
        }
        if last {
            self.sie_status |= sie_status::TRANS_COMPLETE;
        }
        let raise = cfg.int_per_buf || (cfg.int_per_double && (b == 1 || !cfg.double));
        if raise {
            self.buff_status |= bit;
            if b == 1 {
                self.buff_cpu_should_handle |= bit;
            } else {
                self.buff_cpu_should_handle &= !bit;
            }
        }
    }

    fn addressed(&self, addr: u8) -> bool {
        self.live() && addr == (self.addr_endp[0] & 0x7F) as u8
    }
}

enum Refusal {
    Nak,
    Stall,
}

impl UsbDevicePort for Usb {
    fn pullup(&self) -> bool {
        self.main_ctrl & main_ctrl::CONTROLLER_EN != 0 && self.pullup_on()
    }

    fn bus_reset(&mut self, _now: Time, active: bool) {
        self.bus_in_reset = active;
        if active {
            if self.main_ctrl & main_ctrl::CONTROLLER_EN != 0 {
                self.sie_status |= sie_status::BUS_RESET;
            }
            self.set_connected(false);
        } else if self.pullup() {
            self.set_connected(true);
        }
    }

    fn suspend(&mut self, now: Time, suspended: bool) {
        if !self.live() || !self.connected {
            return;
        }
        if suspended && !self.suspended {
            self.sie_status |= sie_status::SUSPENDED;
            self.report(now, "device suspended".into());
        } else if !suspended && self.suspended {
            self.sie_status |= sie_status::RESUME;
            self.report(now, "device resumed by the host".into());
        }
        self.suspended = suspended;
        self.wakeup = false;
    }

    fn remote_wakeup(&self) -> bool {
        self.wakeup
    }

    fn sof(&mut self, _now: Time, frame: u16) {
        if self.live() {
            self.frame = frame & 0x7FF;
            self.sof_latched = true;
        }
    }

    fn setup(&mut self, _now: Time, addr: u8, data: &[u8; 8]) -> Handshake {
        if !self.addressed(addr) {
            return Handshake::Timeout;
        }
        self.dpram[..8].copy_from_slice(data);
        self.sie_status |= sie_status::SETUP_REC;
        // A SETUP clears an EP0 stall (USB 2.0 8.5.3.4): the arm bits drop
        self.ep_stall_arm = 0;
        self.buf_sel[0] = 0;
        self.buf_sel[1] = 0;
        Handshake::Ack
    }

    fn out(&mut self, now: Time, addr: u8, ep: u8, pid: Pid, data: &[u8]) -> Handshake {
        if !self.addressed(addr) || ep > 15 {
            return Handshake::Timeout;
        }
        let cfg = self.ep_config(ep, false);
        if !cfg.enabled {
            return Handshake::Timeout;
        }
        let b = match self.select(now, ep, false, &cfg) {
            Ok(b) => b,
            Err(Refusal::Nak) => return Handshake::Nak,
            Err(Refusal::Stall) => return Handshake::Stall,
        };
        let half = self.half(ep, false, b);
        let expected = if half & buf_ctrl::DATA1 != 0 { Pid::Data1 } else { Pid::Data0 };
        if pid != expected {
            self.sie_status |= sie_status::DATA_SEQ_ERROR;
            self.report(now, format!("EP{ep} OUT: {pid:?} when {expected:?} was expected (data sequence error)"));
            return Handshake::Ack;
        }
        let room = (half & buf_ctrl::LEN) as usize;
        if data.len() > room || data.len() > 64 {
            self.sie_status |= sie_status::RX_OVERFLOW;
            self.report(now, format!("EP{ep} OUT: {} bytes into a {room} byte buffer", data.len()));
            return Handshake::Timeout;
        }
        let at = Self::buffer_offset(&cfg, b);
        self.dpram[at..at + data.len()].copy_from_slice(data);
        let new = (half & !(buf_ctrl::AVAILABLE | buf_ctrl::LEN)) | buf_ctrl::FULL | data.len() as u32;
        self.set_half(ep, false, b, new);
        self.buffer_done(ep, false, b, &cfg, half & buf_ctrl::LAST != 0);
        Handshake::Ack
    }

    fn in_token(&mut self, now: Time, addr: u8, ep: u8) -> InReply {
        if !self.addressed(addr) || ep > 15 {
            return InReply::Timeout;
        }
        let cfg = self.ep_config(ep, true);
        if !cfg.enabled {
            return InReply::Timeout;
        }
        let b = match self.select(now, ep, true, &cfg) {
            Ok(b) => b,
            Err(Refusal::Nak) => return InReply::Nak,
            Err(Refusal::Stall) => return InReply::Stall,
        };
        let half = self.half(ep, true, b);
        let len = ((half & buf_ctrl::LEN) as usize).min(1023);
        let at = Self::buffer_offset(&cfg, b);
        let data = self.dpram[at..(at + len).min(DPRAM_SIZE)].to_vec();
        let pid = if half & buf_ctrl::DATA1 != 0 { Pid::Data1 } else { Pid::Data0 };
        self.set_half(ep, true, b, half & !(buf_ctrl::AVAILABLE | buf_ctrl::FULL));
        self.buffer_done(ep, true, b, &cfg, half & buf_ctrl::LAST != 0);
        InReply::Data(pid, data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::bus::Size;

    /// A controller set up the way the bootrom's `usb_device_start` leaves it, with EP1 IN and EP2 OUT bulk,
    /// double buffered, interrupting per buffer.
    fn device() -> Usb {
        let mut u = Usb { clk_ok: true, vbus: true, ..Usb::default() };
        u.write(USB_MUXING, muxing::TO_PHY | 1 << 3, WriteMode::Normal);
        u.write(USB_PWR, pwr::VBUS_DETECT | pwr::VBUS_DETECT_OVERRIDE_EN, WriteMode::Normal);
        u.write(MAIN_CTRL, main_ctrl::CONTROLLER_EN, WriteMode::Normal);
        u.write(SIE_CTRL, sie_ctrl::PULLUP_EN | sie_ctrl::EP0_INT_1BUF, WriteMode::Normal);
        u.write(INTE, intr::BUFF_STATUS | intr::BUS_RESET | intr::SETUP_REQ, WriteMode::Normal);
        let ctrl = ep_ctrl::ENABLE | ep_ctrl::DOUBLE_BUFFERED | ep_ctrl::INT_PER_BUFF | 2 << 26;
        u.dpram_write(0x08, Size::Word, ctrl | 0x180); // EP1 IN
        u.dpram_write(0x14, Size::Word, ctrl | 0x200); // EP2 OUT
        u
    }

    fn w(u: &mut Usb, off: usize, v: u32) {
        u.dpram_write(off, Size::Word, v);
    }

    #[test]
    fn registers_and_w1c_through_any_alias() {
        let mut u = device();
        assert_eq!(u.read(NAK_POLL, 0), Some(0x0010_0010));
        assert_eq!(u.read(USBPHY_TRIM, 0), Some(0x1F1F));
        u.sie_status = sie_status::SETUP_REC | sie_status::BUS_RESET;
        assert!(u.irq());
        assert_eq!(u.ints(), intr::SETUP_REQ | intr::BUS_RESET);
        // The CLR alias clears W1C bits, as `usb_hw_clear->sie_status = SETUP_REC` expects
        u.write(SIE_STATUS, sie_status::SETUP_REC, WriteMode::Clear);
        assert_eq!(u.sie_status, sie_status::BUS_RESET);
        u.write(SIE_STATUS, sie_status::BUS_RESET, WriteMode::Normal);
        assert!(!u.irq());
        // RW registers take the aliases as usual; INTF forces an interrupt
        u.write(INTE, intr::DEV_SOF, WriteMode::Set);
        assert_eq!(u.read(INTE, 0).unwrap() & intr::DEV_SOF, intr::DEV_SOF);
        u.write(INTF, intr::BUFF_STATUS, WriteMode::Normal);
        assert!(u.irq());
        u.write(INTF, 0, WriteMode::Normal);
        // VBUS detect override, connected and line state in SIE_STATUS
        let s = u.read(SIE_STATUS, 1).unwrap();
        assert_eq!(s & (sie_status::VBUS_DETECTED | 3 << 2), sie_status::VBUS_DETECTED | 1 << 2);
        assert!(!u.write(0x9C, 1, WriteMode::Normal), "0x9c is past the registers");
    }

    #[test]
    fn setup_reset_and_connection() {
        let mut u = device();
        assert!(u.pullup());
        assert_eq!(u.line_state(0, |_| 0), 1, "J while idle");
        u.bus_reset(Time::ZERO, true);
        assert_eq!(u.line_state(0, |_| 0), 0, "SE0 during reset");
        assert_ne!(u.ints() & intr::BUS_RESET, 0);
        u.bus_reset(Time::from_ms(10), false);
        assert_ne!(u.peek(SIE_STATUS, 1).unwrap() & sie_status::CONNECTED, 0);
        // Connection changes latch DEV_CONN_DIS until SIE_STATUS.CONNECTED is written; CONNECTED stays set
        assert_ne!(u.intr() & intr::DEV_CONN_DIS, 0);
        u.write(SIE_STATUS, sie_status::CONNECTED, WriteMode::Clear);
        assert_eq!(u.intr() & intr::DEV_CONN_DIS, 0);
        assert_ne!(u.peek(SIE_STATUS, 1).unwrap() & sie_status::CONNECTED, 0);
        // SETUP to the wrong address gets no answer; to address 0 it lands in DPRAM and disarms EP0 stalls
        u.ep_stall_arm = 3;
        let pkt = [0x80, 6, 0, 1, 0, 0, 64, 0];
        assert_eq!(u.setup(Time::ZERO, 5, &pkt), Handshake::Timeout);
        assert_eq!(u.setup(Time::ZERO, 0, &pkt), Handshake::Ack);
        assert_eq!(&u.dpram[..8], &pkt);
        assert_eq!(u.ep_stall_arm, 0);
        assert_ne!(u.ints() & intr::SETUP_REQ, 0);
        // Pull-up off: disconnected
        u.write(SIE_CTRL, sie_ctrl::PULLUP_EN, WriteMode::Clear);
        assert!(!u.pullup());
        assert_eq!(u.peek(SIE_STATUS, 0).unwrap() & sie_status::CONNECTED, 0);
        // Pulling the cable disconnects too
        u.write(SIE_CTRL, sie_ctrl::PULLUP_EN, WriteMode::Set);
        u.bus_reset(Time::ZERO, true);
        u.bus_reset(Time::from_ms(10), false);
        u.write(SIE_STATUS, sie_status::CONNECTED, WriteMode::Normal);
        u.set_vbus(false);
        assert_eq!(u.peek(SIE_STATUS, 0).unwrap() & sie_status::CONNECTED, 0);
        assert_ne!(u.intr() & intr::DEV_CONN_DIS, 0);
    }

    #[test]
    fn ep0_in_nak_then_data_then_stall() {
        let mut u = device();
        assert_eq!(u.in_token(Time::ZERO, 0, 0), InReply::Nak);
        // 18 bytes, DATA1, AVAILABLE and FULL
        for i in 0..18 {
            u.dpram[0x100 + i] = i as u8;
        }
        w(&mut u, 0x80, 18 | buf_ctrl::FULL | buf_ctrl::DATA1 | buf_ctrl::AVAILABLE);
        assert!(u.kick, "arming a buffer kicks the host");
        assert_eq!(u.in_token(Time::ZERO, 0, 0), InReply::Data(Pid::Data1, (0..18).collect()));
        assert_eq!(u.word(0x80), 18 | buf_ctrl::DATA1, "AVAILABLE and FULL cleared, LEN kept");
        assert_eq!(u.buff_status, 1);
        assert_eq!(u.in_token(Time::ZERO, 0, 0), InReply::Nak);
        // STALL on EP0 needs the arm bit
        w(&mut u, 0x80, buf_ctrl::STALL);
        assert_eq!(u.in_token(Time::ZERO, 0, 0), InReply::Nak);
        u.write(EP_STALL_ARM, 1, WriteMode::Normal);
        assert_eq!(u.in_token(Time::ZERO, 0, 0), InReply::Stall);
    }

    #[test]
    fn buffer_addresses_past_the_dpram_wrap_instead_of_panicking() {
        // BUFFER_ADDRESS is a 16-bit field but the DPRAM is 4 KiB: firmware that writes an offset past it (or puts
        // a double buffered endpoint's second buffer at 0x1000) gets the DPRAM's address decoding, not a crash
        let mut u = device();
        let ctrl = ep_ctrl::ENABLE | ep_ctrl::DOUBLE_BUFFERED | ep_ctrl::INT_PER_BUFF;
        w(&mut u, 0x14, ctrl | 0xFFC0); // EP2 OUT, buffers at 0xFFC0 and 0x10000
        let b0 = 8 | buf_ctrl::AVAILABLE | buf_ctrl::RESET;
        let b1 = 8 | buf_ctrl::AVAILABLE | buf_ctrl::DATA1;
        w(&mut u, 0x80 + 2 * 8 + 4, b0 | b1 << 16);
        assert_eq!(u.out(Time::ZERO, 0, 2, Pid::Data0, &[1; 8]), Handshake::Ack);
        assert_eq!(&u.dpram[0xFC0..0xFC8], &[1; 8]);
        assert_eq!(u.out(Time::ZERO, 0, 2, Pid::Data1, &[2; 8]), Handshake::Ack);
        assert_eq!(&u.dpram[0..8], &[2; 8]);
        w(&mut u, 0x08, ctrl | 0x1000); // EP1 IN at 0x1000: the start of the DPRAM again
        w(&mut u, 0x88, 4 | buf_ctrl::AVAILABLE | buf_ctrl::FULL | buf_ctrl::RESET);
        assert_eq!(u.in_token(Time::ZERO, 0, 1), InReply::Data(Pid::Data0, vec![2; 4]));
    }

    #[test]
    fn double_buffered_out_alternates_and_checks_pid() {
        let mut u = device();
        // Both buffers armed, buffer 0 DATA0 with RESET, buffer 1 DATA1
        let b0 = 64 | buf_ctrl::AVAILABLE | buf_ctrl::RESET;
        let b1 = 64 | buf_ctrl::AVAILABLE | buf_ctrl::DATA1;
        w(&mut u, 0x80 + 2 * 8 + 4, b0 | b1 << 16);
        assert_eq!(u.out(Time::ZERO, 0, 2, Pid::Data0, &[1; 64]), Handshake::Ack);
        assert_eq!(u.buff_status, 1 << 5);
        assert_eq!(u.buff_cpu_should_handle & (1 << 5), 0, "buffer 0");
        assert_eq!(u.half(2, false, 0), 64 | buf_ctrl::FULL);
        // A repeated DATA0 is a sequence error: acknowledged, dropped
        assert_eq!(u.out(Time::ZERO, 0, 2, Pid::Data0, &[9; 3]), Handshake::Ack);
        assert_ne!(u.sie_status & sie_status::DATA_SEQ_ERROR, 0);
        assert_eq!(u.out(Time::ZERO, 0, 2, Pid::Data1, &[2; 10]), Handshake::Ack);
        assert_eq!(u.half(2, false, 1), 10 | buf_ctrl::FULL | buf_ctrl::DATA1);
        assert_ne!(u.buff_cpu_should_handle & (1 << 5), 0, "buffer 1");
        assert_eq!(&u.dpram[0x240..0x24A], &[2; 10]);
        assert_eq!(&u.dpram[0x200..0x240], &[1; 64]);
        // Nothing armed: NAK; too long for the buffer: no handshake
        assert_eq!(u.out(Time::ZERO, 0, 2, Pid::Data0, &[0; 8]), Handshake::Nak);
        w(&mut u, 0x80 + 2 * 8 + 4, 8 | buf_ctrl::AVAILABLE | buf_ctrl::RESET);
        assert_eq!(u.out(Time::ZERO, 0, 2, Pid::Data0, &[0; 9]), Handshake::Timeout);
        // A disabled endpoint does not answer at all
        assert_eq!(u.out(Time::ZERO, 0, 3, Pid::Data0, &[0; 8]), Handshake::Timeout);
    }

    #[test]
    fn interrupt_per_double_buffer_and_abort() {
        let mut u = device();
        let ctrl = ep_ctrl::ENABLE | ep_ctrl::DOUBLE_BUFFERED | ep_ctrl::INT_PER_DOUBLE_BUFF | 0x180;
        w(&mut u, 0x08, ctrl);
        let b0 = 64 | buf_ctrl::AVAILABLE | buf_ctrl::FULL | buf_ctrl::RESET;
        let b1 = 5 | buf_ctrl::AVAILABLE | buf_ctrl::FULL | buf_ctrl::DATA1 | buf_ctrl::LAST;
        w(&mut u, 0x88, b0 | b1 << 16);
        assert!(matches!(u.in_token(Time::ZERO, 0, 1), InReply::Data(Pid::Data0, d) if d.len() == 64));
        assert_eq!(u.buff_status, 0, "no interrupt after the first of two buffers");
        assert!(matches!(u.in_token(Time::ZERO, 0, 1), InReply::Data(Pid::Data1, d) if d.len() == 5));
        assert_eq!(u.buff_status, 1 << 2);
        assert_ne!(u.sie_status & sie_status::TRANS_COMPLETE, 0, "LAST buffer");
        // EP_ABORT: NAK while aborting, ABORT_DONE at once
        w(&mut u, 0x88, 1 | buf_ctrl::AVAILABLE | buf_ctrl::FULL | buf_ctrl::RESET);
        u.write(EP_ABORT, 1 << 2, WriteMode::Set);
        assert_eq!(u.read(EP_ABORT_DONE, 0), Some(1 << 2));
        assert_ne!(u.intr() & intr::ABORT_DONE, 0);
        assert_eq!(u.in_token(Time::ZERO, 0, 1), InReply::Nak);
        u.write(EP_ABORT, 0, WriteMode::Normal);
        u.write(EP_ABORT_DONE, 1 << 2, WriteMode::Clear);
        assert!(matches!(u.in_token(Time::ZERO, 0, 1), InReply::Data(..)));
    }

    #[test]
    fn needs_48mhz_clk_usb_and_sof_read_clears() {
        let mut u = device();
        u.clk_ok = clk_usb_ok(12_000_000);
        assert!(!u.clk_ok);
        assert_eq!(u.setup(Time::ZERO, 0, &[0; 8]), Handshake::Timeout);
        u.clk_ok = clk_usb_ok(48_000_000);
        u.sof(Time::ZERO, 0x123);
        assert_ne!(u.intr() & intr::DEV_SOF, 0);
        assert_eq!(u.read(SOF_RD, 0), Some(0x123));
        assert_eq!(u.intr() & intr::DEV_SOF, 0);
    }

    #[test]
    fn suspend_resume_and_remote_wakeup() {
        let mut u = device();
        u.bus_reset(Time::ZERO, true);
        u.bus_reset(Time::from_ms(10), false);
        u.write(SIE_STATUS, 0xFFFF_FFFF, WriteMode::Normal);
        u.write(INTE, intr::DEV_SUSPEND | intr::DEV_RESUME_FROM_HOST, WriteMode::Normal);
        u.write(SIE_CTRL, sie_ctrl::RESUME, WriteMode::Set);
        assert!(!u.remote_wakeup(), "no wakeup unless suspended");
        u.suspend(Time::from_ms(20), true);
        assert_eq!(u.ints(), intr::DEV_SUSPEND);
        u.write(SIE_STATUS, sie_status::SUSPENDED, WriteMode::Clear);
        u.write(SIE_CTRL, sie_ctrl::RESUME, WriteMode::Set);
        assert!(u.remote_wakeup());
        assert_eq!(u.read(SIE_CTRL, 0).unwrap() & sie_ctrl::RESUME, 0, "self-clearing");
        u.suspend(Time::from_ms(30), false);
        assert_eq!(u.ints(), intr::DEV_RESUME_FROM_HOST);
        assert!(!u.remote_wakeup());
    }

    #[test]
    fn digital_pad_line_state_follows_gpio15() {
        let mut u = device();
        u.write(USB_MUXING, muxing::TO_DIGITAL_PAD, WriteMode::Normal);
        assert_eq!(u.line_state(1 << 15, |p| if p == 15 { 8 } else { 0 }), 1, "forced J");
        assert_eq!(u.line_state(1 << 15, |_| 5), 0);
    }
}
