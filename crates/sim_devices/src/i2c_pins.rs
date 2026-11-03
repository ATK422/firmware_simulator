//! An I2C target on two pins, for buses the SoC bit-bangs (PIO SMBus, software I2C).
//!
//! The adapter follows SDA and SCL like a real target: START and STOP are SDA edges while SCL is high, bits are
//! sampled on SCL rising edges, and the target drives SDA (ACKs, data bits) a hold time after SCL falls. It wraps any
//! [`I2cDevice`] model; the model's clock stretching holds SCL low after the falling edge. Both lines are open
//! drain: the target only ever pulls low or lets go.

use sim_core::time::Time;
use sim_core::wire::{External, Level};

use crate::DeviceCtx;
use crate::i2c::I2cDevice;
use crate::pin::PinDevice;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    /// Receiving the address byte.
    Addr,
    /// Driving (or not) the ACK of the address; `read` is the direction.
    AddrAck {
        acked: bool,
        read: bool,
    },
    /// Receiving a byte from the controller.
    Write,
    WriteAck,
    /// Sending a byte to the controller.
    Read,
    /// The controller's ACK/NAK of a byte read.
    ReadAck,
    /// NAKed or finished: wait for STOP or START.
    Ignore,
}

#[derive(Clone)]
pub struct I2cPinTarget {
    pub sda: u8,
    pub scl: u8,
    pub dev: Box<dyn I2cDevice>,
    /// SDA hold time after SCL falls.
    pub hold: Time,
    sda_level: bool,
    scl_level: bool,
    state: State,
    bits: u8,
    byte: u8,
    /// Byte being sent and the controller's last ACK.
    out: u8,
    master_ack: bool,
    driving_sda: bool,
    /// Transfers seen (STARTs, including repeated).
    pub starts: u32,
    pub stops: u32,
}

impl I2cPinTarget {
    pub fn new(sda: u8, scl: u8, dev: Box<dyn I2cDevice>) -> Self {
        I2cPinTarget {
            sda,
            scl,
            dev,
            hold: Time::from_ns(300),
            sda_level: true,
            scl_level: true,
            state: State::Idle,
            bits: 0,
            byte: 0,
            out: 0,
            master_ack: false,
            driving_sda: false,
            starts: 0,
            stops: 0,
        }
    }

    fn drive_sda(&mut self, ctx: &mut dyn DeviceCtx, low: bool) {
        if low != self.driving_sda {
            let t = ctx.now() + self.hold;
            ctx.drive_at(t, self.sda, if low { External::Driven(Level::Low) } else { External::Floating });
            self.driving_sda = low;
        }
    }

    fn stretch(&mut self, ctx: &mut dyn DeviceCtx) {
        let s = self.dev.stretch(ctx);
        if s > Time::ZERO {
            let now = ctx.now();
            ctx.drive_at(now, self.scl, External::Driven(Level::Low));
            ctx.drive_at(now + s, self.scl, External::Floating);
        }
    }

    fn scl_fell(&mut self, ctx: &mut dyn DeviceCtx) {
        match self.state {
            State::Addr if self.bits == 8 => {
                let addr = self.byte >> 1;
                let read = self.byte & 1 != 0;
                let acked = self.dev.responds_to(addr) && self.dev.start(ctx, addr, read);
                if acked {
                    self.drive_sda(ctx, true);
                }
                self.state = State::AddrAck { acked, read };
            }
            State::AddrAck { acked, read } => {
                if !acked {
                    self.state = State::Ignore;
                } else if read {
                    self.stretch(ctx);
                    self.out = self.dev.read(ctx, true);
                    self.bits = 0;
                    self.send_bit(ctx);
                    self.state = State::Read;
                } else {
                    self.drive_sda(ctx, false);
                    self.bits = 0;
                    self.byte = 0;
                    self.state = State::Write;
                }
            }
            State::Write if self.bits == 8 => {
                let ack = self.dev.write(ctx, self.byte);
                if ack {
                    self.drive_sda(ctx, true);
                }
                self.state = if ack { State::WriteAck } else { State::Ignore };
            }
            State::WriteAck => {
                self.drive_sda(ctx, false);
                self.stretch(ctx);
                self.bits = 0;
                self.byte = 0;
                self.state = State::Write;
            }
            State::Read if self.bits < 8 => self.send_bit(ctx),
            State::Read => {
                // All 8 bits out: let go for the controller's ACK
                self.drive_sda(ctx, false);
                self.state = State::ReadAck;
            }
            State::ReadAck => {
                if self.master_ack {
                    self.out = self.dev.read(ctx, true);
                    self.bits = 0;
                    self.send_bit(ctx);
                    self.state = State::Read;
                } else {
                    self.drive_sda(ctx, false);
                    self.state = State::Ignore;
                }
            }
            _ => {}
        }
    }

    fn send_bit(&mut self, ctx: &mut dyn DeviceCtx) {
        let bit = self.out & (0x80 >> self.bits) != 0;
        self.drive_sda(ctx, !bit);
        self.bits += 1;
    }

    fn scl_rose(&mut self) {
        match self.state {
            State::Addr | State::Write if self.bits < 8 => {
                self.byte = self.byte << 1 | self.sda_level as u8;
                self.bits += 1;
            }
            State::ReadAck => self.master_ack = !self.sda_level,
            _ => {}
        }
    }
}

impl PinDevice for I2cPinTarget {
    fn name(&self) -> String {
        format!("{} on GPIO {}/{}", self.dev.name(), self.sda, self.scl)
    }

    fn watches(&self, pin: u8) -> bool {
        pin == self.sda || pin == self.scl
    }

    fn pin_changed(&mut self, ctx: &mut dyn DeviceCtx, pin: u8, level: bool) {
        if pin == self.sda {
            if level == self.sda_level {
                return;
            }
            self.sda_level = level;
            if self.scl_level && !self.driving_sda {
                if level {
                    // STOP
                    if self.state != State::Idle {
                        self.dev.stop(ctx);
                    }
                    self.stops += 1;
                    self.state = State::Idle;
                } else {
                    // START or repeated START
                    self.starts += 1;
                    self.state = State::Addr;
                    self.bits = 0;
                    self.byte = 0;
                }
            }
        } else {
            if level == self.scl_level {
                return;
            }
            self.scl_level = level;
            if level {
                self.scl_rose();
            } else {
                self.scl_fell(ctx);
            }
        }
    }

    fn clone_box(&self) -> Box<dyn PinDevice> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TestCtx;
    use crate::mcp342x::Mcp342x;

    /// A bit-banging controller against the target, applying the target's drives as they come due.
    struct Bus {
        t: I2cPinTarget,
        ctx: TestCtx,
        sda_ctl: bool,
        scl_ctl: bool,
        sda_dev: bool,
        scl_dev: bool,
    }

    impl Bus {
        fn sda(&self) -> bool {
            self.sda_ctl && self.sda_dev
        }
        fn scl(&self) -> bool {
            self.scl_ctl && self.scl_dev
        }
        /// Advances 2.5 us, applying due drives and telling the target about level changes.
        fn tick(&mut self) {
            let before = (self.sda(), self.scl());
            self.ctx.now += Time::from_ns(2500);
            let due: Vec<_> = self.ctx.scheduled.iter().copied().filter(|d| d.0 <= self.ctx.now).collect();
            self.ctx.scheduled.retain(|d| d.0 > self.ctx.now);
            for (_, pin, e) in due {
                let v = e != External::Driven(Level::Low);
                if pin == self.t.sda { self.sda_dev = v } else { self.scl_dev = v }
            }
            self.notify(before);
        }
        fn notify(&mut self, before: (bool, bool)) {
            if self.sda() != before.0 {
                let l = self.sda();
                self.t.pin_changed(&mut self.ctx, 0, l);
            }
            if self.scl() != before.1 {
                let l = self.scl();
                self.t.pin_changed(&mut self.ctx, 1, l);
            }
        }
        fn set(&mut self, sda: bool, scl: bool) {
            let before = (self.sda(), self.scl());
            self.sda_ctl = sda;
            self.scl_ctl = scl;
            self.notify(before);
            self.tick();
        }
        fn start(&mut self) {
            self.set(true, true);
            self.set(false, true);
            self.set(false, false);
        }
        fn stop(&mut self) {
            self.set(false, false);
            self.set(false, true);
            self.set(true, true);
        }
        /// Clocks one bit; returns SDA sampled while SCL is high.
        fn bit(&mut self, b: bool) -> bool {
            self.set(b, false);
            self.set(b, true);
            while !self.scl() {
                self.tick(); // stretched
            }
            let v = self.sda();
            self.set(b, false);
            v
        }
        fn byte_out(&mut self, b: u8) -> bool {
            for i in (0..8).rev() {
                self.bit(b >> i & 1 != 0);
            }
            !self.bit(true)
        }
        fn byte_in(&mut self, ack: bool) -> u8 {
            let v = (0..8).fold(0, |a, _| a << 1 | self.bit(true) as u8);
            self.bit(!ack);
            v
        }
    }

    #[test]
    fn bit_banged_transfers_reach_the_model() {
        let dev = Mcp342x::mcp3426(1.0, 0.0);
        let mut b = Bus {
            t: I2cPinTarget::new(0, 1, Box::new(dev)),
            ctx: TestCtx::default(),
            sda_ctl: true,
            scl_ctl: true,
            sda_dev: true,
            scl_dev: true,
        };
        // Write the config byte (16-bit continuous), then read three bytes 70 ms later
        b.start();
        assert!(b.byte_out(0x68 << 1));
        assert!(b.byte_out(0x18));
        b.stop();
        b.ctx.now += Time::from_ms(70);
        b.start();
        assert!(b.byte_out(0x68 << 1 | 1));
        let hi = b.byte_in(true);
        let lo = b.byte_in(true);
        let cfg = b.byte_in(false);
        b.stop();
        assert_eq!((u16::from_be_bytes([hi, lo]), cfg), (16000, 0x18));
        // Wrong address: NAK
        b.start();
        assert!(!b.byte_out(0x50 << 1));
        b.stop();
        assert_eq!((b.t.starts, b.t.stops), (3, 3));
    }
}
