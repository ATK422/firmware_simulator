//! UART device interface: a device on a UART's TX and RX lines, byte by byte.
//!
//! The SoC hands the device every byte it transmits; the device answers with bytes for the SoC to receive, which
//! arrive at the UART's baud rate. [`Scripted`] answers fixed requests, for tests and simple peripherals.

use crate::DeviceCtx;

pub trait UartDevice: Send {
    fn name(&self) -> String;
    /// A byte the SoC transmitted. Returns the bytes the device sends back.
    fn receive(&mut self, ctx: &mut dyn DeviceCtx, byte: u8) -> Vec<u8>;
    fn clone_box(&self) -> Box<dyn UartDevice>;
    fn as_any(&self) -> &dyn std::any::Any;
}

sim_core::clone_box!(UartDevice);

/// Answers requests ending in a terminator byte: each request matching a key (exactly) gets its reply. Every
/// received byte is kept in `received`.
#[derive(Clone, Debug, Default)]
pub struct Scripted {
    pub terminator: u8,
    pub replies: Vec<(Vec<u8>, Vec<u8>)>,
    pub received: Vec<u8>,
    line: Vec<u8>,
}

impl Scripted {
    pub fn lines(replies: &[(&str, &str)]) -> Self {
        Scripted {
            terminator: b'\n',
            replies: replies.iter().map(|(k, v)| (k.as_bytes().to_vec(), v.as_bytes().to_vec())).collect(),
            ..Default::default()
        }
    }
}

impl UartDevice for Scripted {
    fn name(&self) -> String {
        "scripted UART device".into()
    }

    fn receive(&mut self, _ctx: &mut dyn DeviceCtx, byte: u8) -> Vec<u8> {
        self.received.push(byte);
        if byte != self.terminator {
            self.line.push(byte);
            return Vec::new();
        }
        let line = std::mem::take(&mut self.line);
        self.replies.iter().find(|(k, _)| *k == line).map(|(_, v)| v.clone()).unwrap_or_default()
    }

    fn clone_box(&self) -> Box<dyn UartDevice> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TestCtx;

    #[test]
    fn scripted_replies() {
        let mut ctx = TestCtx::default();
        let mut d = Scripted::lines(&[("PING", "PONG\n")]);
        let mut out = Vec::new();
        for &b in b"PING\nNOPE\n" {
            out.extend(d.receive(&mut ctx, b));
        }
        assert_eq!(out, b"PONG\n");
        assert_eq!(d.received.len(), 10);
    }
}
