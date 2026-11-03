//! Structured trace events.
//!
//! Machines emit [`TraceEvent`]s into a [`TraceSink`]; the CLI prints them, tests collect them. Events that are
//! expensive to produce (instruction traces) are gated by [`TraceSink::wants`] so a disabled trace costs one call.

use std::fmt;
use std::sync::{Arc, Mutex};

use crate::time::Time;

/// Categories a sink can subscribe to.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum Category {
    /// Every executed instruction.
    Instruction,
    /// Exception entry and return.
    Exception,
    /// GPIO pin level changes.
    Gpio,
    /// Peripheral register accesses.
    Peripheral,
    /// Accesses to registers the emulator does not model.
    Unmodelled,
    /// Anything else a device wants to report (CAN frames, UART bytes, …).
    Device,
}

#[derive(Clone, PartialEq, Debug)]
pub enum TraceEvent {
    Instruction {
        core: u8,
        pc: u32,
        opcode: u32,
        text: String,
    },
    ExceptionEntry {
        core: u8,
        exception: u32,
        return_pc: u32,
    },
    ExceptionReturn {
        core: u8,
        exception: u32,
    },
    /// A pin's resolved level changed.
    Gpio {
        pin: u8,
        level: bool,
    },
    /// The chip's registered output on a pin changed (SIO/peripheral output value and enable).
    GpioOutput {
        pin: u8,
        value: bool,
        enabled: bool,
    },
    PeripheralAccess {
        core: u8,
        write: bool,
        addr: u32,
        value: u32,
        name: String,
    },
    Unmodelled {
        core: u8,
        write: bool,
        addr: u32,
        value: u32,
        name: String,
    },
    Device {
        source: String,
        message: String,
    },
}

impl TraceEvent {
    pub fn category(&self) -> Category {
        match self {
            TraceEvent::Instruction { .. } => Category::Instruction,
            TraceEvent::ExceptionEntry { .. } | TraceEvent::ExceptionReturn { .. } => Category::Exception,
            TraceEvent::Gpio { .. } | TraceEvent::GpioOutput { .. } => Category::Gpio,
            TraceEvent::PeripheralAccess { .. } => Category::Peripheral,
            TraceEvent::Unmodelled { .. } => Category::Unmodelled,
            TraceEvent::Device { .. } => Category::Device,
        }
    }
}

impl fmt::Display for TraceEvent {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TraceEvent::Instruction { core, pc, opcode, text } => {
                write!(f, "core{core} {pc:08x}: {opcode:08x} {text}")
            }
            TraceEvent::ExceptionEntry { core, exception, return_pc } => {
                write!(f, "core{core} exception {exception} entry (return to {return_pc:08x})")
            }
            TraceEvent::ExceptionReturn { core, exception } => {
                write!(f, "core{core} exception {exception} return")
            }
            TraceEvent::Gpio { pin, level } => write!(f, "GPIO{pin} = {}", *level as u8),
            TraceEvent::GpioOutput { pin, value, enabled } => {
                write!(f, "GPIO{pin} out = {}{}", *value as u8, if *enabled { "" } else { " (oe off)" })
            }
            TraceEvent::PeripheralAccess { core, write, addr, value, name } => {
                let dir = if *write { "write" } else { "read" };
                write!(f, "core{core} {dir} {name} ({addr:08x}) = {value:08x}")
            }
            TraceEvent::Unmodelled { core, write, addr, value, name } => {
                let dir = if *write { "write" } else { "read" };
                write!(f, "core{core} unmodelled {dir} {name} ({addr:08x}) = {value:08x}")
            }
            TraceEvent::Device { source, message } => write!(f, "{source}: {message}"),
        }
    }
}

/// Receives trace events.
pub trait TraceSink: Send {
    /// Whether events of `category` should be produced at all.
    fn wants(&self, category: Category) -> bool;
    fn event(&mut self, time: Time, event: TraceEvent);
}

/// Discards everything.
#[derive(Clone, Copy, Default, Debug)]
pub struct NullSink;

impl TraceSink for NullSink {
    fn wants(&self, _: Category) -> bool {
        false
    }
    fn event(&mut self, _: Time, _: TraceEvent) {}
}

/// Collects events in memory, shareable with the test that installed it.
#[derive(Clone, Default, Debug)]
pub struct VecSink {
    pub categories: Vec<Category>,
    pub events: Arc<Mutex<Vec<(Time, TraceEvent)>>>,
}

impl VecSink {
    pub fn new(categories: &[Category]) -> Self {
        VecSink { categories: categories.to_vec(), events: Arc::default() }
    }
    pub fn take(&self) -> Vec<(Time, TraceEvent)> {
        std::mem::take(&mut self.events.lock().unwrap())
    }
}

impl TraceSink for VecSink {
    fn wants(&self, category: Category) -> bool {
        self.categories.contains(&category)
    }
    fn event(&mut self, time: Time, event: TraceEvent) {
        self.events.lock().unwrap().push((time, event));
    }
}

/// Writes one line per event to a writer (stderr, a file).
pub struct WriterSink<W: std::io::Write + Send> {
    pub categories: Vec<Category>,
    pub out: W,
    pub prefix: String,
}

impl<W: std::io::Write + Send> TraceSink for WriterSink<W> {
    fn wants(&self, category: Category) -> bool {
        self.categories.contains(&category)
    }
    fn event(&mut self, time: Time, event: TraceEvent) {
        let _ = writeln!(self.out, "{}[{time}] {event}", self.prefix);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vec_sink_filters_by_category() {
        let sink = VecSink::new(&[Category::Gpio]);
        assert!(sink.wants(Category::Gpio));
        assert!(!sink.wants(Category::Instruction));
        let mut s = sink.clone();
        s.event(Time(1), TraceEvent::Gpio { pin: 25, level: true });
        let ev = sink.take();
        assert_eq!(ev.len(), 1);
        assert_eq!(ev[0].1.to_string(), "GPIO25 = 1");
    }
}
