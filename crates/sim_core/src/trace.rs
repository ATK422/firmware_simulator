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
        // One write per line, so lines of boards on several threads sharing a writer do not mix
        let line = format!("{}[{time}] {event}\n", self.prefix);
        let _ = self.out.write_all(line.as_bytes());
    }
}

/// Passes each event to every sink that wants its category (a printed trace and a waveform file at once).
#[derive(Default)]
pub struct MultiSink {
    pub sinks: Vec<Box<dyn TraceSink>>,
}

impl MultiSink {
    pub fn new(sinks: Vec<Box<dyn TraceSink>>) -> Self {
        MultiSink { sinks }
    }

    /// `sink` combined with `extra`, unless `sink` wants nothing (then `extra` alone).
    pub fn combine(sink: Box<dyn TraceSink>, extra: Box<dyn TraceSink>) -> Box<dyn TraceSink> {
        const ALL: [Category; 6] = [
            Category::Instruction,
            Category::Exception,
            Category::Gpio,
            Category::Peripheral,
            Category::Unmodelled,
            Category::Device,
        ];
        if ALL.iter().any(|&c| sink.wants(c)) { Box::new(MultiSink::new(vec![sink, extra])) } else { extra }
    }
}

impl TraceSink for MultiSink {
    fn wants(&self, category: Category) -> bool {
        self.sinks.iter().any(|s| s.wants(category))
    }
    fn event(&mut self, time: Time, event: TraceEvent) {
        let c = event.category();
        let n = self.sinks.iter().filter(|s| s.wants(c)).count();
        let mut seen = 0;
        let mut event = Some(event);
        for s in &mut self.sinks {
            if s.wants(c) {
                seen += 1;
                // The last interested sink gets the event itself, the others a copy
                let ev = if seen == n { event.take().unwrap() } else { event.clone().unwrap() };
                s.event(time, ev);
            }
        }
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

    #[test]
    fn multi_sink_fans_out_by_category() {
        let gpio = VecSink::new(&[Category::Gpio]);
        let both = VecSink::new(&[Category::Gpio, Category::Device]);
        let mut m = MultiSink::new(vec![Box::new(gpio.clone()), Box::new(both.clone())]);
        assert!(m.wants(Category::Device) && m.wants(Category::Gpio) && !m.wants(Category::Instruction));
        m.event(Time(1), TraceEvent::Gpio { pin: 1, level: true });
        m.event(Time(2), TraceEvent::Device { source: "x".into(), message: "y".into() });
        assert_eq!(gpio.take().len(), 1);
        assert_eq!(both.take().len(), 2);
        // Combining with a sink that wants nothing keeps just the other
        let c = MultiSink::combine(Box::new(NullSink), Box::new(gpio.clone()));
        assert!(c.wants(Category::Gpio) && !c.wants(Category::Device));
    }
}
