//! A structured log for tools: one JSON object per line for every log line, UART and USB serial line, CAN frame
//! and injected fault of a run, each with its virtual time (`t` in seconds, `t_ns` exact) and the board or bus it
//! came from.
//!
//! ```text
//! {"kind":"log","board":"esc0","t":0.137,"t_ns":137000676,"level":"info","fw_time":"0.003398","message":"CAN controller online as client 2"}
//! {"kind":"uart","board":"power","t":0.2,"t_ns":200000000,"uart":0,"line":"Hello"}
//! {"kind":"can","bus":"internal","t":0.8,"t_ns":800658092,"start_ns":800600000,"id":"0x2020801","extended":true,"fd":true,"brs":true,"data":"0100","acked":true,"error":null,"sender":"node 0","candump":"02020801##10100"}
//! {"kind":"fault","board":"camera_cage","t":1.0,"t_ns":1000000000,"fault":"i2c-absent:0x76","ok":true,"error":null}
//! {"kind":"end","t":3.0,"t_ns":3000000000,"reason":"TimeLimit"}
//! ```

use std::io::Write;
use std::sync::{Arc, Mutex};

use serde_json::Value;
use sim_core::time::Time;
use sim_devices::can::{BusRecord, Sender};

use crate::Session;
use crate::robot::RobotBus;

/// A JSON lines log being written; clone the handle to share it between boards.
#[derive(Clone)]
pub struct JsonLog(Arc<Mutex<Box<dyn Write + Send>>>);

fn stamp(kind: &str, t: Time) -> serde_json::Map<String, Value> {
    let mut m = serde_json::Map::new();
    m.insert("kind".into(), kind.into());
    m.insert("t".into(), t.as_secs_f64().into());
    m.insert("t_ns".into(), t.as_ns().into());
    m
}

/// A bus record as a JSON object.
pub fn can_json(bus: &str, r: &BusRecord) -> Value {
    let mut m = stamp("can", r.end);
    m.insert("bus".into(), bus.into());
    m.insert("start_ns".into(), r.start.as_ns().into());
    let f = &r.frame;
    m.insert("id".into(), format!("{:#x}", f.id).into());
    m.insert("extended".into(), f.extended.into());
    m.insert("fd".into(), f.fd.into());
    m.insert("brs".into(), f.brs.into());
    m.insert("rtr".into(), f.rtr.into());
    m.insert("data".into(), f.data.iter().map(|b| format!("{b:02x}")).collect::<String>().into());
    m.insert("acked".into(), r.acked.into());
    m.insert("error".into(), r.error.as_ref().map(|e| format!("{e:?}")).into());
    let sender = match r.sender {
        Sender::Node(n) => format!("node {n}"),
        Sender::Endpoint(n) => format!("endpoint {n}"),
        Sender::Flood(n) => format!("flood {n}"),
    };
    m.insert("sender".into(), sender.into());
    m.insert("candump".into(), sim_devices::can::candump(f).into());
    Value::Object(m)
}

impl JsonLog {
    pub fn create(path: &std::path::Path) -> std::io::Result<JsonLog> {
        let f = std::fs::File::create(path)?;
        Ok(JsonLog::new(Box::new(std::io::BufWriter::new(f))))
    }

    pub fn new(out: Box<dyn Write + Send>) -> JsonLog {
        JsonLog(Arc::new(Mutex::new(out)))
    }

    /// Writes one record.
    pub fn write(&self, v: &Value) {
        let mut out = self.0.lock().unwrap();
        let _ = writeln!(out, "{v}");
    }

    /// Records `board`'s log, UART and USB serial lines (keeping what the session already does with them) and,
    /// for a board with its own bus, that bus's frames as bus `bus`.
    pub fn tap_session(&self, board: &str, bus: Option<&str>, s: &mut Session) {
        let (log, name) = (self.clone(), board.to_string());
        let mut prev = s.on_log.take();
        s.on_log = Some(Box::new(move |t, line| {
            if let Some(p) = &mut prev {
                p(t, line);
            }
            let mut m = stamp("log", t);
            m.insert("board".into(), name.clone().into());
            m.insert("level".into(), line.level.as_ref().map(|l| l.to_lowercase()).into());
            m.insert("fw_time".into(), line.timestamp.clone().into());
            m.insert("message".into(), line.message.clone().into());
            log.write(&Value::Object(m));
        }));
        let (log, name) = (self.clone(), board.to_string());
        let mut prev = s.on_uart_line.take();
        s.on_uart_line = Some(Box::new(move |t, n, line| {
            if let Some(p) = &mut prev {
                p(t, n, line);
            }
            let mut m = stamp("uart", t);
            m.insert("board".into(), name.clone().into());
            m.insert("uart".into(), n.into());
            m.insert("line".into(), line.into());
            log.write(&Value::Object(m));
        }));
        let (log, name) = (self.clone(), board.to_string());
        let mut prev = s.on_usb_cdc_line.take();
        s.on_usb_cdc_line = Some(Box::new(move |t, line| {
            if let Some(p) = &mut prev {
                p(t, line);
            }
            let mut m = stamp("usb", t);
            m.insert("board".into(), name.clone().into());
            m.insert("line".into(), line.into());
            log.write(&Value::Object(m));
        }));
        if let (Some(bus), Some(b)) = (bus, &mut s.can_bus) {
            b.record.get_or_insert_with(Vec::new);
            let (log, bus) = (self.clone(), bus.to_string());
            let mut prev = s.on_can.take();
            s.on_can = Some(Box::new(move |r| {
                if let Some(p) = &mut prev {
                    p(r);
                }
                log.write(&can_json(&bus, r));
            }));
        }
    }

    /// Records the frames of a robot bus.
    pub fn tap_bus(&self, bus: &str, rb: &mut RobotBus) {
        rb.bus.record.get_or_insert_with(Vec::new);
        let (log, bus) = (self.clone(), bus.to_string());
        let mut prev = rb.on_can.take();
        rb.on_can = Some(Box::new(move |r| {
            if let Some(p) = &mut prev {
                p(r);
            }
            log.write(&can_json(&bus, r));
        }));
    }

    /// Records a fault applied to `board`.
    pub fn fault(&self, t: Time, board: &str, fault: &str, result: &Result<(), String>) {
        let mut m = stamp("fault", t);
        m.insert("board".into(), board.into());
        m.insert("fault".into(), fault.into());
        m.insert("ok".into(), result.is_ok().into());
        m.insert("error".into(), result.as_ref().err().cloned().into());
        self.write(&Value::Object(m));
    }

    /// Records the end of the run and flushes the file.
    pub fn finish(&self, t: Time, reason: &str) -> std::io::Result<()> {
        let mut m = stamp("end", t);
        m.insert("reason".into(), reason.into());
        self.write(&Value::Object(m));
        self.0.lock().unwrap().flush()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Default)]
    struct Shared(Arc<Mutex<Vec<u8>>>);
    impl Write for Shared {
        fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(b);
            Ok(b.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn records_are_json_lines() {
        let buf = Shared::default();
        let log = JsonLog::new(Box::new(buf.clone()));
        let rec = BusRecord {
            start: Time::from_us(10),
            end: Time::from_us(120),
            sender: Sender::Node(1),
            frame: sim_devices::can::CanFrame::classic(0x123, false, &[1, 2, 0xab]),
            acked: true,
            error: None,
        };
        log.write(&can_json("internal", &rec));
        log.fault(Time::from_ms(5), "esc0", "can-stuck", &Err("nope".into()));
        log.finish(Time::from_ms(7), "TimeLimit").unwrap();
        let text = String::from_utf8(buf.0.lock().unwrap().clone()).unwrap();
        let lines: Vec<Value> = text.lines().map(|l| serde_json::from_str(l).unwrap()).collect();
        assert_eq!(lines.len(), 3);
        assert_eq!(lines[0]["kind"], "can");
        assert_eq!(lines[0]["id"], "0x123");
        assert_eq!(lines[0]["data"], "0102ab");
        assert_eq!(lines[0]["t_ns"], 120_000);
        assert_eq!(lines[0]["sender"], "node 1");
        assert_eq!(lines[0]["candump"], "123#0102AB");
        assert_eq!((&lines[1]["ok"], &lines[1]["error"]), (&Value::from(false), &Value::from("nope")));
        assert_eq!(lines[2]["reason"], "TimeLimit");
    }
}
