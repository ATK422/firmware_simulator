//! Messages as micro-CDR lays them out (Micro-CDR 2.0.2, which libnanoros builds in), to and from JSON.
//!
//! Little-endian (the RP2040's, `UCDR_MACHINE_ENDIANNESS`); each primitive aligned to its size (8-byte types to 8)
//! counting from the start of the micro-CDR buffer; `bool` one byte; a string a `u32` length that counts its
//! terminating NUL, then its bytes and the NUL; a fixed array its elements one after another (aligned once, to the
//! element); a nested message its fields in place. In JSON a message is an object of its fields, an array a list,
//! `char`/`byte` a number. Missing fields encode as zero (an empty string), as the firmware's zeroed structs do.
//!
//! Where the buffer starts is what differs between the directions (see `docs/nanoros.md`): the board's
//! `nros_publish` writes the 4-byte encapsulation header `00 01 00 00` into the same buffer and the body after it,
//! so a board's payload is aligned from the start of the header ([`decode_payload`]); nano_ros_agent strips the
//! header off what it forwards to the board, which aligns from the start of the body ([`encode`]).

use serde_json::{Map, Number, Value};

use super::msgs::{FieldType, Prim, Registry};

/// The CDR encapsulation header the board puts first (`CDR_ENCAP_HEADER`: little-endian plain CDR).
pub const ENCAPSULATION: [u8; 4] = [0x00, 0x01, 0x00, 0x00];
/// The longest string the generated C structs hold, NUL included (`char name[256]`): a longer one does not
/// deserialize on the board (`ucdr_deserialize_string` with `sizeof` the field as its capacity).
pub const STRING_CAPACITY: usize = 256;

/// `msg` (JSON) as type `type_name`, aligned from the start of the returned bytes.
pub fn encode(reg: &Registry, type_name: &str, msg: &Value) -> Result<Vec<u8>, String> {
    let mut w = Writer { buf: Vec::new() };
    w.message(reg, type_name, msg, "")?;
    Ok(w.buf)
}

/// A board's DATA payload: the encapsulation header, then the message, aligned from the header's start.
pub fn decode_payload(reg: &Registry, type_name: &str, payload: &[u8]) -> Result<Value, String> {
    if payload.len() < 4 {
        return Err(format!("payload of {} byte(s): no encapsulation header", payload.len()));
    }
    if payload[..4] != ENCAPSULATION {
        return Err(format!(
            "encapsulation header {:02x?}, not {:02x?} (little-endian CDR)",
            &payload[..4],
            ENCAPSULATION
        ));
    }
    decode_from(reg, type_name, payload, 4)
}

/// A message aligned from the start of `bytes`, as the board deserializes what the agent forwards.
pub fn decode(reg: &Registry, type_name: &str, bytes: &[u8]) -> Result<Value, String> {
    decode_from(reg, type_name, bytes, 0)
}

fn decode_from(reg: &Registry, type_name: &str, bytes: &[u8], start: usize) -> Result<Value, String> {
    let mut r = Reader { buf: bytes, pos: start };
    let v = r.message(reg, type_name, "")?;
    if r.pos != bytes.len() {
        return Err(format!("{} byte(s) left over after a {type_name}", bytes.len() - r.pos));
    }
    Ok(v)
}

fn path(prefix: &str, name: &str) -> String {
    if prefix.is_empty() { name.to_string() } else { format!("{prefix}.{name}") }
}

struct Writer {
    buf: Vec<u8>,
}

impl Writer {
    fn align(&mut self, n: usize) {
        while !self.buf.len().is_multiple_of(n) {
            self.buf.push(0);
        }
    }

    fn message(&mut self, reg: &Registry, type_name: &str, v: &Value, at: &str) -> Result<(), String> {
        let t = reg.get(type_name)?;
        let empty = Map::new();
        let obj = match v {
            Value::Object(o) => o,
            Value::Null => &empty,
            other => return Err(format!("{}: a {type_name} is an object of its fields, not {other}", at_or(at))),
        };
        if let Some(k) = obj.keys().find(|k| !t.fields.iter().any(|f| &f.name == *k)) {
            let names: Vec<&str> = t.fields.iter().map(|f| f.name.as_str()).collect();
            let why = if t.dropped.contains(k) { " (a dynamic array: nano_ros_codegen leaves it out)" } else { "" };
            return Err(format!(
                "{}: {type_name} has no field {k:?}{why} (its fields: {})",
                at_or(at),
                names.join(", ")
            ));
        }
        for f in &t.fields {
            let fv = obj.get(&f.name).unwrap_or(&Value::Null);
            let p = path(at, &f.name);
            match &f.ty {
                FieldType::Prim(prim) => self.prim(*prim, fv, &p)?,
                FieldType::PrimArray(prim, n) => {
                    let items = array(fv, *n, &p)?;
                    // A string in an array is a char (ucdr_serialize_array_char)
                    let prim = if *prim == Prim::String { Prim::Char } else { *prim };
                    self.align(prim.size());
                    for k in 0..*n {
                        self.prim(prim, items.get(k).unwrap_or(&Value::Null), &format!("{p}[{k}]"))?;
                    }
                }
                FieldType::Nested(ty) => self.message(reg, ty, fv, &p)?,
                FieldType::NestedArray(ty, n) => {
                    let items = array(fv, *n, &p)?;
                    for k in 0..*n {
                        self.message(reg, ty, items.get(k).unwrap_or(&Value::Null), &format!("{p}[{k}]"))?;
                    }
                }
            }
        }
        Ok(())
    }

    fn prim(&mut self, p: Prim, v: &Value, at: &str) -> Result<(), String> {
        self.align(p.size());
        let bad = |what: &str| format!("{at}: expected {what}, found {v}");
        match p {
            Prim::Bool => self.buf.push(match v {
                Value::Null => 0,
                Value::Bool(b) => *b as u8,
                Value::Number(n) if n.as_u64() == Some(0) || n.as_u64() == Some(1) => n.as_u64().unwrap() as u8,
                _ => return Err(bad("true or false")),
            }),
            Prim::Float32 => self.buf.extend((float(v).ok_or_else(|| bad("a number"))? as f32).to_le_bytes()),
            Prim::Float64 => self.buf.extend(float(v).ok_or_else(|| bad("a number"))?.to_le_bytes()),
            Prim::String => {
                let s = match v {
                    Value::Null => "",
                    Value::String(s) => s.as_str(),
                    _ => return Err(bad("a string")),
                };
                if s.len() >= STRING_CAPACITY || s.contains('\0') {
                    return Err(format!(
                        "{at}: a string of {} byte(s): the board's struct holds at most {} and no NUL",
                        s.len(),
                        STRING_CAPACITY - 1
                    ));
                }
                self.buf.extend((s.len() as u32 + 1).to_le_bytes());
                self.buf.extend(s.as_bytes());
                self.buf.push(0);
            }
            _ => {
                let (lo, hi) = int_range(p);
                let x: i128 = match v {
                    Value::Null => 0,
                    Value::Number(n) => n
                        .as_i64()
                        .map(i128::from)
                        .or_else(|| n.as_u64().map(i128::from))
                        .ok_or_else(|| bad("an integer"))?,
                    _ => return Err(bad("an integer")),
                };
                if x < lo || x > hi {
                    return Err(format!("{at}: {x} is out of range for a {p:?} ({lo} to {hi})"));
                }
                let bytes = (x as u64).to_le_bytes();
                self.buf.extend(&bytes[..p.size()]);
            }
        }
        Ok(())
    }
}

fn at_or(at: &str) -> &str {
    if at.is_empty() { "the message" } else { at }
}

fn float(v: &Value) -> Option<f64> {
    match v {
        Value::Null => Some(0.0),
        Value::Number(n) => n.as_f64(),
        Value::String(s) => match s.as_str() {
            "NaN" | "nan" => Some(f64::NAN),
            "inf" | "Infinity" => Some(f64::INFINITY),
            "-inf" | "-Infinity" => Some(f64::NEG_INFINITY),
            _ => None,
        },
        _ => None,
    }
}

fn array<'a>(v: &'a Value, n: usize, at: &str) -> Result<&'a [Value], String> {
    match v {
        Value::Null => Ok(&[]),
        Value::Array(a) if a.len() <= n => Ok(a),
        Value::Array(a) => Err(format!("{at}: {} items, but the array holds {n}", a.len())),
        other => Err(format!("{at}: expected a list of up to {n}, found {other}")),
    }
}

/// The range of an integer type.
fn int_range(p: Prim) -> (i128, i128) {
    match p {
        Prim::Byte | Prim::Char | Prim::Uint8 => (0, u8::MAX as i128),
        Prim::Int8 => (i8::MIN as i128, i8::MAX as i128),
        Prim::Int16 => (i16::MIN as i128, i16::MAX as i128),
        Prim::Uint16 => (0, u16::MAX as i128),
        Prim::Int32 => (i32::MIN as i128, i32::MAX as i128),
        Prim::Uint32 => (0, u32::MAX as i128),
        Prim::Int64 => (i64::MIN as i128, i64::MAX as i128),
        Prim::Uint64 => (0, u64::MAX as i128),
        _ => unreachable!("not an integer type"),
    }
}

struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl Reader<'_> {
    fn take(&mut self, n: usize, at: &str) -> Result<&[u8], String> {
        if self.pos + n > self.buf.len() {
            return Err(format!("{}: the message ends at byte {} (needs {n} more)", at_or(at), self.buf.len()));
        }
        let s = &self.buf[self.pos..self.pos + n];
        self.pos += n;
        Ok(s)
    }

    fn align(&mut self, n: usize) {
        self.pos = self.pos.div_ceil(n) * n;
    }

    fn message(&mut self, reg: &Registry, type_name: &str, at: &str) -> Result<Value, String> {
        let t = reg.get(type_name)?;
        let mut out = Map::new();
        for f in &t.fields {
            let p = path(at, &f.name);
            let v = match &f.ty {
                FieldType::Prim(prim) => self.prim(*prim, &p)?,
                FieldType::PrimArray(prim, n) => {
                    let prim = if *prim == Prim::String { Prim::Char } else { *prim };
                    self.align(prim.size());
                    Value::Array((0..*n).map(|k| self.prim(prim, &format!("{p}[{k}]"))).collect::<Result<_, _>>()?)
                }
                FieldType::Nested(ty) => self.message(reg, ty, &p)?,
                FieldType::NestedArray(ty, n) => Value::Array(
                    (0..*n).map(|k| self.message(reg, ty, &format!("{p}[{k}]"))).collect::<Result<_, _>>()?,
                ),
            };
            out.insert(f.name.clone(), v);
        }
        Ok(Value::Object(out))
    }

    fn prim(&mut self, p: Prim, at: &str) -> Result<Value, String> {
        self.align(p.size());
        let b = self.take(p.size(), at)?;
        let mut le = [0u8; 8];
        le[..b.len()].copy_from_slice(b);
        let u = u64::from_le_bytes(le);
        Ok(match p {
            Prim::Bool => Value::Bool(u != 0),
            Prim::Byte | Prim::Char | Prim::Uint8 | Prim::Uint16 | Prim::Uint32 | Prim::Uint64 => Value::from(u),
            Prim::Int8 => Value::from(u as u8 as i8),
            Prim::Int16 => Value::from(u as u16 as i16),
            Prim::Int32 => Value::from(u as u32 as i32),
            Prim::Int64 => Value::from(u as i64),
            Prim::Float32 => number(f32::from_bits(u as u32) as f64),
            Prim::Float64 => number(f64::from_bits(u)),
            Prim::String => {
                let len = u as usize;
                if len == 0 {
                    return Err(format!("{at}: a string of length 0 (micro-CDR counts the NUL)"));
                }
                let bytes = self.take(len, at)?;
                let text = bytes.strip_suffix(&[0]).unwrap_or(bytes);
                Value::String(String::from_utf8_lossy(text).into_owned())
            }
        })
    }
}

/// A float as JSON; NaN and infinities (which JSON has no numbers for) as strings.
fn number(x: f64) -> Value {
    match Number::from_f64(x) {
        Some(n) => Value::Number(n),
        None if x.is_nan() => Value::from("NaN"),
        None if x > 0.0 => Value::from("inf"),
        None => Value::from("-inf"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn reg() -> Registry {
        Registry::builtin()
    }

    #[test]
    fn primitives_little_endian_and_aligned() {
        let mut r = reg();
        r.add("t/msg/A", "uint8 a\nuint32 b\nbool c\nint16 d\nfloat64 e\nint8 f\nstring s\nchar[3] cs").unwrap();
        let v = json!({"a": 1, "b": 0x01020304, "c": true, "d": -2, "e": 1.5, "f": -1, "s": "hi", "cs": [65, 66]});
        let b = encode(&r, "t/A", &v).unwrap();
        let mut want = vec![1, 0, 0, 0, 4, 3, 2, 1, 1, 0, 0xFE, 0xFF, 0, 0, 0, 0];
        want.extend(1.5f64.to_le_bytes());
        want.extend([0xFF, 0, 0, 0, 3, 0, 0, 0, b'h', b'i', 0, 65, 66, 0]);
        assert_eq!(b, want);
        let back = decode(&r, "t/A", &b).unwrap();
        assert_eq!(
            back,
            json!({"a": 1, "b": 0x01020304, "c": true, "d": -2, "e": 1.5, "f": -1, "s": "hi", "cs": [65, 66, 0]})
        );
    }

    /// The board aligns from the encapsulation header: a float64 right after a u32 needs no padding there (offset
    /// 8 of the payload), where ROS 2 would pad it to offset 8 of the body.
    #[test]
    fn a_board_payload_aligns_from_the_header() {
        let mut r = reg();
        r.add("t/msg/B", "uint32 n\nfloat64 x").unwrap();
        let mut payload = ENCAPSULATION.to_vec();
        payload.extend(7u32.to_le_bytes());
        payload.extend(2.25f64.to_le_bytes());
        assert_eq!(decode_payload(&r, "t/B", &payload).unwrap(), json!({"n": 7, "x": 2.25}));
        // From the agent to the board the body is aligned from its own start: 4 bytes of padding
        assert_eq!(encode(&r, "t/B", &json!({"n": 7, "x": 2.25})).unwrap().len(), 16);
        assert!(decode_payload(&r, "t/B", &[0, 0, 0, 0, 1]).unwrap_err().contains("encapsulation"));
    }

    #[test]
    fn imu_and_firmware_status_round_trip() {
        let r = reg();
        let imu = json!({
            "header": {"stamp": {"sec": 3, "nanosec": 250}, "frame_id": "imu_link"},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.5, "w": 0.75},
            "orientation_covariance": [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
            "linear_acceleration": {"z": 9.81},
        });
        let body = encode(&r, "sensor_msgs/msg/Imu", &imu).unwrap();
        let back = decode(&r, "sensor_msgs/msg/Imu", &body).unwrap();
        assert_eq!(back["orientation"]["w"], json!(0.75));
        assert_eq!(back["header"]["frame_id"], json!("imu_link"));
        assert_eq!(back["linear_acceleration"], json!({"x": 0.0, "y": 0.0, "z": 9.81}));
        assert_eq!(back["angular_velocity_covariance"].as_array().unwrap().len(), 9);
        // 4 + 4 + (4 + 9) -> orientation from 24: 24 + 32 + 72 + 24 + 72 + 24 + 72
        assert_eq!(body.len(), 24 + 32 + 72 + 24 + 72 + 24 + 72);
        let st = json!({"board_name": "led", "client_id": 6, "uptime_ms": 1000, "faults": 0});
        let b = encode(&r, "riptide_msgs2/FirmwareStatus", &st).unwrap();
        let mut payload = ENCAPSULATION.to_vec();
        payload.extend(&b);
        // The body's own alignment and the header's agree for 4-byte types
        let back = decode_payload(&r, "riptide_msgs2/FirmwareStatus", &payload).unwrap();
        assert_eq!(back["board_name"], json!("led"));
        assert_eq!(back["uptime_ms"], json!(1000));
        assert_eq!(back["kill_switches_timed_out"], json!(0));
    }

    #[test]
    fn rejects_what_the_board_could_not_take() {
        let r = reg();
        let e = |v: Value| encode(&r, "std_msgs/msg/UInt8", &v).unwrap_err();
        assert!(e(json!({"data": 256})).contains("out of range"));
        assert!(e(json!({"data": -1})).contains("out of range"));
        assert!(e(json!({"data": 1.5})).contains("integer"));
        assert!(e(json!({"value": 1})).contains("no field \"value\""));
        assert!(e(json!(3)).contains("object"));
        let long = "x".repeat(256);
        assert!(encode(&r, "std_msgs/msg/String", &json!({"data": long})).unwrap_err().contains("at most 255"));
        let too_many = json!({"values": [0, 0, 0, 0, 0, 0, 0, 0, 0]});
        assert!(encode(&r, "riptide_msgs2/msg/DshotCommand", &too_many).unwrap_err().contains("holds 8"));
        assert!(decode(&r, "std_msgs/msg/UInt32", &[1, 2]).unwrap_err().contains("ends at byte 2"));
        assert!(decode(&r, "std_msgs/msg/UInt8", &[1, 2]).unwrap_err().contains("left over"));
    }

    #[test]
    fn defaults_and_special_floats() {
        let r = reg();
        let b = encode(&r, "geometry_msgs/msg/Vector3", &Value::Null).unwrap();
        assert_eq!(b, [0u8; 24]);
        let b = encode(&r, "std_msgs/msg/Float32", &json!({"data": "NaN"})).unwrap();
        assert_eq!(decode(&r, "std_msgs/msg/Float32", &b).unwrap(), json!({"data": "NaN"}));
        let b = encode(&r, "std_msgs/msg/Int64", &json!({"data": i64::MIN})).unwrap();
        assert_eq!(decode(&r, "std_msgs/msg/Int64", &b).unwrap(), json!({"data": i64::MIN}));
        let b = encode(&r, "std_msgs/msg/UInt64", &json!({"data": u64::MAX})).unwrap();
        assert_eq!(decode(&r, "std_msgs/msg/UInt64", &b).unwrap(), json!({"data": u64::MAX}));
    }
}
