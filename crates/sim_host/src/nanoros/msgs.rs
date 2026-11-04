//! ROS message types from `.msg` files, read the way nano_ros_codegen reads them (`msg_file_parser.rs`), so the
//! mock agent lays a message out exactly as the generated firmware code does.
//!
//! Like the codegen: `#` starts a comment; a line with `=` is a constant and skipped (unless its type is a bounded
//! `string<=N`); a third word (a default value) is ignored; `string<=N` is a string; `T[N]` is a fixed array;
//! **dynamic and bounded arrays (`T[]`, `T[<=N]`) are dropped** from the message (the codegen warns and leaves
//! them out of the C struct and the serializer, so they are not on the wire either); `Header` is
//! `std_msgs/msg/Header`; an unqualified type is the message's own package's.
//!
//! The emulator carries the definitions in `crates/sim_host/nanoros_msgs/` (std_msgs, builtin_interfaces,
//! geometry_msgs, sensor_msgs and riptide_msgs2's, a package per directory, usable as nano_ros_codegen's
//! `--ros-prefix`); [`Registry::add_dir`] adds more (a ROS install's `share/`, a workspace's `src/`).

use std::collections::BTreeMap;
use std::path::Path;

/// A primitive field type (`PrimType`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Prim {
    Bool,
    Byte,
    Char,
    Float32,
    Float64,
    Int8,
    Int16,
    Int32,
    Int64,
    Uint8,
    Uint16,
    Uint32,
    Uint64,
    /// `string`, `string<=N`, `wstring`: a micro-CDR string (a `char[256]` in the C struct).
    String,
}

impl Prim {
    pub fn parse(s: &str) -> Option<Prim> {
        Some(match s {
            "bool" => Prim::Bool,
            "byte" => Prim::Byte,
            "char" => Prim::Char,
            "float32" => Prim::Float32,
            "float64" => Prim::Float64,
            "int8" => Prim::Int8,
            "int16" => Prim::Int16,
            "int32" => Prim::Int32,
            "int64" => Prim::Int64,
            "uint8" => Prim::Uint8,
            "uint16" => Prim::Uint16,
            "uint32" => Prim::Uint32,
            "uint64" => Prim::Uint64,
            "string" | "wstring" => Prim::String,
            _ => return None,
        })
    }

    /// Size on the wire, which is also its CDR alignment. A string is aligned as its `u32` length; in a fixed
    /// array it is a `char` (the codegen's `ucdr_serialize_array_char`).
    pub fn size(self) -> usize {
        match self {
            Prim::Bool | Prim::Byte | Prim::Char | Prim::Int8 | Prim::Uint8 => 1,
            Prim::Int16 | Prim::Uint16 => 2,
            Prim::Float32 | Prim::Int32 | Prim::Uint32 | Prim::String => 4,
            Prim::Float64 | Prim::Int64 | Prim::Uint64 => 8,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FieldType {
    Prim(Prim),
    /// `T[N]` of a primitive.
    PrimArray(Prim, usize),
    /// A message, by its full name (`geometry_msgs/msg/Quaternion`).
    Nested(String),
    NestedArray(String, usize),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Field {
    pub name: String,
    pub ty: FieldType,
}

/// A message type: its fields as they are on the wire.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MsgType {
    /// `pkg/msg/Name`.
    pub name: String,
    pub fields: Vec<Field>,
    /// Fields the codegen leaves out (dynamic and bounded arrays).
    pub dropped: Vec<String>,
}

/// `pkg/Name` or `pkg/msg/Name` as `pkg/msg/Name`.
pub fn full_name(type_name: &str) -> Result<String, String> {
    match type_name.split('/').collect::<Vec<_>>()[..] {
        [pkg, name] | [pkg, "msg", name] if !pkg.is_empty() && !name.is_empty() => Ok(format!("{pkg}/msg/{name}")),
        _ => Err(format!("message type {type_name:?}: expected pkg/msg/Name (or pkg/Name)")),
    }
}

/// A nested type reference in package `pkg`'s `.msg` file, qualified (`resolve_nested_type`).
fn qualify(type_str: &str, pkg: &str) -> String {
    match type_str.split('/').collect::<Vec<_>>()[..] {
        [p, n] => format!("{p}/msg/{n}"),
        [_, _, _] => type_str.to_string(),
        _ if type_str == "Header" => "std_msgs/msg/Header".to_string(),
        _ => format!("{pkg}/msg/{type_str}"),
    }
}

/// Parses the text of `pkg/msg/Name`'s `.msg` file.
pub fn parse_msg(full: &str, text: &str) -> Result<MsgType, String> {
    let pkg = full.split('/').next().unwrap_or_default();
    let mut fields = Vec::new();
    let mut dropped = Vec::new();
    for (k, raw) in text.lines().enumerate() {
        let line = raw.split('#').next().unwrap_or_default().trim();
        if line.is_empty() {
            continue;
        }
        let mut words = line.split_whitespace();
        let type_token = words.next().unwrap_or_default();
        // A constant (`uint8 X=1`, `int16 X = -1`); a bounded string's `<=` is not one
        if line.contains('=') && !type_token.contains("<=") {
            continue;
        }
        let Some(name) = words.next() else { continue };
        let (base, array) = match type_token.find('[') {
            Some(b) => (&type_token[..b], Some(type_token[b + 1..].trim_end_matches(']'))),
            None => (type_token, None),
        };
        let base = base.split("<=").next().unwrap_or(base);
        let len = match array {
            None => None,
            Some(spec) if spec.is_empty() || spec.starts_with("<=") => {
                dropped.push(name.to_string());
                continue;
            }
            Some(spec) => Some(
                spec.parse::<usize>()
                    .map_err(|_| format!("{full}.msg line {}: bad array length in {type_token:?}", k + 1))?,
            ),
        };
        let ty = match (Prim::parse(base), len) {
            (Some(p), None) => FieldType::Prim(p),
            (Some(p), Some(n)) => FieldType::PrimArray(p, n),
            (None, None) => FieldType::Nested(qualify(base, pkg)),
            (None, Some(n)) => FieldType::NestedArray(qualify(base, pkg), n),
        };
        fields.push(Field { name: name.to_string(), ty });
    }
    Ok(MsgType { name: full.to_string(), fields, dropped })
}

/// The `.msg` files the emulator carries, as `(package, name, text)`.
macro_rules! builtin {
    ($($pkg:literal: [$($name:literal),*]),* $(,)?) => {
        &[$($(($pkg, $name, include_str!(concat!("../../nanoros_msgs/", $pkg, "/msg/", $name, ".msg"))),)*)*]
    };
}

const BUILTIN: &[(&str, &str, &str)] = builtin! {
    "builtin_interfaces": ["Time", "Duration"],
    "std_msgs": ["Bool", "Byte", "Char", "ColorRGBA", "Empty", "Float32", "Float64", "Header", "Int8", "Int16",
        "Int32", "Int64", "String", "UInt8", "UInt16", "UInt32", "UInt64"],
    "geometry_msgs": ["Point", "Pose", "PoseStamped", "Quaternion", "Twist", "TwistStamped", "Vector3",
        "Vector3Stamped", "Wrench"],
    "sensor_msgs": ["FluidPressure", "Imu", "RelativeHumidity", "Temperature"],
    "riptide_msgs2": ["Depth", "DshotCommand", "DshotRPMFeedback", "ElectricalCommand", "ElectricalReadings",
        "FirmwareStatus", "Int32Stamped", "KillSwitchReport", "LedCommand", "UInt8Stamped"],
};

/// Message types by full name.
#[derive(Clone, Debug, Default)]
pub struct Registry {
    types: BTreeMap<String, MsgType>,
}

impl Registry {
    /// The definitions the emulator carries.
    pub fn builtin() -> Registry {
        let mut r = Registry::default();
        for (pkg, name, text) in BUILTIN {
            r.add(&format!("{pkg}/msg/{name}"), text).expect("a built-in .msg file parses");
        }
        r
    }

    /// The built-in definitions and those under each directory of `$NANOROS_MSG_PATH` (`:`-separated), which win.
    pub fn from_env() -> Result<Registry, String> {
        let mut r = Registry::builtin();
        if let Some(path) = std::env::var_os("NANOROS_MSG_PATH") {
            for dir in std::env::split_paths(&path).filter(|d| !d.as_os_str().is_empty()) {
                r.add_dir(&dir).map_err(|e| format!("NANOROS_MSG_PATH: {e}"))?;
            }
        }
        Ok(r)
    }

    /// Adds (or replaces) `pkg/msg/Name` from its `.msg` text.
    pub fn add(&mut self, type_name: &str, text: &str) -> Result<(), String> {
        let full = full_name(type_name)?;
        let t = parse_msg(&full, text)?;
        self.types.insert(full, t);
        Ok(())
    }

    /// Adds every `PKG/msg/NAME.msg` under `dir` (searched four levels down, skipping hidden, `build`, `install`
    /// and `log` directories); the package is the `<name>` of a `package.xml` next to the `msg` directory, or the
    /// directory's name. Returns how many it added.
    pub fn add_dir(&mut self, dir: &Path) -> Result<usize, String> {
        if !dir.is_dir() {
            return Err(format!("{} is not a directory", dir.display()));
        }
        let mut n = 0;
        self.scan(dir, 4, &mut n)?;
        Ok(n)
    }

    fn scan(&mut self, dir: &Path, depth: usize, n: &mut usize) -> Result<(), String> {
        let msg_dir = dir.join("msg");
        if msg_dir.is_dir() {
            let pkg = std::fs::read_to_string(dir.join("package.xml"))
                .ok()
                .and_then(|x| Some(x.split("<name>").nth(1)?.split("</name>").next()?.trim().to_string()))
                .or_else(|| dir.file_name().map(|f| f.to_string_lossy().into_owned()))
                .unwrap_or_default();
            let mut files: Vec<_> = std::fs::read_dir(&msg_dir)
                .map_err(|e| format!("{}: {e}", msg_dir.display()))?
                .filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| p.extension().is_some_and(|x| x == "msg"))
                .collect();
            files.sort();
            for f in files {
                let text = std::fs::read_to_string(&f).map_err(|e| format!("{}: {e}", f.display()))?;
                let name = f.file_stem().unwrap_or_default().to_string_lossy();
                self.add(&format!("{pkg}/msg/{name}"), &text).map_err(|e| format!("{}: {e}", f.display()))?;
                *n += 1;
            }
        }
        if depth == 0 {
            return Ok(());
        }
        let Ok(rd) = std::fs::read_dir(dir) else { return Ok(()) };
        let mut subs: Vec<_> = rd.filter_map(|e| e.ok().map(|e| e.path())).filter(|p| p.is_dir()).collect();
        subs.sort();
        for s in subs {
            let name = s.file_name().unwrap_or_default().to_string_lossy();
            if name.starts_with('.') || matches!(&*name, "build" | "install" | "log" | "msg") {
                continue;
            }
            self.scan(&s, depth - 1, n)?;
        }
        Ok(())
    }

    /// The type `type_name` (`pkg/msg/Name` or `pkg/Name`).
    pub fn get(&self, type_name: &str) -> Result<&MsgType, String> {
        let full = full_name(type_name)?;
        self.types.get(&full).ok_or_else(|| {
            format!("unknown message type {full} (add its .msg file's package directory to NANOROS_MSG_PATH)")
        })
    }

    /// Checks that `type_name` and every type in it are known (and not nested in themselves).
    pub fn check(&self, type_name: &str) -> Result<(), String> {
        fn walk(r: &Registry, name: &str, depth: usize) -> Result<(), String> {
            if depth > 32 {
                return Err(format!("{name}: nested too deep (a type containing itself?)"));
            }
            for f in &r.get(name)?.fields {
                if let FieldType::Nested(t) | FieldType::NestedArray(t, _) = &f.ty {
                    walk(r, t, depth + 1).map_err(|e| format!("{name}.{}: {e}", f.name))?;
                }
            }
            Ok(())
        }
        walk(self, type_name, 0)
    }

    /// Whether `type_name` has a field of 8-byte alignment anywhere in it (float64, int64, uint64).
    pub fn has_8_byte_field(&self, type_name: &str) -> bool {
        let Ok(t) = self.get(type_name) else { return false };
        t.fields.iter().any(|f| match &f.ty {
            FieldType::Prim(p) | FieldType::PrimArray(p, _) => p.size() == 8,
            FieldType::Nested(n) | FieldType::NestedArray(n, _) => self.has_8_byte_field(n),
        })
    }

    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.types.keys().map(String::as_str)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_like_the_codegen() {
        let t = parse_msg(
            "pkg/msg/All",
            "# comment\nuint8 A=1\nint16 B = -2\nstring<=10 bounded\nfloat64 x 0 # default\nint16[8] values\n\
             uint8[] dynamic\nfloat32[<=4] bounded_array\nHeader header\nOther o\ngeometry_msgs/Vector3[2] vs\n\
             char c\nwstring w\n",
        )
        .unwrap();
        let f: Vec<(&str, &FieldType)> = t.fields.iter().map(|f| (f.name.as_str(), &f.ty)).collect();
        assert_eq!(
            f,
            [
                ("bounded", &FieldType::Prim(Prim::String)),
                ("x", &FieldType::Prim(Prim::Float64)),
                ("values", &FieldType::PrimArray(Prim::Int16, 8)),
                ("header", &FieldType::Nested("std_msgs/msg/Header".into())),
                ("o", &FieldType::Nested("pkg/msg/Other".into())),
                ("vs", &FieldType::NestedArray("geometry_msgs/msg/Vector3".into(), 2)),
                ("c", &FieldType::Prim(Prim::Char)),
                ("w", &FieldType::Prim(Prim::String)),
            ]
        );
        assert_eq!(t.dropped, ["dynamic", "bounded_array"]);
        assert!(parse_msg("pkg/msg/Bad", "uint8[x] y").is_err());
    }

    #[test]
    fn the_builtin_types_resolve() {
        let r = Registry::builtin();
        for name in r.names() {
            r.check(name).unwrap();
        }
        let imu = r.get("sensor_msgs/Imu").unwrap();
        assert_eq!(imu.fields.len(), 7);
        assert_eq!(r.get("riptide_msgs2/msg/FirmwareStatus").unwrap().fields.len(), 12);
        assert!(r.get("riptide_msgs2/msg/ElectricalCommand").unwrap().fields.len() == 1, "constants are skipped");
        assert!(r.has_8_byte_field("sensor_msgs/msg/Imu"));
        assert!(!r.has_8_byte_field("riptide_msgs2/msg/FirmwareStatus"));
        assert!(r.get("nope/msg/X").unwrap_err().contains("NANOROS_MSG_PATH"));
        assert!(full_name("just_a_name").is_err());
        assert_eq!(full_name("std_msgs/UInt8").unwrap(), "std_msgs/msg/UInt8");
    }

    #[test]
    fn adds_a_directory_of_packages() {
        let dir = std::env::temp_dir().join(format!("nanoros-msgs-{}", std::process::id()));
        let pkg = dir.join("src/my_pkg");
        std::fs::create_dir_all(pkg.join("msg")).unwrap();
        std::fs::write(pkg.join("package.xml"), "<package><name>my_msgs</name></package>").unwrap();
        std::fs::write(pkg.join("msg/Thing.msg"), "Header header\nuint16 n\n").unwrap();
        let mut r = Registry::builtin();
        assert_eq!(r.add_dir(&dir).unwrap(), 1);
        r.check("my_msgs/msg/Thing").unwrap();
        assert!(r.add_dir(&dir.join("nope")).is_err());
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
