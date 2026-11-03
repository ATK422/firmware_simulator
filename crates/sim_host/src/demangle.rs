//! Rust symbol demangling for profiles and symbolized addresses: the legacy `_ZN...E` scheme and the v0 scheme
//! (`_R...`, which titan_firmware_rs builds with). Names come out as `rustc-demangle`'s alternate form prints them:
//! no hashes or crate disambiguators. C names and anything this does not understand come back unchanged.

/// Demangles a Rust symbol: `_ZN4core3fmt5write17h0123456789abcdefE` and `_RNvNtCs1234_4core3fmt5write` both
/// become `core::fmt::write`. LLVM's `.llvm.NNN` suffixes are removed first.
pub fn demangle(name: &str) -> String {
    let base = match name.find(".llvm.") {
        Some(i) => &name[..i],
        None => name,
    };
    let v0 = base.strip_prefix("_R").or_else(|| base.strip_prefix("__R"));
    let out = match v0 {
        Some(rest) => v0::demangle(rest),
        None => legacy(base),
    };
    out.unwrap_or_else(|| name.to_string())
}

fn legacy(base: &str) -> Option<String> {
    let inner = base.strip_prefix("__ZN").or_else(|| base.strip_prefix("_ZN")).or_else(|| base.strip_prefix("ZN"))?;
    let bytes = inner.as_bytes();
    let mut i = 0;
    let mut parts: Vec<&str> = Vec::new();
    loop {
        match bytes.get(i)? {
            b'E' => break,
            b'0'..=b'9' => {
                let start = i;
                while bytes.get(i)?.is_ascii_digit() {
                    i += 1;
                }
                let len: usize = inner[start..i].parse().ok()?;
                let part = inner.get(i..i + len)?;
                parts.push(part);
                i += len;
            }
            _ => return None,
        }
    }
    // Anything after the closing E other than nothing means this is not what we think it is
    if i + 1 != bytes.len() {
        return None;
    }
    if parts.is_empty() {
        return None;
    }
    if parts.len() > 1 && is_hash(parts[parts.len() - 1]) {
        parts.pop();
    }
    let mut out = String::new();
    for (k, p) in parts.iter().enumerate() {
        if k > 0 {
            out.push_str("::");
        }
        decode_part(p, &mut out)?;
    }
    Some(out)
}

fn is_hash(s: &str) -> bool {
    s.len() == 17 && s.starts_with('h') && s[1..].bytes().all(|b| b.is_ascii_hexdigit())
}

fn decode_part(p: &str, out: &mut String) -> Option<()> {
    let mut rest = p;
    // A leading `_$` escapes a `$` at the start of an identifier
    if rest.starts_with("_$") {
        rest = &rest[1..];
    }
    while !rest.is_empty() {
        if let Some(r) = rest.strip_prefix("..") {
            out.push_str("::");
            rest = r;
        } else if rest.starts_with('$') {
            let end = rest[1..].find('$')? + 1;
            let esc = &rest[1..end];
            let c = match esc {
                "SP" => '@',
                "BP" => '*',
                "RF" => '&',
                "LT" => '<',
                "GT" => '>',
                "LP" => '(',
                "RP" => ')',
                "C" => ',',
                _ => {
                    let hex = esc.strip_prefix('u')?;
                    char::from_u32(u32::from_str_radix(hex, 16).ok()?)?
                }
            };
            out.push(c);
            rest = &rest[end + 1..];
        } else {
            let n = rest.find(['$', '.']).unwrap_or(rest.len());
            // A single `.` (not `..`) is kept as is
            let n = if n == 0 { 1 } else { n };
            out.push_str(&rest[..n]);
            rest = &rest[n..];
        }
    }
    Some(())
}

/// The v0 mangling scheme (RFC 2603).
mod v0 {
    /// Recursion bound against hostile or broken input.
    const MAX_DEPTH: u32 = 200;

    struct Parser<'a> {
        s: &'a [u8],
        pos: usize,
        depth: u32,
        out: String,
    }

    pub fn demangle(sym: &str) -> Option<String> {
        let mut p = Parser { s: sym.as_bytes(), pos: 0, depth: 0, out: String::new() };
        // An optional encoding version (decimal) comes first
        if p.peek()?.is_ascii_digit() {
            return None;
        }
        p.path(false)?;
        // What follows (the instantiating crate, a vendor suffix after `.`) is not part of the name
        Some(p.out)
    }

    impl Parser<'_> {
        fn peek(&self) -> Option<u8> {
            self.s.get(self.pos).copied()
        }

        fn next(&mut self) -> Option<u8> {
            let b = self.peek()?;
            self.pos += 1;
            Some(b)
        }

        fn eat(&mut self, b: u8) -> bool {
            if self.peek() == Some(b) {
                self.pos += 1;
                true
            } else {
                false
            }
        }

        fn enter(&mut self) -> Option<()> {
            self.depth += 1;
            (self.depth < MAX_DEPTH).then_some(())
        }

        /// `{0-9a-zA-Z} _`: `_` is 0, otherwise the digits plus one.
        fn base62(&mut self) -> Option<u64> {
            if self.eat(b'_') {
                return Some(0);
            }
            let mut v: u64 = 0;
            loop {
                let b = self.next()?;
                let d = match b {
                    b'0'..=b'9' => b - b'0',
                    b'a'..=b'z' => b - b'a' + 10,
                    b'A'..=b'Z' => b - b'A' + 36,
                    b'_' => return v.checked_add(1),
                    _ => return None,
                };
                v = v.checked_mul(62)?.checked_add(d as u64)?;
            }
        }

        fn opt_base62(&mut self, tag: u8) -> Option<u64> {
            if self.eat(tag) { self.base62().map(|v| v + 1) } else { Some(0) }
        }

        fn decimal(&mut self) -> Option<usize> {
            let start = self.pos;
            if self.peek() == Some(b'0') {
                self.pos += 1;
                return Some(0);
            }
            while self.peek().is_some_and(|b| b.is_ascii_digit()) {
                self.pos += 1;
            }
            std::str::from_utf8(&self.s[start..self.pos]).ok()?.parse().ok()
        }

        /// An identifier without its disambiguator: `[u] <decimal> [_] <bytes>`.
        fn ident(&mut self) -> Option<String> {
            let puny = self.eat(b'u');
            let len = self.decimal()?;
            self.eat(b'_');
            let bytes = self.s.get(self.pos..self.pos + len)?;
            self.pos += len;
            let s = std::str::from_utf8(bytes).ok()?.to_string();
            Some(if puny { format!("punycode{{{s}}}") } else { s })
        }

        /// Runs `f` at the position a backref (`B <base62>`) points to, then comes back.
        fn backref(&mut self, f: impl FnOnce(&mut Self) -> Option<()>) -> Option<()> {
            let target = self.base62()? as usize;
            if target >= self.pos {
                return None;
            }
            let back = std::mem::replace(&mut self.pos, target);
            f(self)?;
            self.pos = back;
            Some(())
        }

        fn path(&mut self, in_type: bool) -> Option<()> {
            self.enter()?;
            match self.next()? {
                b'C' => {
                    self.opt_base62(b's')?;
                    let name = self.ident()?;
                    self.out.push_str(&name);
                }
                b'M' => {
                    self.opt_base62(b's')?;
                    self.skip_path()?;
                    self.out.push('<');
                    self.ty()?;
                    self.out.push('>');
                }
                b'X' => {
                    self.opt_base62(b's')?;
                    self.skip_path()?;
                    self.out.push('<');
                    self.ty()?;
                    self.out.push_str(" as ");
                    self.path(true)?;
                    self.out.push('>');
                }
                b'Y' => {
                    self.out.push('<');
                    self.ty()?;
                    self.out.push_str(" as ");
                    self.path(true)?;
                    self.out.push('>');
                }
                b'N' => {
                    let ns = self.next()?;
                    self.path(in_type)?;
                    let dis = self.opt_base62(b's')?;
                    let name = self.ident()?;
                    match ns {
                        // A tuple struct or variant constructor (`c`) has no name of its own
                        b'a'..=b'z' if name.is_empty() => {}
                        b'a'..=b'z' => {
                            self.out.push_str("::");
                            self.out.push_str(&name);
                        }
                        _ => {
                            let kind = match ns {
                                b'C' => "closure".to_string(),
                                b'S' => "shim".to_string(),
                                other => (other as char).to_string(),
                            };
                            self.out.push_str("::{");
                            self.out.push_str(&kind);
                            if !name.is_empty() {
                                self.out.push(':');
                                self.out.push_str(&name);
                            }
                            self.out.push_str(&format!("#{dis}}}"));
                        }
                    }
                }
                b'I' => {
                    self.path(in_type)?;
                    self.out.push_str(if in_type { "<" } else { "::<" });
                    let mut first = true;
                    while !self.eat(b'E') {
                        if !first {
                            self.out.push_str(", ");
                        }
                        first = false;
                        self.generic_arg()?;
                    }
                    self.out.push('>');
                }
                b'B' => self.backref(|p| p.path(in_type))?,
                _ => return None,
            }
            self.depth -= 1;
            Some(())
        }

        /// Parses a path without printing it (the impl path of `M`/`X`, which is not shown).
        fn skip_path(&mut self) -> Option<()> {
            let len = self.out.len();
            self.path(false)?;
            self.out.truncate(len);
            Some(())
        }

        fn generic_arg(&mut self) -> Option<()> {
            if self.eat(b'L') {
                self.base62()?;
                self.out.push_str("'_");
                Some(())
            } else if self.eat(b'K') {
                self.konst()
            } else {
                self.ty()
            }
        }

        fn lifetime_opt(&mut self) -> Option<()> {
            if self.eat(b'L') {
                self.base62()?;
            }
            Some(())
        }

        fn binder_opt(&mut self) -> Option<()> {
            if self.eat(b'G') {
                self.base62()?;
            }
            Some(())
        }

        fn ty(&mut self) -> Option<()> {
            self.enter()?;
            let b = self.peek()?;
            let basic = match b {
                b'a' => Some("i8"),
                b'b' => Some("bool"),
                b'c' => Some("char"),
                b'd' => Some("f64"),
                b'e' => Some("str"),
                b'f' => Some("f32"),
                b'h' => Some("u8"),
                b'i' => Some("isize"),
                b'j' => Some("usize"),
                b'l' => Some("i32"),
                b'm' => Some("u32"),
                b'n' => Some("i128"),
                b'o' => Some("u128"),
                b's' => Some("i16"),
                b't' => Some("u16"),
                b'u' => Some("()"),
                b'v' => Some("..."),
                b'x' => Some("i64"),
                b'y' => Some("u64"),
                b'z' => Some("!"),
                b'p' => Some("_"),
                _ => None,
            };
            if let Some(name) = basic {
                self.pos += 1;
                self.out.push_str(name);
                self.depth -= 1;
                return Some(());
            }
            self.pos += 1;
            match b {
                b'A' | b'S' => {
                    self.out.push('[');
                    self.ty()?;
                    if b == b'A' {
                        self.out.push_str("; ");
                        self.konst()?;
                    }
                    self.out.push(']');
                }
                b'T' => {
                    self.out.push('(');
                    let mut n = 0;
                    while !self.eat(b'E') {
                        if n > 0 {
                            self.out.push_str(", ");
                        }
                        self.ty()?;
                        n += 1;
                    }
                    if n == 1 {
                        self.out.push(',');
                    }
                    self.out.push(')');
                }
                b'R' | b'Q' => {
                    self.lifetime_opt()?;
                    self.out.push_str(if b == b'R' { "&" } else { "&mut " });
                    self.ty()?;
                }
                b'P' | b'O' => {
                    self.out.push_str(if b == b'P' { "*const " } else { "*mut " });
                    self.ty()?;
                }
                b'F' => {
                    self.binder_opt()?;
                    if self.eat(b'U') {
                        self.out.push_str("unsafe ");
                    }
                    if self.eat(b'K') {
                        let abi = if self.eat(b'C') { "C".to_string() } else { self.ident()?.replace('_', "-") };
                        self.out.push_str(&format!("extern \"{abi}\" "));
                    }
                    self.out.push_str("fn(");
                    let mut first = true;
                    while !self.eat(b'E') {
                        if !first {
                            self.out.push_str(", ");
                        }
                        first = false;
                        self.ty()?;
                    }
                    self.out.push(')');
                    let len = self.out.len();
                    self.out.push_str(" -> ");
                    self.ty()?;
                    if self.out[len..] == *" -> ()" {
                        self.out.truncate(len);
                    }
                }
                b'D' => {
                    self.binder_opt()?;
                    self.out.push_str("dyn ");
                    let mut first = true;
                    while !self.eat(b'E') {
                        if !first {
                            self.out.push_str(" + ");
                        }
                        first = false;
                        self.dyn_trait()?;
                    }
                    self.lifetime_opt()?;
                }
                b'B' => self.backref(|p| p.ty())?,
                _ => {
                    // A named type: a path
                    self.pos -= 1;
                    self.path(true)?;
                }
            }
            self.depth -= 1;
            Some(())
        }

        fn dyn_trait(&mut self) -> Option<()> {
            let len = self.out.len();
            self.path(true)?;
            let mut open = false;
            while self.eat(b'p') {
                // `Trait<Item = T>`: merge into the trait's generic list if it has one
                if !open {
                    if self.out.ends_with('>') && self.out.len() > len {
                        self.out.pop();
                        self.out.push_str(", ");
                    } else {
                        self.out.push('<');
                    }
                    open = true;
                } else {
                    self.out.push_str(", ");
                }
                let name = self.ident()?;
                self.out.push_str(&name);
                self.out.push_str(" = ");
                self.ty()?;
            }
            if open {
                self.out.push('>');
            }
            Some(())
        }

        fn konst(&mut self) -> Option<()> {
            self.enter()?;
            let b = self.next()?;
            match b {
                b'p' => self.out.push('_'),
                b'B' => self.backref(|p| p.konst())?,
                b'h' | b't' | b'm' | b'y' | b'o' | b'j' | b'a' | b's' | b'l' | b'x' | b'n' | b'i' => {
                    let neg = self.eat(b'n');
                    let v = self.hex()?;
                    if neg {
                        self.out.push('-');
                    }
                    self.out.push_str(&v.to_string());
                }
                b'b' => {
                    let v = self.hex()?;
                    self.out.push_str(if v == 0 { "false" } else { "true" });
                }
                b'c' => {
                    let v = self.hex()?;
                    let c = char::from_u32(v as u32)?;
                    self.out.push_str(&format!("{c:?}"));
                }
                _ => return None,
            }
            self.depth -= 1;
            Some(())
        }

        fn hex(&mut self) -> Option<u128> {
            let mut v: u128 = 0;
            loop {
                let b = self.next()?;
                if b == b'_' {
                    return Some(v);
                }
                let d = (b as char).to_digit(16)?;
                v = v.checked_mul(16)?.checked_add(d as u128)?;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plain_paths_lose_their_hash() {
        assert_eq!(demangle("_ZN4core3fmt5write17h0123456789abcdefE"), "core::fmt::write");
        assert_eq!(demangle("_ZN9esc_board4main17h5b4bd8a1d8a4f4c0E"), "esc_board::main");
    }

    #[test]
    fn escapes_are_decoded() {
        assert_eq!(
            demangle(
                "_ZN67_$LT$embassy_executor..raw..TaskStorage$LT$F$GT$$u20$as$u20$Foo$GT$4poll17h1111111111111111E"
            ),
            "<embassy_executor::raw::TaskStorage<F> as Foo>::poll"
        );
        assert_eq!(demangle("_ZN4core3ptr35drop_in_place$LT$$RF$mut$u20$u8$GT$17h2222222222222222E"), {
            "core::ptr::drop_in_place<&mut u8>"
        });
        assert_eq!(demangle("_ZN3foo34closure$u7b$$u7b$closure$u7d$$u7d$17h3333333333333333E"), {
            "foo::closure{{closure}}"
        });
        assert_eq!(demangle("_ZN21tuple$LP$u8$C$u16$RP$E"), "tuple(u8,u16)");
    }

    #[test]
    fn llvm_suffixes_and_other_prefixes() {
        assert_eq!(demangle("_ZN3foo3bar17h0123456789abcdefE.llvm.12345"), "foo::bar");
        assert_eq!(demangle("__ZN3foo3barE"), "foo::bar");
    }

    #[test]
    fn v0_paths_impls_and_closures() {
        // Checked against binutils' c++filt (with the crate disambiguators it prints left out)
        assert_eq!(demangle("_RNvCs9hJ03s5DiqP_7___rustc17rust_begin_unwind"), "__rustc::rust_begin_unwind");
        assert_eq!(
            demangle(
                "_RNvMs1_NtCskgAysEWbKMR_16embassy_executor3rawINtB5_11TaskStorageNCNvNvCs7oVxjE0vJcs_8template21_____embassy_main_task36_____embassy_main_task_inner_function0E4pollB16_"
            ),
            "<embassy_executor::raw::TaskStorage<template::____embassy_main_task::____embassy_main_task_inner_function::{closure#0}>>::poll"
        );
        assert_eq!(
            demangle(
                "_RINvMNtNtCskgAysEWbKMR_16embassy_executor8platform6threadNtB3_8Executor3runNCNvCs7oVxjE0vJcs_8template18___cortex_m_rt_main0EB1f_"
            ),
            "<embassy_executor::platform::thread::Executor>::run::<template::__cortex_m_rt_main::{closure#0}>"
        );
        assert_eq!(
            demangle(
                "_RNvXNtCskihDoYKjTNZ_10embassy_rp21critical_section_implNtB2_12RpSpinlockCsNtCsja446QzUmEa_16critical_section4Impl7acquire"
            ),
            "<embassy_rp::critical_section_impl::RpSpinlockCs as critical_section::Impl>::acquire"
        );
    }

    #[test]
    fn v0_types() {
        // Real symbols from titan_firmware_rs images, checked against c++filt
        assert_eq!(
            demangle(
                "_RINvXs1t_NtNtCsj6Yl71vyKPP_10serde_core2de5implsAsj8_NtB9_11Deserialize11deserializeQINtNtNtCs8tXflx8ljP2_8postcard2de12deserializer12DeserializerNtNtB1q_7flavors5SliceEECsbs1mcHoSUOw_12titan_rp2040"
            ),
            "<[i16; 8] as serde_core::de::Deserialize>::deserialize::<&mut postcard::de::deserializer::Deserializer<postcard::de::flavors::Slice>>"
        );
        assert_eq!(
            demangle("_RNvXs1g_NtCs8Vz1rmbUfIk_4core3fmtRDNtB6_5DebugEL_Bx_3fmtB8_"),
            "<&dyn core::fmt::Debug as core::fmt::Debug>::fmt"
        );
        assert_eq!(
            demangle("_RINvNtCshQdu0uZHxqE_5defmt6export3fmtReECsbs1mcHoSUOw_12titan_rp2040"),
            "defmt::export::fmt::<&str>"
        );
        // A variant constructor in a generic argument
        assert!(
            demangle("_RINvMs_NtNtCs8dbknvxPaF4_7smoltcp7storage11ring_bufferINtB5_10RingBufferINtNtB7_13packet_buffer14PacketMetadataNtNtNtB9_6socket3udp11UdpMetadataEE16enqueue_one_withQB18_NtB7_4FullNcNtINtNtCs8Vz1rmbUfIk_4core6result6ResultB2C_B2H_E2Ok0EB9_")
                .ends_with("smoltcp::storage::Full>::Ok>")
        );
        // Hand-made: tuples, pointers, fn pointers
        assert_eq!(demangle("_RINvCs1_1a1fTRmOhEE"), "a::f::<(&u32, *mut u8)>");
        assert_eq!(demangle("_RINvCs1_1a1fTlEE"), "a::f::<(i32,)>");
        assert_eq!(demangle("_RINvCs1_1a1fFUKCmEuEE"), "a::f::<unsafe extern \"C\" fn(u32)>");
        // Malformed input is left alone
        assert_eq!(demangle("_RNvC"), "_RNvC");
        assert_eq!(demangle("_RB0_"), "_RB0_");
    }

    #[test]
    fn non_rust_names_are_unchanged() {
        for n in ["main", "__aeabi_uidiv", "_ZN", "_ZN3fooX", "_ZN99fooE", "SysTick", "_ZN3fooEjunk"] {
            assert_eq!(demangle(n), n, "{n}");
        }
    }
}

/// Prints the demangled form of every symbol in `$DEMANGLE_FILE` (one per line), for comparing against another
/// demangler: `DEMANGLE_FILE=syms.txt cargo test --release -p sim_host --lib -- --ignored demangle_file`.
#[cfg(test)]
#[test]
#[ignore]
fn demangle_file() {
    let Ok(path) = std::env::var("DEMANGLE_FILE") else { return };
    for line in std::fs::read_to_string(path).unwrap().lines() {
        println!("{}", demangle(line.trim()));
    }
}
