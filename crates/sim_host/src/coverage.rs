//! Firmware code coverage: which functions and source lines ran, from the machines' hit bitmaps
//! ([`rp2040::coverage`]), the ELF's DWARF line table (`.debug_line`) and its symbol table.
//!
//! [`Coverage`] collects bitmaps by image (boards running the same image are merged, as are several runs), and
//! [`Coverage::report`] maps them to lines and functions: a line is instrumented when the line table gives it an
//! address, covered when an instruction there ran; a function (a `FUNC` symbol with a size) is covered when any
//! instruction in it ran. [`Report::lcov`] writes the lcov tracefile format (`genhtml`, VS Code's Coverage Gutters)
//! and [`Report::summary`] a table of functions and lines hit per crate.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use rp2040::coverage::CoverageMap;
use sim_core::image::Image;

/// Coverage collected from runs, by image.
#[derive(Default, Clone)]
pub struct Coverage {
    /// Image name (its file name), the image, and what ran of it.
    images: BTreeMap<String, (Image, CoverageMap)>,
}

impl std::fmt::Debug for Coverage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let counts: Vec<(&String, usize)> = self.images.iter().map(|(n, (_, m))| (n, m.count())).collect();
        f.debug_struct("Coverage").field("halfwords", &counts).finish()
    }
}

impl Coverage {
    pub fn new() -> Coverage {
        Coverage::default()
    }

    /// Adds what ran of `image` (named `name`, e.g. its file name): merged with what other boards or runs of an
    /// image of the same name ran.
    pub fn add(&mut self, name: &str, image: &Image, map: &CoverageMap) {
        match self.images.get_mut(name) {
            Some((_, m)) => m.merge(map),
            None => {
                self.images.insert(name.to_string(), (image.clone(), map.clone()));
            }
        }
    }

    /// Adds a machine's coverage (if it collected any), under its image's name.
    pub fn add_machine(&mut self, name: &str, m: &rp2040::Machine) {
        if let (Some(map), Some(image)) = (&m.coverage, &m.image) {
            self.add(name, image, map);
        }
    }

    /// Adds everything `other` collected.
    pub fn merge(&mut self, other: &Coverage) {
        for (name, (image, map)) in &other.images {
            self.add(name, image, map);
        }
    }

    pub fn is_empty(&self) -> bool {
        self.images.is_empty()
    }

    /// The image names collected.
    pub fn names(&self) -> Vec<String> {
        self.images.keys().cloned().collect()
    }

    /// Lines and functions of every image (merged where images share source files).
    pub fn report(&self) -> Report {
        let mut r = Report::default();
        for (name, (image, map)) in &self.images {
            r.add_image(name, image, map);
        }
        r
    }

    /// The report of one image.
    pub fn report_of(&self, name: &str) -> Option<Report> {
        let (image, map) = self.images.get(name)?;
        let mut r = Report::default();
        r.add_image(name, image, map);
        Some(r)
    }
}

/// One function: where it starts and whether it ran.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Function {
    /// The demangled name.
    pub name: String,
    /// The line of its first instruction (0 if the line table does not say).
    pub line: u32,
    pub hit: bool,
}

/// Coverage of one source file.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FileCoverage {
    /// Instrumented lines: whether an instruction of each ran.
    pub lines: BTreeMap<u32, bool>,
    /// Functions by (mangled) symbol name.
    pub functions: BTreeMap<String, Function>,
}

/// Coverage mapped to source files, lines and functions.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Report {
    pub files: BTreeMap<String, FileCoverage>,
    /// Functions with no line information (no DWARF, or a file it does not name), by image.
    pub unplaced: BTreeMap<String, Vec<Function>>,
}

impl Report {
    fn add_image(&mut self, name: &str, image: &Image, map: &CoverageMap) {
        let rows = image.elf.as_deref().map(line_rows).unwrap_or_default();
        // Address ranges of each row, and per line whether any halfword of its ranges ran
        for (i, row) in rows.iter().enumerate() {
            let Some(next) = rows.get(i + 1).filter(|n| n.seq == row.seq) else { continue };
            if row.line == 0 || next.addr <= row.addr {
                continue;
            }
            let hit = map.any(row.addr, next.addr - row.addr);
            let f = self.files.entry(row.file.clone()).or_default();
            let e = f.lines.entry(row.line).or_insert(false);
            *e |= hit;
        }
        // Functions: their first instruction's row gives the file and line
        for sym in image.symbols.iter().filter(|s| s.is_function && s.size > 0) {
            let addr = sym.addr & !1;
            let hit = map.any(addr, sym.size);
            let row = rows.iter().filter(|r| r.addr <= addr && r.line != 0).max_by_key(|r| (r.addr, r.seq));
            let f = Function { name: crate::demangle::demangle(&sym.name), line: row.map_or(0, |r| r.line), hit };
            match row.filter(|r| r.addr + 0x1000 > addr) {
                Some(r) => {
                    let e = self.files.entry(r.file.clone()).or_default().functions.entry(sym.name.clone());
                    let e = e.or_insert(f.clone());
                    e.hit |= f.hit;
                }
                None => self.unplaced.entry(name.to_string()).or_default().push(f),
            }
        }
    }

    /// The lcov tracefile (`TN`, then per file `SF`, `FN`/`FNDA`/`FNF`/`FNH`, `DA`, `LF`/`LH`, `end_of_record`).
    /// Hit counts are 1 or 0: the bitmap says whether an instruction ran, not how often.
    pub fn lcov(&self, test_name: &str) -> String {
        let mut out = String::new();
        for (file, f) in &self.files {
            out.push_str(&format!("TN:{test_name}\nSF:{file}\n"));
            for (sym, func) in &f.functions {
                out.push_str(&format!("FN:{},{sym}\n", func.line));
            }
            for (sym, func) in &f.functions {
                out.push_str(&format!("FNDA:{},{sym}\n", func.hit as u32));
            }
            out.push_str(&format!(
                "FNF:{}\nFNH:{}\n",
                f.functions.len(),
                f.functions.values().filter(|x| x.hit).count()
            ));
            for (line, hit) in &f.lines {
                out.push_str(&format!("DA:{line},{}\n", *hit as u32));
            }
            out.push_str(&format!(
                "LF:{}\nLH:{}\nend_of_record\n",
                f.lines.len(),
                f.lines.values().filter(|h| **h).count()
            ));
        }
        out
    }

    /// Writes [`Report::lcov`] to `path`.
    pub fn write_lcov(&self, path: &Path, test_name: &str) -> std::io::Result<()> {
        if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
            std::fs::create_dir_all(dir)?;
        }
        std::fs::write(path, self.lcov(test_name))
    }

    /// Per crate: functions hit and total, lines hit and total.
    pub fn by_crate(&self) -> BTreeMap<String, [usize; 4]> {
        let mut out: BTreeMap<String, [usize; 4]> = BTreeMap::new();
        for (file, f) in &self.files {
            for func in f.functions.values() {
                let e = out.entry(crate_of(&func.name)).or_default();
                e[0] += func.hit as usize;
                e[1] += 1;
            }
            let e = out.entry(crate_of_file(file)).or_default();
            e[2] += f.lines.values().filter(|h| **h).count();
            e[3] += f.lines.len();
        }
        for fs in self.unplaced.values() {
            for func in fs {
                let e = out.entry(crate_of(&func.name)).or_default();
                e[0] += func.hit as usize;
                e[1] += 1;
            }
        }
        out
    }

    /// A table of [`Report::by_crate`] with the totals.
    pub fn summary(&self) -> String {
        let rows = self.by_crate();
        let mut out = format!("{:<32} {:>16} {:>18}\n", "crate", "functions", "lines");
        let mut total = [0; 4];
        let pct = |a: usize, b: usize| if b == 0 { 0.0 } else { 100.0 * a as f64 / b as f64 };
        for (name, c) in &rows {
            for i in 0..4 {
                total[i] += c[i];
            }
            out.push_str(&format!(
                "{name:<32} {:>7}/{:<5} {:>3.0}% {:>7}/{:<5} {:>3.0}%\n",
                c[0],
                c[1],
                pct(c[0], c[1]),
                c[2],
                c[3],
                pct(c[2], c[3])
            ));
        }
        out.push_str(&format!(
            "{:<32} {:>7}/{:<5} {:>3.0}% {:>7}/{:<5} {:>3.0}%\n",
            "total",
            total[0],
            total[1],
            pct(total[0], total[1]),
            total[2],
            total[3],
            pct(total[2], total[3])
        ));
        out
    }

    /// The function called `name` (demangled, or the symbol), with its file.
    pub fn function(&self, name: &str) -> Option<(&str, &Function)> {
        self.files.iter().find_map(|(file, f)| {
            f.functions
                .iter()
                .find(|(sym, func)| *sym == name || func.name == name)
                .map(|(_, func)| (file.as_str(), func))
        })
    }
}

/// The crate of a demangled function name: its first path segment; for `<T as Trait>::f` that of `T`, or of the
/// trait when `T` is not a path (`<str as core::fmt::Display>::fmt`: `core`).
pub fn crate_of(name: &str) -> String {
    let first = |p: &str| {
        let p = p.trim_start_matches(['<', '&', '*', ' ', '[']).trim_start_matches("mut ").trim_start_matches("dyn ");
        p.find("::").map(|i| p[..i].to_string()).filter(|c| !c.is_empty() && !c.contains([' ', ';', '<', ']']))
    };
    if let Some(inner) = name.strip_prefix('<')
        && let Some((ty, rest)) = inner.split_once(" as ")
    {
        if let Some(c) = first(ty) {
            return c;
        }
        if let Some(c) = first(rest) {
            return c;
        }
    }
    first(name).unwrap_or_else(|| "(no crate)".to_string())
}

/// The crate a source file belongs to, from its path: the directory above `src/` (`.../embassy-rp-0.4.0/src/x.rs`:
/// `embassy-rp`, the version cut off), or the file's directory.
pub fn crate_of_file(file: &str) -> String {
    let p = Path::new(file);
    let parts: Vec<&str> = p.iter().filter_map(|c| c.to_str()).collect();
    if let Some(i) = parts.iter().rposition(|c| *c == "src")
        && i > 0
    {
        let dir = parts[i - 1];
        // A registry crate's directory carries its version
        let name = match dir.rfind('-') {
            Some(k) if dir[k + 1..].starts_with(|c: char| c.is_ascii_digit()) => &dir[..k],
            _ => dir,
        };
        // As in paths: embassy-rp is embassy_rp
        return name.replace('-', "_");
    }
    p.parent().and_then(|d| d.file_name()).map_or("(no crate)".into(), |d| d.to_string_lossy().into_owned())
}

/// One row of a line table.
#[derive(Clone, Debug)]
struct Row {
    addr: u32,
    file: String,
    line: u32,
    /// The sequence it belongs to (rows of different sequences do not bound each other).
    seq: usize,
}

/// Every row of every line program in `.debug_line`, in sequence order, addresses ascending within a sequence.
fn line_rows(elf: &[u8]) -> Vec<Row> {
    use object::{Object, ObjectSection};
    let Ok(file) = object::File::parse(elf) else { return Vec::new() };
    let load = |id: gimli::SectionId| -> Result<std::borrow::Cow<'_, [u8]>, gimli::Error> {
        Ok(file.section_by_name(id.name()).and_then(|s| s.uncompressed_data().ok()).unwrap_or_default())
    };
    let Ok(sections) = gimli::DwarfSections::load(load) else { return Vec::new() };
    let dwarf = sections.borrow(|s| gimli::EndianSlice::new(s, gimli::LittleEndian));
    let mut rows = Vec::new();
    let mut seq = 0;
    let mut units = dwarf.units();
    while let Ok(Some(header)) = units.next() {
        let Ok(unit) = dwarf.unit(header) else { continue };
        let Some(program) = unit.line_program.clone() else { continue };
        let comp_dir = unit.comp_dir.map(|d| d.to_string_lossy().into_owned()).unwrap_or_default();
        let mut paths: BTreeMap<u64, String> = BTreeMap::new();
        let mut state = program.rows();
        while let Ok(Some((header, row))) = state.next_row() {
            if row.end_sequence() {
                rows.push(Row { addr: row.address() as u32, file: String::new(), line: 0, seq });
                seq += 1;
                continue;
            }
            let index = row.file_index();
            let path = match paths.get(&index) {
                Some(p) => p.clone(),
                None => {
                    let p =
                        row.file(header).map(|f| file_path(&dwarf, &unit, header, f, &comp_dir)).unwrap_or_default();
                    paths.insert(index, p.clone());
                    p
                }
            };
            let line = row.line().map_or(0, |l| l.get() as u32);
            rows.push(Row { addr: row.address() as u32, file: path, line, seq });
        }
        seq += 1;
    }
    rows
}

type Slice<'a> = gimli::EndianSlice<'a, gimli::LittleEndian>;

/// A line table file entry's full path: the compilation directory, the entry's directory and its name.
fn file_path(
    dwarf: &gimli::Dwarf<Slice>,
    unit: &gimli::Unit<Slice>,
    header: &gimli::LineProgramHeader<Slice>,
    f: &gimli::FileEntry<Slice>,
    comp_dir: &str,
) -> String {
    let attr = |a: gimli::AttributeValue<Slice>| {
        dwarf.attr_string(unit, a).map(|s| s.to_string_lossy().into_owned()).unwrap_or_default()
    };
    let name = attr(f.path_name());
    let dir = f.directory(header).map(attr).unwrap_or_default();
    let mut p = PathBuf::from(comp_dir);
    p.push(dir);
    p.push(name);
    // Collapse `a/./b` and `a/b/../c` without touching the file system
    let mut out = PathBuf::new();
    for c in p.components() {
        match c {
            std::path::Component::CurDir => {}
            std::path::Component::ParentDir => {
                out.pop();
            }
            c => out.push(c),
        }
    }
    out.to_string_lossy().into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crates_of_names_and_files() {
        assert_eq!(crate_of("power_board::main"), "power_board");
        assert_eq!(crate_of("<embassy_rp::uart::Uart as core::fmt::Write>::write_str"), "embassy_rp");
        assert_eq!(crate_of("memcpy"), "(no crate)");
        assert_eq!(crate_of("<str as core::fmt::Display>::fmt"), "core");
        assert_eq!(crate_of("<[i16; 8] as serde_core::Serialize>::serialize"), "serde_core");
        assert_eq!(
            crate_of_file("/root/.cargo/registry/src/index.crates.io-1/embassy-rp-0.4.0/src/uart/mod.rs"),
            "embassy_rp"
        );
        assert_eq!(crate_of_file("/w/firmware/power_board/src/main.rs"), "power_board");
        assert_eq!(crate_of_file("/w/test_programs/coverage.S"), "test_programs");
    }

    #[test]
    fn lcov_format() {
        let mut r = Report::default();
        let f = r.files.entry("/x/a.rs".into()).or_default();
        f.lines.insert(3, true);
        f.lines.insert(5, false);
        f.functions.insert("_ZN1a1fE".into(), Function { name: "a::f".into(), line: 3, hit: true });
        assert_eq!(
            r.lcov("t"),
            "TN:t\nSF:/x/a.rs\nFN:3,_ZN1a1fE\nFNDA:1,_ZN1a1fE\nFNF:1\nFNH:1\nDA:3,1\nDA:5,0\nLF:2\nLH:1\nend_of_record\n"
        );
        assert!(r.summary().contains("a "), "{}", r.summary());
    }
}
