//! Firmware images: ELF files (what `cargo build` and the pico-sdk produce) and UF2 files (what BOOTSEL takes).

use object::{Object, ObjectSegment, ObjectSymbol, SymbolKind};
use thiserror::Error;

/// RP2040 UF2 family ID.
pub const UF2_FAMILY_RP2040: u32 = 0xE48B_FF56;
const UF2_MAGIC0: u32 = 0x0A32_4655;
const UF2_MAGIC1: u32 = 0x9E5D_5157;
const UF2_MAGIC_END: u32 = 0x0AB1_6F30;
const UF2_FLAG_NOT_MAIN_FLASH: u32 = 0x0000_0001;
const UF2_FLAG_FAMILY_ID: u32 = 0x0000_2000;

#[derive(Debug, Error)]
pub enum ImageError {
    #[error("not an ELF or UF2 file")]
    UnknownFormat,
    #[error("ELF: {0}")]
    Elf(#[from] object::Error),
    #[error("UF2 block {0}: {1}")]
    Uf2(usize, &'static str),
    #[error("image is empty")]
    Empty,
}

/// A contiguous piece of the image, at its load (physical) address.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Segment {
    pub addr: u32,
    pub data: Vec<u8>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Symbol {
    pub name: String,
    pub addr: u32,
    pub size: u32,
    pub is_function: bool,
}

/// A loaded firmware image.
#[derive(Clone, Debug, Default)]
pub struct Image {
    pub segments: Vec<Segment>,
    /// ELF entry point, if the image came from an ELF.
    pub entry: Option<u32>,
    pub symbols: Vec<Symbol>,
    /// The original ELF, kept for defmt and debug info.
    pub elf: Option<Vec<u8>>,
}

impl Image {
    /// Parses an ELF or UF2 file, detected from its contents.
    pub fn parse(bytes: &[u8]) -> Result<Image, ImageError> {
        if bytes.starts_with(b"\x7fELF") {
            Self::from_elf(bytes)
        } else if bytes.len() >= 512 && u32::from_le_bytes(bytes[0..4].try_into().unwrap()) == UF2_MAGIC0 {
            Self::from_uf2(bytes)
        } else {
            Err(ImageError::UnknownFormat)
        }
    }

    pub fn load(path: &std::path::Path) -> anyhow_like::Result<Image> {
        let bytes = std::fs::read(path).map_err(|e| anyhow_like::Error(format!("{}: {e}", path.display())))?;
        Image::parse(&bytes).map_err(|e| anyhow_like::Error(format!("{}: {e}", path.display())))
    }

    /// Loads the `PT_LOAD` segments at their physical addresses (so initialised `.data` lands in flash, where the
    /// startup code copies it from) and collects the symbol table.
    pub fn from_elf(bytes: &[u8]) -> Result<Image, ImageError> {
        let file = object::File::parse(bytes)?;
        let mut segments = Vec::new();
        for seg in file.segments() {
            let data = seg.data()?;
            if data.is_empty() {
                continue;
            }
            // object reports p_vaddr as address(); the load address is p_paddr
            let addr = match &file {
                object::File::Elf32(elf) => {
                    use object::read::elf::ProgramHeader;
                    let endian = elf.endian();
                    let phdr = elf
                        .elf_program_headers()
                        .iter()
                        .find(|p| {
                            p.p_type(endian) == object::elf::PT_LOAD
                                && p.p_vaddr(endian) as u64 == seg.address()
                                && p.p_filesz(endian) as u64 == data.len() as u64
                        })
                        .expect("segment has a program header");
                    phdr.p_paddr(endian)
                }
                _ => seg.address() as u32,
            };
            segments.push(Segment { addr, data: data.to_vec() });
        }
        if segments.is_empty() {
            return Err(ImageError::Empty);
        }
        let mut symbols: Vec<Symbol> = file
            .symbols()
            .filter(|s| s.is_definition() && matches!(s.kind(), SymbolKind::Text | SymbolKind::Data))
            .filter_map(|s| {
                let name = s.name().ok()?;
                if name.is_empty() {
                    return None;
                }
                Some(Symbol {
                    name: name.to_string(),
                    addr: s.address() as u32,
                    size: s.size() as u32,
                    is_function: s.kind() == SymbolKind::Text,
                })
            })
            .collect();
        symbols.sort_by_key(|s| s.addr);
        Ok(Image { segments, entry: Some(file.entry() as u32), symbols, elf: Some(bytes.to_vec()) })
    }

    /// Collects the main-flash blocks of a UF2 file (RP2040 family, or blocks without a family ID).
    pub fn from_uf2(bytes: &[u8]) -> Result<Image, ImageError> {
        let mut blocks: Vec<(u32, Vec<u8>)> = Vec::new();
        for (i, block) in bytes.chunks(512).enumerate() {
            if block.len() < 512 {
                return Err(ImageError::Uf2(i, "truncated block"));
            }
            let w = |o: usize| u32::from_le_bytes(block[o..o + 4].try_into().unwrap());
            if w(0) != UF2_MAGIC0 || w(4) != UF2_MAGIC1 || w(508) != UF2_MAGIC_END {
                return Err(ImageError::Uf2(i, "bad magic"));
            }
            let flags = w(8);
            let addr = w(12);
            let len = w(16) as usize;
            if flags & UF2_FLAG_NOT_MAIN_FLASH != 0 {
                continue;
            }
            if flags & UF2_FLAG_FAMILY_ID != 0 && w(28) != UF2_FAMILY_RP2040 {
                continue;
            }
            if len > 476 {
                return Err(ImageError::Uf2(i, "payload too large"));
            }
            blocks.push((addr, block[32..32 + len].to_vec()));
        }
        if blocks.is_empty() {
            return Err(ImageError::Empty);
        }
        blocks.sort_by_key(|b| b.0);
        // Merge adjacent blocks into segments
        let mut segments: Vec<Segment> = Vec::new();
        for (addr, data) in blocks {
            match segments.last_mut() {
                Some(s) if s.addr + s.data.len() as u32 == addr => s.data.extend_from_slice(&data),
                _ => segments.push(Segment { addr, data }),
            }
        }
        Ok(Image { segments, entry: None, symbols: Vec::new(), elf: None })
    }

    /// Address of a symbol by exact name.
    pub fn symbol(&self, name: &str) -> Option<&Symbol> {
        self.symbols.iter().find(|s| s.name == name)
    }

    /// The function symbol containing `addr`, with the offset into it.
    pub fn symbolize(&self, addr: u32) -> Option<(&Symbol, u32)> {
        let addr = addr & !1;
        let idx = self.symbols.partition_point(|s| s.addr & !1 <= addr);
        self.symbols[..idx]
            .iter()
            .rev()
            .find(|s| s.is_function && addr < (s.addr & !1) + s.size.max(1))
            .map(|s| (s, addr - (s.addr & !1)))
    }

    /// Builds a UF2 file of this image's segments (RP2040 family, 256 byte payloads), as `elf2uf2` does.
    pub fn to_uf2(&self) -> Vec<u8> {
        let mut pages: Vec<(u32, [u8; 256])> = Vec::new();
        for seg in &self.segments {
            for (i, &b) in seg.data.iter().enumerate() {
                let a = seg.addr + i as u32;
                let page = a & !0xFF;
                let idx = match pages.iter().position(|p| p.0 == page) {
                    Some(idx) => idx,
                    None => {
                        pages.push((page, [0; 256]));
                        pages.len() - 1
                    }
                };
                pages[idx].1[(a & 0xFF) as usize] = b;
            }
        }
        pages.sort_by_key(|p| p.0);
        let n = pages.len() as u32;
        let mut out = Vec::with_capacity(pages.len() * 512);
        for (i, (addr, data)) in pages.iter().enumerate() {
            let mut block = [0u8; 512];
            let words = [UF2_MAGIC0, UF2_MAGIC1, UF2_FLAG_FAMILY_ID, *addr, 256, i as u32, n, UF2_FAMILY_RP2040];
            for (j, w) in words.iter().enumerate() {
                block[j * 4..j * 4 + 4].copy_from_slice(&w.to_le_bytes());
            }
            block[32..288].copy_from_slice(data);
            block[508..512].copy_from_slice(&UF2_MAGIC_END.to_le_bytes());
            out.extend_from_slice(&block);
        }
        out
    }
}

/// Minimal error type so `sim_core` does not depend on `anyhow`.
pub mod anyhow_like {
    #[derive(Debug)]
    pub struct Error(pub String);
    impl std::fmt::Display for Error {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            f.write_str(&self.0)
        }
    }
    impl std::error::Error for Error {}
    pub type Result<T> = std::result::Result<T, Error>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uf2_round_trip() {
        let image = Image {
            segments: vec![
                Segment { addr: 0x1000_0000, data: (0..300).map(|i| i as u8).collect() },
                Segment { addr: 0x1000_1000, data: vec![0xAA; 10] },
            ],
            ..Default::default()
        };
        let uf2 = image.to_uf2();
        assert_eq!(uf2.len(), 3 * 512);
        let back = Image::parse(&uf2).unwrap();
        assert_eq!(back.segments.len(), 2);
        assert_eq!(back.segments[0].addr, 0x1000_0000);
        assert_eq!(&back.segments[0].data[..300], &image.segments[0].data[..]);
        assert_eq!(back.segments[1].addr, 0x1000_1000);
        assert_eq!(&back.segments[1].data[..10], &[0xAA; 10]);
    }

    #[test]
    fn uf2_skips_other_families() {
        let image = Image { segments: vec![Segment { addr: 0x1000_0000, data: vec![1; 256] }], ..Default::default() };
        let mut uf2 = image.to_uf2();
        uf2[28..32].copy_from_slice(&0x1234_5678u32.to_le_bytes());
        assert!(matches!(Image::parse(&uf2), Err(ImageError::Empty)));
    }

    #[test]
    fn unknown_format() {
        assert!(matches!(Image::parse(b"hello"), Err(ImageError::UnknownFormat)));
    }
}
