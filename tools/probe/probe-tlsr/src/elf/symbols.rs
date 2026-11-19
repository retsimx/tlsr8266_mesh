//! Code-symbol ranges for turning crash addresses into `name+0xNN`.
//!
//! The firmware is built without usable DWARF, so `.symtab` is the only symbol
//! source. Only the executable sections are kept: data symbols share address
//! space with code but must never name a program counter.

use object::{Object, ObjectSection, ObjectSymbol};

use super::ElfImage;
use crate::CliError;

/// Sections whose symbols are executable code.
const CODE_SECTIONS: [&str; 3] = [".vectors", ".ram_code", ".text"];

/// A symbol's `[start, end)` address range.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SymbolRange {
    /// First address covered by the symbol.
    pub start: u32,
    /// One past the last covered address; always greater than `start`.
    pub end: u32,
    /// Symbol name as it appears in `.symtab`.
    pub name: String,
}

impl ElfImage {
    /// Collects the defined code symbols in `.vectors`, `.ram_code` and `.text`.
    ///
    /// Only symbols with a non-empty name and a known extent (`st_size > 0`)
    /// are kept: a size-less symbol is a Thumb/ARM mapping symbol (`$t`, `$d`),
    /// a linker marker (`ZERO`, `ASMEND`) or a plain label, so it has no range
    /// and would shadow the real function enclosing it. Function `st_value`s
    /// carry the Thumb bit, which no PC comparison matches, so every start is
    /// made even. Symbols that share a start collapse to the longest one, and a
    /// symbol with a non-UTF-8 name is skipped rather than failing the whole
    /// table.
    pub fn code_symbols(&self) -> Result<Vec<SymbolRange>, CliError> {
        let elf = self.parsed()?;
        let mut ranges = Vec::new();

        for symbol in elf.symbols() {
            if symbol.is_undefined() {
                continue;
            }
            let Some(index) = symbol.section_index() else {
                continue;
            };
            let Ok(section) = elf.section_by_index(index) else {
                continue;
            };
            let Ok(section_name) = section.name() else {
                continue;
            };
            if !CODE_SECTIONS.contains(&section_name) {
                continue;
            }
            let Ok(name) = symbol.name() else {
                continue;
            };
            if name.is_empty() {
                continue;
            }
            let (Ok(start), Ok(size)) = (
                u32::try_from(symbol.address()),
                u32::try_from(symbol.size()),
            ) else {
                continue;
            };
            if size == 0 {
                continue;
            }
            let start = start & !1;
            ranges.push(SymbolRange {
                start,
                end: start.saturating_add(size),
                name: name.to_owned(),
            });
        }

        ranges.sort_by_key(|range| range.start);

        // Equal starts: keep the range that covers the most addresses.
        let mut out: Vec<SymbolRange> = Vec::with_capacity(ranges.len());
        for range in ranges {
            match out.last_mut() {
                Some(last) if last.start == range.start => {
                    if range.end > last.end {
                        *last = range;
                    }
                }
                _ => out.push(range),
            }
        }

        Ok(out)
    }
}

/// Renders `addr` as `name` or `name+0xNN` using a start-sorted range table.
///
/// Binary-searches the last range that starts at or before `addr` and requires
/// `addr < end`, so ranges are only claimed where they actually cover.
pub fn symbolize(ranges: &[SymbolRange], addr: u32) -> Option<String> {
    let index = ranges.partition_point(|range| range.start <= addr);
    let range = ranges.get(index.checked_sub(1)?)?;
    if addr >= range.end {
        return None;
    }
    if addr == range.start {
        return Some(range.name.clone());
    }
    Some(format!("{}+0x{:x}", range.name, addr - range.start))
}
