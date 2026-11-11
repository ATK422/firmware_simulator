//! Builds without the `jit` feature, and hosts Cranelift does not generate code for: nothing is compiled, and the
//! core runs decoded blocks.

use sim_core::bus::DIRECT_REGIONS;

use super::Trampoline;
use super::service::Shape;
use crate::block::Entry;

/// A block of a region.
#[derive(Clone, Copy)]
pub(crate) struct RegionBlock {
    pub entry: Entry,
    pub writable: bool,
    pub return_site: bool,
}

pub(crate) enum Compiler {}

pub(crate) struct Defined;

pub(crate) fn code_taken() -> usize {
    0
}

impl Compiler {
    pub(crate) fn new() -> Option<Compiler> {
        None
    }

    pub(crate) fn trampoline(&self) -> Trampoline {
        match *self {}
    }

    pub(crate) fn full(&self) -> bool {
        match *self {}
    }

    pub(crate) fn define(&mut self, _: &[RegionBlock], _: &[Shape; DIRECT_REGIONS], _: &[u32; 16]) -> Option<Defined> {
        match *self {}
    }

    pub(crate) fn finish(&mut self, _: &[Defined]) -> Option<Vec<*const u8>> {
        match *self {}
    }
}
