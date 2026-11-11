//! Hosts Cranelift does not generate code for: nothing is compiled, and the core runs decoded blocks.

use std::time::Duration;

use super::JitFn;
use crate::block::Entry;

pub(crate) struct Compiler {
    pub compiled: u64,
    pub time: Duration,
}

impl Compiler {
    pub(crate) fn new() -> Option<Compiler> {
        None
    }

    pub(crate) fn full(&self) -> bool {
        false
    }

    /// # Safety
    ///
    /// Nothing to free.
    pub(crate) unsafe fn reset(&mut self) {}

    pub(crate) fn compile(&mut self, _: &Entry) -> Option<JitFn> {
        None
    }
}
