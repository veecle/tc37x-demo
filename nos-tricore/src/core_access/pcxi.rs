use core::arch::asm;

use bitfield_struct::bitfield;
use defmt::Format;

use crate::csa::{ContextLinkWord, ContextWalker};

#[bitfield(u32)]
pub struct PCXI {
    pub previous_context_pointer: u16,
    #[bits(4)]
    pub previous_segment_address: u8,
    pub is_upper: bool,
    #[bits(2)]
    _reserved: u8,
    #[bits(9)]
    _not_implemented: u16,
}

impl PCXI {
    pub(crate) unsafe fn get_link_word(&self) -> Option<ContextLinkWord> {
        if self.previous_context_pointer() != 0 && self.previous_segment_address() != 0 {
            Some(unsafe {
                ContextLinkWord::from(
                    self.previous_segment_address(),
                    self.previous_context_pointer(),
                    self.is_upper(),
                )
            })
        } else {
            None
        }
    }
}

impl Format for PCXI {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "PCX {{ previous_segment_address: 0x{:X}, previous_context_pointer: 0x{:X} }}",
            self.previous_segment_address(),
            self.previous_context_pointer()
        )
    }
}

pub fn get_pcxi() -> PCXI {
    let mut raw_value: u32;
    unsafe {
        asm!(
            "mfcr {raw_value}, $pcxi",
            raw_value = out(reg32) raw_value
        );
    }

    PCXI::from(raw_value)
}

pub fn current_link_word<R>(with: impl FnOnce(Option<ContextLinkWord<'_>>) -> R) -> R {
    let current_pcxi = get_pcxi();

    let current_link_word = unsafe { current_pcxi.get_link_word() };
    let result = with(current_link_word);

    drop(current_link_word);

    result
}

pub fn walk_contexts<R>(with: impl FnOnce(ContextWalker) -> R) -> R {
    current_link_word(|current| {
        let current = if let Some(word) = current {
            unsafe { word.get_context().pcxi().get_link_word() }
        } else {
            None
        };
        let walker = ContextWalker::from_link_word(current);
        with(walker)
    })
}
