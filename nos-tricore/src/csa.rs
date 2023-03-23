use crate::core_access::{pcxi::PCXI, psw::PSW};
use core::marker::PhantomData;
use defmt::Format;

#[derive(Format, Clone, Copy)]
pub struct ContextLinkWord<'validity> {
    segment_address: u8,
    context_offset: u16,
    is_upper: bool,
    marker: PhantomData<&'validity ()>,
}

impl<'validity> ContextLinkWord<'validity> {
    pub unsafe fn from(segment_address: u8, context_offset: u16, is_upper: bool) -> Self {
        ContextLinkWord {
            segment_address,
            context_offset,
            is_upper,
            marker: PhantomData,
        }
    }

    pub fn get_context(&self) -> Context<'validity> {
        let address =
            ((self.segment_address as usize) << 28) + ((self.context_offset as usize) << 6);

        if self.is_upper {
            let mem_pointer = unsafe { &*(address as *const UpperContext) };
            Context::Upper(mem_pointer)
        } else {
            let mem_pointer = unsafe { &*(address as *const LowerContext) };
            Context::Lower(mem_pointer)
        }
    }
}

pub struct ContextWalker<'validity> {
    last_link_word: Option<ContextLinkWord<'validity>>,
}

impl<'validity> ContextWalker<'validity> {
    pub fn from_link_word(link_word: Option<ContextLinkWord<'validity>>) -> Self {
        ContextWalker {
            last_link_word: link_word,
        }
    }
}

impl<'validity> Iterator for ContextWalker<'validity> {
    type Item = Context<'validity>;

    fn next(&mut self) -> Option<Self::Item> {
        let current_link_word = self.last_link_word.take()?;

        let context = current_link_word.get_context();
        self.last_link_word = unsafe { context.pcxi().get_link_word() };

        Some(context)
    }
}
#[derive(Format)]
pub enum Context<'a> {
    Upper(&'a UpperContext),
    Lower(&'a LowerContext),
}

impl<'a> Context<'a> {
    pub fn pcxi(&self) -> &'a PCXI {
        match self {
            Context::Upper(c) => &c.pcxi,
            Context::Lower(c) => &c.pcxi,
        }
    }

    pub fn stored_return_address(&self) -> u32 {
        match self {
            Context::Upper(c) => c.a11,
            Context::Lower(c) => c.a11,
        }
    }
}

#[derive(Format)]
#[repr(C)]
pub struct UpperContext {
    pub pcxi: PCXI,
    pub psw: PSW,
    a10: u32,
    a11: u32,
    d8: u32,
    d9: u32,
    d10: u32,
    d11: u32,
    a12: u32,
    a13: u32,
    a14: u32,
    a15: u32,
    d12: u32,
    d13: u32,
    d14: u32,
    d15: u32,
}

#[derive(Format)]
#[repr(C)]
pub struct LowerContext {
    pcxi: PCXI,
    a11: u32,
    a2: u32,
    a3: u32,
    d0: u32,
    d1: u32,
    d2: u32,
    d3: u32,
    a4: u32,
    a5: u32,
    a6: u32,
    a7: u32,
    d4: u32,
    d5: u32,
    d6: u32,
    d7: u32,
}
