use core::arch::asm;

use bitfield_struct::bitfield;
use defmt::{Debug2Format, Format};

#[derive(Format, Debug)]
pub enum Privilege {
    User0,
    User1,
    Supervisor,
}

impl From<u32> for Privilege {
    fn from(value: u32) -> Self {
        match value {
            0 => Self::User0,
            1 => Self::User1,
            2 => Self::Supervisor,
            _ => panic!("Invalid privilege bits: {:b}", value),
        }
    }
}

impl From<Privilege> for u32 {
    fn from(value: Privilege) -> Self {
        match value {
            Privilege::User0 => 0,
            Privilege::User1 => 1,
            Privilege::Supervisor => 2,
        }
    }
}

#[bitfield(u8)]
#[derive(Default)]
pub struct UserStatusBits {
    carry: bool,
    overflow: bool,
    sticky_overflow: bool,
    advance_overflow: bool,
    sticky_advance_overflow: bool,
    #[bits(3)]
    _reserved: u8,
}

impl From<u32> for UserStatusBits {
    fn from(value: u32) -> Self {
        // UserStatusBits::default()
        value.try_into().expect("Invalid user status bits")
    }
}

impl From<UserStatusBits> for u32 {
    fn from(value: UserStatusBits) -> Self {
        value.into()
    }
}

#[bitfield(u32)]
pub struct PSW {
    #[bits(7)]
    pub call_depth_counter: u8,
    cde: bool,
    gw: bool,
    is: bool,
    #[bits(2)]
    pub current_privilege: Privilege,
    #[bits(2)]
    prs: u8,
    s: bool,
    prs_2: bool,
    _reserved: u8,
    #[bits(8)]
    usb: UserStatusBits,
}

impl Format for PSW {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}", Debug2Format(self));
    }
}

pub fn get_psw() -> PSW {
    let mut raw_value: u32;

    unsafe {
        asm!(
            "mfcr {raw_value}, $psw",
            raw_value = out(reg32) raw_value
        );
    }

    PSW::from(raw_value)
}
