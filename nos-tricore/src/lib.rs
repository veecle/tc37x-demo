//!
//! Basic runtime to support [`defmt`] logging and stack-traces in primitive manner
//!
#![no_std]
#![feature(panic_info_message)]
use core::panic::PanicInfo;
use core::sync::atomic::Ordering;
use core::{arch::asm, sync::atomic::AtomicBool};
use critical_section::{Impl, RawRestoreState};

pub mod core_access {
    pub mod pcxi;
    pub mod psw;
}

pub mod csa;

/// In-case of recursive panic
static PANICKING: AtomicBool = AtomicBool::new(false);

/// Panic-handler: print the panic & stack-trace and loop forever
#[cfg_attr(not(test), panic_handler)]
fn panic(panic: &PanicInfo<'_>) -> ! {
    if !PANICKING.swap(true, Ordering::Relaxed) {
        defmt::error!("Panic! {}", defmt::Display2Format(panic));
    }
    // This adds a breakpoint for an attached debugger
    unsafe { asm!("debug") };
    loop {}
}

/// Naive critical section for non-multi-core tri-core :)
struct Section;

unsafe impl Impl for Section {
    unsafe fn acquire() -> RawRestoreState {
        unsafe { asm!("disable") };
        true
    }

    unsafe fn release(_restore_state: RawRestoreState) {
        unsafe { asm!("enable") };
    }
}

critical_section::set_impl!(Section);
