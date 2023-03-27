//! Simple example to demonstrate RUST on AURIX.
//!
//! This code allows to configure the clocks in `IfxCcuClockInit` & work with CAN is `IfxCan_Module` and `IfxCanNode` fashion
//!
//! ## Some pattern to look into here
//! - Safe and structured memory abstraction over [`memory`] module: this allows the driver to work with lifetimes,
//!   owned memory, and slices. Less runtime issues with memory alignment and/or accesses due to the setup
//!
//! - State-machine access when working with [`CanNode`]: this enforces methods to be hidden behind 'states', and
//!   those states enforce programmer (at compile time) to use the node in the correct mode-of-operations
//!
//! - Proc-macro auto-generation of alignments via [`bitfield_struct`] crate: compilation will fail if the bits and
//!   sizes won't match the expected structure size. This can be seen in [`CanTxFrame`] or [`CanRxFrame`]
//!
//! - Inner mutability & access via protected closure (very native) for the [`reg::SystemAccess`] registers (SCU)
//!
//! ## Notes
//! No code here is 'complete', but the goal was to show some Rust patterns in their most basic form. Must of this
//! can be improved later-on to be 'truly' safe
//!
//! Also, most 'variables' are hardcoded for simplicity
#![no_std]
#![no_main]
#![feature(never_type)]
#![feature(generic_const_exprs)]
#![allow(clippy::result_unit_err)]
#![allow(incomplete_features)]
use core::time::Duration;

use byteorder::ByteOrder;
use joystick::JoystickData;
use tc37x_hal::can::can0::{Available, CanModule0, CanModule0RAM};
use tc37x_hal::can::memory::module_ram::{BufferSize8, NodeMemory, NodeMemoryBuilder};
use tc37x_hal::can::node::connection;
use tc37x_hal::can::timing::{CanBitrate, U32Ext};
use tc37x_hal::can::{CanID, CanRxFrame, CanTxFrame};
use tc37x_hal::clocks::config::Clocks;
use tc37x_hal::clocks::oscillator::config::Oscillator;
use tc37x_hal::clocks::setup::SetupClocks;
use tc37x_hal::time::Instant;
use tc37x_pac::{PORT_15, PORT_20};
use tc37x_rt::asm_calls::read_cpu_core_id;
use tc37x_rt::isr::load_interrupt_table;
use tc37x_rt::util::wait_nop;
use tc37x_rt::wdtcon::*;
use tc37x_rt::*;

extern crate defmt_rtt;
extern crate nos_tricore;

// Configure the entry-point, pre and post init
pre_init!(pre_init_fn);
post_init!(post_init_fn);
entry!(main);

mod joystick;
mod led;

/// Copied from hightec (this would belong to nos-tricore runtime)
fn pre_init_fn() {
    if read_cpu_core_id() == 0 {
        disable_safety_watchdog();
    }
    disable_cpu_watchdog();
}

/// Copied from hightec (this would belong to nos-tricore runtime)
fn post_init_fn() {
    load_interrupt_table();
}

/// Main wrapper that early return on error, while the 'main' will print the error out
fn checked_main() -> Result<!, &'static str> {
    // Setup the led first, on trap the led will stop blinking, quite helpful for
    // debugging since the trap setup is not complete for now
    led::setup_led();
    let mut p = unsafe { tc37x_pac::Peripherals::steal() };

    // Enable the can transceiver
    call_without_endinit(|| {
        p.PORT_20.iocr4.modify(|_, w| w.pc6().variant(0b10000)); // output, push pull
        p.PORT_20.pdr0.modify(|_, w| w.pd6().variant(0)); // strong driver
        p.PORT_20.out.modify(|_, w| w.p6().clear_bit()); // pull pin low
    });

    // Configure the clocks for the device and early return if failure
    p.setup(Clocks::new(Oscillator::new(20)))
        .map_err(|_| "clock config err")?;

    // -----------------
    // Start of CAN code
    // -----------------

    defmt::info!("Enabling CAN(0) module");
    let can = CanModule0::new(&mut p.CAN0);

    // -----------------
    // Organize the CAN memory blocks via type-safe wrapper
    // -----------------
    // We create a type-safe and semantically correct memory area for the nodes to work with...
    // addresses starts from 0x00 and increments in chunk, starting from a base memory
    let mut memory = unsafe { NodeMemoryBuilder::steal_module_mem() };

    can_with_loopback(&can, &mut memory)
    //can_with_pins(can, &mut memory, &p.PORT_20, &p.PORT_15)
}

#[allow(unused)]
fn can_with_loopback(
    can: CanModule0<Available, Available>,
    memory: &mut NodeMemoryBuilder<'_, CanModule0RAM>,
) -> Result<!, &'static str> {
    // Type-state configuration: node won't be usable until initialized
    defmt::info!("Configuring Node<0> as Tx");
    let bit_rate = CanBitrate::from_frequency(50u32.kbps()).map_err(|_| "unsupported frequency")?;

    let (can, can_node) = can.node0();
    let tx_memory: NodeMemory<CanTxFrame<BufferSize8>, _> = memory.take_expect(8);

    let mut can_node = can_node
        .set_bitrate(&bit_rate)
        .connect_internal_loopback()
        .set_tx(tx_memory)
        .finalize();

    defmt::info!("Configuring Node<1> as Rx");
    let (can, loop_back_node) = can.node1();

    let rx_memory: NodeMemory<CanRxFrame<BufferSize8>, _> = memory.take_expect(8);

    let mut loop_back_node = loop_back_node
        .set_bitrate(&bit_rate)
        .connect_internal_loopback()
        .set_rx_fifo0(rx_memory)
        .finalize();

    //
    // Naive loop to send/recv frame in loop-back and to externals: right not this blink the LED every cycle
    // for visualization purposes
    // At every cycle it will:
    // - Send a frame
    // - Recv one or more frame (until FIFO<0> is empty)
    //
    let mut _msg: u64 = 0x12_00_00_78_12_34_56_78;
    loop {
        // Iterate over an ID range to show changes in ID
        for id in 0xAA_u16..0xBB_u16 {
            // Data in big-endian to see it :)
            let data = _msg.to_be_bytes();

            if let Some(transmit_buffer) = can_node.acquire_transmit_buffer() {
                let mut can_frame = CanTxFrame::default();

                can_frame.set_data(&data);
                can_frame.set_id(CanID::Extended(id as u32));
                transmit_buffer.set_frame(can_frame).send();

                defmt::info!("Frame is now in buffer!");
            } else {
                defmt::warn!("nb::WouldBlock (buffer still full)");
            }

            // Add the ID to the message counter
            _msg = _msg.wrapping_add(id as u64);
            wait_nop(10_000);

            // Empty FIFO
            let Some(frame) = loop_back_node.try_receive_fifo0() else {
                defmt::panic!("We expected to immediately receive the packet!");
            };

            defmt::assert_eq!(frame.get_id(), CanID::Extended(id as u32));
            defmt::assert_eq!(frame.data(), &data);
            // panic!("We got frame back");

            // Finally, blink LED<1> at every cycle
            led::BoardLed::Led1.toggle_led();
            wait_nop(1_000);
        }
    }
}

#[allow(unused)]
fn can_with_pins(
    can: CanModule0<Available, Available>,
    memory: &mut NodeMemoryBuilder<'_, CanModule0RAM>,
    port20: &PORT_20,
    port15: &PORT_15,
) -> Result<!, &'static str> {
    // Type-state configuration: node won't be usable until initialized
    defmt::info!("Configuring Node<0> as Tx");
    let bit_rate_joystick_can =
        CanBitrate::from_frequency(50u32.kbps()).map_err(|_| "unsupported frequency")?;
    let bit_rate_vesc =
        CanBitrate::from_frequency(50u32.kbps()).map_err(|_| "unsupported frequency")?;

    let tx_memory: NodeMemory<CanTxFrame<BufferSize8>, _> = memory.take_expect(8);
    let rx_memory: NodeMemory<CanRxFrame<BufferSize8>, _> = memory.take_expect(8);
    let (can, vesc_node) = can.node0();
    let mut vesc_node = vesc_node
        .set_bitrate(&bit_rate_vesc)
        .set_rx_fifo0(rx_memory)
        .set_pins(connection::Node0Pin::Rxdb, port20)
        .set_tx(tx_memory)
        .finalize();

    let (_, joystick_can) = can.node1();
    let tx_memory: NodeMemory<CanTxFrame<BufferSize8>, _> = memory.take_expect(8);
    let rx_memory: NodeMemory<CanRxFrame<BufferSize8>, _> = memory.take_expect(8);
    let mut joystick_can = joystick_can
        .set_bitrate(&bit_rate_joystick_can)
        .set_rx_fifo0(rx_memory)
        .set_pins(connection::Node1Pin::Rxda, port15)
        .set_tx(tx_memory)
        .finalize();

    defmt::info!("Initial setup done!");
    let mut vesc_discard_counter = 0;
    let mut joystick_message_counter = 0;
    let mut last_report = Instant::now();
    let mut vesc_duty = 0f32;
    loop {
        let current_time = Instant::now();
        if &last_report + Duration::from_secs(1) < current_time {
            defmt::info!("Now: {}, VESC duty={:?}%; discarded status messages on VESC bus cnt={}, total number of received joystick message={}", 
                current_time,
                vesc_duty * 100.0,
                vesc_discard_counter,
                joystick_message_counter
            );
            last_report = current_time;
        }
        // Empty FIFO
        while let Some(frame) = joystick_can.try_receive_fifo0() {
            if let CanID::Standard(0x70) = frame.get_id() {
                joystick_message_counter += 1;
                let joystick_data: JoystickData = frame.data().try_into().expect("Invalid packet");
                // Invert slider scale, and scale from 0 to 1.0
                vesc_duty = ((255 - joystick_data.slider) as f32) / 255f32;

                // Clamp to avoid slider jitter
                if vesc_duty < 0.10 {
                    vesc_duty = 0.0;
                }

                vesc_node.with_transmit_buffer(|transmit_buffer| {
                    let mut data = [0; 4];
                    byteorder::BigEndian::write_i32(&mut data, (vesc_duty * 100_000.0) as i32);

                    let mut can_frame = CanTxFrame::default();

                    can_frame.set_data(&data);

                    const MOTOR_ID: u32 = 97;
                    const PACKET_TYPE: u32 = 0; // 0 = set duty

                    can_frame.set_id(CanID::Extended(MOTOR_ID + (PACKET_TYPE << 8)));

                    transmit_buffer.set_frame(can_frame).send();
                });

                led::BoardLed::Led1.toggle_led();
            } else {
                defmt::warn!("Invalid frame id {:?}", frame.get_id());
            }
        }

        while vesc_node.try_receive_fifo0().is_some() {
            vesc_discard_counter += 1;
        }
    }
}


fn main() -> ! {
    let result = checked_main();
    if let Err(e) = result {
        defmt::error!("Main terminated with error {:?}", e);
        panic!("Main failed");
    } else {
        defmt::unreachable!("Main should never return Ok()");
    }
}
