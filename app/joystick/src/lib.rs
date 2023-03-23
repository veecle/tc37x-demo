#![no_std]

use bitreader::BitReader;
use defmt::Format;

/// https://chome.nerpa.tech/mcu/using-logitech-extreme-3d-pro-joystick-with-arduino-hid-library/
#[derive(Default, Debug, Format)]
pub struct JoystickData {
    pub x: u16,       // 0 .. 1024
    pub y: u16,       // 0 .. 1024
    pub twist: u8,   // 0 .. 255
    pub hat: u8,      // States: TBD
    pub button_a: u8, // States: TBD
    pub slider: u8,   // States: TBD
    pub button_b: u8, // States: TBD
}


impl TryFrom<&[u8]> for JoystickData {
    type Error = ();
    fn try_from(value: &[u8]) -> Result<Self, ()> {
        if value.len() != 7 {
            return Err(())
        }
        // Process bits
        // This can be likely be done better, but not worth the effort
        let mut bits = BitReader::new(value);
        // Smallest first 8 bits from X In little endian
        let mut x = bits.read_u16(8).unwrap();
        // Smallest 6 bits from Y
        let mut y = bits.read_u16(6).unwrap();
        // Adds bits to x to account for '10'
        x += bits.read_u16(2).unwrap() << 8;
        // Smallest 'hat' but only 4
        let hat = bits.read_u8(4).unwrap();
        // Adds bits to y to account for '10'
        y += bits.read_u16(4).unwrap() << 6;

        // Following, 1 byte per field
        let twist = bits.read_u8(8).unwrap();
        let button_a = bits.read_u8(8).unwrap();
        let slider = bits.read_u8(8).unwrap();
        let button_b = bits.read_u8(8).unwrap();

        //
        // Return data and normalize value to min/max
        //
        Ok(JoystickData {
            x,
            y,
            twist,
            hat,
            button_a,
            slider,
            button_b,
        })
    }
}