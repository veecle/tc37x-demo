//! HighTec code to blink LED via interrupt: useful to see state
use tc37x_rt::asm_calls::enable_interrupts;
use tc37x_rt::{call_without_endinit, interrupt};

/// Ticks per sec.
const SYSTEM_TIMER_FREQ_HZ: u32 = 50_000_000;

#[allow(dead_code)]
enum BoardLedAction {
    NoChange,
    TurnOff,
    TurnOn,
    Toggle,
}

impl BoardLedAction {
    pub fn as_mask(&self) -> u32 {
        match self {
            BoardLedAction::NoChange => 0,
            BoardLedAction::TurnOff => 1,
            BoardLedAction::TurnOn => 1 << 16,
            BoardLedAction::Toggle => (1 << 16) | 1,
        }
    }
}

// defines the LEDs port pin for a APPKIT tc375 Lit Kit ADAS
pub enum BoardLed {
    Led1 = 5,
    Led2 = 6,
}

impl BoardLed {
    fn set_led(self, action: BoardLedAction) {
        let p = unsafe { tc37x_pac::Peripherals::steal() };
        let p00 = p.PORT_00;

        let led_no = self as u32;
        let mask = action.as_mask();
        let mask = mask << led_no;
        p00.omr.write(|w| unsafe { w.bits(mask) });
    }

    pub fn toggle_led(self) {
        self.set_led(BoardLedAction::Toggle);
    }
}

// the called interrupt handler
interrupt!(__INTERRUPT_HANDLER_2, interrupt_handler);

/// Initialise LED code via interrupt (peripherals are stolen)
pub fn setup_led() {
    board_led_init();
    create_timer_interrupt();
    enable_interrupts();
}

fn board_led_init() {
    let p = unsafe { tc37x_pac::Peripherals::steal() };
    let p00 = p.PORT_00;
    change_led_port_mode_to_output(&p00);
    set_led_port_pad_driver_mode(&p00);
}

fn change_led_port_mode_to_output(p00: &tc37x_pac::PORT_00) {
    //clear pin
    p00.omr.write(|w| w.ps5().set_bit());
    //set to output
    p00.iocr4.modify(|_, w| w.pc5().variant(0x10));

    p00.omr.write(|w| w.ps6().set_bit());
    p00.iocr4.modify(|_, w| w.pc6().variant(0x10));
}

fn set_led_port_pad_driver_mode(p00: &tc37x_pac::PORT_00) {
    call_without_endinit(|| {
        //See Aurix TC3XX target specification, section of General Purpose IO/registers

        //Setting the pad driver mode to medium.
        p00.pdr0.modify(|_, w| w.pd5().variant(2));
        //Setting the pad driver selection to automotive.
        p00.pdr0.modify(|_, w| w.pl5().variant(0));

        p00.pdr0.modify(|_, w| w.pd6().variant(2));
        p00.pdr0.modify(|_, w| w.pl6().variant(0));
    });
}

fn create_timer_interrupt() {
    init_timer_compare_registers();
    enable_timer0_interrupt();
}

fn init_timer_compare_registers() {
    let p = unsafe { tc37x_pac::Peripherals::steal() };

    // setup the stm
    let stm0 = p.STM0;
    let mut time = stm0.tim0.read().bits();
    time = time.wrapping_add(SYSTEM_TIMER_FREQ_HZ);
    //See Aurix TC3XX target specification, section of System timers
    //Set compare match control register
    stm0.cmcon.modify(|_, w| w.msize0().variant(31));
    //Set compare
    stm0.cmp0.write(|w| unsafe { w.bits(time) });
    //Interrupt set/clear register - reset compare register
    stm0.iscr.write(|w| w.cmp0irr().set_bit());
    //Interrupt control register - enable compare
    stm0.icr.write(|w| w.cmp0en().set_bit());
}

fn enable_timer0_interrupt() {
    let p = unsafe { tc37x_pac::Peripherals::steal() };
    let src = p.SRC;
    // See 'Interrupt Router section' in specification
    src.stm0sr0.modify(|_, w| w.tos().variant(0));
    src.stm0sr0.modify(|_, w| w.srpn().variant(2));
    src.stm0sr0.modify(|_, w| w.sre().set_bit());
}

fn interrupt_handler() {
    BoardLed::Led2.toggle_led();

    let p = unsafe { tc37x_pac::Peripherals::steal() };
    let stm0 = p.STM0;
    let mut time = stm0.cmp0.read().bits();
    time = time.wrapping_add(SYSTEM_TIMER_FREQ_HZ);
    stm0.cmp0.write(|w| unsafe { w.bits(time) });
    stm0.iscr.write(|w| w.cmp0irr().set_bit());
}
