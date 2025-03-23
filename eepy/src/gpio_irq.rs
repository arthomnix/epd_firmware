use core::cell::RefCell;
use core::sync::atomic::Ordering;
use critical_section::Mutex;
use defmt::{debug, trace, warn};
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use mcp9808::MCP9808;
use mcp9808::reg_conf::ShutdownMode;
use mcp9808::reg_conf::Configuration;
use eepy_sys::input_common::{Event, TouchEvent, TouchEventType};
use fw16_epd_bsp::{pac, EpdPowerSwitch, EpdTouchInt, EpdTouchReset, I2CScl, I2CSda, LaptopSleep};
use fw16_epd_bsp::hal::gpio::Interrupt::{EdgeHigh, EdgeLow};
use fw16_epd_bsp::hal::{Sio, I2C};
use fw16_epd_bsp::pac::{interrupt, I2C0};
use crate::{reset_touch, EVENT_QUEUE, TOUCH_ENABLED};
use crate::core1::{ToCore1Message, IDLE};

pub(super) static GLOBAL_TOUCH_INT_PIN: Mutex<RefCell<Option<EpdTouchInt>>> = Mutex::new(RefCell::new(None));
pub(super) static GLOBAL_I2C: Mutex<RefCell<Option<I2C<I2C0, (I2CSda, I2CScl)>>>> = Mutex::new(RefCell::new(None));
pub(super) static GLOBAL_SLEEP_PIN: Mutex<RefCell<Option<LaptopSleep>>> = Mutex::new(RefCell::new(None));
pub(super) static GLOBAL_EPD_POWER_PIN: Mutex<RefCell<Option<EpdPowerSwitch>>> = Mutex::new(RefCell::new(None));
pub(super) static GLOBAL_TOUCH_RESET_PIN: Mutex<RefCell<Option<EpdTouchReset>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn IO_IRQ_BANK0() {
    static mut TOUCH_INT_PIN: Option<EpdTouchInt> = None;
    static mut SLEEP_PIN: Option<LaptopSleep> = None;
    static mut EPD_POWER_PIN: Option<EpdPowerSwitch> = None;
    static mut TOUCH_RESET_PIN: Option<EpdTouchReset> = None;

    trace!("IO_IRQ_BANK0");

    if TOUCH_INT_PIN.is_none() {
        critical_section::with(|cs| *TOUCH_INT_PIN = GLOBAL_TOUCH_INT_PIN.borrow(cs).take());
    }

    if SLEEP_PIN.is_none() {
        critical_section::with(|cs| *SLEEP_PIN = GLOBAL_SLEEP_PIN.borrow(cs).take());
    }

    if EPD_POWER_PIN.is_none() {
        critical_section::with(|cs| *EPD_POWER_PIN = GLOBAL_EPD_POWER_PIN.borrow(cs).take());
    }

    if TOUCH_RESET_PIN.is_none() {
        critical_section::with(|cs| *TOUCH_RESET_PIN = GLOBAL_TOUCH_RESET_PIN.borrow(cs).take());
    }

    let mut i2c = critical_section::with(|cs| GLOBAL_I2C.borrow(cs).take());

    if let Some(pin) = TOUCH_INT_PIN {
        if pin.interrupt_status(EdgeLow) {

            if TOUCH_ENABLED.load(Ordering::Relaxed) {
                if let Some(i2c) = &mut i2c {
                    let mut buf = [0u8; 9];
                    i2c.write_read(0x38u8, &[0x00], &mut buf).unwrap();
                    let x = (((buf[3] & 0x0f) as u16) << 8) | buf[4] as u16;
                    let y = (((buf[5] & 0x0f) as u16) << 8) | buf[6] as u16;

                    let state = match buf[3] >> 6 {
                        0 => TouchEventType::Down,
                        1 => TouchEventType::Up,
                        2 => TouchEventType::Move,
                        _ => panic!("received invalid touch event type {}", buf[3] >> 6),
                    };

                    let event = TouchEvent {
                        ev_type: state,
                        x,
                        y,
                    };

                    debug!("touch event: {}", event);

                    if !critical_section::with(|cs| EVENT_QUEUE.borrow_ref_mut(cs).push(Event::Touch(event))) {
                        warn!("touch event buffer full");
                    }
                }
            }

        }
        pin.clear_interrupt(EdgeLow);
    }

    if let Some(pin) = SLEEP_PIN {
        if pin.interrupt_status(EdgeLow) {
            debug!("sleeping");
            pin.clear_interrupt(EdgeLow);

            if let Some(touch_int) = TOUCH_INT_PIN {
                touch_int.set_interrupt_enabled(EdgeLow, false);
            }

            // Wait until refresh has finished
            while !IDLE.load(Ordering::Relaxed) {}

            // Power down temp sensor
            if let Some(i2c) = &mut i2c {
                let mut mcp9808 = MCP9808::new(i2c);
                let mut config = mcp9808.read_configuration().unwrap();
                config.set_shutdown_mode(ShutdownMode::Shutdown);
                mcp9808.write_register(config).unwrap();
            }

            // Power down display
            // (pin is connected to P-ch MOSFET so high = off)
            if let Some(power_pin) = EPD_POWER_PIN {
                power_pin.set_high().unwrap();
            }

            pac::NVIC::pend(interrupt::SW0_IRQ);
        }

        if pin.interrupt_status(EdgeHigh) {
            pin.clear_interrupt(EdgeHigh);

            // Power up temp sensor
            if let Some(i2c) = &mut i2c {
                let mut mcp9808 = MCP9808::new(i2c);
                let mut config = mcp9808.read_configuration().unwrap();
                config.set_shutdown_mode(ShutdownMode::Continuous);
                mcp9808.write_register(config).unwrap();
            }

            // Power up EPD
            if let Some(power_pin) = EPD_POWER_PIN {
                power_pin.set_low().unwrap();
            }

            let mut fifo = Sio::new(unsafe { pac::Peripherals::steal() }.SIO).fifo;
            fifo.write_blocking(ToCore1Message::HardResetEpd as u32);

            if let Some(reset) = TOUCH_RESET_PIN {
                reset_touch(reset);
            }

            if let Some(touch_int) = TOUCH_INT_PIN {
                touch_int.clear_interrupt(EdgeLow);
                touch_int.set_interrupt_enabled(EdgeLow, true);
            }

            debug!("woken up");
        }
    }

    critical_section::with(|cs| GLOBAL_I2C.borrow(cs).replace(i2c));
}

#[interrupt]
fn SW0_IRQ() {
    trace!("SW0_IRQ");
    cortex_m::asm::wfi();
}