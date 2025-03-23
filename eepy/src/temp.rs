use core::sync::atomic::Ordering;
use defmt::{debug, trace};
use mcp9808::MCP9808;
use mcp9808::reg_temp_generic::ReadableTempRegister;
use fw16_epd_bsp::hal::fugit::ExtU32;
use fw16_epd_bsp::hal::timer::{Alarm, Alarm0};
use fw16_epd_bsp::pac::interrupt;
use crate::{GLOBAL_ALARM0, GLOBAL_I2C, TEMP};

#[interrupt]
fn TIMER_IRQ_0() {
    static mut ALARM0: Option<Alarm0> = None;

    trace!("TIMER_IRQ_0");


    if ALARM0.is_none() {
        critical_section::with(|cs| *ALARM0 = GLOBAL_ALARM0.borrow(cs).take());
    }

    let mut i2c = critical_section::with(|cs| GLOBAL_I2C.borrow(cs).take());

    if let Some(alarm) = ALARM0 {
        if let Some(i2c) = &mut i2c {
            let mut mcp9808 = MCP9808::new(i2c);

            let temp = ((mcp9808.read_temperature().unwrap().get_raw_value() & 0x1ff0) >> 4) as i16;
            let clamped = match temp {
                ..=0 => 0u8,
                60.. => 60u8,
                t => t as u8,
            };

            debug!("read temperature {}C (using {}C)", temp, clamped);

            TEMP.store(clamped, Ordering::Relaxed);

            alarm.clear_interrupt();
            alarm.schedule(1.minutes()).unwrap();
        } else {
            // I2C bus was in use, so try again in a short time
            alarm.clear_interrupt();
            alarm.schedule(10.millis()).unwrap();
        }
    }

    critical_section::with(|cs| GLOBAL_I2C.borrow(cs).replace(i2c));
}