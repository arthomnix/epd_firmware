#![no_std]

pub extern crate rp2040_hal as hal;

pub use hal::pac;

#[cfg(feature = "rt")]
pub use rp2040_hal::entry;

#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

hal::bsp_pins! {
    Gpio2 {
        name: epd_touch_int,
        aliases: {
            FunctionSioInput, PullUp: EpdTouchInt
        }
    },

    Gpio3 {
        name: epd_touch_rst,
        aliases: {
            FunctionSioOutput, PullNone: EpdTouchReset
        }
    },

    Gpio4 {
        name: i2c_sda,
        aliases: {
            FunctionI2C, PullUp: I2CSda
        }
    }

    Gpio5 {
        name: i2c_scl,
        aliases: {
            FunctionI2C, PullUp: I2CScl
        }
    }

    Gpio6 {
        name: epd_pwr_sw,
        aliases: {
            FunctionSioOutput, PullNone: EpdPowerSwitch
        }
    },

    Gpio8 {
        name: epd_busy,
        aliases: {
            FunctionSioInput, PullUp: EpdBusy
        }
    },

    Gpio9 {
        name: epd_rst,
        aliases: {
            FunctionSioOutput, PullNone: EpdReset
        }
    },

    Gpio10 {
        name: epd_dc,
        aliases: {
            FunctionSioOutput, PullNone: EpdDc
        }
    },

    Gpio11 {
        name: spi3_epd_sda,
        aliases: {
            FunctionSioOutput, PullNone: EpdSdaWrite,
            FunctionSioInput, PullNone: EpdSdaRead
        }
    }

    Gpio13 {
        name: spi3_epd_cs,
        aliases: {
            FunctionSioOutput, PullNone: EpdCs
        }
    }

    Gpio14 {
        name: spi3_epd_sck,
        aliases: {
            FunctionSioOutput, PullNone: EpdSck,
            FunctionPio0, PullNone: EpdSckPio
        }
    }

    Gpio25 {
        name: laptop_sleep,
        aliases: {
            FunctionSioInput, PullUp: LaptopSleep
        }
    },
}

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
