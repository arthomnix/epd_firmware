#![no_main]
#![no_std]

//mod serial;
mod ringbuffer;
mod syscall;
mod launcher;
mod usb;
mod exception;

extern crate panic_probe;
extern crate defmt_rtt;

use core::arch::asm;
use core::cell::RefCell;
use critical_section::Mutex;
use defmt::{debug, info, trace, warn};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use mcp9808::MCP9808;
use mcp9808::reg_conf::{Configuration, ShutdownMode};
use mcp9808::reg_res::{Resolution, ResolutionVal};
use mcp9808::reg_temp_generic::ReadableTempRegister;
use once_cell::sync::OnceCell;
use portable_atomic::{AtomicBool, AtomicU8};
use portable_atomic::Ordering;
use eepy_sys::exec::exec;
use fw16_epd_bsp::{entry, hal, pac, EpdBusy, EpdCs, EpdDc, EpdPowerSwitch, EpdReset, EpdSck, EpdSdaWrite, EpdTouchInt, EpdTouchReset, I2CScl, I2CSda, LaptopSleep, Pins};
use fw16_epd_bsp::hal::{Sio, Timer, I2C};
use fw16_epd_bsp::hal::clocks::ClockSource;
use fw16_epd_bsp::hal::fugit::{RateExtU32, ExtU32};
use fw16_epd_bsp::hal::gpio::Interrupt::{EdgeHigh, EdgeLow};
use fw16_epd_bsp::hal::multicore::{Multicore, Stack};
use fw16_epd_bsp::hal::timer::{Alarm, Alarm0};
use fw16_epd_bsp::pac::I2C0;
use fw16_epd_bsp::pac::interrupt;
use eepy_sys::input_common::{Event, TouchEvent, TouchEventType};
use tp370pgh01::rp2040::{Rp2040PervasiveSpiDelays, IoPin};
use tp370pgh01::{Tp370pgh01, IMAGE_BYTES};
use crate::ringbuffer::RingBuffer;

const SRAM_END: *mut u8 = 0x20042000 as *mut u8;

static CORE1_STACK: Stack<8192> = Stack::new();

static GLOBAL_TOUCH_INT_PIN: Mutex<RefCell<Option<EpdTouchInt>>> = Mutex::new(RefCell::new(None));
static GLOBAL_I2C: Mutex<RefCell<Option<I2C<I2C0, (I2CSda, I2CScl)>>>> = Mutex::new(RefCell::new(None));
static GLOBAL_SLEEP_PIN: Mutex<RefCell<Option<LaptopSleep>>> = Mutex::new(RefCell::new(None));
static GLOBAL_EPD_POWER_PIN: Mutex<RefCell<Option<EpdPowerSwitch>>> = Mutex::new(RefCell::new(None));
static GLOBAL_TOUCH_RESET_PIN: Mutex<RefCell<Option<EpdTouchReset>>> = Mutex::new(RefCell::new(None));

static GLOBAL_ALARM0: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));

static IMAGE_BUFFER: Mutex<RefCell<[u8; IMAGE_BYTES]>> = Mutex::new(RefCell::new([0; IMAGE_BYTES]));
static DO_REFRESH: AtomicBool = AtomicBool::new(false);
static FAST_REFRESH: AtomicBool = AtomicBool::new(false);
static REFRESHING: AtomicBool = AtomicBool::new(false);
static EPD_NEEDS_HARD_RESET: AtomicBool = AtomicBool::new(true);
static TEMP: AtomicU8 = AtomicU8::new(20);
static SERIAL_NUMBER: OnceCell<[u8; 16]> = OnceCell::new();

static EVENT_QUEUE: Mutex<RefCell<RingBuffer<Event>>> = Mutex::new(RefCell::new(RingBuffer::new()));
static TOUCH_ENABLED: AtomicBool = AtomicBool::new(false);

static FLASHING: AtomicBool = AtomicBool::new(false);
static FLASHING_ACK: AtomicBool = AtomicBool::new(false);

/// Function in RAM to be executed by core1 whilst flashing programs (core1 cannot be executing code
/// from flash whilst writing/erasing flash)
///
/// To exit this function from core0, set [FLASHING] to false and SEV
#[link_section = ".data.ram_func"]
fn core1_flash_wait() {
    cortex_m::interrupt::disable();
    FLASHING_ACK.store(true, Ordering::Relaxed);

    while FLASHING.load(Ordering::Relaxed) {
        cortex_m::asm::wfe();
    }

    FLASHING_ACK.store(false, Ordering::Relaxed);
    unsafe { cortex_m::interrupt::enable() };
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        fw16_epd_bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    ).unwrap();

    core.SCB.set_sleepdeep();

    // Set MPU regions for non-privileged code
    unsafe {
        // Region 0: Unprivileged RAM region (first 128K) - start 0x20020000, length 128K
        core.MPU.rnr.write(0);
        core.MPU.rbar.write(0x20020000);
        core.MPU.rasr.write(
            (0b1 << 28) // XN
            | (0b011 << 24) // AP: full access
            | (0b000 << 19) // TEX
            | (0b011 << 16) // S, C, B: non-shareable, writeback
            | (0b00000000 << 8) // all subregions enabled
            | (16 << 1) // size: 2^(16 + 1) = 128K
            | (0b1 << 0) // enable
        );

        // Region 1: Unprivileged RAM region (final 8K) - start 0x20040000, length 8K
        core.MPU.rnr.write(1);
        core.MPU.rbar.write(0x20040000);
        core.MPU.rasr.write(
            (0b1 << 28) // XN
                | (0b011 << 24) // AP: full access
                | (0b000 << 19) // TEX
                | (0b011 << 16) // S, C, B: non-shareable, writeback
                | (0b00000000 << 8) // all subregions enabled
                | (12 << 1) // size: 2^(12 + 1) = 8K
                | (0b1 << 0) // enable
        );

        // Region 3: Flash - start: 0x10000000, length 16M
        core.MPU.rnr.write(3);
        core.MPU.rbar.write(0x10000000);
        core.MPU.rasr.write(
            (0b0 << 28) // disable XN
                | (0b110 << 24) // AP: privileged or unprivileged read-only
                | (0b000 << 19) // TEX
                | (0b110 << 16) // S, C, B: shared, write-through
                | (0b00000000 << 8) // all subregions enabled
                | (23 << 1) // size: 2^(23 + 1) = 16M
                | (0b1 << 0) // enable
        );

        // Enable MPU
        core.MPU.ctrl.write(
            (0b1 << 2) // PRIVDEFENA: enable default memory map for privileged access
            | (0b0 << 1) // HFNMIENA: disable MPU for HardFault and NMI
            | (0b1 << 0) // ENABLE: enable MPU
        );
    }

    // Read flash unique ID
    cortex_m::interrupt::disable();
    let mut id = [0u8; 8];
    unsafe { rp2040_flash::flash::flash_unique_id(&mut id, true) };
    let mut id = u64::from_be_bytes(id);
    let mut serial = [0u8; 16];
    for c in serial.iter_mut().rev() {
        let nibble = (id & 0x0f) as u8;
        *c = match nibble {
            0x0..=0x9 => b'0' + nibble,
            0xa..=0xf => b'A' + nibble - 0xa,
            _ => unreachable!(),
        };
        id >>= 4;
    }
    SERIAL_NUMBER.set(serial).unwrap();
    unsafe { cortex_m::interrupt::enable() };

    info!("eepyOS version {} (c) arthomnix 2025", env!("CARGO_PKG_VERSION"));
    info!("Serial number: {}", unsafe { core::str::from_utf8_unchecked(SERIAL_NUMBER.get().unwrap()) });

    let mut sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );


    let mut epd_power: EpdPowerSwitch = pins.epd_pwr_sw.reconfigure();
    epd_power.set_low().unwrap();
    critical_section::with(|cs| GLOBAL_EPD_POWER_PIN.borrow_ref_mut(cs).replace(epd_power));

    let mut sleep = pins.laptop_sleep.into_pull_up_input();
    sleep.set_interrupt_enabled(EdgeLow, true);
    sleep.set_interrupt_enabled(EdgeHigh, true);
    sleep.clear_interrupt(EdgeLow);
    sleep.clear_interrupt(EdgeHigh);
    critical_section::with(|cs| GLOBAL_SLEEP_PIN.borrow_ref_mut(cs).replace(sleep));

    let mut touch_reset: EpdTouchReset = pins.epd_touch_rst.reconfigure();
    touch_reset.set_high().unwrap();
    cortex_m::asm::delay(1000000);
    touch_reset.set_low().unwrap();
    cortex_m::asm::delay(1000000);
    touch_reset.set_high().unwrap();
    cortex_m::asm::delay(10000000);
    critical_section::with(|cs| GLOBAL_TOUCH_RESET_PIN.borrow_ref_mut(cs).replace(touch_reset));

    let cs: EpdCs = pins.spi3_epd_cs.reconfigure();
    let dc: EpdDc = pins.epd_dc.reconfigure();
    let busy: EpdBusy = pins.epd_busy.reconfigure();
    let rst: EpdReset = pins.epd_rst.reconfigure();
    let sda: EpdSdaWrite = pins.spi3_epd_sda.reconfigure();
    let sck: EpdSck = pins.spi3_epd_sck.reconfigure();

    let i2c_sda: I2CSda = pins.i2c_sda.reconfigure();
    let i2c_scl: I2CScl = pins.i2c_scl.reconfigure();
    let int: EpdTouchInt = pins.epd_touch_int.reconfigure();
    int.set_interrupt_enabled(EdgeLow, true);

    let mut i2c = hal::i2c::I2C::i2c0(pac.I2C0, i2c_sda, i2c_scl, 400.kHz(), &mut pac.RESETS, clocks.system_clock.get_freq());

    {
        let mut mcp9808 = MCP9808::new(&mut i2c);
        let mut res = mcp9808::reg_res::new();
        res.set_resolution(ResolutionVal::Deg_0_5C);
        mcp9808.write_register(res).unwrap();
    }

    critical_section::with(|cs| {
        GLOBAL_TOUCH_INT_PIN.borrow_ref_mut(cs).replace(int);
        GLOBAL_I2C.borrow_ref_mut(cs).replace(i2c);
    });

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    // make sure temperature sensor has read temperature
    timer.delay_ms(35);
    let mut alarm = timer.alarm_0().unwrap();
    alarm.enable_interrupt();
    critical_section::with(|cs| GLOBAL_ALARM0.borrow_ref_mut(cs).replace(alarm));
    unsafe {
        core.NVIC.set_priority(interrupt::TIMER_IRQ_0, 0b10000000);
        pac::NVIC::unmask(interrupt::TIMER_IRQ_0);
    }
    pac::NVIC::pend(interrupt::TIMER_IRQ_0);

    usb::init_usb(pac.USBCTRL_REGS, pac.USBCTRL_DPRAM, clocks.usb_clock);

    unsafe {
        core.NVIC.set_priority(interrupt::USBCTRL_IRQ, 0b11000000);
    }

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let core1 = &mut mc.cores()[1];
    core1.spawn(CORE1_STACK.take().unwrap(), move || {
        info!("core1 init");

        let mut epd = Tp370pgh01::new(cs, IoPin::new(sda), sck, dc, busy, rst, timer, Rp2040PervasiveSpiDelays);

        let mut prev_image = [0u8; IMAGE_BYTES];
        let mut image = [0u8; IMAGE_BYTES];

        loop {
            cortex_m::asm::wfe();

            if FLASHING.load(Ordering::Relaxed) {
                core1_flash_wait();
                continue;
            }

            if EPD_NEEDS_HARD_RESET.swap(false, Ordering::Relaxed) {
                epd.hard_reset().unwrap();
            }

            REFRESHING.store(true, Ordering::Relaxed);

            if DO_REFRESH.swap(false, Ordering::Relaxed) {
                if FAST_REFRESH.load(Ordering::Relaxed) {
                    prev_image.copy_from_slice(&image);
                    critical_section::with(|cs| image.copy_from_slice(IMAGE_BUFFER.borrow_ref(cs).as_ref()));
                    epd.soft_reset().unwrap();
                    epd.refresh_fast(&image, &prev_image, TEMP.load(Ordering::Relaxed)).unwrap();
                } else {
                    critical_section::with(|cs| image.copy_from_slice(IMAGE_BUFFER.borrow_ref(cs).as_ref()));
                    epd.soft_reset().unwrap();
                    epd.refresh(&image, TEMP.load(Ordering::Relaxed)).unwrap();
                }

                critical_section::with(|cs| EVENT_QUEUE.borrow_ref_mut(cs).push(Event::RefreshFinished));
                cortex_m::asm::sev();
            }

            REFRESHING.store(false, Ordering::Relaxed);
        }
    }).unwrap();

    unsafe {
        core.NVIC.set_priority(interrupt::IO_IRQ_BANK0, 0);
        core.NVIC.set_priority(interrupt::SW0_IRQ, 0b01000000);
        pac::NVIC::unmask(interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(interrupt::SW0_IRQ);
    };

    unsafe {
        asm!(
            "msr psp, {sram_end}",
            "msr control, {control_0b10}",
            "isb",
            sram_end = in(reg) SRAM_END,
            control_0b10 = in(reg) 0b10,
        );

        exec(0);
    }
}

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
            while REFRESHING.load(Ordering::Relaxed) {}

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

            EPD_NEEDS_HARD_RESET.store(true, Ordering::Relaxed);

            if let Some(reset) = TOUCH_RESET_PIN {
                cortex_m::asm::delay(1000000);
                reset.set_high().unwrap();
                cortex_m::asm::delay(1000000);
                reset.set_low().unwrap();
                cortex_m::asm::delay(1000000);
                reset.set_high().unwrap();
                cortex_m::asm::delay(10000000);
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