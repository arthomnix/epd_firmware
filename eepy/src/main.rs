#![no_main]
#![no_std]

mod programs;
mod gui;
mod serial;
mod ringbuffer;

#[allow(unused_imports)]
use panic_probe as _;
#[allow(unused_imports)]
use defmt_rtt as _;

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
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::SerialPort;
use fw16_epd_bsp::{entry, hal, pac, EpdBusy, EpdCs, EpdDc, EpdPowerSwitch, EpdReset, EpdSck, EpdSdaWrite, EpdTouchInt, EpdTouchReset, I2CScl, I2CSda, LaptopSleep, Pins};
use fw16_epd_bsp::hal::{Sio, Timer, I2C};
use fw16_epd_bsp::hal::clocks::ClockSource;
use fw16_epd_bsp::hal::fugit::{RateExtU32, ExtU32};
use fw16_epd_bsp::hal::gpio::Interrupt::{EdgeHigh, EdgeLow};
use fw16_epd_bsp::hal::multicore::{Multicore, Stack};
use fw16_epd_bsp::hal::timer::{Alarm, Alarm0};
use fw16_epd_bsp::pac::I2C0;
use fw16_epd_bsp::pac::interrupt;
use eepy_gui::draw_target::EpdDrawTarget;
use eepy_sys::{Event, RefreshBlockMode, SafeOption, TouchEvent, TouchEventType};
use tp370pgh01::rp2040::{Rp2040PervasiveSpiDelays, IoPin};
use tp370pgh01::{Tp370pgh01, IMAGE_BYTES};
use crate::ringbuffer::RingBuffer;
//use crate::programs::Programs;

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

static mut GLOBAL_USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut GLOBAL_USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut GLOBAL_USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

static SERIAL_NUMBER: OnceCell<[u8; 16]> = OnceCell::new();

static EVENT_QUEUE: Mutex<RefCell<RingBuffer<Event>>> = Mutex::new(RefCell::new(RingBuffer::new()));
static TOUCH_ENABLED: AtomicBool = AtomicBool::new(false);

static FLASHING: AtomicBool = AtomicBool::new(false);
static FLASHING_ACK: AtomicBool = AtomicBool::new(false);

extern "C" fn write_image(image: &[u8; IMAGE_BYTES]) {
    critical_section::with(|cs| IMAGE_BUFFER.borrow_ref_mut(cs).copy_from_slice(image));
}

extern "C" fn refresh(fast_refresh: bool, block_mode: RefreshBlockMode) {
    DO_REFRESH.store(true, Ordering::Relaxed);
    FAST_REFRESH.store(fast_refresh, Ordering::Relaxed);
    cortex_m::asm::sev();

    if matches!(block_mode, RefreshBlockMode::BlockAcknowledge | RefreshBlockMode::BlockFinish) {
        while DO_REFRESH.load(Ordering::Relaxed) {}
    }

    if matches!(block_mode, RefreshBlockMode::BlockFinish) {
        while REFRESHING.load(Ordering::Relaxed) {}
    }
}

extern "C" fn next_event() -> SafeOption<Event> {
    critical_section::with(|cs| EVENT_QUEUE.borrow_ref_mut(cs).pop()).into()
}

unsafe extern "C" fn set_touch_enabled(enable: bool) {
    TOUCH_ENABLED.store(enable, Ordering::Relaxed);
    if !enable {
        critical_section::with(|cs| EVENT_QUEUE.borrow_ref_mut(cs).clear());
    }
}

extern "C" fn serial_number() -> &'static [u8; 16] {
    SERIAL_NUMBER.get().unwrap()
}

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

    info!("Framework 16 EPD firmware version {}, serial no. {}", env!("CARGO_PKG_VERSION"), unsafe { core::str::from_utf8_unchecked(serial_number()) });

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

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    unsafe {
        GLOBAL_USB_BUS = Some(usb_bus);
    }

    // Safety: These are only accessed within this interrupt handler, or in main() before the
    // interrupt is enabled.
    #[allow(static_mut_refs)]
    let bus_ref = unsafe { GLOBAL_USB_BUS.as_ref().unwrap() };

    let serial = SerialPort::new(bus_ref);
    let usb_device = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x2e8a, 0x000a))
        .strings(&[StringDescriptors::default()
            .manufacturer("arthomnix")
            .product("Touchscreen EPD Input Module for Framework 16")
            .serial_number(unsafe { core::str::from_utf8_unchecked(serial_number()) })
        ])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    unsafe {
        GLOBAL_USB_SERIAL = Some(serial);
        GLOBAL_USB_DEVICE = Some(usb_device);
    }

    unsafe { pac::NVIC::unmask(interrupt::USBCTRL_IRQ) };

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
            }

            critical_section::with(|cs| EVENT_QUEUE.borrow_ref_mut(cs).push(Event::RefreshFinished));
            REFRESHING.store(false, Ordering::Relaxed);
        }
    }).unwrap();

    unsafe {
        core.NVIC.set_priority(interrupt::IO_IRQ_BANK0, 0);
        core.NVIC.set_priority(interrupt::SW0_IRQ, 0b01000000);
        pac::NVIC::unmask(interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(interrupt::SW0_IRQ);
    };

    /*let program = Programs::new().next();
    if let Some(program) = program {
        unsafe {
            program.load();
            let pft = ProgramFunctionTable {
                write_image,
                refresh,
                next_event,
                set_touch_enabled,
                serial_number,
            };
            program.entry()(&pft);
        }
    }*/

    let draw_target = EpdDrawTarget::new(write_image, refresh);
    gui::gui_main(draw_target);
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