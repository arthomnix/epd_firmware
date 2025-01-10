#![no_main]
#![no_std]

mod programs;

#[allow(unused_imports)]
use panic_probe as _;
#[allow(unused_imports)]
use defmt_rtt as _;

use core::cell::RefCell;
use critical_section::Mutex;
use defmt::{debug, error, info, trace};
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle};
use embedded_graphics::text::Text;
use embedded_hal::digital::{OutputPin, PinState};
use embedded_hal::i2c::I2c;
use embedded_hal_bus::i2c::RefCellDevice;
use mcp9808::MCP9808;
use mcp9808::reg_res::ResolutionVal;
use mcp9808::reg_temp_generic::ReadableTempRegister;
use portable_atomic::{AtomicBool, AtomicU8};
use portable_atomic::Ordering;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::SerialPort;
use fw16_epd_bsp::{entry, hal, pac, EpdBusy, EpdCs, EpdDc, EpdReset, EpdSck, EpdSdaWrite, EpdTouchInt, I2CScl, I2CSda, Pins};
use fw16_epd_bsp::hal::{Sio, Timer, I2C};
use fw16_epd_bsp::hal::clocks::ClockSource;
use fw16_epd_bsp::hal::fugit::{RateExtU32, ExtU32};
use fw16_epd_bsp::hal::gpio::Interrupt::EdgeLow;
use fw16_epd_bsp::hal::multicore::{Multicore, Stack};
use fw16_epd_bsp::hal::timer::{Alarm, Alarm0};
use fw16_epd_bsp::pac::I2C0;
use fw16_epd_bsp::pac::interrupt;
use fw16_epd_program_interface::eg::EpdDrawTarget;
use fw16_epd_program_interface::ProgramFunctionTable;
use tp370pgh01::rp2040::{Rp2040PervasiveSpiDelays, IoPin};
use tp370pgh01::{Tp370pgh01, IMAGE_BYTES};
use crate::programs::Programs;

static CORE1_STACK: Stack<8192> = Stack::new();

static GLOBAL_TOUCH_INT_PIN: Mutex<RefCell<Option<EpdTouchInt>>> = Mutex::new(RefCell::new(None));
static GLOBAL_I2C: Mutex<RefCell<Option<I2C<I2C0, (I2CSda, I2CScl)>>>> = Mutex::new(RefCell::new(None));

static GLOBAL_ALARM0: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));

static IMAGE_BUFFER: Mutex<RefCell<[u8; IMAGE_BYTES]>> = Mutex::new(RefCell::new([0; IMAGE_BYTES]));
static DO_REFRESH: AtomicBool = AtomicBool::new(false);
static FAST_REFRESH: AtomicBool = AtomicBool::new(false);
static TEMP: AtomicU8 = AtomicU8::new(20);

static mut GLOBAL_USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut GLOBAL_USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut GLOBAL_USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

extern "C" fn write_image(image: &[u8; IMAGE_BYTES]) {
    critical_section::with(|cs| IMAGE_BUFFER.borrow_ref_mut(cs).copy_from_slice(image));
}

extern "C" fn refresh() {
    DO_REFRESH.store(true, Ordering::Relaxed);
    FAST_REFRESH.store(false, Ordering::Relaxed);
    cortex_m::asm::sev();
}

extern "C" fn refresh_fast() {
    DO_REFRESH.store(true, Ordering::Relaxed);
    FAST_REFRESH.store(true, Ordering::Relaxed);
    cortex_m::asm::sev();
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
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

    // Read flash unique ID
    cortex_m::interrupt::disable();
    let mut id = [0u8; 8];
    unsafe { rp2040_flash::flash::flash_unique_id(&mut id, true) };
    let mut id = u64::from_be_bytes(id);
    info!("Framework 16 EPD firmware version {}, serial no. {:x}", env!("CARGO_PKG_VERSION"), id);
    let mut serial_no = [0u8; 16];
    for c  in serial_no.iter_mut().rev() {
        let nibble = (id & 0x0f) as u8;
        *c = match nibble {
            0x0..=0x9 => b'0' + nibble,
            0xa..=0xf => b'A' + nibble - 0xa,
            _ => unreachable!(),
        };
        id >>= 4;
    }
    // Safety: this function never returns, so we should be fine right?
    let serial_no: &'static str = unsafe { &*&raw const *core::str::from_utf8(&serial_no).unwrap() };
    unsafe { cortex_m::interrupt::enable() };

    let mut sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );


    let _power = pins.epd_pwr_sw.into_push_pull_output_in_state(PinState::Low);

    let mut touch_reset = pins.epd_touch_rst.into_push_pull_output_in_state(PinState::High);
    cortex_m::asm::delay(120000);
    touch_reset.set_low().unwrap();
    cortex_m::asm::delay(120000);
    touch_reset.set_high().unwrap();
    cortex_m::asm::delay(12000000);


    let cs: EpdCs = pins.spi3_epd_cs.reconfigure();
    let dc: EpdDc = pins.epd_dc.reconfigure();
    let busy: EpdBusy = pins.epd_busy.reconfigure();
    let rst: EpdReset = pins.epd_rst.reconfigure();
    let sda: EpdSdaWrite = pins.spi3_epd_sda.reconfigure();
    let sck: EpdSck = pins.spi3_epd_sck.reconfigure();

    let i2c_sda: I2CSda = pins.i2c_sda.reconfigure();
    let i2c_scl: I2CScl = pins.i2c_scl.reconfigure();
    let int: EpdTouchInt = pins.epd_touch_int.reconfigure();

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut alarm = timer.alarm_0().unwrap();
    alarm.enable_interrupt();
    critical_section::with(|cs| GLOBAL_ALARM0.borrow_ref_mut(cs).replace(alarm));
    unsafe { pac::NVIC::unmask(interrupt::TIMER_IRQ_0); }
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
            .serial_number(serial_no)
        ])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    unsafe {
        GLOBAL_USB_SERIAL = Some(serial);
        GLOBAL_USB_DEVICE = Some(usb_device);
    }

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let core1 = &mut mc.cores()[1];
    core1.spawn(CORE1_STACK.take().unwrap(), move || {
        info!("core1 init");

        int.set_interrupt_enabled(EdgeLow, true);

        let i2c = hal::i2c::I2C::i2c0(pac.I2C0, i2c_sda, i2c_scl, 400.kHz(), &mut pac.RESETS, clocks.system_clock.get_freq());

        critical_section::with(|cs| {
            GLOBAL_TOUCH_INT_PIN.borrow_ref_mut(cs).replace(int);
            GLOBAL_I2C.borrow_ref_mut(cs).replace(i2c);
        });

        unsafe {
            pac::NVIC::unmask(interrupt::IO_IRQ_BANK0);
            pac::NVIC::unmask(interrupt::USBCTRL_IRQ);
        }

        let mut epd = Tp370pgh01::new(cs, IoPin::new(sda), sck, dc, busy, rst, timer, Rp2040PervasiveSpiDelays);
        epd.hard_reset().unwrap();

        let mut prev_image = [0u8; IMAGE_BYTES];
        let mut image = [0u8; IMAGE_BYTES];

        loop {
            cortex_m::asm::wfe();

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
        }
    }).unwrap();


    let mut draw_target = EpdDrawTarget::new(ProgramFunctionTable {
        write_image,
        refresh,
        refresh_fast,
    });

    Text::new("Hello, World!", Point::new(20, 20), MonoTextStyle::new(&FONT_10X20, BinaryColor::On))
        .draw(&mut draw_target)
        .unwrap();
    draw_target.refresh();

    let programs = Programs::new();
    for _program in programs {
        cortex_m::asm::delay(0);
    }

    loop {
        cortex_m::asm::wfi();
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

            let temp = mcp9808.read_temperature().unwrap().get_celsius(ResolutionVal::Deg_0_0625C);
            let clamped = match temp {
                ..=0.0 => 0u8,
                60.0.. => 60u8,
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
fn USBCTRL_IRQ() {
    static mut INDEX: usize = 0;

    trace!("USBCTRL_IRQ");

    // Safety: These are only accessed within this interrupt handler, or in main() before the
    // interrupt is enabled.
    #[allow(static_mut_refs)]
    let usb_dev = unsafe { GLOBAL_USB_DEVICE.as_mut().unwrap() };
    #[allow(static_mut_refs)]
    let serial = unsafe { GLOBAL_USB_SERIAL.as_mut().unwrap() };

    if usb_dev.poll(&mut [serial]) {
        critical_section::with(|cs| {
            let mut buf = IMAGE_BUFFER.borrow_ref_mut(cs);
            match serial.read(&mut buf.as_mut()[*INDEX..]) {
                Err(UsbError::WouldBlock) => {},
                Err(e) => panic!("{e:?}"),
                Ok(count) => {
                    *INDEX += count;
                    if *INDEX >= (240 * 416) / 8 {
                        info!("Finished rx frame over USB");
                        *INDEX = 0;
                        FAST_REFRESH.store(false, Ordering::Relaxed);
                        DO_REFRESH.store(true, Ordering::Relaxed);
                    }
                }
            }
        })
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    static mut FIRST_EVENT: bool = true;
    static mut TOUCH_INT_PIN: Option<EpdTouchInt> = None;
    static mut PREV_POS: Option<Point> = None;
    static mut DRAW_TARGET: EpdDrawTarget = EpdDrawTarget::new(ProgramFunctionTable {
        write_image,
        refresh,
        refresh_fast,
    });

    trace!("IO_IRQ_BANK0");

    if TOUCH_INT_PIN.is_none() {
        critical_section::with(|cs| *TOUCH_INT_PIN = GLOBAL_TOUCH_INT_PIN.borrow(cs).take());
    }

    let mut i2c = critical_section::with(|cs| GLOBAL_I2C.borrow(cs).take());

    if let Some(i2c) = &mut i2c {
        let rc = RefCell::new(i2c);
        let mut i2c = RefCellDevice::new(&rc);


        if *FIRST_EVENT == true {
            *FIRST_EVENT = false;
        } else if let Some(int) = TOUCH_INT_PIN {
            if int.interrupt_status(EdgeLow) {
                let mut buf = [0u8; 9];
                i2c.write_read(0x38u8, &[0x00], &mut buf).unwrap();
                let x = (((buf[3] & 0x0f) as i32) << 8) | buf[4] as i32;
                let y = (((buf[5] & 0x0f) as i32) << 8) | buf[6] as i32;
                debug!("touch event at ({}, {})", x, y);
                let pos = Point::new(x, y);

                let state = buf[3] >> 6;

                if state == 1 && y > 400 {
                    DRAW_TARGET.clear(BinaryColor::Off).unwrap();
                    DRAW_TARGET.refresh();
                } else if state == 1 && y < 20 {
                    hal::rom_data::reset_to_usb_boot(0, 0);
                } else {
                    if state == 1 || state == 2 {
                        if let Some(prev) = *PREV_POS {
                            Line::new(prev, pos)
                                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 5))
                                .draw(DRAW_TARGET)
                                .unwrap();
                            DRAW_TARGET.refresh_fast();
                        }
                    }

                    if state == 0 || state == 2 {
                        *PREV_POS = Some(Point::new(x, y));
                    }
                }
                int.clear_interrupt(EdgeLow);
            }
        } else {
            error!("int pin is None");
        }
    } else {
        error!("i2c is None");
    }

    critical_section::with(|cs| GLOBAL_I2C.borrow(cs).replace(i2c));
}