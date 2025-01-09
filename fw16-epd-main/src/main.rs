#![no_main]
#![no_std]

mod programs;

#[allow(unused_imports)]
use panic_probe as _;
#[allow(unused_imports)]
use defmt_rtt as _;

use core::cell::{RefCell, RefMut};
use critical_section::Mutex;
use defmt::{debug, error, info, trace};
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle, PrimitiveStyleBuilder};
use embedded_graphics::text::Text;
use embedded_hal::digital::PinState;
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
use fw16_epd_bsp::hal::fugit::RateExtU32;
use fw16_epd_bsp::hal::gpio::Interrupt::EdgeLow;
use fw16_epd_bsp::hal::multicore::{Multicore, Stack};
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

static IMAGE_BUFFER: Mutex<RefCell<[u8; tp370pgh01::IMAGE_BYTES]>> = Mutex::new(RefCell::new([0; tp370pgh01::IMAGE_BYTES]));
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
    let core = pac::CorePeripherals::take().unwrap();
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

    let mut sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );


    let _power = pins.epd_pwr_sw.into_push_pull_output_in_state(PinState::Low);

    let cs: EpdCs = pins.spi3_epd_cs.reconfigure();
    let dc: EpdDc = pins.epd_dc.reconfigure();
    let busy: EpdBusy = pins.epd_busy.reconfigure();
    let rst: EpdReset = pins.epd_rst.reconfigure();
    let sda: EpdSdaWrite = pins.spi3_epd_sda.reconfigure();
    let sck: EpdSck = pins.spi3_epd_sck.reconfigure();

    let i2c_sda: I2CSda = pins.i2c_sda.reconfigure();
    let i2c_scl: I2CScl = pins.i2c_scl.reconfigure();
    let int: EpdTouchInt = pins.epd_touch_int.reconfigure();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

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

    let bus_ref = unsafe { GLOBAL_USB_BUS.as_ref().unwrap() };

    let serial = SerialPort::new(bus_ref);
    let usb_device = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x2e8a, 0x000a))
        .strings(&[StringDescriptors::default()
            .manufacturer("arthomnix")
            .product("Touchscreen EPD Input Module for Framework 16")
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
fn USBCTRL_IRQ() {
    static mut INDEX: usize = 0;

    trace!("USBCTRL_IRQ");

    let usb_dev = unsafe { GLOBAL_USB_DEVICE.as_mut().unwrap() };
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
    static mut I2C: Option<I2C<I2C0, (I2CSda, I2CScl)>> = None;
    static mut PREV_POS: Point = Point::new(0, 0);
    static mut DRAW_TARGET: EpdDrawTarget = EpdDrawTarget::new(ProgramFunctionTable {
        write_image,
        refresh,
        refresh_fast,
    });

    trace!("IO_IRQ_BANK0");

    if TOUCH_INT_PIN.is_none() {
        critical_section::with(|cs| *TOUCH_INT_PIN = GLOBAL_TOUCH_INT_PIN.borrow(cs).take());
    }

    if I2C.is_none() {
        critical_section::with(|cs| *I2C = GLOBAL_I2C.borrow(cs).take());
    }

    if let Some(i2c) = I2C {
        let rc = RefCell::new(i2c);
        let mut i2c = RefCellDevice::new(&rc);
        let mut mcp9808 = MCP9808::new(RefCellDevice::new(&rc));

        let temp = match mcp9808.read_temperature().unwrap().get_celsius(ResolutionVal::Deg_0_0625C) {
            ..=0.0 => 0u8,
            60.0.. => 60u8,
            t => t as u8,
        };
        TEMP.store(temp, Ordering::Relaxed);

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
                        Line::new(PREV_POS.clone(), pos)
                            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 5))
                            .draw(DRAW_TARGET)
                            .unwrap();
                        DRAW_TARGET.refresh_fast();
                    }

                    if state == 0 || state == 2 {
                        *PREV_POS = Point::new(x, y);
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
}