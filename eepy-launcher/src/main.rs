#![no_std]
#![no_main]

mod serial;
mod ui;

extern crate panic_halt;

use core::arch::asm;
use core::sync::atomic::Ordering;
use usb_device::bus::UsbBusAllocator;
use eepy_gui::draw_target::EpdDrawTarget;
use eepy_gui::element::Gui;
use eepy_sys::input::{has_event, next_event, set_touch_enabled};
use eepy_sys::misc::get_serial;
use eepy_sys::usb::{self, UsbBus};
use usb_device::prelude::*;
use usbd_serial::SerialPort;
use eepy_sys::eepy_app;
use crate::serial::{HOST_APP, NEEDS_REFRESH, NEEDS_REFRESH_PROGRAMS};
use crate::ui::MainGui;

static mut USB: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;

#[eepy_app(name = "Launcher")]
fn main() {
    #[allow(static_mut_refs)]
    unsafe {
        let bus = UsbBusAllocator::new(UsbBus::init());
        USB = Some(bus);
        let bus_ref = USB.as_ref().unwrap();

        let serial = SerialPort::new(bus_ref);
        USB_SERIAL = Some(serial);

        let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x2e8a, 0x000a))
            .strings(&[StringDescriptors::default()
                .manufacturer("arthomnix")
                .product("Touchscreen EPD for FW16 [eepyOS Launcher]")
                .serial_number(get_serial())
            ])
            .unwrap()
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();
        USB_DEVICE = Some(usb_dev);
    }

    usb::set_handler(serial::usb_handler);

    let mut draw_target = EpdDrawTarget::default();
    set_touch_enabled(true);
    let mut gui = MainGui::new();
    gui.draw_init(&mut draw_target);
    draw_target.refresh(false);

    loop {
        if !HOST_APP.load(Ordering::Relaxed) {
            while let Some(ev) = next_event() {
                gui.tick(&mut draw_target, ev);
            }

            if NEEDS_REFRESH_PROGRAMS.swap(false, Ordering::Relaxed) {
                if let MainGui::MainPage(page) = &mut gui {
                    page.refresh_buttons();
                    page.draw_init(&mut draw_target);
                    draw_target.refresh(false);
                }
            } else if NEEDS_REFRESH.swap(false, Ordering::Relaxed) {
                gui.draw_init(&mut draw_target);
                draw_target.refresh(false);
            } else if !has_event() {
                // has_event() is a syscall. The SVCall exception is a WFE wakeup event, so we need two
                // WFEs so we don't immediately wake up.
                unsafe { asm!("wfe", "wfe") };
            }
        }
    }
}