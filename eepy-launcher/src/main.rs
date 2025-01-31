#![no_std]
#![no_main]

extern crate panic_halt;

use core::arch::asm;
use embedded_graphics::geometry::AnchorPoint;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, Rectangle};
use usb_device::bus::UsbBusAllocator;
use eepy_gui::draw_target::EpdDrawTarget;
use eepy_gui::element::button::Button;
use eepy_gui::element::Gui;
use eepy_gui::element::slider::Slider;
use eepy_sys::exec::exec;
use eepy_sys::image::RefreshBlockMode;
use eepy_sys::input::{has_event, next_event, set_touch_enabled, Event, TouchEventType};
use eepy_sys::header::{ProgramSlotHeader, Programs};
use eepy_sys::misc::{get_serial, info, trace};
use eepy_sys::usb;
use eepy_sys::usb::UsbBus;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

#[link_section = ".header"]
#[used]
static HEADER: ProgramSlotHeader = ProgramSlotHeader::partial(
    "Launcher",
    env!("CARGO_PKG_VERSION"),
    entry,
);

enum Page {
    MainPage,
    ScratchpadPage,
}

struct MainPage {
    scratchpad_button: Button<'static>,
    app_buttons: [Option<(Button<'static>, u8)>; 32],
}

impl MainPage {
    fn new() -> Self {
        let mut buttons = [const { None }; 32];
        let mut programs = Programs::new();

        for y in 0..16 {
            for x in 0..2 {
                if let Some(prog) = programs.next() {
                    let bi = y * 2 + x;
                    let x_coord = if x == 0 { 10 } else { 125 };
                    let y_coord = 35 + 23 * y as i32;
                    let button = Button::with_default_style(
                        Rectangle::new(Point::new(x_coord, y_coord), Size::new(105, 20)),
                        unsafe { (*prog).name().unwrap() },
                        false,
                    );
                    let slot_num = unsafe { (&*prog).slot() };
                    buttons[bi] = Some((button, slot_num))
                }
            }
        }

        Self {
            scratchpad_button: Button::with_default_style_auto_sized(Point::new(10, 10), "Scratchpad", true),
            app_buttons: buttons,
        }
    }
}

impl Gui for MainPage {
    type Output = Option<Page>;

    fn draw_init(&self, draw_target: &mut EpdDrawTarget) {
        self.scratchpad_button.draw_init(draw_target);
        for b in &self.app_buttons {
            if let Some((button, _)) = b {
                button.draw_init(draw_target);
            }
        }
    }

    fn tick(&mut self, draw_target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        let mut needs_refresh = false;

        let s = self.scratchpad_button.tick(draw_target, ev);
        if s.clicked {
            return Some(Page::ScratchpadPage);
        } else if s.needs_refresh {
            draw_target.refresh(true, RefreshBlockMode::BlockAcknowledge);
        }

        for b in &mut self.app_buttons {
            if let Some((button, s)) = b {
                let response = button.tick(draw_target, ev);

                if response.clicked {
                    cleanup_usb();
                    exec(*s);
                }

                needs_refresh |= response.needs_refresh;
            }
        }

        if needs_refresh {
            draw_target.refresh(true, RefreshBlockMode::NonBlocking);
        }

        None
    }
}

struct ScratchpadPage {
    exit_button: Button<'static>,
    clear_button: Button<'static>,
    toggle_button: Button<'static>,
    slider: Slider,
    eraser: bool,
    prev_pos: Option<Point>,
}

impl ScratchpadPage {
    fn new() -> Self {
        let exit_button = Button::with_default_style_auto_sized(Point::new(10, 416 - 10 - 20), "Exit", true);

        let next_pos = exit_button
            .bounding_box()
            .translate(Point::new(10, 0))
            .anchor_point(AnchorPoint::TopRight);
        let clear_button = Button::with_default_style_auto_sized(next_pos, "Clear", true);

        let next_pos = clear_button
            .bounding_box()
            .translate(Point::new(10, 0))
            .anchor_point(AnchorPoint::TopRight);
        let toggle_button = Button::with_default_style_auto_sized(next_pos, "Eraser", true);

        let slider = Slider::with_default_style(Point::new(20, 20), 100, 1, 10, 2);

        Self {
            exit_button,
            clear_button,
            toggle_button,
            slider,
            eraser: false,
            prev_pos: None,
        }
    }
}

impl Gui for ScratchpadPage {
    type Output = Option<Page>;

    fn draw_init(&self, draw_target: &mut EpdDrawTarget) {
        self.exit_button.draw_init(draw_target);
        self.clear_button.draw_init(draw_target);
        self.toggle_button.draw_init(draw_target);
        self.slider.draw_init(draw_target);
    }

    fn tick(&mut self, draw_target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        let mut refresh: Option<RefreshBlockMode> = None;
        let mut handle_drawing: bool = true;

        let e = self.exit_button.tick(draw_target, ev);
        if e.clicked {
            return Some(Page::MainPage);
        }
        if e.needs_refresh {
            refresh = Some(RefreshBlockMode::BlockAcknowledge);
        }

        let c = self.clear_button.tick(draw_target, ev);
        if c.clicked {
            draw_target.clear(BinaryColor::Off).unwrap();
            self.draw_init(draw_target);
            draw_target.refresh(false, RefreshBlockMode::BlockAcknowledge);
            return None;
        }
        if c.needs_refresh {
            refresh = Some(RefreshBlockMode::BlockAcknowledge);
        }

        let t = self.toggle_button.tick(draw_target, ev);
        if t.clicked {
            self.eraser = !self.eraser;
            self.toggle_button.label = if self.eraser { "Pen" } else { "Eraser" };
            refresh = Some(RefreshBlockMode::NonBlocking);
        }
        if t.needs_refresh {
            refresh = Some(RefreshBlockMode::BlockAcknowledge);
        }

        if self.slider.tick(draw_target, ev) {
            refresh = Some(RefreshBlockMode::NonBlocking);
            handle_drawing = false;
        }

        if let Event::Touch(ev) = ev {
            if handle_drawing && matches!(ev.ev_type, TouchEventType::Move | TouchEventType::Up) {
                if let Some(prev) = self.prev_pos {
                    let style = PrimitiveStyle::with_stroke(BinaryColor::from(!self.eraser), self.slider.value as u32);
                    Line::new(prev, ev.eg_point())
                        .into_styled(style)
                        .draw(draw_target)
                        .unwrap();
                    // Draw a circle at each end of the line
                    let circle_style = PrimitiveStyle::with_fill(BinaryColor::from(!self.eraser));
                    Circle::with_center(prev, self.slider.value as u32 - 1)
                        .into_styled(circle_style)
                        .draw(draw_target)
                        .unwrap();
                    Circle::with_center(ev.eg_point(), self.slider.value as u32 - 1)
                        .into_styled(circle_style)
                        .draw(draw_target)
                        .unwrap();

                    self.draw_init(draw_target);

                    if refresh.is_none() {
                        refresh = Some(RefreshBlockMode::NonBlocking);
                    }
                }
            }

            if matches!(ev.ev_type, TouchEventType::Down | TouchEventType::Move) {
                self.prev_pos = Some(ev.eg_point());
            }
        }

        if let Some(mode) = refresh {
            draw_target.refresh(true, mode);
        }

        None
    }
}

struct MainGui {
    current_page: Page,
    main_page: MainPage,
    scratchpad_page: ScratchpadPage,
}

impl MainGui {
    fn new() -> Self {
        Self {
            current_page: Page::MainPage,
            main_page: MainPage::new(),
            scratchpad_page: ScratchpadPage::new(),
        }
    }

    fn get_current_page(&self) -> &dyn Gui<Output = Option<Page>> {
        match self.current_page {
            Page::MainPage => &self.main_page,
            Page::ScratchpadPage => &self.scratchpad_page,
        }
    }

    fn get_current_page_mut(&mut self) -> &mut dyn Gui<Output = Option<Page>> {
        match self.current_page {
            Page::MainPage => &mut self.main_page,
            Page::ScratchpadPage => &mut self.scratchpad_page,
        }
    }
}

impl Gui for MainGui {
    type Output = ();

    fn draw_init(&self, draw_target: &mut EpdDrawTarget) {
        self.get_current_page().draw_init(draw_target);
    }

    fn tick(&mut self, draw_target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        if let Some(page) = self.get_current_page_mut().tick(draw_target, ev) {
            self.current_page = page;
            draw_target.clear(BinaryColor::Off).unwrap();
            self.draw_init(draw_target);
            draw_target.refresh(false, RefreshBlockMode::BlockAcknowledge);
        }
    }
}

static mut USB: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;

#[allow(static_mut_refs)]
pub extern "C" fn testing_usb_handler() {
    let dev: &mut UsbDevice<UsbBus> = unsafe { USB_DEVICE.as_mut().unwrap() };
    let serial: &mut SerialPort<UsbBus> = unsafe { USB_SERIAL.as_mut().unwrap() };

    trace("Launcher USB handler");

    if dev.poll(&mut [serial]) {
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Err(_) => {},
            Ok(0) => {},
            Ok(_) => {
                let s = core::str::from_utf8(&buf);
                if let Ok(s) = s {
                    info(s);
                }
            }
        }
    }
}

fn cleanup_usb() {
    #[allow(static_mut_refs)]
    unsafe {
        let _ = USB.take();
        let _ = USB_DEVICE.take();
        let _ = USB_SERIAL.take();
    }
}

#[no_mangle]
pub extern "C" fn entry() {
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

    usb::set_handler(testing_usb_handler);

    let mut draw_target = EpdDrawTarget::new();
    set_touch_enabled(true);
    let mut gui = MainGui::new();
    gui.draw_init(&mut draw_target);
    draw_target.refresh(false, RefreshBlockMode::BlockAcknowledge);

    loop {
        while let Some(ev) = next_event() {
            gui.tick(&mut draw_target, ev);
        }

        if !has_event() {
            // has_event() is a syscall. The SVCall exception is a WFE wakeup event, so we need two
            // WFEs so we don't immediately wake up.
            unsafe { asm!("wfe", "wfe") };
        }
    }
}