#![no_std]
#![no_main]

mod serial;

extern crate panic_halt;

use core::arch::asm;
use core::mem::offset_of;
use core::sync::atomic::Ordering;
use embedded_graphics::geometry::AnchorPoint;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::Text;
use usb_device::bus::UsbBusAllocator;
use eepy_gui::draw_target::EpdDrawTarget;
use eepy_gui::element::button::Button;
use eepy_gui::element::{Gui, DEFAULT_TEXT_STYLE};
use eepy_gui::element::slider::Slider;
use eepy_sys::exec::exec;
use eepy_sys::input::{has_event, next_event, set_touch_enabled};
use eepy_sys::input_common::{Event, TouchEventType};
use eepy_sys::header::{slot, slot_ptr, ProgramSlotHeader, Programs};
use eepy_sys::misc::get_serial;
use eepy_sys::{flash, usb};
use eepy_sys::usb::UsbBus;
use usb_device::prelude::*;
use usbd_serial::SerialPort;
use crate::serial::{HOST_APP, NEEDS_REFRESH, NEEDS_REFRESH_PROGRAMS};

#[link_section = ".header"]
#[used]
static HEADER: ProgramSlotHeader = ProgramSlotHeader::partial(
    "Launcher",
    env!("CARGO_PKG_VERSION"),
    main,
);

struct MainPage {
    scratchpad_button: Button<'static>,
    app_buttons: [Option<(Button<'static>, u8)>; 32],
}

impl MainPage {
    fn refresh_buttons(&mut self) {
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
                    self.app_buttons[bi] = Some((button, slot_num))
                }
            }
        }
    }

    fn new() -> Self {
        let mut res = Self {
            scratchpad_button: Button::with_default_style_auto_sized(Point::new(10, 10), "Scratchpad", true),
            app_buttons: [const { None }; 32],
        };
        res.refresh_buttons();
        res
    }
}

impl Gui for MainPage {
    type Output = Option<PageType>;

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
            return Some(PageType::ScratchpadPage);
        } else if s.needs_refresh {
            draw_target.refresh(true);
        }

        for b in &mut self.app_buttons {
            if let Some((button, s)) = b {
                let response = button.tick(draw_target, ev);

                if response.long_clicked {
                    return Some(PageType::AppInfoPage(*s));
                } else if response.clicked {
                    unsafe { cleanup_usb() };
                    exec(*s);
                }

                needs_refresh |= response.needs_refresh;
            }
        }

        if needs_refresh {
            draw_target.refresh(true);
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
    type Output = Option<PageType>;

    fn draw_init(&self, draw_target: &mut EpdDrawTarget) {
        self.exit_button.draw_init(draw_target);
        self.clear_button.draw_init(draw_target);
        self.toggle_button.draw_init(draw_target);
        self.slider.draw_init(draw_target);
    }

    fn tick(&mut self, draw_target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        let mut refresh = false;
        let mut maybe_refresh = false;
        let mut handle_drawing: bool = true;

        let e = self.exit_button.tick(draw_target, ev);
        if e.clicked {
            return Some(PageType::MainPage);
        }
        refresh |= e.needs_refresh;

        let c = self.clear_button.tick(draw_target, ev);
        if c.clicked {
            draw_target.clear(BinaryColor::Off).unwrap();
            self.draw_init(draw_target);
            draw_target.refresh(false);
            return None;
        }
        refresh |= c.needs_refresh;

        let t = self.toggle_button.tick(draw_target, ev);
        if t.clicked {
            self.eraser = !self.eraser;
            self.toggle_button.label = if self.eraser { "Pen" } else { "Eraser" };
        }
        maybe_refresh |= t.clicked;
        refresh |= t.needs_refresh;

        if self.slider.tick(draw_target, ev) {
            maybe_refresh = true;
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

                    maybe_refresh = true;
                }
            }

            if matches!(ev.ev_type, TouchEventType::Down | TouchEventType::Move) {
                self.prev_pos = Some(ev.eg_point());
            }
        }

        if refresh {
            draw_target.refresh(true);
        } else if maybe_refresh {
            draw_target.maybe_refresh(true);
        }

        None
    }
}

struct AppInfoPage {
    slot: u8,
    back_button: Button<'static>,
    delete_button: Button<'static>,
}

impl AppInfoPage {
    fn new(slot: u8) -> Self {
        Self {
            slot,
            back_button: Button::with_default_style_auto_sized(Point::new(10, 386), "Back", true),
            delete_button: Button::with_default_style_auto_sized(Point::new(10, 60), "Delete program", true),
        }
    }
}

impl Gui for AppInfoPage {
    type Output = Option<PageType>;

    fn draw_init(&self, target: &mut EpdDrawTarget) {
        let header = unsafe { slot(self.slot) };

        Text::new("Program: ", Point::new(10, 20), DEFAULT_TEXT_STYLE)
            .draw(target)
            .unwrap();

        if let Ok(name) = unsafe { (*header).name() } {
            Text::new(name, Point::new(120, 20), DEFAULT_TEXT_STYLE)
                .draw(target)
                .unwrap();
        } else {
            Text::new("<invalid>", Point::new(120, 20), DEFAULT_TEXT_STYLE)
                .draw(target)
                .unwrap();
        }


        Text::new("Version: ", Point::new(10, 40), DEFAULT_TEXT_STYLE)
            .draw(target)
            .unwrap();

        if let Ok(vers) = unsafe { (*header).version() } {
            Text::new(vers, Point::new(120, 40), DEFAULT_TEXT_STYLE)
                .draw(target)
                .unwrap();
        } else {
            Text::new("<invalid>", Point::new(120, 40), DEFAULT_TEXT_STYLE)
                .draw(target)
                .unwrap();
        }

        self.back_button.draw_init(target);
        self.delete_button.draw_init(target);
    }

    fn tick(&mut self, target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        let b = self.back_button.tick(target, ev);
        if b.clicked {
            return Some(PageType::MainPage);
        }

        let d = self.delete_button.tick(target, ev);
        if d.clicked {
            let ptr = unsafe { slot_ptr(self.slot) };
            let mut buf = [0u8; 256];
            unsafe { buf.copy_from_slice(core::slice::from_raw_parts(ptr, 256)) };
            let offset = offset_of!(ProgramSlotHeader, len);
            buf[offset..offset + 4].copy_from_slice(&[0; 4]);

            unsafe { flash::program(self.slot as u32 * 512 * 1024, &buf) };
            return Some(PageType::MainPage);
        }

        None
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum PageType {
    MainPage,
    ScratchpadPage,
    AppInfoPage(u8),
}

enum MainGui {
    MainPage(MainPage),
    ScratchpadPage(ScratchpadPage),
    AppInfoPage(AppInfoPage),
}

impl MainGui {
    fn new() -> Self {
        Self::MainPage(MainPage::new())
    }

    fn get_current_page(&self) -> &dyn Gui<Output = Option<PageType>> {
        match self {
            MainGui::MainPage(page) => page,
            MainGui::ScratchpadPage(page) => page,
            MainGui::AppInfoPage(page) => page,
        }
    }

    fn get_current_page_mut(&mut self) -> &mut dyn Gui<Output = Option<PageType>> {
        match self {
            MainGui::MainPage(page) => page,
            MainGui::ScratchpadPage(page) => page,
            MainGui::AppInfoPage(page) => page,
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
            *self = match page {
                PageType::MainPage => MainGui::MainPage(MainPage::new()),
                PageType::ScratchpadPage => MainGui::ScratchpadPage(ScratchpadPage::new()),
                PageType::AppInfoPage(slot) => MainGui::AppInfoPage(AppInfoPage::new(slot)),
            };

            draw_target.clear(BinaryColor::Off).unwrap();
            self.draw_init(draw_target);
            draw_target.refresh(false);
        }
    }
}

static mut USB: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;

unsafe fn cleanup_usb() {
    #[allow(static_mut_refs)]
    unsafe {
        let _ = USB.take();
        let _ = USB_DEVICE.take();
        let _ = USB_SERIAL.take();
    }
}

extern "C" fn main() {
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