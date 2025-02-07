#![no_std]
#![no_main]

extern crate panic_halt;

use core::arch::asm;
use core::fmt::Write;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::Drawable;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::Point;
use embedded_graphics::text::Text;
use heapless::String;
use eepy_gui::draw_target::EpdDrawTarget;
use eepy_gui::element::button::Button;
use eepy_gui::element::{Gui, DEFAULT_TEXT_STYLE};
use eepy_sys::header::ProgramSlotHeader;
use eepy_sys::image::RefreshBlockMode;
use eepy_sys::input::{has_event, next_event, set_touch_enabled};

#[link_section = ".header"]
#[used]
static HEADER: ProgramSlotHeader = ProgramSlotHeader::partial(
    "ExampleApp",
    env!("CARGO_PKG_VERSION"),
    entry,
);

#[no_mangle]
pub extern "C" fn entry() {
    set_touch_enabled(true);

    let mut draw_target = EpdDrawTarget::default();

    let text = Text::new("Example App", Point::new(10, 20), DEFAULT_TEXT_STYLE);
    let mut button = Button::with_default_style_auto_sized(Point::new(10, 40), "Click me", true);
    let mut exit_button = Button::with_default_style_auto_sized(Point::new(10, 386), "Exit", false);

    text.draw(&mut draw_target).unwrap();
    button.draw_init(&mut draw_target);
    exit_button.draw_init(&mut draw_target);
    draw_target.refresh(false, RefreshBlockMode::BlockAcknowledge);

    let mut counter = 0;

    loop {
        while let Some(ev) = next_event() {
            if exit_button.tick(&mut draw_target, ev).clicked {
                return;
            }

            let mut needs_refresh = false;

            let response = button.tick(&mut draw_target, ev);
            if response.clicked {
                draw_target.clear(BinaryColor::Off).unwrap();
                text.draw(&mut draw_target).unwrap();
                button.draw_init(&mut draw_target);
                exit_button.draw_init(&mut draw_target);

                counter += 1;
                let mut s = String::<16>::new();
                write!(s, "{counter}").unwrap();
                Text::new(&s, Point::new(10, 80), DEFAULT_TEXT_STYLE)
                    .draw(&mut draw_target)
                    .unwrap();
                needs_refresh = true;
            }
            needs_refresh |= response.needs_refresh;

            if needs_refresh {
                draw_target.refresh(true, RefreshBlockMode::NonBlocking);
            }
        }

        if !has_event() {
            unsafe { asm!("wfe", "wfe") };
        }
    }
}