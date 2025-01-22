#![no_std]
#![no_main]

extern crate panic_halt;

use core::fmt::Write;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::Drawable;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::Point;
use embedded_graphics::text::Text;
use heapless::String;
use fw16_epd_gui::draw_target::EpdDrawTarget;
use fw16_epd_gui::element::button::Button;
use fw16_epd_gui::element::{Gui, DEFAULT_TEXT_STYLE};
use fw16_epd_program_interface::{ProgramFunctionTable, RefreshBlockMode, SafeOption};
use fw16_epd_program_interface::header::ProgramSlotHeader;

extern "C" {
    static _end: *const u8;
}

#[link_section = ".header"]
#[used]
static HEADER: ProgramSlotHeader = ProgramSlotHeader::partial(
    "Example TESTING FOO BAR BAZ",
    env!("CARGO_PKG_VERSION"),
    entry,
);

#[used]
static mut FOO: u32 = 10;

#[no_mangle]
pub extern "C" fn entry(pft: &ProgramFunctionTable) {

    unsafe { (pft.set_touch_enabled)(true) };

    let mut draw_target = EpdDrawTarget::new(pft.write_image, pft.refresh);

    let mut button = Button::with_default_style_auto_sized(Point::new(10, 40), "Click me", true);
    button.draw_init(&mut draw_target);
    draw_target.refresh(false, RefreshBlockMode::BlockAcknowledge);

    let mut counter = 0;

    loop {
        while let SafeOption::Some(ev) = (pft.next_event)() {
            let mut needs_refresh = false;

            let response = button.tick(&mut draw_target, ev);
            if response.clicked {
                draw_target.clear(BinaryColor::Off).unwrap();
                button.draw_init(&mut draw_target);

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
    }
}