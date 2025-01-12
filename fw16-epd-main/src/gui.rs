use core::fmt::Write;
use defmt::debug;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle, RoundedRectangle};
use embedded_graphics::text::Text;
use heapless::String;
use fw16_epd_gui::draw_target::EpdDrawTarget;
use fw16_epd_gui::element::button::Button;
use fw16_epd_gui::element::GuiElement;
use crate::{next_touch_event, set_touch_enabled};

pub(crate) fn gui_main(mut draw_target: EpdDrawTarget) -> ! {
    debug!("gui_main");

    draw_target.refresh(false, true);

    unsafe { set_touch_enabled(true) };

    let mut button = Button::new(
        RoundedRectangle::with_equal_corners(Rectangle::new(Point::new(10, 40), Size::new(100, 20)), Size::new(3, 3)),
        "Click me",
        PrimitiveStyle::with_stroke(BinaryColor::On, 2),
        MonoTextStyle::new(&FONT_10X20, BinaryColor::On)
    );
    button.draw_element(&mut draw_target);
    draw_target.refresh(true, false);

    let mut counter = 0;

    loop {
        while let Some(ev) = next_touch_event().into() {
            button.handle_touch(ev);
        }

        if button.clicked(true) {
            counter += 1;

            let mut s = String::<10>::new();
            write!(s, "{counter}").unwrap();

            draw_target.clear(BinaryColor::Off).unwrap();
            button.draw_element(&mut draw_target);

            Text::new(&s, Point::new(10, 80), MonoTextStyle::new(&FONT_10X20, BinaryColor::On))
                .draw_element(&mut draw_target);
            draw_target.refresh(true, false);
        }
    }
}