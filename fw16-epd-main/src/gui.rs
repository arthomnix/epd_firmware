use core::fmt::Write;
use defmt::debug;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use heapless::String;
use fw16_epd_program_interface::eg::EpdDrawTarget;
use fw16_epd_program_interface::TouchEvent;
use crate::{next_touch_event, set_touch_enabled};

pub(crate) fn gui_main(mut draw_target: EpdDrawTarget) -> ! {
    draw_target.refresh(false, true);

    unsafe { set_touch_enabled(true) };

    loop {
        let mut touch_event: Option<TouchEvent> = None;
        while let Some(ev) = next_touch_event().into() {
            debug!("{}", ev);
            touch_event = Some(ev);
        }

        if let Some(ev) = touch_event {
            draw_target.clear(BinaryColor::Off).unwrap();

            let mut s = String::<32>::new();
            write!(s, "{ev}").unwrap();

            Text::new(s.as_str(), Point::new(10, 40), MonoTextStyle::new(&FONT_10X20, BinaryColor::On))
                .draw(&mut draw_target)
                .unwrap();
            debug!("triggering refresh");
            draw_target.refresh(true, false);
        }
    }
}