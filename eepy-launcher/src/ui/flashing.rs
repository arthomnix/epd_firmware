use core::fmt::Write;
use embedded_graphics::Drawable;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use eepy_gui::draw_target::EpdDrawTarget;
use eepy_gui::element::DEFAULT_TEXT_STYLE;

pub(crate) fn draw_flashing_ui(page: usize, num_pages: Option<usize>) {
    let mut target = EpdDrawTarget::default();

    Text::new("Receiving program...", Point::new(10, 20), DEFAULT_TEXT_STYLE)
        .draw(&mut target)
        .unwrap();

    if let Some(num_pages) = num_pages {
        let mut s = heapless::String::<16>::new();
        write!(s, "({page} / {num_pages})").unwrap();

        Text::new(&s, Point::new(10, 40), DEFAULT_TEXT_STYLE)
            .draw(&mut target)
            .unwrap();
    }
    target.maybe_refresh(true);
}