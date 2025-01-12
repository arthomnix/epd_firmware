pub mod button;

use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, PrimitiveStyleBuilder};
use fw16_epd_program_interface::TouchEvent;
use crate::draw_target::EpdDrawTarget;

pub const DEFAULT_PRIMITIVE_STYLE: PrimitiveStyle<BinaryColor> = PrimitiveStyleBuilder::new()
    .stroke_width(2)
    .stroke_color(BinaryColor::On)
    .fill_color(BinaryColor::Off)
    .build();
pub const DEFAULT_TEXT_STYLE: MonoTextStyle<BinaryColor> = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

pub trait GuiElement {
    fn draw_element(&self, target: &mut EpdDrawTarget);

    fn handle_touch(&mut self, ev: TouchEvent);
}

impl<T: Drawable<Color = BinaryColor>> GuiElement for T {
    fn draw_element(&self, target: &mut EpdDrawTarget) {
        self.draw(target).unwrap();
    }

    fn handle_touch(&mut self, _ev: TouchEvent) {
        // no-op
    }
}