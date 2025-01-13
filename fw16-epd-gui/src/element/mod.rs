pub mod button;

use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, PrimitiveStyleBuilder};
use fw16_epd_program_interface::{Event, TouchEvent};
use crate::draw_target::EpdDrawTarget;

pub const DEFAULT_PRIMITIVE_STYLE: PrimitiveStyle<BinaryColor> = PrimitiveStyleBuilder::new()
    .stroke_width(2)
    .stroke_color(BinaryColor::On)
    .fill_color(BinaryColor::Off)
    .build();
pub const DEFAULT_TEXT_STYLE: MonoTextStyle<BinaryColor> = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

pub trait Gui {
    type Output;

    fn draw_init(&self, target: &mut EpdDrawTarget);

    fn tick(&mut self, target: &mut EpdDrawTarget, ev: Event) -> Self::Output;
}

impl<T: Drawable<Color = BinaryColor>> Gui for T {
    type Output = ();

    fn draw_init(&self, target: &mut EpdDrawTarget) {
        self.draw(target).unwrap();
    }

    fn tick(&mut self, _target: &mut EpdDrawTarget, _ev: Event) {
        // no-op
    }
}