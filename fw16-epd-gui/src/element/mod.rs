pub mod button;
pub mod slider;

use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle};
use fw16_epd_program_interface::Event;
use tp370pgh01::{DIM_X, DIM_Y};
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

    // By default, assume element fills the display
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(Point::zero(), Size::new(DIM_X as u32, DIM_Y as u32))
    }
}