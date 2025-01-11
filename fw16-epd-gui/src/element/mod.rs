pub mod button;

use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use fw16_epd_program_interface::TouchEvent;
use crate::draw_target::EpdDrawTarget;

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