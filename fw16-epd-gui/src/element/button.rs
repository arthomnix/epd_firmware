use core::fmt::Binary;
use embedded_graphics::prelude::*;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle, RoundedRectangle};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyle, TextStyleBuilder};
use fw16_epd_program_interface::{TouchEvent, TouchEventType};
use crate::draw_target::EpdDrawTarget;
use crate::element::GuiElement;

const CENTRE_STYLE: TextStyle = TextStyleBuilder::new()
    .alignment(Alignment::Center)
    .baseline(Baseline::Middle)
    .build();

#[derive(Copy, Clone, Debug, Eq, PartialEq, defmt::Format)]
enum ClickState {
    /// The button has not been clicked
    None,
    /// The button has been pressed, but not released
    Pressed,
    /// The button has been fully clicked and released
    Clicked,
}

#[derive(Debug, defmt::Format)]
pub struct Button<'a> {
    rect: RoundedRectangle,
    label: &'a str,
    rect_style: PrimitiveStyle<BinaryColor>,
    char_style: MonoTextStyle<'a, BinaryColor>,
    click_state: ClickState,
}

impl<'a> Button<'a> {
    pub fn new(rect: RoundedRectangle, label: &'a str, rect_style: PrimitiveStyle<BinaryColor>, char_style: MonoTextStyle<'a, BinaryColor>) -> Self {
        Self {
            rect,
            label,
            rect_style,
            char_style,
            click_state: ClickState::None,
        }
    }

    pub fn clicked(&mut self, clear: bool) -> bool {
        if self.click_state == ClickState::Clicked {
            if clear {
                self.click_state = ClickState::None;
            }

            true
        } else {
            false
        }
    }
}

impl<'a> GuiElement for Button<'a> {
    fn draw_element(&self, target: &mut EpdDrawTarget) {
        self.rect
            .into_styled(self.rect_style)
            .draw(target)
            .unwrap();

        Text::with_text_style(
            self.label,
            self.rect.bounding_box().center(),
            self.char_style,
            CENTRE_STYLE
        )
            .draw(target)
            .unwrap();
    }

    fn handle_touch(&mut self, ev: TouchEvent) {
        if self.rect.contains(ev.eg_point()) {
            match (self.click_state, ev.ev_type) {
                (ClickState::None, TouchEventType::Down) => self.click_state = ClickState::Pressed,
                (ClickState::Pressed, TouchEventType::Up) => self.click_state = ClickState::Clicked,
                _ => {},
            }
        } else if self.click_state == ClickState::Pressed {
            // user dragged their finger out of the button bounding box
            self.click_state = ClickState::None;
        }
    }
}