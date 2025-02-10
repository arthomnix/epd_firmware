use embedded_graphics::prelude::*;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::primitives::{CornerRadii, PrimitiveStyle, Rectangle, RoundedRectangle};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyle, TextStyleBuilder};
use embedded_graphics::text::renderer::TextRenderer;
use fugit::ExtU32;
use eepy_sys::input_common::{Event, TouchEventType};
use eepy_sys::misc::{now, Instant};
use crate::draw_target::EpdDrawTarget;
use crate::element::{Gui, DEFAULT_PRIMITIVE_STYLE, DEFAULT_TEXT_STYLE};

const CENTRE_STYLE: TextStyle = TextStyleBuilder::new()
    .alignment(Alignment::Center)
    .baseline(Baseline::Middle)
    .build();

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct ButtonOutput {
    pub clicked: bool,
    pub long_clicked: bool,
    pub needs_refresh: bool,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct Button<'a> {
    pub rect: RoundedRectangle,
    pub label: &'a str,
    pub rect_style: PrimitiveStyle<BinaryColor>,
    pub char_style: MonoTextStyle<'a, BinaryColor>,
    pub touch_feedback: bool,
    pub touch_feedback_immediate_release: bool,

    click_begin_time: Option<Instant>,
    inverted: bool,
    should_uninvert: bool,
}

impl<'a> Button<'a> {
    pub fn new(rect: RoundedRectangle, label: &'a str, rect_style: PrimitiveStyle<BinaryColor>, char_style: MonoTextStyle<'a, BinaryColor>, touch_feedback: bool, touch_feedback_immediate_release: bool) -> Self {
        Self {
            rect,
            label,
            rect_style,
            char_style,
            touch_feedback,
            touch_feedback_immediate_release,
            click_begin_time: None,
            inverted: false,
            should_uninvert: false,
        }
    }

    pub fn auto_sized(top_left: Point, corner_radii: CornerRadii, label: &'a str, rect_style: PrimitiveStyle<BinaryColor>, char_style: MonoTextStyle<'a, BinaryColor>, touch_feedback: bool, touch_feedback_immediate_release: bool) -> Self {
        let size = Size::new((char_style.font.character_size.width + char_style.font.character_spacing) * (label.len() as u32 + 1), char_style.line_height());
        Self {
            rect: RoundedRectangle::new(Rectangle::new(top_left, size), corner_radii),
            label,
            rect_style,
            char_style,
            touch_feedback,
            touch_feedback_immediate_release,
            click_begin_time: None,
            inverted: false,
            should_uninvert: false,
        }
    }

    pub fn with_default_style(rect: Rectangle, label: &'a str, touch_feedback_immediate_release: bool) -> Self {
        Self {
            rect: RoundedRectangle::new(rect, CornerRadii::new(Size::new(3, 3))),
            label,
            rect_style: DEFAULT_PRIMITIVE_STYLE,
            char_style: DEFAULT_TEXT_STYLE,
            touch_feedback: true,
            touch_feedback_immediate_release,
            click_begin_time: None,
            inverted: false,
            should_uninvert: false,
        }
    }

    pub fn with_default_style_auto_sized(top_left: Point, label: &'a str, touch_feedback_immediate_release: bool) -> Self {
        Self::auto_sized(
            top_left,
            CornerRadii::new(Size::new(3, 3)),
            label,
            DEFAULT_PRIMITIVE_STYLE,
            DEFAULT_TEXT_STYLE,
            true,
            touch_feedback_immediate_release,
        )
    }

    fn invert(&mut self) {
        self.rect_style.fill_color = self.rect_style.fill_color.map(|c| c.invert());
        self.char_style.text_color = self.char_style.text_color.map(|c| c.invert());
        self.inverted = !self.inverted;
    }
}

impl<'a> Gui for Button<'a> {
    type Output = ButtonOutput;

    fn draw_init(&self, target: &mut EpdDrawTarget) {
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

    fn tick(&mut self, target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        let mut ret = ButtonOutput::default();

        if let Event::Touch(ev) = ev {
            if self.rect.contains(ev.eg_point()) {
                match (self.click_begin_time, ev.ev_type) {
                    (None, TouchEventType::Down) => {
                        self.click_begin_time = Some(now());
                        if self.touch_feedback {
                            self.invert();
                            self.draw_init(target);
                            ret.needs_refresh = true;
                        }
                    },
                    (Some(t), TouchEventType::Up) => {
                        self.click_begin_time = None;
                        if self.inverted {
                            if self.touch_feedback_immediate_release {
                                self.invert();
                                self.draw_init(target);
                                ret.needs_refresh = true;
                            } else {
                                self.should_uninvert = true;
                            }
                        }

                        ret.clicked = true;
                        if now().checked_duration_since(t).unwrap() > 500.millis::<1, 1_000_000>() {
                            ret.long_clicked = true;
                        }
                    },
                    _ => {},
                }
            } else {
                self.click_begin_time = None;
                if self.inverted {
                    if self.touch_feedback_immediate_release {
                        self.invert();
                        self.draw_init(target);
                        ret.needs_refresh = true;
                    } else {
                        self.should_uninvert = true;
                    }
                }
            }
        }

        if ev == Event::RefreshFinished && self.should_uninvert {
            self.should_uninvert = false;
            self.invert();
            self.draw_init(target);
            ret.needs_refresh = true;
        }

        ret
    }

    fn bounding_box(&self) -> Rectangle {
        self.rect.into_styled(self.rect_style).bounding_box()
    }
}