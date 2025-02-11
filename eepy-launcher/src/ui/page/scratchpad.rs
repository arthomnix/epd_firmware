use embedded_graphics::geometry::AnchorPoint;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle};
use eepy_gui::draw_target::EpdDrawTarget;
use eepy_gui::element::button::Button;
use eepy_gui::element::Gui;
use eepy_gui::element::slider::Slider;
use eepy_sys::input_common::{Event, TouchEventType};
use crate::ui::MainGui;
use crate::ui::page::main::MainPage;

pub(crate) struct ScratchpadPage {
    exit_button: Button<'static>,
    clear_button: Button<'static>,
    toggle_button: Button<'static>,
    slider: Slider,
    eraser: bool,
    prev_pos: Option<Point>,
}

impl ScratchpadPage {
    pub(crate) fn new() -> Self {
        let exit_button = Button::with_default_style_auto_sized(Point::new(10, 416 - 10 - 20), "Exit", true);

        let next_pos = exit_button
            .bounding_box()
            .translate(Point::new(10, 0))
            .anchor_point(AnchorPoint::TopRight);
        let clear_button = Button::with_default_style_auto_sized(next_pos, "Clear", true);

        let next_pos = clear_button
            .bounding_box()
            .translate(Point::new(10, 0))
            .anchor_point(AnchorPoint::TopRight);
        let toggle_button = Button::with_default_style_auto_sized(next_pos, "Eraser", true);

        let slider = Slider::with_default_style(Point::new(20, 20), 100, 1, 10, 2);

        Self {
            exit_button,
            clear_button,
            toggle_button,
            slider,
            eraser: false,
            prev_pos: None,
        }
    }
}

impl Gui for ScratchpadPage {
    type Output = Option<MainGui>;

    fn draw_init(&self, draw_target: &mut EpdDrawTarget) {
        self.exit_button.draw_init(draw_target);
        self.clear_button.draw_init(draw_target);
        self.toggle_button.draw_init(draw_target);
        self.slider.draw_init(draw_target);
    }

    fn tick(&mut self, draw_target: &mut EpdDrawTarget, ev: Event) -> Self::Output {
        let mut refresh = false;
        let mut maybe_refresh = false;
        let mut handle_drawing: bool = true;

        let e = self.exit_button.tick(draw_target, ev);
        if e.clicked {
            return Some(MainGui::MainPage(MainPage::new()));
        }
        refresh |= e.needs_refresh;

        let c = self.clear_button.tick(draw_target, ev);
        if c.clicked {
            draw_target.clear(BinaryColor::Off).unwrap();
            self.draw_init(draw_target);
            draw_target.refresh(false);
            return None;
        }
        refresh |= c.needs_refresh;

        let t = self.toggle_button.tick(draw_target, ev);
        if t.clicked {
            self.eraser = !self.eraser;
            self.toggle_button.label = if self.eraser { "Pen" } else { "Eraser" };
        }
        maybe_refresh |= t.clicked;
        refresh |= t.needs_refresh;

        if self.slider.tick(draw_target, ev) {
            maybe_refresh = true;
            handle_drawing = false;
        }

        if let Event::Touch(ev) = ev {
            if handle_drawing && matches!(ev.ev_type, TouchEventType::Move | TouchEventType::Up) {
                if let Some(prev) = self.prev_pos {
                    let style = PrimitiveStyle::with_stroke(BinaryColor::from(!self.eraser), self.slider.value as u32);
                    Line::new(prev, ev.eg_point())
                        .into_styled(style)
                        .draw(draw_target)
                        .unwrap();
                    // Draw a circle at each end of the line
                    let circle_style = PrimitiveStyle::with_fill(BinaryColor::from(!self.eraser));
                    Circle::with_center(prev, self.slider.value as u32 - 1)
                        .into_styled(circle_style)
                        .draw(draw_target)
                        .unwrap();
                    Circle::with_center(ev.eg_point(), self.slider.value as u32 - 1)
                        .into_styled(circle_style)
                        .draw(draw_target)
                        .unwrap();

                    self.draw_init(draw_target);

                    maybe_refresh = true;
                }
            }

            if matches!(ev.ev_type, TouchEventType::Down | TouchEventType::Move) {
                self.prev_pos = Some(ev.eg_point());
            }
        }

        if refresh {
            draw_target.refresh(true);
        } else if maybe_refresh {
            draw_target.maybe_refresh(true);
        }

        None
    }
}
