use embedded_graphics::prelude::*;
use embedded_graphics::geometry::Point;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, Rectangle};
use eepy_sys::{Event, TouchEventType};
use crate::draw_target::EpdDrawTarget;
use crate::element::{Gui, DEFAULT_PRIMITIVE_STYLE};

pub struct Slider {
    pub start_point: Point,
    pub length: i32,
    pub min: i32,
    pub max: i32,
    pub value: i32,

    pub line_style: PrimitiveStyle<BinaryColor>,
    pub marker_style: PrimitiveStyle<BinaryColor>,
    pub marker_radius: i32,

    sliding: bool,
}

impl Slider {
    pub fn new(start_point: Point, length: i32, min: i32, max: i32, starting_value: i32, line_style: PrimitiveStyle<BinaryColor>, marker_style: PrimitiveStyle<BinaryColor>, marker_radius: i32) -> Self {
        Self {
            start_point,
            length,
            min,
            max,
            value: starting_value,

            line_style,
            marker_style,
            marker_radius,

            sliding: false,
        }
    }

    pub fn with_default_style(start_point: Point, length: i32, min: i32, max: i32, starting_value: i32) -> Self {
        Self {
            start_point,
            length,
            min,
            max,
            value: starting_value,

            line_style: DEFAULT_PRIMITIVE_STYLE,
            marker_style: DEFAULT_PRIMITIVE_STYLE,
            marker_radius: 9,

            sliding: false,
        }
    }
}

impl Gui for Slider {
    type Output = bool;

    fn draw_init(&self, target: &mut EpdDrawTarget) {
        target.fill_solid(&self.bounding_box(), BinaryColor::Off).unwrap();

        Line::new(self.start_point, self.start_point + Point::new(self.length, 0))
            .into_styled(self.line_style)
            .draw(target)
            .unwrap();
        
        let x = self.start_point.x + (self.length * (self.value - self.min)) / (self.max - self.min);
        let marker_point = Point::new(x, self.start_point.y);
        Circle::with_center(marker_point, self.marker_radius as u32 * 2)
            .into_styled(self.marker_style)
            .draw(target)
            .unwrap();
    }

    fn tick(&mut self, target: &mut EpdDrawTarget, ev: Event) -> bool {
        if let Event::Touch(ev) = ev {
            let p = ev.eg_point();

            if self.bounding_box().contains(p) && ev.ev_type == TouchEventType::Down {
                self.sliding = true;
            } else if ev.ev_type == TouchEventType::Up {
                self.sliding = false;
            }

            if self.sliding {
                if p.x >= self.start_point.x && p.x <= (self.start_point.x + self.length) {
                    let xmin = self.start_point.x;
                    let xmax = xmin + self.length;

                    self.value = self.min + ((self.max - self.min) * (ev.eg_point().x - xmin)) / (xmax - xmin);
                    self.draw_init(target);
                } else if p.x < self.start_point.x {
                    self.value = self.min;
                    self.draw_init(target);
                } else {
                    self.value = self.max;
                    self.draw_init(target);
                }

                return true;
            }
        }

        false
    }

    fn bounding_box(&self) -> Rectangle {
        let real_radius = self.marker_radius + self.marker_style.stroke_width as i32;
        let top_left = self.start_point + Point::new(-real_radius, -real_radius);
        Rectangle::new(top_left, Size::new((self.length + 2 * real_radius) as u32, 2 * real_radius as u32))
    }

}