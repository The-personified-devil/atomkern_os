extern crate bitvec;
use self::bitvec::prelude::*;

extern crate lazy_static;
extern crate spin;
// extern crate volatile;
use self::lazy_static::lazy_static;
use self::spin::Mutex;
// use self::volatile::Volatile;
use core::arch::asm;
use core::fmt;
use core::intrinsics::breakpoint;

pub mod pixel;

// type Char = [u8; 16];
// TODO: Make sane, maybe use index wrapper type or some fucking shit, or just fucking transmute,
// fuck do I know
const FONT: &[u8] = include_bytes!("font");

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Position {
    pub x: usize,
    pub y: usize,
}

pub struct Dimensions {
    pub width: usize,
    pub height: usize,
}

// TODO: Is volatile needed here
// TODO: Support for different framebuffer formats
pub struct Buffer {
    pub pixels: &'static mut [u8],
    pub dimensions: Dimensions,
}

const CELL_WIDTH: usize = 8;
const CELL_HEIGHT: usize = 16;

// TODO: Find alternative to storing as a pixel, because that may cause issues with colored output
// when output through a Grayscale buffer
pub struct CellBuffer<'a, P: pixel::Color> {
    pub buffer: pixel::Buffer<P, &'a mut [P::SubPixel]>,
}

impl<'a, P: pixel::Color> CellBuffer<'a, P> {
    fn write_cell(&mut self, x: usize, y: usize, data: &[P]) {
        assert_eq!(data.len(), CELL_WIDTH * CELL_HEIGHT);
        // TODO: Make more iDiOmAtIc
        for (y_off, part) in data.chunks(CELL_WIDTH).enumerate() {
            for (x_off, pixel) in part.iter().enumerate() {
                self.buffer[(x * CELL_WIDTH + x_off, y * CELL_HEIGHT + y_off)] = pixel.clone();
            }
        }
    }

    fn new_line(&mut self) {
        // unsafe {
        //     asm!("sfence");
        // }
        for row in 0..self.buffer.height() - CELL_HEIGHT {
            for col in 0..self.buffer.width() {
                self.buffer[(col, row)] = self.buffer[(col, row + CELL_HEIGHT)];
            }
        }
    }

    fn width(&self) -> usize {
        self.buffer.width() / CELL_WIDTH
    }

    fn height(&self) -> usize {
        self.buffer.height() / CELL_HEIGHT
    }
}

pub struct Writer<P: pixel::Color + 'static> {
    pub buffer: CellBuffer<'static, P>,
    pub position: Position,
    pub pending_newline: bool,
}

const FONT_WIDTH: usize = 8;
const FONT_HEIGHT: usize = 16;

// const FOREGROUND: pixel::RGBA = pixel::RGBA::from_channels(&[256u8, 256, 256, 256]);
// const BACKGROUND: pixel::RGBA = pixel::RGBA::from_channels(&[0u8, 0, 0, 0]);

// TODO: Use fixed size slices where applicable
impl<P: pixel::Color> Writer<P> {
    // TODO: Fix
    fn char_to_pixels(character: u8) -> [P; CELL_WIDTH * CELL_HEIGHT] {
        // unsafe {
        //     breakpoint();
        // }
        let offset = character as usize * (FONT_WIDTH * FONT_HEIGHT / 8);
        let bits = &FONT[offset..offset + (FONT_WIDTH * FONT_HEIGHT / 8)];
        let mut pixels: [P; CELL_WIDTH * CELL_HEIGHT] =
            [P::from_channels(&[0, 0, 0, 0]); CELL_WIDTH * CELL_HEIGHT];
        for (bit, pixel) in bits
            .view_bits::<Msb0>()
            .iter()
            .by_vals()
            .zip(pixels.iter_mut())
        {
            *pixel = if bit {
                // P::from_channels(&[34, 34, 178, 255])
                // P::from_channels(&[247, 246, 89, 255])
                P::from_channels(&[219, 178, 235, 255])
            } else {
                // P::from_channels(&[89, 246, 247, 1])
                P::from_channels(&[40, 40, 40, 255])
            }
        }
        pixels
    }

    pub fn write_byte(&mut self, byte: u8) {
        match byte {
            b'\n' => self.new_line(), // self.pending_newline = true,
            byte => {
                if self.position.x >= self.buffer.width() || self.pending_newline {
                    self.new_line()
                }

                self.buffer.write_cell(
                    self.position.x,
                    self.position.y,
                    Self::char_to_pixels(byte).as_slice(),
                );
                self.position.x += 1;
            }
        }
    }

    // Carry newline until new characters are actually printed
    fn new_line(&mut self) {
        // Clear rest of line to avoid mixing up of lines
        while self.position.x < self.buffer.width() {
            self.write_byte(b'\0');
        }
        if self.position.y >= self.buffer.height() - 1 {
            self.buffer.new_line();
            self.position.y = self.buffer.height() - 1;
        } else {
            self.position.y += 1;
        }
        self.position.x = 0;

        self.pending_newline = false;
    }
}

// impl Writer {
//     fn clear_row(&mut self, row: usize) {
//         for col in 0..self.buf.dimensions.width {
//             self.buf.pixels[row * col] = ColorCode::new(Color::Black).0;
//         }
//     }
// }

impl<P: pixel::Color> fmt::Write for Writer<P> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write_string(s);
        Ok(())
    }
}

impl<P: pixel::Color> Writer<P> {
    pub fn write_string(&mut self, s: &str) {
        for byte in s.bytes() {
            match byte {
                // printable ASCII byte or newline
                0x20..=0x7e | b'\n' => self.write_byte(byte),
                // not part of printable ASCII range
                _ => self.write_byte(0xfe),
            }
        }
    }
}

pub static WRITER: Mutex<Option<Writer<pixel::RGBA>>> = Mutex::new(None);

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => ($crate::framebuffer::_print(format_args!($($arg)*)));
}

#[macro_export]
macro_rules! println {
    () => ($crate::print!("\n"));
    ($($arg:tt)*) => ($crate::print!("{}\n", format_args!($($arg)*)));
}

#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    use core::fmt::Write;
    WRITER.lock().as_mut().unwrap().write_fmt(args).unwrap();
}
