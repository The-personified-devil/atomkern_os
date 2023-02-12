use core::convert::TryInto;
// use core::convert::TryInto;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut, Index, IndexMut};

pub trait Primitive: Copy + Clone + PartialOrd<Self> + 'static {}

impl Primitive for u8 {}

/// Copy of a part of the buffer, but behaves like a view
pub trait Color: Copy {
    type SubPixel: Primitive;

    fn channel_count() -> usize;
    fn from_slice<'a>(slice: &'a [Self::SubPixel]) -> &'a Self;
    fn from_slice_mut<'a>(slice: &'a mut [Self::SubPixel]) -> &'a mut Self;
    fn from_channels(slice: &[usize]) -> Self;
}

// TODO: Make it runtime, sacrificing performance, but avoiding unsafe and other hacks
#[derive(Clone, Copy)]
pub struct RGBA([u8; 4]);

impl Color for RGBA {
    type SubPixel = u8;

    #[inline]
    fn channel_count() -> usize {
        4
    }

    /// Act like a transparent wrapper type
    #[inline]
    fn from_slice<'a>(slice: &'a [Self::SubPixel]) -> &'a Self {
        unsafe {
            assert_eq!(slice.len(), Self::channel_count());
            core::mem::transmute(slice.as_ptr())
        }
    }

    /// Act like a transparent wrapper type
    #[inline]
    fn from_slice_mut<'a>(slice: &'a mut [Self::SubPixel]) -> &'a mut Self {
        unsafe {
            assert_eq!(slice.len(), Self::channel_count());
            core::mem::transmute(slice.as_ptr())
        }
    }

    #[inline]
    // TODO: Cannel count not nice, consider when doing other types
    fn from_channels(slice: &[usize]) -> RGBA {
        let mut arr: [u8; 4] = [08; 4];
        for (num, a) in slice.iter().zip(arr.iter_mut()) {
            *a = (*num).try_into().unwrap();
        }
        Self(arr)
    }
}

// TODO: Why not just transmute the container to one with the right type?
pub struct Buffer<P: Color, Container: Deref<Target = [P::SubPixel]>> {
    pub width: usize,
    pub height: usize,
    pub data: Container,
    pub _pixel_type: PhantomData<P>,
}

impl<P, Container> Buffer<P, Container>
where
    P: Color,
    Container: Deref<Target = [P::SubPixel]>,
{
    fn get_pixel(&self, x: usize, y: usize) -> &P {
        let ch_count = <P as Color>::channel_count() as usize;
        let i = ch_count * (y * self.width + x) as usize;
        <P as Color>::from_slice(&self.data[i..i + ch_count])
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }
}

impl<P, Container> Buffer<P, Container>
where
    P: Color,
    Container: Deref<Target = [P::SubPixel]> + DerefMut,
{
    fn get_pixel_mut(&mut self, x: usize, y: usize) -> &mut P {
        let ch_count = <P as Color>::channel_count() as usize;
        let i = ch_count * (y * self.width + x) as usize;
        <P as Color>::from_slice_mut(&mut self.data[i..i + ch_count])
    }
}

impl<P, Container> Index<(usize, usize)> for Buffer<P, Container>
where
    P: Color,
    Container: Deref<Target = [P::SubPixel]>,
{
    type Output = P;

    fn index(&self, (x, y): (usize, usize)) -> &P {
        self.get_pixel(x, y)
    }
}

impl<P, Container> IndexMut<(usize, usize)> for Buffer<P, Container>
where
    P: Color,
    Container: Deref<Target = [P::SubPixel]> + DerefMut,
{
    fn index_mut(&mut self, (x, y): (usize, usize)) -> &mut P {
        self.get_pixel_mut(x, y)
    }
}
