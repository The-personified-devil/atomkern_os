// Redifining enum depending on platform too hacky
// Individual call for every type preferrable
// Avoid generics, cuz complexity
// Allocator trait
// Type associated constants?
// -> Not it's own type
// => Alias sizes to nttp struct instantiations

// Allocator is not size specific, as it has to manage multiple sizes at a time, typed maybe as a
// wrapper but we'll see

use crate::println;
use core::marker::PhantomData;
pub mod frame;

pub struct PageRange<Size: PageSize> {
    pub start: usize,
    pub end: usize,
    pub phantom: PhantomData<Size>,
}

impl<Size: PageSize> PageRange<Size> {
    fn page_count(&self) -> usize {
        (self.end - self.start) / Size::SIZE
    }
}

pub trait PageSize {
    const SIZE: usize;
}

pub struct Size4k {}

impl PageSize for Size4k {
    const SIZE: usize = 4096;
}
