use crate::println;
use core::ops::IndexMut;
use x86_64::PhysAddr;

// extern crate bitvec;
use bitvec::slice::BitSlice;

#[derive(Debug)]
pub struct Allocator<'a> {
    pub map_4K: &'a mut BitSlice,
    pub map_2M: &'a mut BitSlice,
    pub map_1G: &'a mut BitSlice,
}

// have allocerror just use the enum variants or structs used
#[derive(Debug)]
pub enum AllocError {
    No1G,
    No2M,
    No4k,
}

#[derive(Debug)]
pub enum DeallocError {
    Error,
}

// TODO: Turn into recursive loops
// TODO: Bring in line with general allocator design
impl<'a> Allocator<'a> {
    pub fn setup(&mut self) {
        for mut bit in self.map_4K.iter_mut() {
            bit.set(true);
        }

        for mut bit in self.map_2M.iter_mut() {
            bit.set(true);
        }

        for mut bit in self.map_1G.iter_mut() {
            bit.set(true);
        }
    }
    pub fn allocate(&mut self) -> Result<PhysAddr, AllocError> {
        let index_1g = self
            .map_1G
            .iter()
            .enumerate()
            .find_map(|(i, bit)| (!*bit).then_some(i))
            .unwrap();
        // println!("1G page index: {}", index_1g);

        let offset_2m = index_1g * 512;
        let index_2m = offset_2m
            + self.map_2M[offset_2m..offset_2m + 512]
                .iter()
                .enumerate()
                .find_map(|(i, bit)| (!*bit).then_some(i))
                .unwrap();
        // println!("2M page index: {}", index_2m);

        let offset_4k = index_2m * 512;
        let index_4k = offset_4k
            + self.map_4K[offset_4k..offset_4k + 512]
                .iter()
                .enumerate()
                .find_map(|(i, bit)| (!*bit).then_some(i))
                .unwrap();
        // println!("4K page index: {}", index_4k);

        self.map_4K.set(index_4k, true);

        if self.map_4K[offset_4k..offset_4k + 512].all() {
            self.map_2M.set(index_2m, true);
        }

        if self.map_2M[offset_2m..offset_2m + 512].all() {
            self.map_1G.set(index_1g, true);
        }

        Ok(PhysAddr::new((index_4k as u64) * 0x1000))
    }

    // TODO: Check if it rounds down properly
    pub fn deallocate(&mut self, addr: PhysAddr) -> Result<(), DeallocError> {
        let val = usize::try_from(addr.as_u64()).unwrap() / 0x1000;
        println!("Deallocated val: {}", val);

        self.map_4K.set(val, false);
        self.map_2M.set(val / 512, false);
        self.map_1G.set(val / 512 / 512, false);

        Ok(())
    }

    // End == exclusive
    pub fn register_region(&mut self, start: PhysAddr, end: PhysAddr) {
        println!("registered_region from {:?} to {:?}", start, end);
        let mut current = start;
        while current < end {
            let val = current.as_u64() as usize / 0x1000;
            self.map_4K.set(val, false);
            self.map_2M.set(val / 512, false);
            self.map_1G.set(val / 512 / 512, false);
            current = PhysAddr::new_truncate(current.as_u64() + 0x1000);
        }
    }

    pub fn unregister_region(&mut self, start: PhysAddr, end: PhysAddr) {
        println!("registered_region from {:?} to {:?}", start, end);
        let mut current = start;
        while current < end {
            let val = current.as_u64() as usize / 0x1000;
            // We'll fix up not marking the entire thing later, but for now we'll just do it, cuz
            // checking every time is gonna be hella fucking slow
            self.map_4K.set(val, true);
            self.map_2M.set(val / 512, true);
            self.map_1G.set(val / 512 / 512, true);
            current = PhysAddr::new_truncate(current.as_u64() + 0x1000);
        }
    }
}
