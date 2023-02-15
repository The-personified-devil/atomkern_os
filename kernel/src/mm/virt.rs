use crate::println;
use bitvec::prelude::*;
use x86_64::{
    registers,
    structures::paging::{PageTable, PageTableFlags},
    VirtAddr,
};

// as param?
const PHYS_OFFSET: VirtAddr = VirtAddr::new_truncate(0x10000000000);

// struct Allocator {

// }

pub unsafe fn active_page_table() -> &'static mut PageTable {
    let (page_table, cr3_flags) = registers::control::Cr3::read();
    // println!("start addr: {}", page_table.start_address().as_u64());
    let addr = PHYS_OFFSET + page_table.start_address().as_u64();
    let page_table_ptr: *mut PageTable = addr.as_mut_ptr();
    unsafe { &mut *page_table_ptr }
}

pub fn addr_to_table(addr: x86_64::PhysAddr) -> &'static mut PageTable {
    // println!("addr: {}", addr.as_u64());
    let addr = PHYS_OFFSET + addr.as_u64();
    let page_table_ptr: *mut PageTable = addr.as_mut_ptr();
    unsafe { &mut *page_table_ptr }
}

pub fn map_page(
    allocator: &mut crate::frame::Allocator,
    addr: x86_64::VirtAddr,
    frame: x86_64::PhysAddr,
) {
    // println!("map_page");
    let pml4 = unsafe { active_page_table() };
    let int = addr.as_u64();
    let bits = int.view_bits::<Lsb0>();
    // println!("{:?}", bits);
    let l1 = bits[12..21].load::<u16>();
    let l2 = bits[21..30].load::<u16>();
    let l3 = bits[30..39].load::<u16>();
    let l4 = bits[39..48].load::<u16>();
    // println!("{l1}, {l2}, {l3}, {l4}");
    // println!("{:?}", pml4[l1 as usize]);

    let level3 = &mut pml4[l4 as usize];
    if level3.addr().is_null() {
        let addr = allocator.allocate().unwrap();
        // println!("{:?}", addr);
        level3.set_addr(addr, PageTableFlags::PRESENT);
        let level3 = addr_to_table(level3.addr());
        level3.zero();
    }
    {
        // println!("{:?}", level3);
    }
    let level3 = addr_to_table(level3.addr());
    {
        // println!("l3: {:?}", level3)
    }

    let level2 = &mut level3[l3 as usize];
    if level2.is_unused() {
        let addr = allocator.allocate().unwrap();
        // println!("{:?}", addr);
        level2.set_addr(addr, PageTableFlags::PRESENT);
        let level2 = addr_to_table(level2.addr());
        level2.zero();
    }
    {
        // println!("{:?}", level2);
    }
    let level2 = addr_to_table(level2.addr());
    {
        // println!("l2: {:?}", level2)
    }

    let level1 = &mut level2[l2 as usize];
    if level1.is_unused() {
        let addr = allocator.allocate().unwrap();
        // println!("{:?}", addr);
        level1.set_addr(addr, PageTableFlags::PRESENT);
        let level1 = addr_to_table(level1.addr());
        level1.zero();
    }
    {
        // println!("{:?}", level1);
    }
    let level1 = addr_to_table(level1.addr());
    {
        // println!("l1: {:?}", level1)
    }

    let entry = &mut level1[l1 as usize];
    entry.set_addr(frame, PageTableFlags::PRESENT);
    // println!("{:?}", entry);

    // println!("{:?} {:?} {:?} {:?}", level3, level2, level1, entry);
}
