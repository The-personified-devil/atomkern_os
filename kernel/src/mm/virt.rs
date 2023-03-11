use crate::println;
use bitvec::prelude::*;
use bytemuck::TransparentWrapper;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use proc_bitfield::bitfield;
use x86_64::{
    registers,
    registers::model_specific::Msr,
    structures::paging::{PageTable, PageTableFlags},
    VirtAddr,
};

// as param?
const PHYS_OFFSET: VirtAddr = VirtAddr::new_truncate(0x10000000000);

// struct Allocator {

// }
//
bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct PatEntry(u8): Debug, FromRaw, IntoRaw {
        pub mem_type: u8 [try PatMemType] @ 0..=3,
    }
}

unsafe impl TransparentWrapper<u8> for PatEntry {}

#[repr(transparent)]
#[derive(Debug)]
struct Pat([PatEntry; 8]);

unsafe impl TransparentWrapper<[PatEntry; 8]> for Pat {}

impl Pat {
    pub fn new() -> Pat {
        Pat([
            PatEntry(0).with_mem_type(PatMemType::WriteBack),
            PatEntry(0).with_mem_type(PatMemType::WriteThrough),
            PatEntry(0).with_mem_type(PatMemType::Uncached),
            PatEntry(0).with_mem_type(PatMemType::Unchacheable),
            PatEntry(0).with_mem_type(PatMemType::WriteCombining),
            // PatEntry(0).with_mem_type(PatMemType::WriteCombining),
            // PatEntry(0).with_mem_type(PatMemType::WriteCombining),
            // PatEntry(0).with_mem_type(PatMemType::WriteCombining),
            // PatEntry(0).with_mem_type(PatMemType::WriteCombining),
            // PatEntry(0).with_mem_type(PatMemType::WriteCombining),
            // PatEntry(0).with_mem_type(PatMemType::WriteCombining),
            // PatEntry(0).with_mem_type(PatMemType::WriteCombining),
            PatEntry(0).with_mem_type(PatMemType::WriteProtected),
            PatEntry(0).with_mem_type(PatMemType::Uncached),
            PatEntry(0).with_mem_type(PatMemType::Unchacheable),
        ])
    }
}

#[derive(Debug, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum PatMemType {
    Unchacheable = 0x00,
    WriteCombining = 0x01,
    WriteThrough = 0x04,
    WriteProtected = 0x05,
    WriteBack = 0x06,
    Uncached = 0x07,
}

pub fn set_pat() {
    let mut msr = Msr::new(0x277);
    unsafe {
        // println!("old pat {:?}", Pat::wrap(PatEntry::wrap_slice(&msr.read().to_le_bytes()).try_into().unwrap()));
        msr.write(u64::from_le_bytes(
            PatEntry::peel_slice(&Pat::peel(Pat::new()))
                .try_into()
                .unwrap(),
        ));
    }
}
pub fn wat() {
    let mut msr = Msr::new(0x277);
    unsafe {
        println!(
            "old pat {:?}",
            Pat::wrap(
                PatEntry::wrap_slice(&msr.read().to_le_bytes())
                    .try_into()
                    .unwrap()
            )
        );
    }
}

pub fn add_page_flags(addr: x86_64::VirtAddr) {
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
    let level3 = addr_to_table(level3.addr());

    let level2 = &mut level3[l3 as usize];
    let level2 = addr_to_table(level2.addr());

    let level1 = &mut level2[l2 as usize];
    let level1 = addr_to_table(level1.addr());

    let entry = &mut level1[l1 as usize];

    let flags = entry.flags();
    // println!("old flags: {:?}", flags);
    entry.set_flags(unsafe { PageTableFlags::from_bits_unchecked(1 << 7) } | flags)
    // println!("{:?}", entry);

    // println!("{:?} {:?} {:?} {:?}", level3, level2, level1, entry);
}

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
        level3.set_addr(addr, PageTableFlags::PRESENT | PageTableFlags::WRITABLE);
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
    if level2.addr().is_null() {
        let addr = allocator.allocate().unwrap();
        // println!("{:?}", addr);
        level2.set_addr(addr, PageTableFlags::PRESENT | PageTableFlags::WRITABLE);
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
    if level1.addr().is_null() {
        let addr = allocator.allocate().unwrap();
        // println!("{:?}", addr);
        level1.set_addr(addr, PageTableFlags::PRESENT | PageTableFlags::WRITABLE);
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
    entry.set_addr(frame, PageTableFlags::PRESENT | PageTableFlags::WRITABLE);
    /* println!("{:?}", entry); */

    // println!("{:?} {:?} {:?} {:?}", level3, level2, level1, entry);
}
