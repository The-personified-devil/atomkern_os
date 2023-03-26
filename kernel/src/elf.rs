use crate::{
    frame::Allocator,
    mm::virt::{active_page_table, addr_to_table, proper_map_page, MapFlags},
    println,
};
use bitflags::bitflags;
use core::{arch::asm, ptr::addr_of};
use x86_64::{PhysAddr, VirtAddr};

#[derive(Debug)]
#[repr(C)]
struct Header {
    magic: [u8; 4],
    bitness: u8,
    endianess: u8,
    header_version: u8,
    abi: u8,
    padding: [u8; 6],
    toipe: u16,
    isa: u16,
    elf_version: u32,
    prog_entry_pos: u64,
    prog_head_pos: u64,
    section_head_pos: u64,
    flags: u32,
    header_size: u16,
    prog_entry_size: u16,
    prog_entry_num: u16,
    sec_entry_size: u16,
    sec_entry_num: u16,
    sec_head_index: u16,
}

#[derive(Debug)]
#[repr(C)]
struct ProgramHeader {
    seg_type: u32,
    flags: SegmentFlags,
    offset: u64,
    vaddr: u64,
    _res1: u64,
    size: u64,
    mem_size: u64,
    alignment: u64,
}

bitflags! {
    #[derive(Debug, PartialEq)]
    struct SegmentFlags: u32 {
        const Executable = 0;
        const Writable = 2;
        const Readable = 4;
    }
}

pub fn parse(allocator: &mut Allocator, slice: &[u8]) -> Option<()> {
    let addr = addr_of!(*slice);
    let header = unsafe { &*addr.cast::<Header>() };

    println!("header {:?}", header);

    // if &header.magic != b"\x7FELF"
    //     || header.bitness != 2
    //     || header.endianess != 1
    //     || header.abi != 0
    //     || header.toipe != 2
    //     || header.isa != 0x3E
    // {
    //     return None;
    // }

    let mut sections = arrayvec::ArrayVec::<&ProgramHeader, 50>::new();

    let prog_base = unsafe { addr.byte_add(header.prog_head_pos as usize) };
    for i in 0..header.prog_entry_num {
        let prog_header = unsafe {
            &*prog_base
                .byte_add(header.prog_entry_size as usize * i as usize)
                .cast::<ProgramHeader>()
        };

        println!("prog_header {:?}", prog_header);

        sections.push(prog_header);
    }

    let table = allocator.allocate().unwrap();

    let pml4 = addr_to_table(table);
    let current = unsafe { active_page_table() };

    for i in 1..512 {
        pml4[i] = current[i].clone();
    }

    // If this actually fucking works lmfao
    unsafe {
        asm!("mov cr3, {:r}", in(reg) table.as_u64());
    }

    for section in sections {
        match section.seg_type {
            1 => map_section(allocator, table, section, unsafe {
                addr.cast::<u8>().byte_add(section.offset as usize)
            }),
            _ => (),
        }
    }

    println!("cr3 {:?}", table);
    crate::interrupt::create_proc(allocator, table, header.prog_entry_pos);

    Some(())
}

// this is fucked af
fn map_section(
    allocator: &mut Allocator,
    table: PhysAddr,
    section: &ProgramHeader,
    addr: *const u8,
) {
    let mut flags = MapFlags::empty();

    flags.set(
        MapFlags::Executable,
        section.flags.contains(SegmentFlags::Executable),
    );

    flags.set(
        MapFlags::Writable,
        section.flags.contains(SegmentFlags::Writable),
    );
    flags.set(MapFlags::User, true);
    flags.set(MapFlags::Writable, true);

    println!("flags {:?}", flags);

    for i in 0..=section.mem_size.div_ceil(4096) {
        let page = allocator.allocate().unwrap();

        proper_map_page(
            allocator,
            table,
            VirtAddr::new((section.vaddr / 4096 * 4096) + i * 4096),
            page,
            flags,
        );
    }

    unsafe {
        core::ptr::copy_nonoverlapping(
            addr,
            core::ptr::from_exposed_addr_mut::<u8>(section.vaddr as usize),
            section.size as usize,
        );
    }
}
