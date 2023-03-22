#![no_std]
#![feature(core_intrinsics)]
#![feature(abi_x86_interrupt)]
#![feature(pointer_byte_offsets)]
#![feature(strict_provenance)]
#![feature(naked_functions)]
#![feature(const_maybe_uninit_zeroed)]
#![feature(int_roundings)]
#![feature(const_mut_refs)]
#![no_main]
#![allow(unaligned_references)]
#![warn(clippy::all, clippy::pedantic)]

use arrayvec::ArrayVec;
use bootloader_api::{
    config::Mapping,
    entry_point,
    info::{MemoryRegion, MemoryRegionKind},
    BootInfo, BootloaderConfig,
};
use core::panic::PanicInfo;
use mm::phys::frame;
use mm::virt::*;
use x86_64::{PhysAddr, VirtAddr, structures::paging::{PageTable, PageTableFlags}};

mod acpi;
mod framebuffer;
mod interrupt;
mod mm;
mod pcie;
mod proc;
mod xhci;
mod elf;

const PHYS_OFFSET: VirtAddr = VirtAddr::new_truncate(0x10000000000);

pub static CONFIG: BootloaderConfig = {
    let mut config = BootloaderConfig::new_default();
    config.mappings.physical_memory = Some(Mapping::FixedAddress(PHYS_OFFSET.as_u64()));
    config
};

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("{}", info);
    loop {}
}

entry_point!(kernel_main, config = &CONFIG);

fn kernel_main(boot_info: &'static mut BootInfo) -> ! {
    let fb = boot_info.framebuffer.as_mut().unwrap();

    let addr;
    let len;
    {
        set_pat();

        let info = fb.info();
        addr = fb.buffer().as_ptr().addr() / 4096 * 4096;
        len = info.byte_len;
    }

    framebuffer::init(fb);

    // TODO: Extract into a PAT helper
    let mut val = addr;
    while val < addr + len {
        add_page_flags(VirtAddr::new(val as u64));
        val += 4096;
    }
    wat();

    let mut ranges = ArrayVec::<MemoryRegion, 40>::new();
    let mut last_end = 0;
    let mut range_start = 0;
    let mut region_kind = bootloader_api::info::MemoryRegionKind::UnknownBios(1);
    for (i, region) in boot_info.memory_regions.iter().enumerate() {
        if last_end == region.start && region.kind == region_kind {
            last_end = region.end;
        } else {
            ranges.push(MemoryRegion {
                start: range_start,
                end: last_end,
                kind: region_kind,
            });
            range_start = region.start;
            last_end = region.end;
            region_kind = region.kind;
        }
    }

    ranges.push(MemoryRegion {
        start: range_start,
        end: last_end,
        kind: region_kind,
    });

    // TODO: Figure out the amount of memory in the system dynamically
    let amount_4k = 41943040 / 8;
    let amount_2m = 81920 / 8;
    let amount_1g = 160 / 8;

    for range in ranges.iter() {
        println!(
            "Range {:?} {}-{} = {}",
            range.kind,
            range.start,
            range.end,
            range.end - range.start
        );
    }

    let alloc_range = ranges
        .iter()
        .find(|x| {
            x.end - x.start > amount_1g + amount_2m + amount_4k
                && x.kind == MemoryRegionKind::Usable
        })
        .expect("No suitable region found for frame allocator");

    let mut allocator = frame::Allocator::new(alloc_range, 42949672960);

    ranges
        .iter()
        .filter(|x| x.kind == MemoryRegionKind::Usable)
        .for_each(|x| {
            allocator.register_region(
                PhysAddr::new_truncate(x.start),
                PhysAddr::new_truncate(x.end),
            );
        });

    // TODO: Don't retroactively unregister the memory used by the allocator bitmap
    allocator.unregister_region(
        PhysAddr::new_truncate(alloc_range.start),
        // Bigger == better :bingshrug:
        PhysAddr::new_truncate(alloc_range.start + amount_4k + amount_2m + amount_1g + 0x1000),
    );

    let page_table = unsafe { active_page_table() };
    let mut first_page = false;
    fn iter_table(page_table: &mut PageTable, level: usize, first_page: &mut bool) {
        let mut begin_addr = Some(page_table.iter().next().unwrap().addr());
        let mut prev_addr = begin_addr.unwrap();
        for (i, entry) in page_table.iter_mut().enumerate() {
            let mut flags = entry.flags();
            flags.set(PageTableFlags::USER_ACCESSIBLE, true);
            if !*first_page && level == 3 {
                // println!("yeet");
                flags.set(PageTableFlags::PRESENT, true);
                flags.set(PageTableFlags::WRITABLE, true);
                *first_page = true;
                entry.set_addr(x86_64::PhysAddr::new_truncate(0x1000 * (i as u64)), flags);
            }
            entry.set_flags(flags);
            // if flags.contains(PageTableFlags::PRESENT) {
            //     println!("Level {}, Index {}: {:?}", level, i, entry);
            // }

            prev_addr = entry.addr();
            if entry.addr() == x86_64::PhysAddr::zero() {
                if begin_addr.is_some() {
                    // println!("{:?}, {:?}", begin_addr, prev_addr);
                    begin_addr = None;
                }
                continue;
            }
            if begin_addr.is_none() {
                begin_addr = Some(entry.addr());
            }

            if entry.flags().contains(PageTableFlags::HUGE_PAGE) || level >= 3 {
                continue;
            }

            // println!("{:?}", entry);
            iter_table(
                unsafe { &mut *(PHYS_OFFSET + entry.addr().as_u64()).as_mut_ptr::<PageTable>() },
                level + 1,
                first_page,
            );
        }
        // println!("{:?}, {:?}", begin_addr, prev_addr);
    }
    iter_table(page_table, 0, &mut first_page);

    let bytes = include_bytes!("fuck");


    let acpi = acpi::Acpi::parse(
        boot_info
            .rsdp_addr
            .into_option()
            .expect("Acpi is required by the Atomkern os"),
    );

    *interrupt::ALLOC.lock() = Some(unsafe { core::mem::transmute::<_, &'static mut frame::Allocator>( &mut allocator) });

    interrupt::init_interrupts();

    elf::parse(*interrupt::ALLOC.lock().as_mut().unwrap(), bytes);
    interrupt::init_runtime();

    // crate::pcie::pcie_shenanigans(&mut allocator, &mut acpi.get_pcie_configs());
    
    // interrupt::init_multicore(
    //     interrupt::ALLOC.lock().as_mut().unwrap(),
    //     &acpi.get_apics(),
    // );

    loop {}
}
