#![no_std]
#![feature(core_intrinsics)]
#![feature(iter_array_chunks)]
#![feature(abi_x86_interrupt)]
#![feature(pointer_byte_offsets)]
#![feature(strict_provenance)]
#![feature(naked_functions)]
#![no_main]

use arrayvec::ArrayVec;
use bootloader_api::{
    config::Mapping,
    entry_point,
    info::{MemoryRegion, MemoryRegionKind},
    BootInfo, BootloaderConfig,
};
use core::panic::PanicInfo;
use core::prelude::*;
use core::ptr;
mod mm;
mod proc;
// use core::marker::PhantomData;

use core::convert::TryInto;

use mm::phys::frame;
use mm::phys::{Alloc, Allocator, PageRange};
use mm::virt::active_page_table;
use x86_64::{
    registers,
    structures::paging::{PageTable, PageTableFlags},
    PhysAddr, VirtAddr,
};

mod framebuffer;
mod interrupt;

const PHYS_OFFSET: VirtAddr = VirtAddr::new_truncate(0x10000000000);

pub static CONFIG: BootloaderConfig = {
    let mut config = BootloaderConfig::new_default();
    config.mappings.physical_memory = Some(Mapping::FixedAddress(PHYS_OFFSET.as_u64()));
    // config.mappings.dynamic_range_end = Some(0x1900000000); // 100 GiB cuz I test with 32GB
    config
};

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("{}", info);
    loop {}
}

entry_point!(kernel_main, config = &CONFIG);

fn kernel_main(boot_info: &'static mut BootInfo) -> ! {
    let info_addr = ptr::addr_of!(*boot_info);
    let a = 0;

    let fb = boot_info.framebuffer.as_mut().unwrap();
    let fb_addr = ptr::addr_of!(*fb.buffer());
    *framebuffer::WRITER.lock() = Some(framebuffer::Writer {
        buffer: framebuffer::CellBuffer {
            buffer: framebuffer::pixel::Buffer {
                width: fb.info().width,
                height: fb.info().height,
                data: fb.buffer_mut(),
                _pixel_type: core::marker::PhantomData,
            },
        },
        position: framebuffer::Position { x: 0, y: 0 },
    });

    println!(
        "Stack address: {:?}\nBootinfo address: {:?}\nFramebuffer address: {:?}",
        &a, info_addr, fb_addr
    );
    let mut ranges = ArrayVec::<MemoryRegion, 40>::new();
    let mut last_end = 0;
    let mut range_start = 0;
    let mut region_kind = bootloader_api::info::MemoryRegionKind::UnknownBios(1);
    for (i, region) in boot_info.memory_regions.into_iter().enumerate() {
        if last_end == region.start && region.kind == region_kind {
            last_end = region.end;
        } else {
            // println!(
            //     "Range {:?} {}-{} = {}",
            //     region_kind,
            //     range_start,
            //     last_end,
            //     last_end - range_start
            // );
            ranges.push(MemoryRegion {
                start: range_start,
                end: last_end,
                kind: region_kind,
            });
            range_start = region.start;
            last_end = region.end;
            region_kind = region.kind;
        }

        // println!(
        //     "Region {i}: {:?} {}-{} = {}",
        //     region.kind,
        //     region.start,
        //     region.end,
        //     region.end - region.start
        // );
    }

    ranges.push(MemoryRegion {
        start: range_start,
        end: last_end,
        kind: region_kind,
    });

    let alloc: Alloc = Alloc {};
    let amount_4k = 41943040 / 8;
    let amount_2m = 81920 / 8;
    let amount_1g = 160 / 8;

    let mut allocator: Option<frame::Allocator> = None;
    let mut start = 0;

    for range in ranges.iter() {
        println!(
            "Range {:?} {}-{} = {}",
            range.kind,
            range.start,
            range.end,
            range.end - range.start
        );

        // TODO: Move this to the memory allocator (only figure out region, all the fixing it up is
        // left to the memory allocator)
        if range.end - range.start > amount_4k + amount_2m + amount_1g
            && range.kind == MemoryRegionKind::Usable
            && allocator.is_none()
        {
            allocator = Some(frame::Allocator {
                map_4K: unsafe {
                    bitvec::slice::from_raw_parts_mut(
                        bitvec::ptr::BitPtr::from_mut(
                            &mut *(PHYS_OFFSET + range.start).as_mut_ptr(),
                        ),
                        amount_4k as usize * 8,
                    )
                    .unwrap()
                },
                map_2M: unsafe {
                    bitvec::slice::from_raw_parts_mut(
                        bitvec::ptr::BitPtr::from_mut(
                            &mut *(PHYS_OFFSET + range.start + amount_4k).as_mut_ptr(),
                        ),
                        amount_2m as usize * 8,
                    )
                    .unwrap()
                },
                map_1G: unsafe {
                    bitvec::slice::from_raw_parts_mut(
                        bitvec::ptr::BitPtr::from_mut(
                            &mut *(PHYS_OFFSET + range.start + amount_4k + amount_2m).as_mut_ptr(),
                        ),
                        amount_1g as usize * 8,
                    )
                    .unwrap()
                },
            });
            start = range.start;
        }
    }


    let allocator = allocator.as_mut().unwrap();

    allocator.setup();

    ranges
        .iter()
        .filter(|x| x.kind == MemoryRegionKind::Usable)
        .for_each(|x| {
            allocator.register_region(
                PhysAddr::new_truncate(x.start),
                PhysAddr::new_truncate(x.end),
            )
        });

    allocator.unregister_region(
        PhysAddr::new_truncate(start),
        // Bigger == better :bingshrug:
        PhysAddr::new_truncate(start + amount_4k + amount_2m + amount_1g + 0x1000)
    );

    println!("{:?}", allocator.map_1G);

    let no1 = allocator.allocate().unwrap();
    let no2 = allocator.allocate().unwrap();
    let no3 = allocator.allocate().unwrap();
    allocator.deallocate(no3);
    println!("{:?} {:?} {:?}", no1, no2, no3);

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
            )
        }
        // println!("{:?}, {:?}", begin_addr, prev_addr);
    }
    iter_table(page_table, 0, &mut first_page);

    interrupt::init(
        boot_info.rsdp_addr.into_option().unwrap(),
        x86_64::registers::control::Cr3::read()
            .0
            .start_address()
            .as_u64(),
    );

    proc::sysret();

    loop {}
}
