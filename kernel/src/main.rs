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
use core::panic::PanicInfo;
use limine::{
    LimineBootInfoRequest, LimineFramebufferRequest, LimineHhdmRequest, LimineMemmapRequest,
    LimineMemoryMapEntryType, LimineRsdpRequest,
};
use mm::phys::frame;
use mm::virt::*;
use x86_64::{
    structures::paging::{PageTable, PageTableFlags},
    PhysAddr, VirtAddr,
};

mod acpi;
mod elf;
mod framebuffer;
mod interrupt;
mod mm;
mod pcie;
mod proc;
mod xhci;

static BOOTLOADER_INFO: LimineBootInfoRequest = LimineBootInfoRequest::new(0);
static FRAMEBUFFER: LimineFramebufferRequest = LimineFramebufferRequest::new(0);
static MEMMAP: LimineMemmapRequest = LimineMemmapRequest::new(0);
static HHDM: LimineHhdmRequest = LimineHhdmRequest::new(0);
static RDSP: LimineRsdpRequest = LimineRsdpRequest::new(0);

// TODO: Unfuck this / Don't rely on direct map
static PHYS_OFFSET: spin::RwLock<VirtAddr> = spin::RwLock::new(VirtAddr::new_truncate(0));

pub fn phys_offset() -> VirtAddr {
    *PHYS_OFFSET.read()
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("{}", info);
    loop {}
}

#[no_mangle]
fn _start() -> ! {
    // if let Some(bootinfo) = BOOTLOADER_INFO.get_response().get() {
    //     println!(
    //         "booted by {} v{}",
    //         bootinfo.name.to_str().unwrap().to_str().unwrap(),
    //         bootinfo.version.to_str().unwrap().to_str().unwrap(),
    //     );
    // }

    if let Some(framebuffer) = FRAMEBUFFER.get_response().get() {
        framebuffer::init(&*framebuffer.framebuffers()[0]);
    }

    let memmap = MEMMAP
        .get_response()
        .get()
        .expect("Atomkern requires memory segment information")
        .memmap();

    let hhdm = HHDM.get_response().get().unwrap().offset;
    *PHYS_OFFSET.write() = VirtAddr::new(hhdm);

    // let addr;
    // let len;
    // {
    //     set_pat();

    //     let info = fb.info();
    //     addr = fb.buffer().as_ptr().addr() / 4096 * 4096;
    //     len = info.byte_len;
    // }

    // TODO: Extract into a PAT helper
    // let mut val = addr;
    // while val < addr + len {
    //     add_page_flags(VirtAddr::new(val as u64));
    //     val += 4096;
    // }
    // wat();

    // TODO: Figure out the amount of memory in the system dynamically
    let amount_4k = 41943040 / 8;
    let amount_2m = 81920 / 8;
    let amount_1g = 160 / 8;

    memmap
        .iter()
        .filter(|x| x.typ == LimineMemoryMapEntryType::Usable)
        .for_each(|x| println!("{:?}", x));

    let alloc_range = memmap
        .iter()
        .find(|x| {
            x.len > amount_1g + amount_2m + amount_4k && x.typ == LimineMemoryMapEntryType::Usable
        })
        .expect("No suitable region found for frame allocator");

    let mut allocator = frame::Allocator::new(alloc_range, 42949672960);

    memmap
        .iter()
        .filter(|x| x.typ == LimineMemoryMapEntryType::Usable)
        .for_each(|x| {
            allocator.register_region(PhysAddr::new(x.base), PhysAddr::new(x.base + x.len));
        });

    // TODO: Don't retroactively unregister the memory used by the allocator bitmap
    allocator.unregister_region(
        PhysAddr::new(alloc_range.base),
        // Bigger == better :bingshrug:
        PhysAddr::new(alloc_range.base + amount_4k + amount_2m + amount_1g + 0x1000),
    );

    let rdsp = &RDSP
        .get_response()
        .get()
        .expect("Atomkern requires acpi")
        .address;

    let acpi = acpi::Acpi::parse(rdsp.as_ptr().unwrap());

    *interrupt::ALLOC.lock() =
        Some(unsafe { core::mem::transmute::<_, &'static mut frame::Allocator>(&mut allocator) });

    interrupt::init_interrupts();

    // let bytes = include_bytes!("fuck");
    // elf::parse(*interrupt::ALLOC.lock().as_mut().unwrap(), bytes);

    // interrupt::init_runtime();

    crate::pcie::pcie_shenanigans(&mut allocator, &mut acpi.get_pcie_configs());

    // interrupt::init_multicore(
    //     interrupt::ALLOC.lock().as_mut().unwrap(),
    //     &acpi.get_apics(),
    // );

    loop {}
}
