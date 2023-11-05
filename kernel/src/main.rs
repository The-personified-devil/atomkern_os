#![no_std]
#![feature(core_intrinsics)]
#![feature(abi_x86_interrupt)]
#![feature(pointer_byte_offsets)]
#![feature(strict_provenance)]
#![feature(naked_functions)]
#![feature(const_maybe_uninit_zeroed)]
#![feature(int_roundings)]
#![feature(const_mut_refs)]
#![feature(iter_collect_into)]
#![no_main]
#![warn(clippy::all, clippy::pedantic)]

use arrayvec::ArrayVec;
use atomkern_abi as abi;
use core::{arch::asm, panic::PanicInfo, ptr::addr_of};
use limine::{
    BootInfoRequest, FramebufferRequest, HhdmRequest, MemmapRequest, MemoryMapEntryType,
    RsdpRequest,
};
use mm::phys::frame;
use mm::virt::*;
use x86_64::{PhysAddr, VirtAddr};

mod acpi;
mod elf;
mod framebuffer;
mod interrupt;
mod misc;
mod mm;
mod pcie;
mod proc;
mod xhci;

static BOOTLOADER_INFO: BootInfoRequest = BootInfoRequest::new(0);
static FRAMEBUFFER: FramebufferRequest = FramebufferRequest::new(0);
static MEMMAP: MemmapRequest = MemmapRequest::new(0);
static HHDM: HhdmRequest = HhdmRequest::new(0);
static RDSP: RsdpRequest = RsdpRequest::new(0);

// TODO: Unfuck this / Don't rely on direct map
static PHYS_OFFSET: spin::RwLock<VirtAddr> = spin::RwLock::new(VirtAddr::new_truncate(0));

// Stolen from somewhere lmao
#[macro_use]
mod macros {
    #[repr(C)] // guarantee 'bytes' comes after '_align'
    pub struct AlignedAs<Align, Bytes: ?Sized> {
        pub _align: [Align; 0],
        pub bytes: Bytes,
    }

    macro_rules! include_bytes_align_as {
        ($align_ty:ty, $path:literal) => {{
            // const block expression to encapsulate the static
            use $crate::macros::AlignedAs;

            // this assignment is made possible by CoerceUnsized
            static ALIGNED: &AlignedAs<$align_ty, [u8]> = &AlignedAs {
                _align: [],
                bytes: *include_bytes!($path),
            };

            &ALIGNED.bytes
        }};
    }
}

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
    unsafe {
        x86_64::registers::control::Cr4::update(|x| {
            x.set(x86_64::registers::control::Cr4Flags::FSGSBASE, true)
        });
    }

    if let Some(framebuffer) = FRAMEBUFFER.get_response().get() {
        framebuffer::init(&*framebuffer.framebuffers()[0]);
    }

    let memmap = MEMMAP
        .get_response()
        .get()
        .expect("Atomkern requires memory segment information")
        .memmap();

    println!("hhdm {:?}", HHDM.get_response().get());
    let hhdm = HHDM.get_response().get().unwrap().offset;
    *PHYS_OFFSET.write() = VirtAddr::new(hhdm);
    println!("phys offest {:?}", phys_offset());

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
        .filter(|x| x.typ == MemoryMapEntryType::Usable)
        .for_each(|x| println!("{:?}", x));

    let alloc_range = memmap
        .iter()
        .find(|x| x.len > amount_1g + amount_2m + amount_4k && x.typ == MemoryMapEntryType::Usable)
        .expect("No suitable region found for frame allocator");

    let mut allocator = frame::Allocator::new(alloc_range, 42949672960);

    memmap
        .iter()
        .filter(|x| x.typ == MemoryMapEntryType::Usable)
        .for_each(|x| {
            allocator.register_region(PhysAddr::new(x.base), PhysAddr::new(x.base + x.len));
        });

    // TODO: Don't retroactively unregister the memory used by the allocator bitmap
    // TODO: Use unsafe for things like this that can't be made safe in every context
    allocator.unregister_region(
        PhysAddr::new(alloc_range.base),
        // Bigger == better :bingshrug:
        PhysAddr::new(alloc_range.base + amount_4k + amount_2m + amount_1g + 0x1000),
    );

    // TODO: This should prolly not be initialized this late lmao
    *interrupt::ALLOC.lock() =
        Some(unsafe { core::mem::transmute::<_, &'static mut frame::Allocator>(&mut allocator) });

    misc::cpu_local::setup_cpu_local(interrupt::ALLOC.lock().as_mut().unwrap(), 0);

    let rdsp = &RDSP
        .get_response()
        .get()
        .expect("Atomkern requires acpi")
        .address;

    let acpi = acpi::Acpi::parse(rdsp.as_ptr().unwrap());

    // TODO: Ensure id 0 actually matches bsp
    interrupt::init_interrupts(interrupt::ALLOC.lock().as_mut().unwrap(), 0);

    // interrupt::init_multicore(interrupt::ALLOC.lock().as_mut().unwrap(), &acpi.get_apics());
    
    crate::pcie::pcie_shenanigans(&mut allocator, &mut acpi.get_pcie_configs());

    crate::proc::setup_syscalls();

    let bytes = include_bytes_align_as!(u64, "fuck");
    let (entry, tls) = elf::parse(*interrupt::ALLOC.lock().as_mut().unwrap(), bytes);

    crate::proc::create_kernel_handler();
    // crate::proc::queue_proc(0);
    crate::proc::queue_proc(crate::proc::create_proc(0, entry, tls, 0));
    interrupt::init_runtime();

    loop {}
}
