use crate::misc::cpu_local::setup_cpu_local;
use crate::pcie;
use crate::println;
use bytemuck::TransparentWrapper;
use core::arch::asm;
use core::cell::Cell;
use core::cell::OnceCell;
use core::cell::RefCell;
use core::mem::transmute;
use core::ptr::addr_of;
use core::slice;
use core::sync::atomic::AtomicUsize;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use proc_bitfield::bitfield;
use snafu::prelude::*;
use x86_64::registers::control::Cr2;
use x86_64::registers::model_specific::Msr;
use x86_64::registers::segmentation::Segment;
use x86_64::structures::gdt::GlobalDescriptorTable;
use x86_64::structures::gdt::SegmentSelector;
use x86_64::structures::idt::*;
use x86_64::structures::tss::TaskStateSegment;
use x86_64::PhysAddr;
use x86_64::VirtAddr;

use crate::misc::cpu_local::CpuLocal;

use crate::acpi::X2Apic;

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct PatEntry(u8): Debug, FromRaw, IntoRaw {
        pub mem_type: u8 [try PatMemType] @ 0..=3,
    }
}

#[repr(transparent)]
struct Pat([PatEntry; 8]);

unsafe impl TransparentWrapper<[PatEntry; 8]> for Pat {}

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

#[derive(Debug, Snafu)]
#[snafu(display("Test error mashallahi {id}"))]
struct TestError {
    id: u8,
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct ApicMsr(pub u64): Debug, FromRaw, IntoRaw {
        pub bootstrap: bool @ 8,
        pub enable_x2apic: bool @ 10,
        pub enable: bool @ 11,
        // Only used in xApic mode
        pub base: u64 @ 12..52,
    }
}

// TODO: Make safer in general
bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct LocalVec(pub u64): Debug, FromRaw, IntoRaw {
        pub vec: u8 @ 0..=7,
        // Turn into enum etc...
        pub software_enable: bool @ 8,
        pub message_type: u8 @ 8..=10,
        pub delivery_status: bool @ 12,
        pub remote_irr: bool @ 14,
        pub trigger_mode: bool @ 15,
        pub mask: bool @ 16,
        pub timer_mode: bool @ 17,
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct InterprocessInterrupt(pub u64): Debug, FromRaw, IntoRaw {
        pub vec: u8 @ 0..=7,
        pub message_type: u8 [try MessageType] @ 8..=10,
        pub dest_mode: bool @ 11,
        pub delivery_status: bool @ 12,
        pub level: bool @ 14,
        pub trigger_mode: bool @ 15,
        pub remote_status: u8 @ 16..=17,
        pub dest_shorthand: u8 @ 18..=19,
        pub destination: u32 @ 32..=63,
    }
}

#[derive(Debug, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum MessageType {
    Fixed = 0b000,
    LowPriority = 0b001,
    SMI = 0b010,
    Read = 0b011,
    NMI = 0b100,
    Init = 0b101,
    Startup = 0b110,
    External = 0b111,
}

extern "x86-interrupt" fn page_fault_handler(
    frame: InterruptStackFrame,
    error_code: PageFaultErrorCode,
) {
    println!("Page fault from address {:?} because {:?}", Cr2, error_code);
    loop {}
}

extern "x86-interrupt" fn error(frame: InterruptStackFrame) {
    loop {}
}

extern "x86-interrupt" fn gpf(frame: InterruptStackFrame, id: u64) {
    // println!(
    //     "General protection fault with id: {} and frame {:?}",
    //     id, frame
    // );
    loop {}
}

extern "x86-interrupt" fn error_with_code(_: InterruptStackFrame, id: u64) {
    // println!("Exception with code: {}", id);
    loop {}
}

extern "x86-interrupt" fn div(_: InterruptStackFrame, id: u64) -> ! {
    // println!("General protection fault with id: {}", id);
    loop {}
}

extern "C" fn ap_init() -> ! {
    unsafe {
        asm!("mov cr3, {}", in(reg) core::ptr::from_exposed_addr::<u64>(0xFE0).read(), options(nomem, nostack, preserves_flags));
    }

    ap_init_rs();
}

// TODO: Passing around allocators is pretty good tbh, but keeping them locked for so awfully long
// is very bad
fn ap_init_rs() -> ! {
    let apic_id = unsafe {
        (crate::phys_offset() + 0xFE8 as u64)
            .as_mut_ptr::<u64>()
            .read()
    };

    unsafe {
        x86_64::registers::control::Cr4::update(|x| {
            x.set(x86_64::registers::control::Cr4Flags::FSGSBASE, true)
        });
    }

    setup_cpu_local(ALLOC.lock().as_mut().unwrap(), apic_id as usize);

    println!("Cpu with apic id {apic_id}: Started");

    init_interrupts(ALLOC.lock().as_mut().unwrap(), apic_id as usize);
    println!("Cpu with apic id {apic_id}: Enabled x2Apic, interrupts");

    init_runtime();

    loop {}
}

// TODO: Turn into arena to make reusable (obviously important lel)
static STACK_CNT: AtomicUsize = AtomicUsize::new(0);

pub fn alloc_stack(allocator: &mut crate::frame::Allocator) -> u64 {
    let base = 0x30000000000;
    let idx = STACK_CNT.fetch_add(1, core::sync::atomic::Ordering::AcqRel);

    let mut i = 0;
    while i < 0x20000 {
        i += 1;
        let frame = allocator.allocate().unwrap();
        crate::map_page(
            allocator,
            x86_64::VirtAddr::new((base + idx * 0x20000000 + i * 0x1000) as u64),
            frame,
        );
    }

    (base + (idx + 1) * 0x20000000) as u64
}

fn enable_x2apic() {
    let mut apic_reg = Msr::new(0x0000001B);
    unsafe {
        apic_reg.write(ApicMsr(apic_reg.read()).with_enable_x2apic(true).into());
    }

    let mut spurious_vec_reg = Msr::new(0x80F);
    unsafe {
        spurious_vec_reg.write(LocalVec(0).with_vec(125).with_software_enable(true).into());
    }

    // Reset timer initial, as the timer may still be running in periodic mode even after being
    // cycled and will fire when setting the timer vec
    let mut timer_initial_reg = Msr::new(0x838);
    unsafe {
        timer_initial_reg.write(0);
    }

    let mut error_reg = Msr::new(0x828);
    unsafe {
        error_reg.write(0);
    }
}

extern "C" {
    fn switch_ctx();
    fn create_lmao(_: u64);
}

#[no_mangle]
extern "C" fn lmao() -> ! {
    lmao_rs()
}

fn lmao_rs() -> ! {
    println!("We in thread 2 bby");
    loop {}
}

#[link_section = ".cpu_local"]
static GDT: CpuLocal<OnceCell<GlobalDescriptorTable>> = CpuLocal::new(OnceCell::new());

#[link_section = ".cpu_local"]
static IDT: CpuLocal<OnceCell<InterruptDescriptorTable>> = CpuLocal::new(OnceCell::new());

#[link_section = ".cpu_local"]
static TSS: CpuLocal<OnceCell<TaskStateSegment>> = CpuLocal::new(OnceCell::new());

#[link_section = ".cpu_local"]
static mut SELEC: CpuLocal<SegmentSelector> = CpuLocal::new(SegmentSelector::NULL);

pub fn init_interrupts(allocator: &mut crate::frame::Allocator, id: usize) {
    // TODO: Be able to register from somewhere else

    let mut idt = InterruptDescriptorTable::new();
    idt.page_fault.set_handler_fn(page_fault_handler);
    idt.divide_error.set_handler_fn(error);
    idt.debug.set_handler_fn(error);
    idt.non_maskable_interrupt.set_handler_fn(error);
    idt.breakpoint.set_handler_fn(error);
    idt.overflow.set_handler_fn(error);
    idt.bound_range_exceeded.set_handler_fn(error);
    idt.invalid_opcode.set_handler_fn(error);
    idt.device_not_available.set_handler_fn(error);
    idt.general_protection_fault.set_handler_fn(gpf);
    idt.double_fault.set_handler_fn(div);
    idt.invalid_tss.set_handler_fn(gpf);
    idt.segment_not_present.set_handler_fn(gpf);
    idt.stack_segment_fault.set_handler_fn(gpf);
    idt.x87_floating_point.set_handler_fn(error);
    idt.alignment_check.set_handler_fn(gpf);

    // IDT.invalid_opcode.set_handler_addr(x86_64::VirtAddr::new(
    //     switch_ctx as *const extern "C" fn() as u64,
    // ));

    // TODO: Figure out whether this is reused by virtio
    idt[20].set_handler_fn(crate::xhci::xhci_int);

    unsafe {
        idt[130].set_handler_addr(x86_64::VirtAddr::new(
            switch_ctx as *const extern "C" fn() as u64,
        ));
    }

    IDT.set(idt).unwrap();

    IDT.get().unwrap().load();

    let mut tss = TaskStateSegment::new();
    tss.privilege_stack_table[0] = VirtAddr::new(alloc_stack(allocator));
    TSS.set(tss).unwrap();

    let mut gdt = GlobalDescriptorTable::new();

    // Pad out cuz limine sets it up differently
    gdt.add_entry(x86_64::structures::gdt::Descriptor::kernel_code_segment()); // 1
    let data = gdt.add_entry(x86_64::structures::gdt::Descriptor::kernel_data_segment());
    gdt.add_entry(x86_64::structures::gdt::Descriptor::user_data_segment());
    gdt.add_entry(x86_64::structures::gdt::Descriptor::user_code_segment());

    // Limine's choices
    gdt.add_entry(x86_64::structures::gdt::Descriptor::kernel_code_segment());

    // TODO: unfuck whatever this is doing, cuz it's clearly not good
    // gdt.add_entry(x86_64::structures::gdt::Descriptor::kernel_data_segment());

    let selec = gdt.add_entry(x86_64::structures::gdt::Descriptor::tss_segment(
        &TSS.get().unwrap(),
    ));

    GDT.set(gdt).unwrap();

    GDT.get().unwrap().load();

    // ?????? Sure if it works
    unsafe {
        x86_64::registers::segmentation::SS::set_reg(data);
    }

    unsafe {
        x86_64::instructions::tables::load_tss(selec);
    }
    // println!("Setup the IDT");

    enable_x2apic();
    // println!("Setup the x2Apic");

    x86_64::instructions::interrupts::enable();
    // println!("Enabled interrupts");
}

// TODO: Hide behind api
pub static ALLOC: spin::Mutex<Option<&mut crate::frame::Allocator>> =
    spin::Mutex::<Option<&mut crate::frame::Allocator>>::new(None);

pub fn init_multicore(allocator: &mut crate::frame::Allocator, apics: &[X2Apic]) {
    crate::mm::virt::map_page(allocator, VirtAddr::new(0), PhysAddr::new(0));
    println!("Starting APs");

    // TODO: Reclaim bootloader and put somewhere more safe
    let long_mode = include_bytes!("long_mode.o");
    unsafe {
        core::ptr::copy(
            long_mode.as_ptr(),
            crate::phys_offset().as_mut_ptr::<u8>(),
            4096,
        );
    }

    unsafe {
        (crate::phys_offset() + 0xFF8_u64)
            .as_mut_ptr::<extern "C" fn() -> !>()
            .write(ap_init);
    }

    println!(
        "C43 {}",
        x86_64::registers::control::Cr3::read()
            .0
            .start_address()
            .as_u64()
    );

    unsafe {
        (crate::phys_offset() + 0xFE0_u64)
            .as_mut_ptr::<u64>()
            .write(
                x86_64::registers::control::Cr3::read()
                    .0
                    .start_address()
                    .as_u64(),
            );
    }

    let mut error_reg = Msr::new(0x828);
    let mut ipi = Msr::new(0x830);

    // TODO: Properly yeet invalid apics, preferrably in acpi code, more reliably skip bsp
    for apic in apics.iter().skip(1) {
        if apic.apic_id > 100 {
            continue;
        }
        let init = InterprocessInterrupt(0)
            .with_message_type(MessageType::Init)
            .with_level(true)
            .with_destination(apic.apic_id);

        let init2 = InterprocessInterrupt(0)
            .with_message_type(MessageType::Init)
            .with_destination(apic.apic_id);

        let startup = InterprocessInterrupt(0)
            .with_message_type(MessageType::Startup)
            .with_level(true)
            .with_destination(apic.apic_id)
            .with_vec(0);

        unsafe {
            let stack_addr = alloc_stack(allocator);
            (crate::phys_offset() + 0xFF0_u64)
                .as_mut_ptr::<u64>()
                .write(stack_addr);

            (crate::phys_offset() + 0xFE8_u64)
                .as_mut_ptr::<u64>()
                .write(apic.apic_id as u64);

            error_reg.write(0);
            ipi.write(init.into());
            ipi.write(init2.into());

            error_reg.write(0);
            ipi.write(startup.into());

            println!("Bsp started cpu with apic id {}", apic.apic_id);
        }
    }
    println!("Enabled all cpus");
}

pub fn init_runtime() {
    let mut timer_vec: LocalVec = 0.into();
    // timer_vec.set_timer_mode(true);
    timer_vec.set_vec(130);
    timer_vec.set_mask(false);
    let mut timer_vec_reg = Msr::new(0x832);
    unsafe {
        timer_vec_reg.write(timer_vec.into());
    }
    // println!("Got past timer vec");

    let mut divide = Msr::new(0x83E);
    unsafe {
        divide.write(0);
    }
    // println!("Got past divider");

    // 1 Secs at 4 Ghz
    let timer_initial: u64 = 4000000000;
    let mut timer_initial_reg = Msr::new(0x838);
    unsafe {
        timer_initial_reg.write(timer_initial);
    }
    // println!("Got past initial timer");

    // let mut timer_current = Msr::new(0x839);
    // let current = unsafe { timer_current.read() };
    // println!("{}", current);
}
