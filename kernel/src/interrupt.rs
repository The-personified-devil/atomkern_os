use crate::pcie;
use crate::println;
use crate::proc::Process;
use arrayvec::ArrayVec;
use bytemuck::TransparentWrapper;
use core::arch::asm;
use core::mem::transmute;
use core::ptr::addr_of;
use core::slice;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use proc_bitfield::bitfield;
use snafu::prelude::*;
use x86_64::registers::control::Cr2;
use x86_64::registers::model_specific::Msr;
use x86_64::structures::gdt::GlobalDescriptorTable;
use x86_64::structures::gdt::SegmentSelector;
use x86_64::structures::idt::*;
use x86_64::structures::tss::TaskStateSegment;
use x86_64::PhysAddr;
use x86_64::VirtAddr;

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
    // println!("Page fault from address {:?} because {:?}", Cr2, error_code);
    loop {}
}

extern "x86-interrupt" fn error(frame: InterruptStackFrame) {
    loop {}
}

extern "x86-interrupt" fn gpf(frame: InterruptStackFrame, id: u64) {
    println!(
        "General protection fault with id: {} and frame {:?}",
        id, frame
    );
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

static PROCS: spin::Mutex<ArrayVec<Process, 200>> =
    spin::Mutex::new(ArrayVec::<Process, 200>::new_const());

pub fn create_proc(allocator: &mut crate::frame::Allocator, cr3: PhysAddr, entry: u64) {
    let id;
    {
        let mut procs = PROCS.lock();
        procs.push(Process {
            rflags: 0,
            rsp: alloc_stack(allocator, 30),
            rip: entry,
        cs: 0x23,
        ss: 0x1B,
            regs: [0; 14],
            cr3: cr3.as_u64(),
        });

        id = procs.len() as u64 - 1;
    }

    PROC_QUEUE.lock().put(id);
}

static PROC_QUEUE: spin::Mutex<LoopedQueue> = spin::Mutex::<_>::new(LoopedQueue::new());

pub fn get_cr3() -> u64 {
    let value;

    unsafe {
        asm!("mov {}, cr3", out(reg) value, options(nomem, nostack, preserves_flags));
    }

    value
}

#[no_mangle]
extern "C" fn determine_next_proc(regs: [u64; 19]) -> *const Process {
    println!("regs: {:?}", regs);

    let gs: u64;
    unsafe {
        asm!("mov {}, gs", out(reg) gs);
    }

    let mut binding = PROCS.lock();
    let procs = binding.as_mut();

    let proc = &mut procs[gs as usize];
    let value: u64;

    *proc = Process {
        rip: regs[14],
        cs: regs[15],
        rflags: regs[16],
        rsp: regs[17],
        ss: regs[18],
        regs: regs[0..14].try_into().unwrap(),
        cr3: get_cr3(),
    };

    println!("proc_queue {:?}", PROC_QUEUE.lock());

    // Have 0 be the Os hlt thread (or currently just spin)
    // For now this will automatically happen because we only switch to userspace on the first
    // timer interrupt
    let new = PROC_QUEUE.lock().take().unwrap_or(0);
    println!("new {}", new);

    unsafe {
        asm!("mov gs, {}", in(reg) new);
    }

    let proc2 = &mut procs[new as usize];

    // TODO: Don't clone rflags here that's bs
    proc2.rflags = regs[16];

    println!("determine_next_proc");

    let timer_initial: u64 = 4000000000;
    let mut timer_initial_reg = Msr::new(0x838);
    unsafe {
        timer_initial_reg.write(timer_initial);
    }

    println!("proc2: {:?}", proc2);
    // Save because only this cpu should be accessing tcb data at this point
    addr_of!(*proc2)
}

extern "C" fn ap_init() -> ! {
    ap_init_rs();
}

fn ap_init_rs() -> ! {
    let apic_id = unsafe {
        (crate::phys_offset() + 0xFE8 as u64)
            .as_mut_ptr::<u64>()
            .read()
    };

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

    idt[20].set_handler_fn(crate::xhci::xhci_int);

    unsafe {
        idt[130].set_handler_addr(x86_64::VirtAddr::new(
            switch_ctx as *const extern "C" fn() as u64,
        ));

        idt.load_unsafe();
    }

    let mut tss = TaskStateSegment::new();

    tss.privilege_stack_table[0] = VirtAddr::new(alloc_stack(
        *ALLOC.lock().as_mut().unwrap(),
        32 + apic_id as usize,
    ));

    let mut gdt = GlobalDescriptorTable::new();
    let selec = gdt.add_entry(x86_64::structures::gdt::Descriptor::tss_segment(unsafe {
        transmute::<_, &'static TaskStateSegment>(&tss)
    }));

    unsafe {
        gdt.load_unsafe();

        x86_64::instructions::tables::load_tss(selec);
    }

    println!("Cpu with apic id {apic_id}: Started");
    enable_x2apic();
    x86_64::instructions::interrupts::enable();

    println!("Cpu with apic id {apic_id}: Enabled x2Apic, interrupts");

    init_runtime();

    loop {}
}

pub fn alloc_stack(allocator: &mut crate::frame::Allocator, idx: usize) -> u64 {
    let base = 0x30000000000;

    let mut i = 0;
    while i < 0x2000 {
        i += 1;
        let frame = allocator.allocate().unwrap();
        crate::map_page(
            allocator,
            x86_64::VirtAddr::new((base + idx * 0x2000000 + i * 0x1000) as u64),
            frame,
        );
    }

    (base + (idx + 1) * 0x2000000) as u64
}

fn enable_x2apic() {
    let mut apic_reg = Msr::new(0x0000001B);
    unsafe {
        apic_reg.write(
            ApicMsr(unsafe { apic_reg.read() })
                .with_enable_x2apic(true)
                .into(),
        );
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

static mut GDT: GlobalDescriptorTable = {
    let mut gdt = GlobalDescriptorTable::new();

    // Pad out cuz limine sets it up differently
    gdt.add_entry(x86_64::structures::gdt::Descriptor::kernel_code_segment()); // 1
    gdt.add_entry(x86_64::structures::gdt::Descriptor::kernel_data_segment());
    gdt.add_entry(x86_64::structures::gdt::Descriptor::user_data_segment());
    gdt.add_entry(x86_64::structures::gdt::Descriptor::user_code_segment());

    // Limine's choices
    gdt.add_entry(x86_64::structures::gdt::Descriptor::kernel_code_segment());
    // gdt.add_entry(x86_64::structures::gdt::Descriptor::kernel_data_segment());

    gdt
};

static mut IDT: InterruptDescriptorTable = InterruptDescriptorTable::new();

static mut TSS: TaskStateSegment = TaskStateSegment::new();

pub fn init_interrupts() {
    // TODO: Be able to register from somewhere else

    // Unsafe cuz static mut, but otherwise valid, except sync
    unsafe {
        IDT.page_fault.set_handler_fn(page_fault_handler);
        IDT.divide_error.set_handler_fn(error);
        IDT.debug.set_handler_fn(error);
        IDT.non_maskable_interrupt.set_handler_fn(error);
        IDT.breakpoint.set_handler_fn(error);
        IDT.overflow.set_handler_fn(error);
        IDT.bound_range_exceeded.set_handler_fn(error);
        IDT.invalid_opcode.set_handler_fn(error);
        IDT.device_not_available.set_handler_fn(error);
        IDT.general_protection_fault.set_handler_fn(gpf);
        IDT.double_fault.set_handler_fn(div);
        IDT.invalid_tss.set_handler_fn(gpf);
        IDT.segment_not_present.set_handler_fn(gpf);
        IDT.stack_segment_fault.set_handler_fn(gpf);
        IDT.x87_floating_point.set_handler_fn(error);
        IDT.alignment_check.set_handler_fn(gpf);

        // IDT.invalid_opcode.set_handler_addr(x86_64::VirtAddr::new(
        //     switch_ctx as *const extern "C" fn() as u64,
        // ));

        IDT[20].set_handler_fn(crate::xhci::xhci_int);

        IDT[130].set_handler_addr(x86_64::VirtAddr::new(
            switch_ctx as *const extern "C" fn() as u64,
        ));

        IDT.load();
    }
    println!("Setup the IDT");

    enable_x2apic();
    println!("Setup the x2Apic");

    x86_64::instructions::interrupts::enable();
    println!("Enabled interrupts");

    // TODO: Major bullshit, but fuck it
    PROCS.lock().push(Process {
        rflags: 0,
        rsp: 0,
        rip: lmao as *const extern "C" fn() -> ! as u64,
        cs: 0x23,
        ss: 0x1B,
        regs: [0; 14],
        cr3: get_cr3(),
    });
}

pub static ALLOC: spin::Mutex<Option<&mut crate::frame::Allocator>> =
    spin::Mutex::<Option<&mut crate::frame::Allocator>>::new(None);

pub fn init_multicore(allocator: &mut crate::frame::Allocator, apics: &[X2Apic]) {
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

    let mut error_reg = Msr::new(0x828);
    let mut ipi = Msr::new(0x830);

    // TODO: Properly yeet invalid apics, preferrably in acpi code, more reliably skip bsp
    for apic in apics.iter().skip(1) {
        if apic.apic_id > 100 {
            continue;
        }
        let mut init = InterprocessInterrupt(0)
            .with_message_type(MessageType::Init)
            .with_level(true)
            .with_destination(apic.apic_id);

        let mut init2 = InterprocessInterrupt(0)
            .with_message_type(MessageType::Init)
            .with_destination(apic.apic_id);

        let mut startup = InterprocessInterrupt(0)
            .with_message_type(MessageType::Startup)
            .with_level(true)
            .with_destination(apic.apic_id)
            .with_vec(0);

        unsafe {
            let stack_addr = alloc_stack(allocator, apic.apic_id as usize);
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

    unsafe {
        TSS.privilege_stack_table[0] = VirtAddr::new(alloc_stack(allocator, 32));

        let selec = GDT.add_entry(x86_64::structures::gdt::Descriptor::tss_segment(&TSS));

        GDT.load();

        x86_64::instructions::tables::load_tss(selec);
    }

    PROCS.lock().push(Process {
        rflags: 0,
        rsp: alloc_stack(allocator, 30),
        rip: unsafe { lmao as *const extern "C" fn() -> ! as u64 },
        cs: 0x23,
        ss: 0x1B,
        regs: [0; 14],
        cr3: get_cr3(),
    });

    PROCS.lock().push(Process {
        rflags: 0,
        rsp: alloc_stack(allocator, 31),
        rip: unsafe { lmao as *const extern "C" fn() -> ! as u64 },
        cs: 0x23,
        ss: 0x1B,
        regs: [0; 14],
        cr3: get_cr3(),
    });
}

pub fn init_runtime() {
    {
        let mut alloc = ALLOC.lock();
        let allocator = alloc.as_mut().unwrap();

        unsafe {
            TSS.privilege_stack_table[0] = VirtAddr::new(alloc_stack(allocator, 32));

            let selec = GDT.add_entry(x86_64::structures::gdt::Descriptor::tss_segment(&TSS));

            GDT.load();
            // SS not used in ring 0, so we disappoint limine
            // x86_64::registers::segmentation::SS

            x86_64::instructions::tables::load_tss(selec);
        }

        PROCS.lock().push(Process {
            rflags: 0,
            rsp: alloc_stack(allocator, 30),
            rip: lmao as *const extern "C" fn() -> ! as u64,
        cs: 0x23,
        ss: 0x1B,
            regs: [0; 14],
            cr3: get_cr3(),
        });

        PROCS.lock().push(Process {
            rflags: 0,
            rsp: alloc_stack(allocator, 31),
            rip: lmao as *const extern "C" fn() -> ! as u64,
        cs: 0x23,
        ss: 0x1B,
            regs: [0; 14],
            cr3: get_cr3(),
        });

        unsafe {
            asm!("mov gs, {:r}", in(reg) 0);
        }
        PROC_QUEUE.lock().put(1);
    }

    let mut timer_vec: LocalVec = 0.into();
    // timer_vec.set_timer_mode(true);
    timer_vec.set_vec(130);
    timer_vec.set_mask(false);
    let mut timer_vec_reg = Msr::new(0x832);
    unsafe {
        timer_vec_reg.write(timer_vec.into());
    }
    println!("Got past timer vec");

    let mut divide = Msr::new(0x83E);
    unsafe {
        divide.write(0);
    }
    println!("Got past divider");

    // 1 Secs at 4 Ghz
    let timer_initial: u64 = 4000000000;
    let mut timer_initial_reg = Msr::new(0x838);
    unsafe {
        timer_initial_reg.write(timer_initial);
    }
    println!("Got past initial timer");

    let mut timer_current = Msr::new(0x839);
    let current = unsafe { timer_current.read() };
    println!("{}", current);

    let mut timer_current = Msr::new(0x839);
    let current = unsafe { timer_current.read() };
    println!("{}", current);

    loop {}
}

#[derive(Debug)]
struct LoopedQueue {
    arr: [u64; 4096],
    cur: usize,
    last: usize,
}

impl LoopedQueue {
    const fn new() -> Self {
        Self {
            arr: [0; 4096],
            cur: 0,
            last: 0,
        }
    }

    fn put(&mut self, t: u64) {
        self.arr[self.cur] = t;
        self.cur = (self.cur + 1) % 4096;
    }

    fn take(&mut self) -> Option<u64> {
        if self.cur == self.last {
            return None;
        }

        let val = self.arr[self.last];
        self.last = (self.last + 1) % 4096;
        Some(val)
    }
}
