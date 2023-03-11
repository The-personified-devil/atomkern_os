use crate::pcie;
use crate::println;
use arrayvec::ArrayVec;
use bytemuck::TransparentWrapper;
use core::slice;
use nom::branch::alt;
use nom::bytes::complete::{tag, take};
use nom::combinator::{map, not};
use nom::number::complete::{le_u32, le_u8};
use nom::sequence::{preceded, tuple};
use nom::IResult;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use proc_bitfield::bitfield;
use snafu::prelude::*;
use x86_64::registers::control::Cr2;
use x86_64::registers::model_specific::Msr;
use x86_64::structures::idt::*;

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
    _: InterruptStackFrame,
    error_code: PageFaultErrorCode,
) {
    println!("Page fault from address {:?} because {:?}", Cr2, error_code);
    loop {}
}

extern "x86-interrupt" fn error(frame: InterruptStackFrame) {
    loop {}
}

extern "x86-interrupt" fn gpf(frame: InterruptStackFrame, id: u64) {
    // println!("General protection fault with id: {}", id);
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

#[no_mangle]
static mut REGS: crate::proc::Registers = crate::proc::Registers {
    rax: 0,
    rbx: 0,
    rcx: 0,
    rdx: 0,
    rsi: 0,
    rdi: 0,
    r8: 0,
    r9: 0,
    r10: 0,
    r11: 0,
    r12: 0,
    r13: 0,
    r14: 0,
    r15: 0,
};

static PROCS: spin::Mutex<Option<ArrayVec<crate::proc::Process, 2>>> = spin::Mutex::new(None);

#[no_mangle]
static mut PROC: crate::proc::Process = unsafe { core::mem::MaybeUninit::zeroed().assume_init() };

static CURPROC: spin::Mutex<usize> = spin::Mutex::new(0);

static STATE: spin::Mutex<u64> = spin::Mutex::new(0);

fn is_valid(id: u8) -> Result<(), TestError> {
    ensure!(id >= 10, TestSnafu { id });
    Ok(())
}

#[no_mangle]
extern "C" fn determine_next_proc() {
    let mut binding = PROCS.lock();
    let procs = binding.as_mut().unwrap();

    let mut cur = CURPROC.lock();
    let mut state = STATE.lock();
    let proc = &mut procs[*cur as usize];

    unsafe {
        *proc = PROC;
        proc.regs = REGS;
    }

    let mut proc;

    if *state == 0 {
        proc = procs[0];
        *state = 1;
        *cur = 1;
    } else if *state == 1 {
        proc = procs[0];
        *cur = 0;
        *state = 2;
    } else {
        if *cur == 1 {
            proc = procs[0];
            *cur = 0;
        } else {
            proc = procs[1];
            *cur = 1;
        }
    }

    println!("determine_next_proc");

    unsafe {
        REGS = proc.regs;
        PROC = proc;
    }

    let timer_initial: u64 = 4000000000;
    let mut timer_initial_reg = Msr::new(0x838);
    unsafe {
        timer_initial_reg.write(timer_initial);
    }
}

#[derive(Clone, Copy, Debug)]
#[repr(C, packed)]
struct RSDP2 {
    pub signature: [u8; 8],
    pub checksum: u8,
    pub oemid: [u8; 6],
    pub revision: u8,
    pub rsdt_addr: u32,

    pub length: u32,
    pub xsdt_addr: u64,
    pub extended_checksum: u8,
    pub reserved: [u8; 3],
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
struct SystemDescriptionHeader {
    pub signature: [u8; 4],
    pub length: u32,
    pub revision: u8,
    pub checksum: u8,
    pub oem_id: [u8; 6],
    pub oem_table_id: [u8; 8],
    pub oem_revision: u32,
    pub creator_id: u32,
    pub creator_revision: u32,
    pub data: [u8; 0],
}

#[derive(Clone, Debug)]
#[repr(C)]
struct ApicHeader {
    pub entry_type: u8,
    pub length: u8,
}

pub fn acpi(allocator: &mut crate::frame::Allocator, rdsp_addr: u64) -> ArrayVec<X2Apic, 256> {
    let rdsp = unsafe { &*((crate::PHYS_OFFSET + rdsp_addr).as_ptr() as *const RSDP2) };
    println!("{:?}", rdsp);

    let xsdt_ptr = (crate::PHYS_OFFSET + rdsp.xsdt_addr).as_ptr() as *const SystemDescriptionHeader;
    let xsdt = unsafe { &*xsdt_ptr };
    println!("{:?}", xsdt);
    println!("{}", core::mem::size_of::<SystemDescriptionHeader>());

    let length = unsafe { core::ptr::addr_of!(xsdt.length).read_unaligned() };
    println!("{:?}", length);

    fn check_signature(ptr: *const SystemDescriptionHeader) {
        let header = unsafe { &*ptr };
        let length = unsafe { core::ptr::addr_of!(header.length).read_unaligned() };
        let sum = unsafe { slice::from_raw_parts(ptr as *const u8, length as usize) }
            .iter()
            .fold(0_u64, |acc, x| acc + (*x) as u64);
        assert_eq!(sum % 0x100, 0);
    }

    // let tables = unsafe {
    //     slice::from_raw_parts(
    //         core::ptr::addr_of!(xsdt.tables) as *const u64,
    //         (length as usize - core::mem::size_of::<SystemDescriptionHeader>()) / 8,
    //     )
    // };
    let mut tables = ArrayVec::<_, 100>::new();
    for offset in (0..length as usize - core::mem::size_of::<SystemDescriptionHeader>()).step_by(8)
    {
        let addr = unsafe {
            ((core::ptr::addr_of!(xsdt.data).addr() + offset) as *const u64).read_unaligned()
        };
        let table = unsafe { &*(crate::PHYS_OFFSET + addr).as_ptr::<SystemDescriptionHeader>() };

        check_signature(table);
        tables.push(table)
    }

    println!("size: {}", core::mem::size_of::<SystemDescriptionHeader>());

    let pcie = tables
        .iter()
        .find(|x| core::str::from_utf8(x.signature.as_slice()).unwrap() == "MCFG")
        .unwrap();

    let data = unsafe {
        slice::from_raw_parts(
            core::ptr::addr_of!(pcie.data).byte_offset(8isize) as *const u8,
            (pcie.length as usize) - core::mem::size_of::<SystemDescriptionHeader>() - 8,
        )
    };
    println!("data: {:?}", data);

    let mut configs = ArrayVec::<PcieRef, 256>::new();
    let mut data = data;
    while data.len() >= core::mem::size_of::<PcieRef>() {
        let config = unsafe { &*(data.as_ptr() as *const PcieRef) };
        configs.push(config.clone());
        data = &data[core::mem::size_of::<PcieRef>()..];
    }
    println!("configs: {:?}", configs);
    pcie::pcie_shenanigans(allocator, configs[0].addr);

    for table in tables.iter() {
        let signature = core::str::from_utf8(table.signature.as_slice()).unwrap();
        println!("{}:, {:?}", signature, table);
    }

    let apic = tables
        .iter()
        .find(|x| core::str::from_utf8(x.signature.as_slice()).unwrap() == "APIC")
        .unwrap();

    let data = unsafe {
        slice::from_raw_parts(
            core::ptr::addr_of!(apic.data).byte_offset(8isize) as *const u8,
            (apic.length as usize) - core::mem::size_of::<SystemDescriptionHeader>(),
        )
    };
    println!("{:?}", data);

    let mut apics = ArrayVec::<X2Apic, 256>::new();
    let mut remainder = data;
    while let Ok((remaining, apic)) = parse(remainder).map_err(|x| println!("{:?}", x)) {
        remainder = remaining;
        apics.push(apic);
    }
    apics.iter().for_each(|x| println!("{:?}", x));

    println!("{:?}", xsdt);
    apics
}

#[derive(Debug)]
pub struct X2Apic {
    pub processor_id: u32,
    pub flags: u32,
    pub apic_id: u32,
}

// #[derive(Debug)]
// struct Apic {
//     pub processor_id: u8,
//     pub flags: u32,
//     pub apic_id: u8,
// }

fn to_apic(data: (u8, u8, u32)) -> ParseReturn {
    println!("to_apic");
    ParseReturn::X2Apic(X2Apic {
        processor_id: data.0.into(),
        flags: data.2,
        apic_id: data.1.into(),
    })
}

fn to_x2apic(data: (u32, u32, u32)) -> ParseReturn {
    println!("to_x2apic");
    ParseReturn::X2Apic(X2Apic {
        processor_id: data.2,
        flags: data.1,
        apic_id: data.0,
    })
}

enum ParseReturn {
    // Apic(Apic),
    X2Apic(X2Apic),
    Skip(u8),
}

#[derive(Clone, Copy, Debug)]
#[repr(C, packed)]
struct PcieRef {
    addr: u64,
    segment_group: u16,
    start_bus: u8,
    end_bus: u8,
}

fn to_skip(data: u8) -> ParseReturn {
    ParseReturn::Skip(data)
}

fn parse(input: &[u8]) -> IResult<&[u8], X2Apic> {
    let entry_type = alt((
        map(
            preceded(
                tuple((tag([9u8, 16]), take(2usize))),
                tuple((le_u32, le_u32, le_u32)),
            ),
            to_x2apic,
        ),
        map(
            preceded(tag([0u8, 8]), tuple((le_u8, le_u8, le_u32))),
            to_apic,
        ),
        map(preceded(not(tag([255u8])), le_u8), to_skip),
    ))(input)?;

    match entry_type {
        // (remaining, ParseReturn::Apic(apic)) => Ok((remaining, apic)),
        (remaining, ParseReturn::X2Apic(apic)) => Ok((remaining, apic)),
        (remaining, ParseReturn::Skip(skip)) => {
            parse(take(skip.checked_sub(2).unwrap_or(0))(remaining)?.0)
        }
    }
}

extern "C" fn ap_init() -> ! {
    ap_init_rs();
}

fn ap_init_rs() -> ! {
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
    idt.invalid_tss.set_handler_fn(error_with_code);
    idt.segment_not_present.set_handler_fn(error_with_code);
    idt.stack_segment_fault.set_handler_fn(error_with_code);
    idt.stack_segment_fault.set_handler_fn(error_with_code);
    idt.x87_floating_point.set_handler_fn(error);
    idt.alignment_check.set_handler_fn(error_with_code);
    unsafe {
        idt.load_unsafe();
    }

    let apic_id = unsafe {
        (crate::PHYS_OFFSET + 0xFE8 as u64)
            .as_mut_ptr::<u64>()
            .read()
    };

    println!("Cpu with apic id {apic_id}: Started");
    enable_x2apic();
    println!("Cpu with apic id {apic_id}: Enabled x2Apic");

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

    // The lapic might have been started by the bios, so we disable it first to put it into a more
    // well known state
    unsafe {
        apic_reg.write(
            ApicMsr(unsafe { apic_reg.read() })
                .with_enable(false)
                .with_enable_x2apic(false)
                .into(),
        );
    }
    unsafe {
        apic_reg.write(ApicMsr(unsafe { apic_reg.read() }).with_enable(true).into());
    }

    println!("Enabled apic");

    unsafe {
        apic_reg.write(
            ApicMsr(unsafe { apic_reg.read() })
                .with_enable_x2apic(true)
                .into(),
        );
    }
    println!("{:?}", ApicMsr(unsafe { apic_reg.read() }));
    println!("Enabled x2Apic");

    let mut spurious_vec_reg = Msr::new(0x80F);
    unsafe {
        spurious_vec_reg.write(LocalVec(0).with_vec(125).with_software_enable(true).into());
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
    loop {
        x86_64::instructions::interrupts::without_interrupts(|| println!("We in thread 2 bby"));
    }
}

pub fn init(allocator: &mut crate::frame::Allocator, rdsp_addr: u64, page_addr: u64) {
    *PROCS.lock() = Some(ArrayVec::new());
    println!("survived first one");
    PROCS.lock().as_mut().unwrap().push(crate::proc::Process {
        rflags: 0,
        rsp: 0,
        rip: 0,
        regs: crate::proc::Registers::default(),
    });

    PROCS.lock().as_mut().unwrap().push(crate::proc::Process {
        rflags: 0,
        rsp: 0,
        rip: unsafe { lmao as *const extern "C" fn() -> ! as u64 },
        regs: crate::proc::Registers::default(),
    });

    println!("Got to idt setup");
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
    idt.stack_segment_fault.set_handler_fn(gpf);
    idt.x87_floating_point.set_handler_fn(error);
    idt.alignment_check.set_handler_fn(gpf);
    idt[20].set_handler_fn(crate::pcie::xhci_int);
    unsafe {
        idt[130].set_handler_addr(x86_64::VirtAddr::new(
            switch_ctx as *const extern "C" fn() as u64,
        ));
    }
    unsafe {
        idt.load_unsafe();
    }
    println!("Got past idt setup");

    enable_x2apic();
    println!("Got past x2apic setup");

    x86_64::instructions::interrupts::enable();
    println!("Enabled interrupts");

    // Reset timer initial, as the timer may still be running in periodic mode even after being
    // cycled and will fire when setting the timer vec
    let mut timer_initial_reg = Msr::new(0x838);
    unsafe {
        timer_initial_reg.write(0);
    }

    let apics = acpi(allocator, rdsp_addr);
    // TODO: Make static, this shit lives shorter than ur mom

    let mut error_reg = Msr::new(0x828);
    unsafe {
        error_reg.write(0);
    }

    // let apic_id = Msr::new(0x802);
    // println!("{:?}", reg);

    println!("\nStarting APs");

    // TODO: Reclaim bootloader and put somewhere more safe
    let long_mode = include_bytes!("long_mode.o");
    unsafe {
        core::ptr::copy(
            long_mode.as_ptr(),
            crate::PHYS_OFFSET.as_mut_ptr::<u8>(),
            4096,
        );
    }
    unsafe {
        (crate::PHYS_OFFSET + 0xFF8_u64)
            .as_mut_ptr::<extern "C" fn() -> !>()
            .write(ap_init);
    }

    let mut ipi = Msr::new(0x830);
    // TODO: Parse ACPI
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
            (crate::PHYS_OFFSET + 0xFF0_u64)
                .as_mut_ptr::<u64>()
                .write(stack_addr);

            (crate::PHYS_OFFSET + 0xFE8_u64)
                .as_mut_ptr::<u64>()
                .write(apic.apic_id as u64);

            error_reg.write(0);
            ipi.write(init.into());
            ipi.write(init2.into());

            error_reg.write(0);
            ipi.write(startup.into());

            // let mut a: u64 = 15000000;
            // while a > 0 {
            //     a -= 1;
            //     asm!("pause");
            // }

            // for i in 0..15000000 {
            //     asm!("pause");
            // }
            // println!("Bsp started cpu with apic id {}", apic.apic_id);
        }
    }
    println!("Enabled all cpus");

    x86_64::instructions::interrupts::enable();
    println!("Enabled interrupts");

    // Reset timer initial, as the timer may still be running in periodic mode even after being
    // cycled and will fire when setting the timer vec
    let mut timer_initial_reg = Msr::new(0x838);
    unsafe {
        timer_initial_reg.write(0);
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

    loop {
        let mut state;
        {
            x86_64::instructions::interrupts::disable();
            state = *STATE.lock();
            unsafe {
                STATE.force_unlock();
            }
            x86_64::instructions::interrupts::enable();
        }
        if state == 1 {
            unsafe {
                create_lmao(alloc_stack(allocator, 22));
            }
        }
        x86_64::instructions::interrupts::without_interrupts(|| println!("Main thread bb"));
    }
}

// let base = (crate::PHYS_OFFSET + reg.base()).as_mut_ptr::<u64>();
// unsafe {
//     base.byte_add(0xF0).write(spurious_vec.into());
// }
// (0xFF8 as *mut u64).write(stack)
// base.byte_add(0x280).write(0);
// (base.byte_add(0x310) as *mut u32).write_volatile((init.0 >> 32) as u32 );
// (base.byte_add(0x300) as *mut u32).write_volatile(init.0 as u32);
// println!("{:?}", InterprocessInterrupt((base.byte_add(0x300) as *mut u32).read_volatile() as u64));
// (base.byte_add(0x310) as *mut u32).write_volatile((init2.0 >> 32) as u32 );
// (base.byte_add(0x300) as *mut u32).write_volatile(init2.0 as u32);
// base.byte_add(0x280).write(0);
// println!("{:?}", InterprocessInterrupt((base.byte_add(0x300) as *mut u32).read_volatile() as u64));
// println!("llllll");
// (base.byte_add(0x310) as *mut u32).write_volatile((startup.0 >> 32) as u32 );
// (base.byte_add(0x300) as *mut u32).write_volatile(startup.0 as u32);
