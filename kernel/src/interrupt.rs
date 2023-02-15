use crate::println;
use arrayvec::ArrayVec;
use core::arch::asm;
use core::prelude::*;
use core::slice;
use nom::branch::alt;
use nom::bytes::complete::{tag, take};
use nom::combinator::{map, not};
use nom::number::complete::{le_u32, le_u8};
use nom::sequence::{preceded, tuple};
use nom::IResult;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use proc_bitfield::bitfield;
use x86_64::instructions::tables::{sgdt, sidt};
use x86_64::registers::control::Cr2;
use x86_64::registers::model_specific::Msr;
use x86_64::structures::{gdt::*, idt::*};

// #[link(name = "shit")]
// extern "C" {
//     #[naked]
//     fn SwitchToLongMode();
// }

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
}

extern "x86-interrupt" fn error(_: InterruptStackFrame) {
    // println!("Startup lmao");
    loop {}
}
extern "x86-interrupt" fn gpf(_: InterruptStackFrame, id: u64) {
    // println!("General protection fault with id: {}", id);
    loop {}
}

extern "x86-interrupt" fn div(_: InterruptStackFrame, id: u64) -> ! {
    // println!("General protection fault with id: {}", id);
    loop {}
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

pub fn acpi(rdsp_addr: u64) -> ArrayVec<X2Apic, 256> {
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
        let sum = unsafe { slice::from_raw_parts(ptr as *const u8, (length + 0) as usize) }
            .iter()
            .fold(0 as u64, |acc, x| acc + (*x) as u64);
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
    println!("{}", length);

    // tables.iter().for_each(|x| {
    //     println!("{:?}", unsafe {
    //         &*(crate::PHYS_OFFSET + *x).as_ptr::<SystemDescriptionHeader>()
    //     });
    // });
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
    ParseReturn::X2Apic(X2Apic {
        processor_id: data.0.into(),
        flags: data.2,
        apic_id: data.1.into(),
    })
}

fn to_x2apic(data: (u32, u32, u32)) -> ParseReturn {
    ParseReturn::X2Apic(X2Apic {
        processor_id: data.0,
        flags: data.1,
        apic_id: data.2,
    })
}
enum ParseReturn {
    // Apic(Apic),
    X2Apic(X2Apic),
    Skip(u8),
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
    idt.invalid_tss.set_handler_fn(gpf);
    idt.segment_not_present.set_handler_fn(gpf);
    idt.stack_segment_fault.set_handler_fn(gpf);
    idt.stack_segment_fault.set_handler_fn(gpf);
    idt.x87_floating_point.set_handler_fn(error);
    idt.alignment_check.set_handler_fn(gpf);
    unsafe {
        idt.load_unsafe();
    }
    let mut a: u64 = 15000000;
    while a > 0 {
        a -= 1;
        unsafe {
        asm!("pause");
        }
    }
    println!("lmao");
    println!("lmao");
    loop {}
}

pub fn alloc_stack(allocator: &mut crate::frame::Allocator, idx: usize) -> u64 {
    let base = 0x30000000000;

    let mut i = 0;
    while i < 0x2000 {
        i += 1;
        let frame = allocator.allocate().unwrap();
        // println!("{}, {}, {}", base, idx * 0x2000000 , i * 0x1000);
        // println!("{}", (base + idx * 0x3000000 + i * 0x1000));
            // println!("{:?}", x86_64::VirtAddr::try_new((base + idx * 0x2000000 + i * 0x1000) as u64));
        crate::map_page(
            allocator,
            x86_64::VirtAddr::new((base + idx * 0x2000000 + i * 0x1000) as u64),
            frame,
        );
        // println!("eyo wtf");
    }

    (base + (idx + 1) * 0x2000000) as u64
}

pub fn init(allocator: &mut crate::frame::Allocator, rdsp_addr: u64, page_addr: u64) {
    let apics = acpi(rdsp_addr);
    // TODO: Make static, this shit lives shorter than ur mom

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
    unsafe {
        idt.load_unsafe();
    }

    // Effectively an infinite loop, cuz the cpu just resumes it's prior doings and we still
    // haven't mapped 0x0 (which makes a lot of sense, lmao)
    // let ptr: *const i32 = core::ptr::null();
    // let val = unsafe {*ptr};
    // println!(
    //     "{:?}, {:?}",
    //     unsafe { &*(sgdt().base.as_ptr() as *const GlobalDescriptorTable) },
    //     unsafe { &*(sidt().base.as_ptr() as *const InterruptDescriptorTable) }
    // );
    let mut apic_reg = Msr::new(0x0000001B);
    let mut reg: ApicMsr = unsafe { apic_reg.read() }.into();
    reg.set_enable(true);
    unsafe {
        apic_reg.write(reg.into());
    }

    let mut reg: ApicMsr = unsafe { apic_reg.read() }.into();
    reg.set_enable_x2apic(true);
    unsafe {
        apic_reg.write(reg.into());
    }
    // let base = (crate::PHYS_OFFSET + reg.base()).as_mut_ptr::<u64>();

    let mut reg: ApicMsr = unsafe { apic_reg.read() }.into();

    let mut spurious_vec: LocalVec = 0.into();
    spurious_vec.set_vec(125);
    spurious_vec.set_software_enable(true);
    // unsafe {
    //     base.byte_add(0xF0).write(spurious_vec.into());
    // }
    let mut spurious_vec_reg = Msr::new(0x80F);
    unsafe {
        spurious_vec_reg.write(Into::<u64>::into(spurious_vec));
    }
    let mut error_reg = Msr::new(0x828);
    unsafe {
        error_reg.write(0);
    }

    // let mut timer_vec: LocalVec = 0.into();
    // // timer_vec.set_message_type(0b100);
    // timer_vec.set_timer_mode(true);
    // timer_vec.set_vec(255);
    // let mut timer_vec_reg = Msr::new(0x832);
    // unsafe {
    //     timer_vec_reg.write(timer_vec.into());
    // }
    // let timer_initial: u64 = 200;
    // let mut timer_initial_reg = Msr::new(0x838);
    // unsafe {
    //     timer_initial_reg.write(timer_initial);
    // }
    // let timer_current = Msr::new(0x839);
    // let apic_id = Msr::new(0x802);
    println!("{:?}", InterprocessInterrupt((0 & 0xfff00000) | 0x008500));
    println!(
        "{:?}",
        InterprocessInterrupt(((0 & 0x00ffffff) | (1 << 24) << 32) + (0 & 0xfff00000) | 0x00C500)
    );

    println!("{:?}", x86_64::registers::control::Cr3::read());
    // println!("{}", unsafe { apic_id.read() });
    // println!("{}", unsafe { timer_initial_reg.read() });
    // println!("{}", unsafe { timer_vec_reg.read() });
    // println!("{}", unsafe { spurious_vec_reg.read() });
    println!("{:?}", reg);

    let long_mode = include_bytes!("long_mode.o");
    // This definitely does not do the right thing lmfao
    unsafe {
        core::ptr::copy(
            long_mode.as_ptr(),
            (crate::PHYS_OFFSET + 0x0 as u64).as_mut_ptr::<u8>(),
            4096,
        );
    }
    unsafe {
        (crate::PHYS_OFFSET + 0xFF8 as u64)
            .as_mut_ptr::<extern "C" fn() -> !>()
            .write(ap_init);
    }
    println!("{:?}", x86_64::registers::control::Cr3::read());
    let mut ipi = Msr::new(0x830);
    // TODO: Parse ACPI
    for apic in apics.iter().skip(1) {
        if apic.apic_id > 100 {
            continue;
        }
        let mut init = InterprocessInterrupt(0)
            .with_message_type(MessageType::Init)
            .with_level(true)
            // .with_trigger_mode(true)
            .with_destination(apic.apic_id);

        let mut init2 = InterprocessInterrupt(0)
            .with_message_type(MessageType::Init)
            // .with_trigger_mode(true)
            .with_destination(apic.apic_id);

        let mut startup = InterprocessInterrupt(0)
            .with_message_type(MessageType::Startup)
            .with_level(true)
            .with_destination(apic.apic_id)
            .with_vec(0);

        unsafe {
            println!("alloc stack");
            let stack_addr = alloc_stack(allocator, apic.apic_id as usize);
            (crate::PHYS_OFFSET + 0xFF0 as u64)
                .as_mut_ptr::<u64>()
                .write(stack_addr);
            println!("after");
            // (0xFF8 as *mut u64).write(stack)
            error_reg.write(0);
            // base.byte_add(0x280).write(0);
            // (base.byte_add(0x310) as *mut u32).write_volatile((init.0 >> 32) as u32 );
            // (base.byte_add(0x300) as *mut u32).write_volatile(init.0 as u32);
            ipi.write(init.into());
            // let mut a: u64 = 55000000;
            // while a > 0 {
            //     a -= 1;
            //     unsafe {
            //         asm!("pause");
            //     }
            // }
            // println!("{:?}", InterprocessInterrupt((base.byte_add(0x300) as *mut u32).read_volatile() as u64));
            // (base.byte_add(0x310) as *mut u32).write_volatile((init2.0 >> 32) as u32 );
            // (base.byte_add(0x300) as *mut u32).write_volatile(init2.0 as u32);
            ipi.write(init2.into());
            println!("first_cycle");
            let mut a: u64 = 15000000;
            while a > 0 {
                a -= 1;
                asm!("pause");
            }
            println!("no pause");
            error_reg.write(0);
            // base.byte_add(0x280).write(0);
            // println!("{:?}", InterprocessInterrupt((base.byte_add(0x300) as *mut u32).read_volatile() as u64));
            // println!("llllll");
            // (base.byte_add(0x310) as *mut u32).write_volatile((startup.0 >> 32) as u32 );
            // (base.byte_add(0x300) as *mut u32).write_volatile(startup.0 as u32);
            ipi.write(startup.into());
            println!("second cycle");
            let mut a: u64 = 15000000;
            while a > 0 {
                a -= 1;
                asm!("pause");
            }
            println!("lmao");
        }
    }
    loop {}
}
