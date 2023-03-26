use x86_64::{
    registers::model_specific::{self, Efer, EferFlags},
    structures::gdt::SegmentSelector,
    PrivilegeLevel, VirtAddr, PhysAddr,
};
use num_enum::TryFromPrimitive;

use crate::{println, mm::virt::MapFlags};
use core::arch::asm;

#[no_mangle]
extern "C" fn test() {
    println!("Hopefully this is userspace, it might segfault though");
    loop {}
}

extern "C" {
    fn syscall_handler();
}

#[derive(Debug, TryFromPrimitive)]
#[repr(u64)]
enum Syscall {
    Alloc = 2,
}

#[derive(Debug, TryFromPrimitive)]
#[repr(u64)]
enum PageType {
    Normal = 0,
    Large = 1,
    Huge = 2,
}

#[no_mangle]
extern "C" fn syscall_handler_rs(header: u64, param1: u64, param2: u64, rip: u64, param4: u64, param5: u64) {
    println!("Syscall done {header}");
    if (header & 0b1) != 0 {
        println!("rpc not supported yet");
        return;
    }
    let toipe = header & !0b1;
    // println!("{header} {param1} {param2} {param3} {param4} {param5}");

    match toipe.try_into().unwrap() {
        Syscall::Alloc => {
            alloc_memory_user(param1.try_into().unwrap(), VirtAddr::new(param2), param4);
        }
    } 
    println!("leaving syscall");
}

// Just assume cr3 is still right, as we call it from a context not changing cr3
fn alloc_memory_user(meta: PageType, addr: VirtAddr, page_count: u64) {
    println!("in alloc");
    match meta {
        PageType::Normal => {
            // TODO: Security check

            let mut alloc = crate::interrupt::ALLOC.lock();
            let alloc = alloc.as_mut().unwrap();
            let page_table = PhysAddr::new(crate::interrupt::get_cr3());
            println!("page_count {page_count}");

            for i in 0..page_count {
                println!("{i}");
                let virt_addr = addr + 0x1000 * i;
                let phys_addr = alloc.allocate().unwrap();
                let flags = MapFlags::User | MapFlags::Writable;
                crate::mm::virt::proper_map_page(alloc, page_table,virt_addr, phys_addr, flags);
            }
        }
        _ => todo!(),
    }
}

pub fn sysret() {
    unsafe {
        asm!(
            "mov r11, 0x202",
            "sysretq",
            in("rcx") test
        );
    }
}

pub fn setup_syscalls() {
    // Enable syscall/sysret
    unsafe { Efer::update(|x| x.set(EferFlags::SYSTEM_CALL_EXTENSIONS, true)) };

    model_specific::LStar::write(x86_64::VirtAddr::new(syscall_handler as u64));
    model_specific::Star::write(
        SegmentSelector::new(4, PrivilegeLevel::Ring3),
        SegmentSelector::new(3, PrivilegeLevel::Ring3),
        SegmentSelector::new(1, PrivilegeLevel::Ring0),
        SegmentSelector::new(2, PrivilegeLevel::Ring0),
    )
    .unwrap();
}

#[derive(Default, Debug, Copy, Clone)]
#[repr(C)]
pub struct Registers {
    pub regs: [u64; 14],
}

#[derive(Default, Debug, Copy, Clone)]
#[repr(C)]
pub struct Process {
    pub ss: u64,
    pub rsp: u64,
    pub rflags: u64,
    pub cs: u64,
    pub rip: u64,
    pub regs: [u64; 14],
    pub cr3: u64,
}
