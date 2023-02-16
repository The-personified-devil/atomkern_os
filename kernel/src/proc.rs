use core::arch::asm;
use crate::println;

#[no_mangle]
extern "C" fn test() {
    println!("Hopefully this is userspace, it might segfault though");
    loop{}
}

pub fn sysret() {
    unsafe {
        asm!(
            "mov rcx, 0xc0000082",
            "wrmsr",
            "mov rcx, 0xc0000080",
            "rdmsr",
            "or eax, 1",
            "wrmsr",
            "mov rcx, 0xc0000081",
            "rdmsr",
            "mov edx, 0x00180008",
            "wrmsr",
            // sym test
        );
        asm!(
            "mov r11, 0x202",
            "sysretq",
            in("rcx") test

    )
    }
}

#[derive(Default, Debug, Copy, Clone)]
#[repr(C)]
pub struct Registers {
    pub rax: u64,
    pub rbx: u64,
    pub rcx: u64,
    pub rdx: u64,
    pub rsi: u64,
    pub rdi: u64,
    pub r8: u64,
    pub r9: u64,
    pub r10: u64,
    pub r11: u64,
    pub r12: u64,
    pub r13: u64,
    pub r14: u64,
    pub r15: u64,
}

#[derive(Default, Debug, Copy, Clone)]
#[repr(C)]
pub struct Process {
    pub rflags: u64,
    pub rsp: u64,
    pub rip: u64,
    pub regs: Registers,
    // pub cr3: x86_64::PhysAddr,
}
