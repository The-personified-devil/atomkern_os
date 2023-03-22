use crate::println;
use core::arch::asm;

#[no_mangle]
extern "C" fn test() {
    println!("Hopefully this is userspace, it might segfault though");
    loop {}
}

pub fn sysret() {
    unsafe {
        asm!(
            // LSTAR
            // "mov rcx, 0xc0000082",
            // "wrmsr",
            
            // enable efer shit
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
        );
    }
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
