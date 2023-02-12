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
