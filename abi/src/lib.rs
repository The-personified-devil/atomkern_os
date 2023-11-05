#![no_std]

extern "C" {
    pub fn do_syscall(header: u64, meta: u64, addr: u64, rip: u64, page_count: u64) -> u64;
}

pub type ThreadId = u32;
pub type TaskId = u32;

pub mod proc;
