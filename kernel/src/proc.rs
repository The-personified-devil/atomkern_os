use arrayvec::ArrayVec;
use num_enum::TryFromPrimitive;
use x86_64::{
    registers::model_specific::{self, Efer, EferFlags, Msr},
    structures::gdt::SegmentSelector,
    PhysAddr, PrivilegeLevel, VirtAddr,
};

use crate::{misc::cpu_local::CpuLocal, mm::virt::MapFlags, println};
use abi::TaskId;
use atomkern_abi as abi;
use core::{
    arch::asm,
    cell::Cell,
    mem::size_of,
    ptr::{self, addr_of},
};

static PROCS: spin::Mutex<ArrayVec<Process, 200>> =
    spin::Mutex::new(ArrayVec::<Process, 200>::new_const());

static PROC_QUEUE: spin::Mutex<LoopedQueue> = spin::Mutex::<_>::new(LoopedQueue::new());

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

#[link_section = ".cpu_local"]
static proc_id: CpuLocal<Cell<u64>> = CpuLocal::new(Cell::new(u64::MAX));

#[link_section = ".cpu_local"]
static yeld: CpuLocal<Cell<u64>> = CpuLocal::new(Cell::new(0));

pub fn queue_proc(id: u64) {
    PROC_QUEUE.lock().put(id);
}

pub fn halter() {
    loop {
        // x86_64::instructions::hlt();
    }
}

pub fn create_kernel_handler() {
    let mut alloc = crate::interrupt::ALLOC.lock();
    let alloc = alloc.as_mut().unwrap();
    let stack = crate::interrupt::alloc_stack(alloc);

    PROCS.lock().push(Process {
        stored: Some(Stored {
            rip: halter as *const fn() as u64,
            cr3: get_cr3(),
            rsp: stack,
            ss: 0,
            cs: 40,
            ..Default::default()
        }),
        tls_info: abi::proc::Tls::default(),
        threads_spawned: 0,
    });
}

pub fn create_proc(tid: abi::ThreadId, entry: u64, tls: abi::proc::Tls, additional: u64) -> u64 {
    let mut procs = PROCS.lock();
    let id = procs.len();

    let mut alloc = crate::interrupt::ALLOC.lock();
    let alloc = alloc.as_mut().unwrap();
    let stack = crate::interrupt::alloc_stack(alloc);

    let param_ptr =
        unsafe { ptr::from_exposed_addr_mut::<abi::proc::StartParams>(stack as usize).offset(-1) };
    // let params = unsafe { param_ptr.as_mut() }.unwrap();

    let params = abi::proc::StartParams {
        tls,
        thread_start: additional,
        tid,
    };
    println!("{:?}", params);
    unsafe {
        param_ptr.write_volatile(params);
    }

    println!("stack {stack}");

    let mut regs = [0; 14];
    regs[8] = param_ptr.addr() as u64;

    procs.push(Process {
        stored: Some(Stored {
            rip: entry,
            rsp: param_ptr.expose_addr() as u64,
            cr3: get_cr3(),
            ss: 16,
            cs: 8,
            regs,
            ..Default::default()
        }),
        tls_info: tls,
        threads_spawned: 0,
    });
    // This should be a magic id for being in kernel
    // proc_id.set(id as u64);
    id as u64
}

pub fn get_cr3() -> u64 {
    let value;

    unsafe {
        asm!("mov {}, cr3", out(reg) value, options(nomem, nostack, preserves_flags));
    }

    value
}

// This should gracefully handle the current state being fucked up
// i.e. check if the current proc got deleted and don't requeue it
// also be able to go into the kernel sleeper
#[no_mangle]
extern "C" fn determine_next_proc(regs: [u64; 19]) -> *const Stored {
    // println!("regs: {:?}", regs);

    let prev_id = proc_id.get();

    let fs: u64;
    unsafe {
        asm!("rdfsbase {}", out(reg) fs);
    }

    let mut binding = PROCS.lock();
    let procs = &mut binding;

    // TODO: Rename to context
    let prev_stored = Stored {
        rip: regs[14],
        cs: regs[15],
        rflags: regs[16],
        rsp: regs[17],
        ss: regs[18],
        regs: regs[0..14].try_into().unwrap(),
        cr3: get_cr3(),
        fs,
    };

    if prev_id != u64::MAX && prev_id != 0 {
        procs[prev_id as usize].stored = Some(prev_stored);

        if yeld.get() == 0 {
            PROC_QUEUE.lock().put(prev_id);
        }
        yeld.set(0);
    }

    let new = PROC_QUEUE.lock().take().unwrap_or(0);
    println!("new {}", new);

    proc_id.set(new);

    // TODO: Should we actually consume here, but consuming would make accidentally running the
    // same Process at the same time impossible
    let stored = &mut procs[new as usize].stored.unwrap();

    // if new == 0 {
    //     stored.rsp = regs[17];
    // }

    unsafe {
        asm!("wrfsbase {}", in(reg) stored.fs);
    }

    // TODO: Don't clone rflags here that's bs
    stored.rflags = regs[16];

    println!("determine_next_proc");

    let timer_initial: u64 = 4000000000;
    let mut timer_initial_reg = Msr::new(0x838);
    unsafe {
        timer_initial_reg.write(timer_initial);
    }

    // Safe because only this cpu should be accessing tcb data at this point
    addr_of!(*stored)
}

extern "C" {
    fn syscall_handler();
    // pub fn sysret_executor(stack: u64, entry: u64, rflags: u64, params: u64) -> !;
}

#[derive(Debug, TryFromPrimitive)]
#[repr(u64)]
enum Syscall {
    Alloc = 2,
    Print = 4,
    CreateThread = 6,
    Yield = 8,
    WakeupThread = 10,
    MapPcie = 12,
    VirtToPhys = 14,
    MapAllocPhys = 16,
}

#[derive(Debug, TryFromPrimitive)]
#[repr(u64)]
enum PageType {
    Normal = 0,
    Large = 1,
    Huge = 2,
}

#[no_mangle]
extern "C" fn syscall_handler_rs(
    header: u64,
    param1: u64,
    param2: u64,
    rip: u64,
    param4: u64,
    param5: u64,
) -> u64 {
    println!("rip {rip}");
    println!("Syscall done {header}");
    if (header & 0b1) != 0 {
        println!("rpc not supported yet");
        return 0;
    }
    let toipe = header & !0b1;
    // println!("{header} {param1} {param2} {param3} {param4} {param5}");

    let mut e = 0;
    match toipe.try_into().unwrap() {
        Syscall::Alloc => {
            alloc_memory_user(param1.try_into().unwrap(), VirtAddr::new(param2), param4);
        }
        Syscall::Print => {
            print_user(param1, param2);
        }
        Syscall::CreateThread => {
            e = create_thread(param1, param2);
        }
        Syscall::Yield => {
            println!("yielding");
            // let id = proc_id.get();
            // proc_id.set(u64::MAX);
            yeld.set(1);
            x86_64::instructions::interrupts::enable();
            x86_64::instructions::hlt();
            x86_64::instructions::interrupts::disable();
        }
        Syscall::WakeupThread => {
            println!("thread waking not implemented");
            loop {}
        }
        Syscall::MapPcie => {
            crate::pcie::map_virtio_net(param1);
        }
        Syscall::VirtToPhys => {
            e = crate::mm::virt::virt_to_phys(VirtAddr::new(param1)).as_u64();
        }
        Syscall::MapAllocPhys => {
            e = crate::pcie::map_alloc_phys(crate::pcie::UserSpaceAddr::new(param1), param2 as usize, param4 != 0).as_u64();
        }
    }
    println!("leaving syscall");
    e
}

fn print_user(addr: u64, length: u64) {
    let slice = unsafe {
        core::slice::from_raw_parts(
            core::ptr::from_exposed_addr::<u8>(addr.try_into().unwrap()),
            length.try_into().unwrap(),
        )
    };
    let str = unsafe { core::str::from_utf8_unchecked(slice) };

    println!("Printed by user: {}", str);
}

fn create_thread(entry: u64, actual_start: u64) -> u64 {
    // println!("proc addr: {entry}");
    println!("create thread 0");

    let spawned;
    let tls;
    {
        let proc = &mut PROCS.lock()[proc_id.get() as usize];
        println!("locked the procs");
        proc.threads_spawned += 1;

        spawned = proc.threads_spawned;
        tls = proc.tls_info;
    }

    let id = create_proc(spawned as u32, entry, tls, actual_start);

    println!("created thread");
    queue_proc(id);

    id
}

// Just assume cr3 is still right, as we call it from a context not changing cr3
fn alloc_memory_user(meta: PageType, addr: VirtAddr, page_count: u64) {
    println!("in alloc");
    match meta {
        PageType::Normal => {
            // TODO: Security check

            let mut alloc = crate::interrupt::ALLOC.lock();
            let alloc = alloc.as_mut().unwrap();
            let page_table = PhysAddr::new(get_cr3());
            println!("page_count {page_count}");

            for i in 0..page_count {
                // println!("{i}");
                let virt_addr = addr + 0x1000 * i;
                let phys_addr = alloc.allocate().unwrap();
                let flags = MapFlags::User | MapFlags::Writable;
                crate::mm::virt::proper_map_page(alloc, page_table, virt_addr, phys_addr, flags);
            }
        }
        _ => todo!(),
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
pub struct Process {
    pub stored: Option<Stored>,
    pub tls_info: abi::proc::Tls,
    pub threads_spawned: usize,
}

#[derive(Default, Debug, Copy, Clone)]
#[repr(C)]
pub struct Stored {
    pub ss: u64,
    pub rsp: u64,
    pub rflags: u64,
    pub cs: u64,
    pub rip: u64,
    pub regs: [u64; 14],
    pub cr3: u64,
    pub fs: u64,
}
