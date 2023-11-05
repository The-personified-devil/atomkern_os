use core::ptr::addr_of;
use core::arch::asm;

#[repr(transparent)]
pub struct CpuLocal<T>(T);

impl<T> CpuLocal<T> {
    pub const fn new(arg: T) -> CpuLocal<T> {
        Self(arg)
    }
}

impl<T> core::ops::Deref for CpuLocal<T> {
    type Target = T;

    // TODO: Ensure alignment is consistent
    // Also fix alignment errors lmao
    fn deref(&self) -> &Self::Target {
        let diff = unsafe {
            addr_of!(*self)
                .cast::<u8>()
                .offset_from(addr_of!(__cpu_local_end))
        };
        let tl_ptr: *const T;

        unsafe {
            asm!("mov {}, gs:[0]", lateout(reg) tl_ptr, options(pure, readonly));
            tl_ptr.byte_offset(diff).as_ref().unwrap()
        }
    }
}

// TODO: Check if the whole send sync shit is sane lmao
unsafe impl<T> Sync for CpuLocal<T> {}

extern "C" {
    static mut __cpu_local_start: u8;
    static mut __cpu_local_end: u8;
}

pub fn setup_cpu_local(alloc: &mut crate::frame::Allocator, id: usize) {
    let size = unsafe { addr_of!(__cpu_local_end).offset_from(addr_of!(__cpu_local_start)) }
        .unsigned_abs()
        + 8;

    let pg_count = size.div_ceil(0x1000);

    let base = 0x20000000000;
    for i in 0..pg_count {
        let a = alloc.allocate().unwrap();
        crate::map_page(
            alloc,
            x86_64::VirtAddr::new((base + i * 0x1000 + id * 0x1000 * pg_count) as u64),
            a,
        );
    }

    // This should be the addr of the last qword in the allocated region
    let addr = base + pg_count * 0x1000 + id * 0x1000 * pg_count - 8;

    let a = core::ptr::from_exposed_addr_mut::<usize>(addr);
    unsafe {
        *a = a as usize;
    }
    unsafe {
        core::ptr::copy_nonoverlapping(
            addr_of!(__cpu_local_start),
            a.byte_sub(size - 8).cast::<u8>(),
            size - 8,
        );

        asm!("wrgsbase {}", in(reg) a);
    }
}
