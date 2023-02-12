use x86_64::{registers, structures::paging::PageTable, VirtAddr};

// as param?
const PHYS_OFFSET: VirtAddr = VirtAddr::new_truncate(0x10000000000);

// struct Allocator {

// }


pub unsafe fn active_page_table() -> &'static mut PageTable {
    let (page_table, cr3_flags) = registers::control::Cr3::read();
    let addr = PHYS_OFFSET + page_table.start_address().as_u64();
    let page_table_ptr: *mut PageTable = addr.as_mut_ptr();
    unsafe { &mut *page_table_ptr }
}
