#![feature(int_roundings)]
#![feature(pointer_byte_offsets)]
#![feature(inline_const)]
#![feature(const_for)]
#![feature(const_mut_refs)]

use core::panic::PanicInfo;
use rlibc::*;

// #[panic_handler]
// fn panic(info: &PanicInfo) -> ! {
//     // println!("{}", info);
//     loop {}
// }
//
//
//

extern "C" {
    fn do_syscall(header: u64, meta: u64, addr: u64, rip: u64, page_count: u64);
}



// #[start]
// fn start(argc: isize, argv: *const *const u8) -> isize {
//     // println!("Hello, world!");
//     unsafe { do_syscall(2, 0, 0, 5); }
//     0
// }

#[no_mangle]
fn _start () {
    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {
            println!("Hello world");
        })
}

fn main() {

}
