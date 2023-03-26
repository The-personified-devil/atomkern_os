use std::{env, error::Error};
fn main() {
    println!("cargo:rerun-if-changed=src/syscall.s");
    nasm_rs::compile_library_args("libsyscall.a", &["src/syscall.s"], &["-felf64"]).unwrap();

    println!("cargo:rustc-link-lib=syscall");

    let kernel_name = env::var("CARGO_PKG_NAME").unwrap();
    // Tell rustc to pass the linker script to the linker.
    println!("cargo:rustc-link-arg-bin={kernel_name}=--script=conf/linker.ld");

    // Have cargo rerun this script if the linker script or CARGO_PKG_ENV changes.
    println!("cargo:rerun-if-changed=conf/linker.ld");
}
