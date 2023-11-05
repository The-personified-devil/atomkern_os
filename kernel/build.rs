use std::{env, error::Error};

fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    println!("cargo:rerun-if-changed=src/proc.s");
    nasm_rs::compile_library_args("libbruh.a", &["src/proc.s"], &["-felf64"]).unwrap();

    println!("cargo:rustc-link-lib=bruh");

    // Get the name of the package.
    let kernel_name = env::var("CARGO_PKG_NAME")?;

    // Tell rustc to pass the linker script to the linker.
    println!("cargo:rustc-link-arg=-Tconf/linker.ld");

    // Have cargo rerun this script if the linker script or CARGO_PKG_ENV changes.
    println!("cargo:rerun-if-changed=conf/linker.ld");
    println!("cargo:rerun-if-env-changed=CARGO_PKG_NAME");

    Ok(())
}
