fn main() {
    println!("cargo:rerun-if-changed=src/syscall.s");
    nasm_rs::compile_library_args("libsyscall.a", &["src/syscall.s"], &["-felf64"]).unwrap();

    println!("cargo:rustc-link-lib=static:+bundle=syscall");
}
