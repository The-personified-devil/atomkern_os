fn main() {
    nasm_rs::compile_library_args("libbruh.a", &["src/proc.s"], &["-felf64"]).unwrap();
    println!("cargo:rustc-link-lib=bruh");
    // cc::Build::new().l("src/long_mode.o").compile("libshit.a");
}
