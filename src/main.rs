fn main() {
    // read env variables that were set in build script
    let uefi_path = env!("UEFI_PATH");
    let bios_path = env!("BIOS_PATH");

    // choose whether to start the UEFI or BIOS image
    let uefi = true;

    let mut cmd = std::process::Command::new("qemu-system-x86_64");
    if uefi {
        cmd.arg("-bios").arg("/usr/share/ovmf/x64/OVMF.fd"); //ovmf_prebuilt::ovmf_pure_efi());
        cmd.arg("-s").arg("-S");
        cmd.arg("-m").arg("32G");
        cmd.arg("-d").arg("all");
        cmd.arg("-D").arg("qemulog%d");
        cmd.arg("-no-shutdown");
        cmd.arg("-no-reboot");
        cmd.arg("-monitor").arg("stdio");
        // cmd.arg("--enable-kvm");
        cmd.arg("-machine").arg("type=q35,accel=kvm");
        cmd.arg("-smp").arg("16");
        // cmd.arg("-cpu").arg("qemu64,+x2apic");
        // cmd.arg("-usbdevice").arg("host:0781:55a3");
        // cmd.arg("-drive")
        //     .arg(format!("if=none,id=stick,format=raw,file={uefi_path}"));
        // cmd.arg("-device").arg("nec-usb-xhci,id=xhci");
        // cmd.arg("-device").arg("usb-storage,bus=xhci.0,drive=stick");
        cmd.arg("-drive").arg(format!("format=raw,file={uefi_path}"));
    } else {
        // cmd.arg("-drive").arg(format!("format=raw,file={bios_path}"));
    }
    let mut child = cmd.spawn().unwrap();
    child.wait().unwrap();
}
