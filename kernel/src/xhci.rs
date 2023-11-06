use x86_64::{structures::idt::InterruptStackFrame, registers::model_specific::Msr};

pub extern "x86-interrupt" fn xhci_int(_: InterruptStackFrame) {
    let mut eoi = Msr::new(0x80B);

    // TODO: Extract into a helper
    unsafe {
        eoi.write(0);
    }
}
