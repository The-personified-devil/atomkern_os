#! /bin/sh
#
# This script will be executed by `cargo run`.

set -xe

LIMINE_GIT_URL="https://github.com/limine-bootloader/limine.git"

# Cargo passes the path to the built executable as the first argument.
KERNEL=$1

# Clone the `limine` repository if we don't have it yet.
if [ ! -d target/limine ]; then
    git clone $LIMINE_GIT_URL --depth=1 --branch v3.0-branch-binary target/limine
fi

# Make sure we have an up-to-date version of the bootloader.
cd target/limine
git fetch
make
cd -

# Copy the needed files into an ISO image.
mkdir -p target/iso_root
cp $KERNEL conf/limine.cfg target/limine/limine{.sys,-cd.bin,-cd-efi.bin} target/iso_root

xorriso -as mkisofs                                             \
    -b limine-cd.bin                                            \
    -no-emul-boot -boot-load-size 4 -boot-info-table            \
    --efi-boot limine-cd-efi.bin                                \
    -efi-boot-part --efi-boot-image --protective-msdos-label    \
    target/iso_root -o $KERNEL.iso

# For the image to be bootable on BIOS systems, we must run `limine-deploy` on it.
# target/limine/limine-deploy $KERNEL.iso

# Run the created image with QEMU.
qemu-system-x86_64 \
        -bios /usr/share/ovmf/x64/OVMF.fd \
 \
        -machine type=q35,accel=kvm \
        -cpu host \
        -smp 16 \
        -m 32G \
 \
        -no-shutdown \
        -no-reboot \
        -s -S \
 \
        -d all \
        -D qemulog%d \
        -monitor stdio \
 \
        --trace usb_* \
        --trace msix_* \
        --trace pci_* \
        --trace esp_pci_* \
        --trace apic_* \
        --trace vfio_* \
        --trace virtio_* \
        --trace net_* \
 \
        -device qemu-xhci,id=xhci \
        -netdev user,id=user,hostfwd=tcp::2222-:6970 \
        -net nic,netdev=user,model=virtio-net-pci-non-transitional \
	-object filter-dump,id=user,netdev=user,file=packetdump \
 \
    $KERNEL.iso

        # -device usb-mouse,bus=xhci.0 \
        # -device usb-kbd,bus=xhci.0 \
