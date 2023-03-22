use crate::{
    print, println,
    xhci::{msi_kekw, Config, HostCaps, OpRegs},
};
use arrayvec::ArrayVec;
use bitflags::bitflags;
use bitvec::prelude::*;
use core::ptr::{addr_of, addr_of_mut};
use num_enum::{IntoPrimitive, TryFromPrimitive};
use proc_bitfield::*;
use x86_64::VirtAddr;

#[derive(Debug)]
#[repr(C)]
struct PcieConfig {
    vendor_id: u16,
    device_id: u16,
    command: u16,
    status: u16,
    revision_id: u8,
    class_code: [u8; 3],
    cache_size: u8,
    latency_timer: u8,
    header_type: u8,
    bist: u8,
}

#[derive(Debug)]
#[repr(C)]
pub struct VirtioConfig {
    pcie: PcieConfig,
    bars: [Bar; 6],
    _res: u32,
    system_vendor_id: u16,
    subsystem_id: u16,
    _reserved2: u32,
    capabilities_ptr: u8,
}

#[derive(Debug)]
#[repr(C)]
struct XhciConfig {
    pcie: PcieConfig,
    bar0: Bar,
    bar1: Bar,
    _reserved1: [u32; 5],
    system_vendor_id: u16,
    subsystem_id: u16,
    _reserved2: u32,
    capabilities_ptr: u8,
    _reserved2_2: [u8; 3],
}

#[derive(Debug)]
#[repr(C)]
pub struct CapabilityHeader {
    id: u8,
    next: u8,
}

#[derive(Debug)]
#[repr(C)]
pub struct VirtioHeader {
    header: CapabilityHeader,
    length: u8,
    cfg_type: u8,
    bar: u8,
    _res: [u8; 3],
    offset: u32,
    length_2: u32,
}

#[derive(Debug)]
#[repr(C)]
pub struct Common {
    dev_feature_select: u32,
    dev_feature: Feature,
    drv_feature_select: u32,
    drv_feature: Feature,
    msix_config: u16,
    num_queues: u16,
    dev_status: Status,
    config_generation: u8,

    select: u16,
    size: u16,
    msix_vec: u16,
    enable: u16,
    notify_off: u16,
    desc: u64,
    driver: u64,
    device: u64,
    queues: [Queue; 3],
}

#[derive(Debug)]
#[repr(C, packed)]
struct Queue {
    select: u16,
    size: u16,
    msix_vec: u16,
    enable: u16,
    notify_off: u16,
    desc: u64,
    driver: u64,
    device: u64,
}

#[derive(Debug)]
#[repr(C)]
pub struct MsiCaps {
    pub header: CapabilityHeader,
    pub msg_control: MsiControl,
    pub table: BarOffset,
    pub pending: BarOffset,
}

bitfield! {
    pub struct MsiControl(u16): Debug {
        pub size: u16 @ 0..=10,
        pub mask: bool @ 14,
        pub enable: bool @ 15,
    }
}

bitfield! {
    pub struct BarOffset(u32): Debug {
        pub bar: u8 @ 0..=2,
        pub offset: u32 @ 3..=31,
    }
}

bitfield! {
    struct Bar(pub u32): Debug {
        is_io: bool @ 0,
        bar_type: u8 [try BarType] @ 1..=2,
        prefetchable: bool @ 3,
        addr: u32 @ 4..=31,
    }
}

#[derive(Debug, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
enum BarType {
    Bit32 = 0b00,
    Bit64 = 0b10,
}

fn get_bar(
    allocator: &mut crate::frame::Allocator,
    config: &VirtioConfig,
    index: usize,
    offset: u64,
) -> VirtAddr {
    let bar = &config.bars[index as usize];

    let addr = match bar.bar_type().unwrap() {
        BarType::Bit64 => {
            ((config.bars[index as usize + 1].0 as u64) << 32) | ((bar.addr() as u64) << 4)
        }
        BarType::Bit32 => (bar.addr() as u64) << 4,
    };
    let addr = addr as u64 + offset as u64;

    let virt = crate::phys_offset() + addr / 4096 * 4096;
    crate::mm::virt::map_page(allocator, virt, x86_64::PhysAddr::new(addr / 4096 * 4096));
    virt
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    struct Status: u8 {
        const Acknowledge = 1;
        const Driver = 2;
        const Failed = 128;
        const FeaturesOk = 8;
        const DriverOk = 4;
        const DeviceNeedsReset = 64;
    }
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    struct Feature: u32 {
        const CSum = 1;
        const GuestCSum = 1 << 1;
        const CtrlGuestOffload = 1 << 2;
        const MultiQueue = 1 << 3;
        const Mac = 1 << 5;

        const GuestTSO4 = 1 << 7;
        const GuestTSO6 = 1 << 8;
        const GuestECN = 1 << 9;
        const GuestUFO = 1 << 10;

        const HostTSO4 = 1 << 11;
        const HostTSO6 = 1 << 12;
        const HostECN = 1 << 13;
        const HostUFO = 1 << 14;

        const MergeRxBuf = 1 << 15;
        const Status = 1 << 16;
        const CtrlChan = 1 << 17;
        const CtrlRx = 1 << 18;
        const CtrlVlan = 1 << 19;
        const GuestAnnounce = 1 << 21;
        const MultiQueueAuto = 1 << 22;
        const CtrlMac = 1 << 23;
    }
}

pub fn virtio_shenanigans(allocator: &mut crate::frame::Allocator, config: &mut VirtioConfig) {
    println!("pcie: {:?}", config);
    println!("bars {:?}", config.bars);

    let mut base = unsafe { addr_of_mut!(*config).cast::<CapabilityHeader>() };
    let mut caps_ptr = unsafe { base.byte_offset(config.capabilities_ptr.into()) };

    let mut msi = None;
    let mut common = None;
    let mut notif = None;
    // Int#x
    let mut isr = None;
    let mut device = None;
    // Suboptimal access method
    let mut pci = None;
    loop {
        let caps = unsafe { &*caps_ptr };
        println!("caps: {:?}", caps);
        if caps.id == 0x11 {
            msi = Some(unsafe { &mut *caps_ptr.cast::<MsiCaps>() });
        }
        if caps.id == 0x9 {
            let vendor = unsafe { &*caps_ptr.cast::<VirtioHeader>() };
            match vendor.cfg_type {
                1 => common = Some(vendor),
                2 => notif = Some(vendor),
                3 => isr = Some(vendor),
                4 => device = Some(vendor),
                5 => pci = Some(vendor),
                _ => unreachable!(),
            }
        }
        if caps.next == 0 {
            break;
        }
        caps_ptr = unsafe { base.byte_offset(caps.next.into()) };
    }

    println!("common: {:?}", common);
    println!("notif: {:?}", notif);
    println!("isr: {:?}", isr);
    println!("device: {:?}", device);
    println!("pci: {:?}", pci);
    println!("msi: {:?}", msi);

    let common = common.unwrap();
    let virt = get_bar(allocator, config, common.bar as usize, common.offset as u64);
    let common = unsafe { &mut *virt.as_mut_ptr::<Common>() };
    println!("common: {:?}", common);

    let msi = msi.unwrap();

    let msi_ptr = get_bar(
        allocator,
        config,
        msi.table.bar().into(),
        (msi.table.offset() << 3) as u64,
    )
    .as_mut_ptr::<MsiReg>();

    let pba_ptr = get_bar(
        allocator,
        config,
        msi.pending.bar().into(),
        (msi.pending.offset() << 3) as u64,
    )
    .as_mut_ptr::<u8>();

    let msiregs =
        unsafe { core::slice::from_raw_parts_mut(msi_ptr, (msi.msg_control.size() + 1).into()) };

    let ptr = bitvec::ptr::BitPtr::from_mut(unsafe { &mut *pba_ptr });
    let pending =
        unsafe { bitvec::slice::from_raw_parts_mut(ptr, (msi.msg_control.size() + 1).into()) };

    let mut dev = Device {
        msi: Some(Msi {
            caps: msi,
            table: msiregs,
            pending: pending.unwrap(),
        }),
    };
    dev.enable_msi(0);
    dev.enable_msi_all();

    common.dev_status |= Status::Acknowledge;
    common.dev_status |= Status::Driver;
    common.dev_feature_select = 0;

    println!("feats: {:?}", common.dev_feature);

    common.drv_feature_select = 0;
    common.drv_feature = Feature::empty();
    common.dev_status |= Status::FeaturesOk;
    // common.msix_config = 0;
    
    let desc_arr_addr = allocator.allocate().unwrap().as_u64();
    let desc_arr =
        unsafe { &mut *(crate::phys_offset() + desc_arr_addr).as_mut_ptr::<[Desc; 100]>() };

    let used_addr = allocator.allocate().unwrap().as_u64();
    let used = unsafe { &mut *(crate::phys_offset() + used_addr).as_mut_ptr::<Used>() };

    let avail_addr = allocator.allocate().unwrap().as_u64();
    let avail = unsafe { &mut *(crate::phys_offset() + avail_addr).as_mut_ptr::<Avail>() };

    let queue = common;
    queue.desc = desc_arr_addr;
    queue.device = used_addr;
    queue.driver = avail_addr;
    queue.size = 100;
    queue.select = 0;
    queue.enable = 1;

    for i in 0..100 {
        let addr = allocator.allocate().unwrap().as_u64();
        desc_arr[i].addr = addr;
        desc_arr[i].lenght = 1526;
        desc_arr[i].flags = 2;
        avail.ring[i] = i as u16;
    }

    println!("this is a memory barrier lmfao");
    queue.msix_vec = 0;
    println!("this is a memory barrier lmfao");

    avail.flags = 0;
    println!("this is a memory barrier lmfao");
    avail.idx = 100;
    println!("this is a memory barrier lmfao");
    queue.dev_status |= Status::DriverOk;

    println!("avail {:?}", avail);
    loop {
        println!("used {:?}", used);
    }
}

#[derive(Debug)]
#[repr(C)]
struct Avail {
    flags: u16,
    idx: u16,
    ring: [u16; 100],
}

#[derive(Debug)]
#[repr(C)]
struct UsedElem {
    id: u32,
    len: u32,
}

#[derive(Debug)]
#[repr(C)]
struct Used {
    flags: u16,
    idx: u16,
    ring: [UsedElem; 100],
}

#[derive(Debug)]
#[repr(C)]
struct Desc {
    addr: u64,
    lenght: u32,
    flags: u16,
    next: u16,
}

pub fn pcie_shenanigans(
    allocator: &mut crate::frame::Allocator,
    configs: &mut [crate::acpi::PcieRef],
) -> ArrayVec<Device<'static>, 256> {
    let addr = configs[0].addr;
    let mut xhci = None;
    let mut net = None;
    for i in 0..256 {
        let cfg_addr = crate::phys_offset() + (addr + ((i as u64) << 15));
        let config = unsafe { &*cfg_addr.as_ptr::<PcieConfig>() };

        // println!("pcie_config: {:?}", config);

        // check capabilities bit in status reg status
        if config.class_code == [0x30, 0x03, 0x0C] {
            xhci = Some(unsafe { &mut *cfg_addr.as_mut_ptr::<XhciConfig>() });
        }
        if config.device_id == 0x1041 {
            net = Some(unsafe { &mut *cfg_addr.as_mut_ptr::<VirtioConfig>() });
        }
    }

    let xhci = xhci.unwrap();
    virtio_shenanigans(allocator, net.unwrap());

    // TODO: Flatten call stack
    loop {}

    println!("xhci: {:?}", xhci);
    println!("bar0 {}, bar1 {}", xhci.bar0.addr(), xhci.bar1.0);

    // Also is mmio registers base
    let bar = match xhci.bar0.bar_type().unwrap() {
        BarType::Bit64 => ((xhci.bar1.0 as u64) << 32) + ((xhci.bar0.addr() as u64) << 4),
        BarType::Bit32 => (xhci.bar0.addr() as u64) << 4,
    };

    println!("bar64: {:?}", bar);

    println!("fuck");
    println!("addr: {:?}", (crate::phys_offset() + bar));

    crate::mm::virt::map_page(
        allocator,
        crate::phys_offset() + bar,
        x86_64::PhysAddr::new(bar / 4096 * 4096),
    );

    crate::mm::virt::map_page(
        allocator,
        crate::phys_offset() + bar + 4096usize,
        x86_64::PhysAddr::new((bar + 4096) / 4096 * 4096),
    );

    let hostcaps_ptr = (crate::phys_offset() + bar).as_ptr::<HostCaps>();
    let hostcaps = unsafe { &*(crate::phys_offset() + bar).as_ptr::<HostCaps>() };
    println!("hostcaps: {:?}", hostcaps.clone());

    let opregs =
        unsafe { &mut *(crate::phys_offset() + bar + hostcaps.length as u64).as_mut_ptr::<OpRegs>() };
    unsafe {
        addr_of_mut!(opregs.cmd).write_volatile(opregs.cmd.clone().with_reset(true));
    }
    for i in 0..5 {
        println!("ee");
    }
    println!("opregs: {:?}", opregs.clone());
    unsafe {
        (&mut opregs.config as *mut Config).write_volatile(
            opregs
                .config
                .clone()
                .with_max_slots(hostcaps.sparams1.max_slots()),
        );
    }
    println!("\nopregs: {:?}", opregs.clone());

    let dev_ctx_arr = allocator.allocate().unwrap();
    let cmd_queue = allocator.allocate().unwrap();
    opregs.device_ctx_ptr = dev_ctx_arr.as_u64();
    opregs.cmd_ring.set_ring_ptr(cmd_queue.as_u64());

    let mut base = unsafe { (xhci as *const _) as *const CapabilityHeader };
    let mut caps_ptr = unsafe { base.byte_offset(xhci.capabilities_ptr.into()) };

    let mut msi = None;
    loop {
        let caps = unsafe { &*caps_ptr };
        println!("caps: {:?}", caps);
        if caps.id == 0x11 {
            msi = Some(unsafe { &mut *caps_ptr.cast::<MsiCaps>() });
        }
        if caps.next == 0 {
            break;
        }
        caps_ptr = unsafe { base.byte_offset(caps.next.into()) };
    }

    let msi = msi.unwrap();

    assert_eq!(msi.table.bar(), 0);
    assert_eq!(msi.pending.bar(), 0);

    let msireg =
        (crate::phys_offset() + bar + (msi.table.offset() << 3) as u64).as_mut_ptr::<MsiReg>();

    let pba = (crate::phys_offset() + bar + (msi.pending.offset() << 3) as u64).as_mut_ptr::<u8>();

    crate::mm::virt::map_page(
        allocator,
        x86_64::VirtAddr::new(msireg.addr() as u64),
        x86_64::PhysAddr::new(bar + (msi.table.offset() << 3) as u64),
    );

    let msiregs =
        unsafe { core::slice::from_raw_parts_mut(msireg, (msi.msg_control.size() + 1).into()) };

    let ptr = bitvec::ptr::BitPtr::from_mut(unsafe { &mut *pba });
    let pending =
        unsafe { bitvec::slice::from_raw_parts_mut(ptr, (msi.msg_control.size() + 1).into()) };

    let mut devs = ArrayVec::<Device<'static>, 256>::new();
    devs.push(Device {
        msi: Some(Msi {
            caps: msi,
            table: msiregs,
            pending: pending.unwrap(),
        }),
    });

    msi_kekw(allocator, &mut devs[0], bar, hostcaps_ptr, msi, opregs);
    devs
}

#[derive(Debug)]
#[repr(C)]
pub struct MsiReg {
    pub addr: u64,
    pub data: u32,
    pub mask: u32,
}

struct Msi<'a> {
    caps: &'a mut MsiCaps,
    table: &'a mut [MsiReg],
    pending: &'a mut BitSlice<u8, Lsb0>,
}

pub struct Device<'a> {
    msi: Option<Msi<'a>>,
}

impl<'a> Device<'a> {
    pub fn enable_msi(&mut self, id: u16) {
        let msi = self.msi.as_mut().unwrap();
        for (i, reg) in msi.table.iter_mut().enumerate() {
            reg.addr = 0xFEE << 20;
            reg.data = 20 + i as u32;
            reg.mask = (i > 0).into();
        }
    }

    pub fn enable_msi_all(&mut self) {
        self.msi.as_mut().unwrap().caps.msg_control.set_enable(true);
    }
}
