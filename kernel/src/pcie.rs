use crate::{mm::virt::MapFlags, phys_offset};
#[allow(invalid_reference_casting)]
use crate::{
    println,
    xhci::{msi_kekw, Config, HostCaps, OpRegs},
};
use arrayvec::ArrayVec;
use bitflags::bitflags;
use bitvec::prelude::*;
use core::ptr::{addr_of, addr_of_mut, from_exposed_addr};
use num_enum::{IntoPrimitive, TryFromPrimitive};
use proc_bitfield::*;
use x86_64::{PhysAddr, VirtAddr};

// TODO: Move xhci outta here
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
    virt: u64,
    offset: u64,
) -> VirtAddr {
    let bar = &config.bars[index as usize];
    println!("bar {}", index);

    let addr = match bar.bar_type().unwrap() {
        BarType::Bit64 => {
            ((config.bars[index as usize + 1].0 as u64) << 32) | ((bar.addr() as u64) << 4)
        }
        BarType::Bit32 => (bar.addr() as u64) << 4,
    };
    let addr = addr as u64 + offset as u64;

    let virt = VirtAddr::new(virt);
    crate::mm::virt::map_page(allocator, virt, x86_64::PhysAddr::new(addr / 4096 * 4096));
    virt
}

// pub fn map_bar(bar: usize) {

// }

pub struct UserSpaceAddr {
    raw: u64,
}

// TODO: Make saner and part of the syscall bs
// i.e. not panicing lmao
impl UserSpaceAddr {
    pub fn new(raw: u64) -> Self {
        if raw > 0xffffffff80000000 {
            panic!("this is not a userspace addr :angy:");
        }
        Self { raw }
    }

    pub fn as_raw(&self) -> u64 {
        self.raw
    }
}

// TODO: Introduce below restrictions, i.e. special segments
pub fn map_alloc_phys(addr: UserSpaceAddr, size: usize, small: bool) -> PhysAddr {
    let mut allocator = crate::interrupt::ALLOC.lock();
    let allocator = allocator.as_mut().unwrap();

    let alloc = {
        if size > 4096 {
            // allocator.allocate_small_big().unwrap()
            // TODO: Actually create allocator for this
            unsafe { PhysAddr::new_unsafe(0x370000000000) }
        } else if size > 0x200000 {
            panic!("nuh uh fuck u");
        } else if small {
            allocator.allocate_small().unwrap()
        } else {
            allocator.allocate().unwrap()
        }
    };

    let (page_table, _) = x86_64::registers::control::Cr3::read();
    let pt_addr = page_table.start_address();

    for i in 0..size.div_ceil(4096) {
        crate::mm::virt::proper_map_page(
            allocator,
            pt_addr,
            VirtAddr::new(addr.as_raw() + i as u64 * 4096),
            PhysAddr::new(alloc.as_u64() + i as u64 * 4096),
            MapFlags::User | MapFlags::Writable | MapFlags::Readable,
        );
    }

    alloc
}

pub fn map_virtio_net(uspace_ptr: u64) {
    let mut allocator = crate::interrupt::ALLOC.lock();
    let allocator = allocator.as_mut().unwrap();

    let (page_table, _) = x86_64::registers::control::Cr3::read();
    // println!("start addr: {}", page_table.start_address().as_u64());
    let addr = page_table.start_address();

    let config = unsafe { &mut *NET.unwrap() };

    crate::mm::virt::proper_map_page(
        allocator,
        addr,
        VirtAddr::new(uspace_ptr),
        PhysAddr::new(VirtAddr::new(config as *mut _ as u64) - phys_offset()),
        MapFlags::User | MapFlags::Writable | MapFlags::Readable,
    );
}

// TODO: Unfuck this bullshit, potentially make pcie discovery lazy?
static mut NET: Option<*mut VirtioConfig> = None;

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
            unsafe {
                NET = Some(cfg_addr.as_mut_ptr::<VirtioConfig>());
            }
        }
    }

    let mut devs = ArrayVec::<Device<'static>, 256>::new();
    return devs;

    let xhci = xhci.unwrap();

    // From here onward xhci only

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

    let opregs = unsafe {
        &mut *(crate::phys_offset() + bar + hostcaps.length as u64).as_mut_ptr::<OpRegs>()
    };
    unsafe {
        addr_of_mut!(opregs.cmd).write_volatile(
            addr_of_mut!(opregs.cmd)
                .read_unaligned()
                .clone()
                .with_reset(true),
        );
    }

    for i in 0..5 {
        println!("ee");
    }

    // println!("opregs: {:?}", opregs.clone());
    unsafe {
        addr_of_mut!(opregs.config).write_volatile(
            addr_of_mut!(opregs.config)
                .read_unaligned()
                .clone()
                .with_max_slots(hostcaps.sparams1.max_slots()),
        );
    }
    // println!("\nopregs: {:?}", opregs.clone());

    let dev_ctx_arr = allocator.allocate().unwrap();
    let cmd_queue = allocator.allocate().unwrap();
    opregs.device_ctx_ptr = dev_ctx_arr.as_u64();

    unsafe {
        addr_of_mut!(opregs.cmd_ring).write_unaligned({
            let mut a = addr_of_mut!(opregs.cmd_ring).read_unaligned();
            a.set_ring_ptr(cmd_queue.as_u64());
            a
        });
    }

    let mut base = unsafe { (xhci as *const _) as *const CapabilityHeader };
    let mut caps_ptr = unsafe { base.byte_offset(xhci.capabilities_ptr.into()) };

    let mut msi = None;
    loop {
        let caps = unsafe { &*caps_ptr };
        println!("caps: {:?}", caps);
        if caps.id == 0x11 {
            msi = Some(unsafe { &mut *caps_ptr.cast::<MsiCaps>().cast_mut() });
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

    // let ptr = bitvec::ptr::BitPtr::from_mut(unsafe { &mut *pba });
    // let pending =
    //     unsafe { bitvec::slice::from_raw_parts_mut(ptr, (msi.msg_control.size() + 1).into()) };

    // devs.push(Device {
    //     msi: Some(Msi {
    //         caps: msi,
    //         table: msiregs,
    //         pending: pending.unwrap(),
    //     }),
    // });

    msi_kekw(allocator, &mut devs[0], bar, hostcaps_ptr, msi, opregs);
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
