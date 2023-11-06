use crate::{mm::virt::MapFlags, phys_offset};

use crate::println;

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
) {
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
}
