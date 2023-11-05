#![feature(int_roundings)]
#![feature(pointer_byte_offsets)]
#![feature(inline_const)]
#![feature(const_for)]
#![feature(const_mut_refs)]
#![feature(thread_local)]
#![feature(iter_collect_into)]
#![feature(strict_provenance)]

use bitflags::bitflags;
use bitvec::order::Lsb0;
use bitvec::slice::BitSlice;
use log::{Level, LevelFilter};
use num_enum::*;
use proc_bitfield::bitfield;
// use std::fmt::Write;
use std::io::Write;
use std::mem::size_of;
use std::ptr::{addr_of, addr_of_mut, from_exposed_addr, null_mut};

use atomkern_abi as abi;
use smoltcp::phy::{self, Device, DeviceCapabilities, Medium};
use smoltcp::time::Instant;

use smoltcp::iface::{Config, Interface, SocketSet};
// use smoltcp::phy::wait as phy_wait;
use smoltcp::socket::{dhcpv4, tcp, udp};
use smoltcp::time::Duration;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address, Ipv4Cidr, Ipv6Address};

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

    // I wanna know who in gods name thought this was a good idea
    queue: Queue,
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
struct NetHeader {
    flags: u8,
    gso_type: u8,
    hdr_len: u16,
    gso_size: u16,
    csum_start: u16,
    csum_offset: u16,
    num_buffers: u16,
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

fn map_pcie_bar(ptrs: Ptrs) {
    unsafe {
        abi::do_syscall(12, addr_of!(ptrs) as u64, 0, 0, 0);
    }
}

fn get_phys_addr(ptr: *mut u8) -> u64 {
    unsafe { abi::do_syscall(14, ptr as u64, 0, 0, 0) }
}

#[no_mangle]
pub extern "C" fn pow(x: f64, y: f64) -> f64 {
    libm::pow(x, y)
}

#[thread_local]
static a: usize = 69;
#[derive(Debug, Clone, Copy)]
struct Ptrs {
    cfg: *mut u8,
    common: *mut u8,
    msi: *mut u8,
    pba: *mut u8,
    notif: *mut u8,
}

#[derive(Debug)]
#[repr(C)]
struct Avail {
    flags: u16,
    idx: u16,
    ring: [u16; 100],
}

#[derive(Debug, Clone, Copy)]
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

#[derive(Debug, Clone)]
#[repr(C)]
struct Desc {
    addr: u64,
    length: u32,
    flags: u16,
    next: u16,
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

pub struct PcieDevice<'a> {
    msi: Option<Msi<'a>>,
}

impl<'a> PcieDevice<'a> {
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

// TODO: Give smoltcp mac
impl<'b> phy::Device for Dev<'b> {
    type RxToken<'a> = StmPhyRxToken<'a> where Self: 'a;
    type TxToken<'a> = StmPhyTxToken<'a> where Self: 'a;

    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        // TODO: Somehow move into the token
        // Why the fuck anyone design something like that
        // that's just fucking reference galore
        // whyyy
        let queue = &mut self.queues[0];

        // used.idx is one past
        if queue.last_used != queue.used.idx {
            let elem =
                unsafe { addr_of!(queue.used.ring[queue.last_used as usize]).read_volatile() };
            println!("{:?}", elem);

            let ptr = queue.fuckwit[elem.id as usize];
            println!("{:?}", unsafe { &mut *ptr.cast::<NetHeader>() });

            let slice = unsafe {
                core::slice::from_raw_parts_mut::<u8>(
                    queue.fuckwit[elem.id as usize].byte_add(size_of::<NetHeader>()),
                    elem.len as usize - size_of::<NetHeader>(),
                )
            };

            let mut arr = vec![];

            for b in &mut *slice {
                core::ascii::escape_default(*b).collect_into(&mut arr);
            }
            println!("data {}", core::str::from_utf8(arr.as_slice()).unwrap());

            queue.last_used += 1;

            let prev = self.queues[1].last_used;
            self.queues[1].last_used += 2;

            return Some((
                StmPhyRxToken(slice),
                StmPhyTxToken(&mut self.queues[1], prev, prev + 1),
            ));
        }
        None
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        let prev = self.queues[1].last_used;
        self.queues[1].last_used += 2;

        Some(StmPhyTxToken(&mut self.queues[1], prev, prev + 1))
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 1536;
        caps.max_burst_size = Some(1);
        caps.medium = Medium::Ethernet;
        caps
    }
}

struct StmPhyRxToken<'a>(&'a mut [u8]);

impl<'a> phy::RxToken for StmPhyRxToken<'a> {
    fn consume<R, F>(mut self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        // TODO: receive packet into buffer
        let result = f(&mut self.0);
        println!("rx called");
        result
    }
}

struct StmPhyTxToken<'a>(&'a mut AbstractQueue, u16, u16);

impl<'a> phy::TxToken for StmPhyTxToken<'a> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let queue = self.0;
        let id = self.1;

        let size = size_of::<NetHeader>();
        let desc = &mut queue.descs[id as usize];
        desc.length = (size + len) as u32;
        desc.flags = 0;
        // desc.next = self.2;
        // desc.
        //
        // let desc = &mut queue.descs[self.2 as usize];
        // desc.length = 10;

        let ptr = queue.fuckwit[id as usize];
        unsafe {
            ptr.cast::<NetHeader>().write(NetHeader {
                flags: 0,
                gso_type: 0,
                hdr_len: 0,
                gso_size: 0,
                csum_start: 0,
                csum_offset: 0,
                num_buffers: 0,
            });
        }

        // let ptr = queue.fuckwit[self.2 as usize];
        let slice = unsafe { core::slice::from_raw_parts_mut::<u8>(ptr.byte_add(size), 1536 - size) };

        let result = f(&mut slice[..len]);
        println!("tx called {}", len);

        queue.avail.ring[queue.avail.idx as usize] = id;
        // queue.avail.ring[queue.avail.idx as usize + 1] = self.2;
        queue.avail.idx += 1;

        unsafe {
            queue.notif.cast::<u16>().write_volatile(1);
        }

        result
    }
}

struct Dev<'a> {
    config: &'a mut VirtioConfig,
    common: &'a mut Common,
    queues: [AbstractQueue; 2],
    notif: *mut u8,
}

impl<'a> Dev<'a> {
    fn init() -> Self {
        let mut vec = vec![0_u8; 0x5000];
        let stupi_ptr = vec.as_mut_ptr();

        let ptrs = Ptrs {
            cfg: stupi_ptr,
            common: unsafe { stupi_ptr.byte_offset(0x1000) },
            msi: unsafe { stupi_ptr.byte_offset(0x2000) },
            pba: unsafe { stupi_ptr.byte_offset(0x3000) },
            notif: unsafe { stupi_ptr.byte_offset(0x4000) },
        };

        map_pcie_bar(ptrs);
        let config = unsafe { &mut *(ptrs.cfg as *mut VirtioConfig) };

        let mut base = unsafe { addr_of_mut!(*ptrs.cfg).cast::<CapabilityHeader>() };
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
        // assume notif thingy is 0
        println!("{:?}", notif.unwrap());
        // loop {}

        let msi = msi.unwrap();

        let msiregs = unsafe {
            core::slice::from_raw_parts_mut(
                ptrs.msi as *mut MsiReg,
                (msi.msg_control.size() + 1).into(),
            )
        };

        let ptr = bitvec::ptr::BitPtr::from_mut(unsafe { &mut *ptrs.pba });
        let pending =
            unsafe { bitvec::slice::from_raw_parts_mut(ptr, (msi.msg_control.size() + 1).into()) };

        let mut dev = PcieDevice {
            msi: Some(Msi {
                caps: msi,
                table: msiregs,
                pending: pending.unwrap(),
            }),
        };
        dev.enable_msi(0);
        dev.enable_msi_all();

        let common = unsafe { &mut *ptrs.common.cast::<Common>() };
        println!("common: {:?}", common);

        common.dev_status |= Status::Acknowledge;
        common.dev_status |= Status::Driver;
        common.dev_feature_select = 0;

        println!("feats: {:?}", common.dev_feature);

        common.drv_feature_select = 0;
        common.drv_feature = Feature::empty();
        common.dev_status |= Status::FeaturesOk;
        // common.msix_config = 0;

        common.dev_status |= Status::DriverOk;

        std::mem::forget(vec);

        let factor = unsafe { addr_of!(*notif.unwrap()).offset(1).cast::<u32>().read() };

        println!("factor: {}", factor);
        let notif = unsafe { ptrs.notif.byte_add(notif.unwrap().offset as usize % 4096) };

        let mut rec_queue = AbstractQueue::with_queue_id(&mut common.queue, 0, notif, factor);
        rec_queue.avail.idx = 100;

        let send_queue = AbstractQueue::with_queue_id(&mut common.queue, 1, notif, factor);

        Self {
            config,
            common,
            queues: [rec_queue, send_queue],
            notif,
        }
    }
}

struct AbstractQueue {
    descs: Vec<Desc>,
    used: Box<Used>,
    avail: Box<Avail>,
    fuckwit: [*mut u8; 100],
    last_used: u16,
    notif: *mut u8,
}

impl AbstractQueue {
    fn with_queue_id(queue: &mut Queue, id: u16, notif: *mut u8, factor: u32) -> Self {
        let mut desc_arr = vec![
            Desc {
                addr: 0,
                length: 0,
                flags: 0,
                next: 0
            };
            100
        ];
        let desc_arr_addr = get_phys_addr(desc_arr.as_mut_ptr() as *mut u8);

        let mut used = Box::new(Used {
            flags: 0,
            idx: 0,
            ring: [UsedElem { id: 0, len: 0 }; 100],
        });
        let used_addr = get_phys_addr(addr_of_mut!(*used) as *mut u8);

        let mut avail = Box::new(Avail {
            flags: 0,
            idx: 0,
            ring: [0; 100],
        });
        let avail_addr = get_phys_addr(addr_of_mut!(*avail) as *mut u8);

        queue.select = id;
        queue.desc = desc_arr_addr;
        queue.device = used_addr;
        queue.driver = avail_addr;
        queue.size = 100;

        let mut fuckwit: [*mut u8; 100] = [null_mut(); 100];

        for i in 0..100 {
            let mut data = vec![0_u8; 4096];

            fuckwit[i] = data.as_mut_ptr();

            let addr = get_phys_addr(data.as_mut_ptr());

            desc_arr[i].addr = addr as u64;
            desc_arr[i].length = 1526;
            desc_arr[i].flags = 2;
            avail.ring[i] = i as u16;

            std::mem::forget(data);
        }

        println!("this is a memory barrier lmfao");
        queue.msix_vec = 0;
        println!("this is a memory barrier lmfao");

        avail.flags = 0;
        println!("this is a memory barrier lmfao");
        queue.enable = 1;
        println!("this is a memory barrier lmfao");

        // println!("{}", unsafe { addr_of!(queue.notify_off).read_unaligned() });
        let notif = unsafe { notif.byte_add(queue.notify_off as usize * factor as usize) };

        Self {
            descs: desc_arr,
            used,
            avail,
            fuckwit,
            last_used: 0,
            notif,
        }
    }
}

pub fn setup_logging_with_clock<F>(filter: &str, since_startup: F)
where
    F: Fn() -> Instant + Send + Sync + 'static,
{
    // Builder::new()
    //     .format(move |buf, record| {
    //         let elapsed = since_startup();
    //         let timestamp = format!("[{elapsed}]");
    //         if record.target().starts_with("smoltcp::") {
    //             writeln!(
    //                 buf,
    //                 "\x1b[0m{} ({}): {}\x1b[0m",
    //                 timestamp,
    //                 record.target().replace("smoltcp::", ""),
    //                 record.args()
    //             )
    //         } else if record.level() == Level::Trace {
    //             let message = format!("{}", record.args());
    //             writeln!(
    //                 buf,
    //                 "\x1b[37m{} {}\x1b[0m",
    //                 timestamp,
    //                 message.replace('\n', "\n             ")
    //             )
    //         } else {
    //             writeln!(
    //                 buf,
    //                 "\x1b[32m{} ({}): {}\x1b[0m",
    //                 timestamp,
    //                 record.target(),
    //                 record.args()
    //             )
    //         }
    //     })
    //     .filter(None, LevelFilter::Trace)
    //     .parse_filters(filter)
    //     .parse_env("RUST_LOG")
    //     .init();
}

pub fn setup_logging(filter: &str) {
    setup_logging_with_clock(filter, Instant::now)
}

use log::{Metadata, Record};

struct SimpleLogger;

impl log::Log for SimpleLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Info
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            println!("{} - {}", record.level(), record.args());
        }
    }

    fn flush(&self) {}
}

use log::SetLoggerError;

static LOGGER: SimpleLogger = SimpleLogger;

pub fn init() -> Result<(), SetLoggerError> {
    log::set_logger(&LOGGER).map(|()| log::set_max_level(LevelFilter::Trace))
}

fn main() {
    // panic::set_hook(Box::new(|panic_info| {
    //     println!("panic occurred: {panic_info}");
    //     loop {}
    // }));
    // log

    // #[cfg(feature = "log")]
    // setup_logging("");
    // simple_logger::init().unwrap();
    init().unwrap();

    let mut device = Dev::init();

    // Create interface
    let mut config = match device.capabilities().medium {
        Medium::Ethernet => {
            Config::new(EthernetAddress([0x02, 0x00, 0x00, 0x00, 0x00, 0x01]).into())
        }
        Medium::Ip => Config::new(smoltcp::wire::HardwareAddress::Ip),
        Medium::Ieee802154 => todo!(),
    };

    config.random_seed = 666;

    let mut iface = Interface::new(config, &mut device, Instant::now());

    // // Create sockets
    // let mut dhcp_socket = dhcpv4::Socket::new();

    // // Set a ridiculously short max lease time to show DHCP renews work properly.
    // // This will cause the DHCP client to start renewing after 5 seconds, and give up the
    // // lease after 10 seconds if renew hasn't succeeded.
    // // IMPORTANT: This should be removed in production.
    // dhcp_socket.set_max_lease_duration(Some(Duration::from_secs(10)));

    // let mut sockets = SocketSet::new(vec![]);
    // let dhcp_handle = sockets.add(dhcp_socket);

    // loop {
    //     let timestamp = Instant::now();
    //     iface.poll(timestamp, &mut device, &mut sockets);

    //     let event = sockets.get_mut::<dhcpv4::Socket>(dhcp_handle).poll();
    //     match event {
    //         None => {}
    //         Some(dhcpv4::Event::Configured(config)) => {
    //             println!("DHCP config acquired!");

    //             println!("IP address:      {}", config.address);
    //             set_ipv4_addr(&mut iface, config.address);

    //             if let Some(router) = config.router {
    //                 println!("Default gateway: {}", router);
    //                 iface.routes_mut().add_default_ipv4_route(router).unwrap();
    //             } else {
    //                 println!("Default gateway: None");
    //                 iface.routes_mut().remove_default_ipv4_route();
    //             }

    //             for (i, s) in config.dns_servers.iter().enumerate() {
    //                 println!("DNS server {}:    {}", i, s);
    //             }
    //         }
    //         Some(dhcpv4::Event::Deconfigured) => {
    //             println!("DHCP lost config!");
    //             set_ipv4_addr(&mut iface, Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0));
    //             iface.routes_mut().remove_default_ipv4_route();
    //         }
    //     }

    //     // phy_wait(fd, iface.poll_delay(timestamp, &sockets)).expect("wait error");
    // }

    iface.update_ip_addrs(|ip_addrs| {
        ip_addrs
            .push(IpCidr::new(IpAddress::v4(10, 0, 2, 15), 24))
            .unwrap();
        // ip_addrs
        //     .push(IpCidr::new(IpAddress::v6(0xfdaa, 0, 0, 0, 0, 0, 0, 1), 64))
        //     .unwrap();
        // ip_addrs
        //     .push(IpCidr::new(IpAddress::v6(0xfe80, 0, 0, 0, 0, 0, 0, 1), 64))
        //     .unwrap();
    });
    iface.set_any_ip(true);
    iface
        .routes_mut()
        .add_default_ipv4_route(Ipv4Address::new(192, 168, 69, 100))
        .unwrap();
    iface
        .routes_mut()
        .add_default_ipv6_route(Ipv6Address::new(0xfe80, 0, 0, 0, 0, 0, 0, 0x100))
        .unwrap();

    // Create sockets
    let udp_rx_buffer = udp::PacketBuffer::new(
        vec![udp::PacketMetadata::EMPTY, udp::PacketMetadata::EMPTY],
        vec![0; 65535],
    );
    let udp_tx_buffer = udp::PacketBuffer::new(
        vec![udp::PacketMetadata::EMPTY, udp::PacketMetadata::EMPTY],
        vec![0; 65535],
    );
    let udp_socket = udp::Socket::new(udp_rx_buffer, udp_tx_buffer);

    let tcp1_rx_buffer = tcp::SocketBuffer::new(vec![0; 64]);
    let tcp1_tx_buffer = tcp::SocketBuffer::new(vec![0; 128]);
    let tcp1_socket = tcp::Socket::new(tcp1_rx_buffer, tcp1_tx_buffer);

    let tcp2_rx_buffer = tcp::SocketBuffer::new(vec![0; 64]);
    let tcp2_tx_buffer = tcp::SocketBuffer::new(vec![0; 128]);
    let tcp2_socket = tcp::Socket::new(tcp2_rx_buffer, tcp2_tx_buffer);

    let tcp3_rx_buffer = tcp::SocketBuffer::new(vec![0; 65535]);
    let tcp3_tx_buffer = tcp::SocketBuffer::new(vec![0; 65535]);
    let tcp3_socket = tcp::Socket::new(tcp3_rx_buffer, tcp3_tx_buffer);

    let tcp4_rx_buffer = tcp::SocketBuffer::new(vec![0; 65535]);
    let tcp4_tx_buffer = tcp::SocketBuffer::new(vec![0; 65535]);
    let tcp4_socket = tcp::Socket::new(tcp4_rx_buffer, tcp4_tx_buffer);

    let mut sockets = SocketSet::new(vec![]);
    let udp_handle = sockets.add(udp_socket);
    let tcp1_handle = sockets.add(tcp1_socket);
    let tcp2_handle = sockets.add(tcp2_socket);
    let tcp3_handle = sockets.add(tcp3_socket);
    let tcp4_handle = sockets.add(tcp4_socket);

    let mut tcp_6970_active = false;
    loop {
        let timestamp = Instant::now();
        iface.poll(timestamp, &mut device, &mut sockets);

        // udp:6969: respond "hello"
        let socket = sockets.get_mut::<udp::Socket>(udp_handle);
        if !socket.is_open() {
            socket.bind(6969).unwrap()
        }

        let client = match socket.recv() {
            Ok((data, endpoint)) => {
                println!("udp:6969 recv data: {:?} from {}", data, endpoint);
                let mut data = data.to_vec();
                data.reverse();
                Some((endpoint, data))
            }
            Err(_) => None,
        };
        if let Some((endpoint, data)) = client {
            println!("udp:6969 send data: {:?} to {}", data, endpoint,);
            socket.send_slice(&data, endpoint).unwrap();
        }

        // tcp:6969: respond "hello"
        let socket = sockets.get_mut::<tcp::Socket>(tcp1_handle);
        if !socket.is_open() {
            socket.listen(6969).unwrap();
        }

        if socket.can_send() {
            println!("tcp:6969 send greeting");
            // writeln!(socket, "hello").unwrap();
            println!("tcp:6969 close");
            socket.close();
        }

        // tcp:6970: echo with reverse
        let socket = sockets.get_mut::<tcp::Socket>(tcp2_handle);
        if !socket.is_open() {
            socket.listen(6970).unwrap()
        }

        if socket.is_active() && !tcp_6970_active {
            println!("tcp:6970 connected");
        } else if !socket.is_active() && tcp_6970_active {
            println!("tcp:6970 disconnected");
        }
        tcp_6970_active = socket.is_active();

        if socket.may_recv() {
            let data = socket
                .recv(|buffer| {
                    let recvd_len = buffer.len();
                    let mut data = buffer.to_owned();
                    if !data.is_empty() {
                        println!("tcp:6970 recv data: {:?}", data);
                        data = data.split(|&b| b == b'\n').collect::<Vec<_>>().concat();
                        data.reverse();
                        data.extend(b"\n");
                    }
                    (recvd_len, data)
                })
                .unwrap();
            if socket.can_send() && !data.is_empty() {
                println!("tcp:6970 send data: {:?}", data);
                socket.send_slice(&data[..]).unwrap();
            }
        } else if socket.may_send() {
            println!("tcp:6970 close");
            socket.close();
        }

        // tcp:6971: sinkhole
        let socket = sockets.get_mut::<tcp::Socket>(tcp3_handle);
        if !socket.is_open() {
            socket.listen(6971).unwrap();
            socket.set_keep_alive(Some(Duration::from_millis(1000)));
            socket.set_timeout(Some(Duration::from_millis(2000)));
        }

        if socket.may_recv() {
            socket
                .recv(|buffer| {
                    if !buffer.is_empty() {
                        println!("tcp:6971 recv {:?} octets", buffer.len());
                    }
                    (buffer.len(), ())
                })
                .unwrap();
        } else if socket.may_send() {
            socket.close();
        }

        // tcp:6972: fountain
        let socket = sockets.get_mut::<tcp::Socket>(tcp4_handle);
        if !socket.is_open() {
            socket.listen(6972).unwrap()
        }

        if socket.may_send() {
            socket
                .send(|data| {
                    if !data.is_empty() {
                        println!("tcp:6972 send {:?} octets", data.len());
                        for (i, b) in data.iter_mut().enumerate() {
                            *b = (i % 256) as u8;
                        }
                    }
                    (data.len(), ())
                })
                .unwrap();
        }

        // phy_wait(fd, iface.poll_delay(timestamp, &sockets)).expect("wait error");
    }

    // println!("hi guys, a is {}", { a });
    // println!("addr of a {:?}", core::ptr::addr_of!(a));

    // let rt = tokio::runtime::Runtime::new() // .enable_all()
    //     // .build()
    //     .unwrap()
    //     // let guard = rt.enter();
    //     // tokio::spawn(async {println!("lmao")});
    //     // rt.shutdown_timeout(std::time::Duration::from_secs(10));
    //     .block_on(async {
    //         println!("Hello world");
    //     });
    // println!("and it comes to an end");
}

fn set_ipv4_addr(iface: &mut Interface, cidr: Ipv4Cidr) {
    iface.update_ip_addrs(|addrs| {
        addrs.clear();
        addrs.push(IpCidr::Ipv4(cidr)).unwrap();
    });
}
