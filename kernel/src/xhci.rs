use crate::pcie::MsiCaps;
use crate::pcie::MsiReg;
use crate::println;
use bitvec::prelude::*;
use core::ptr::{addr_of, addr_of_mut};
use num_enum::{IntoPrimitive, TryFromPrimitive};
use proc_bitfield::bitfield;
use x86_64::{registers::model_specific::Msr, structures::idt::InterruptStackFrame};

#[derive(Clone, Debug)]
#[repr(C)]
pub struct HostCaps {
    pub length: u8,
    pub _res1: u8,
    pub version: u16,
    pub sparams1: HParams1,
    pub sparams2: HParams2,
    pub sparams3s: HParams3,
    pub cparams1: CParams1,
    pub dorbell_off: u32,
    pub rt_regs_off: u32,
    pub cparams2: CParams2,
}

bitfield! {
    #[derive(Clone)]
    pub struct HParams1(u32): Debug {
        pub max_slots: u8 @ 0..=7,
        pub max_intrs: u16 @ 8..=18,
        pub max_ports: u8 @ 24..=31,
    }
}

bitfield! {
    #[derive(Clone)]
    pub struct HParams2(u32): Debug {
        pub sched_tresh: u8 @ 0..=3,
        pub max_ring_segment_tbl: u8 @ 4..=7,
        pub max_scratch_bufs_hi: u8 @ 21..25,
        pub scratch_restore: bool @ 26,
        pub max_scratch_bufs_lo: u8 @ 27..=31,
    }
}

#[derive(Clone, Debug)]
#[repr(C)]
pub struct HParams3 {
    pub u1_exit_latency: u8,
    pub _res: u8,
    pub u2_exit_latency: u16,
}

bitfield! {
    #[derive(Clone)]
    pub struct CParams1(u32): Debug {
        pub addressing64: bool @ 0,
        pub bw_negotiation: bool @ 1,
        pub context_size: bool @ 2,
        pub port_pwr_ctrl: bool @ 3,
        pub port_indicator: bool @ 4,
        pub light_hc_reset: bool @ 5,
        pub latency_tolerance: bool @ 6,
        pub no_secondary_sid: bool @ 7,
        pub parse_all_event_data: bool @ 8,
        pub stopped_short_packet: bool @ 9,
        pub stopped_edtla: bool @ 10,
        pub contiguous_frame_id: bool @ 11,
        pub max_prim_stream_size: u8 @ 12..=15,
        pub xhci_ext_caps_ptr: u16 @ 16..=31,
    }
}

bitfield! {
    #[derive(Clone)]
    pub struct CParams2(u32): Debug {
        pub u3_entry: bool @ 0,
        pub max_exit_too_large: bool @ 1,
        pub force_save_context: bool @ 2,
        pub compliance_trans: bool @ 3,
        pub large_esit: bool @ 4,
        pub config_info: bool @ 5,
        pub extended_tbc: bool @ 6,
        pub extended_tbc_trb: bool @ 7,
        pub extended_property: bool @ 8,
        pub virtualization: bool @ 9,
    }
}

#[derive(Clone, Debug)]
#[repr(C, packed)]
pub struct OpRegs {
    pub cmd: UsbCmd,
    pub status: UsbStatus,
    pub page_size: PageSize,
    pub _res1: u64,
    pub notification: bitvec::array::BitArray<[u16; 1]>,
    pub _res2: u16,
    pub cmd_ring: CommandRingCtrl,
    pub _res3: [u64; 2],
    pub device_ctx_ptr: u64,
    pub config: Config,
}

bitfield! {
    #[derive(Clone)]
    pub struct PageSize(u32): Debug {
        pub page_size: u16 @ 0..=15,
    }
}

bitfield! {
    #[derive(Clone)]
    pub struct Config(u32): Debug {
        pub max_slots: u8 @ 0..=7,
        pub u3_enable: bool @ 8,
        pub config_info_enable: bool @ 9,
    }
}

bitfield! {
    #[derive(Clone)]
    pub struct CommandRingCtrl(u64): Debug {
        pub ring_cycle_state: bool @ 0,
        pub command_stop: bool @ 1,
        pub cmd_abort: bool @ 2,
        pub cmd_ring_running: bool @ 3,
        pub ring_ptr: u64 @ 6..=63,
    }
}

bitfield! {
    #[derive(Clone)]
    pub struct UsbCmd(u32): Debug {
        pub run: bool @ 0,
        pub reset: bool @ 1,
        pub interrupter_enable: bool @ 2,
        pub system_error_enable: bool @ 3,
        pub light_reset: bool @ 7,
        pub controller_save: bool @ 8,
        pub restore: bool @ 9,
        pub enable_wrap: bool @ 10,
        pub enable_u3_mfindex: bool @ 11,
        pub enable_cme: bool @ 13,
        pub enable_ext_tbc: bool @ 14,
        pub enable_ext_tbc_trb: bool @ 15,
        pub enable_vtio: bool @ 16,
    }
}

bitfield! {
    #[derive(Clone)]
    pub struct UsbStatus(u32): Debug {
        pub halted: bool @ 0,
        pub system_error: bool @ 2,
        pub event_interrupt: bool @ 3,
        pub port_change_detect: bool @ 4,
        pub save_state: bool @ 8,
        pub restore_state: bool @ 9,
        pub sr_error: bool @ 10,
        pub not_ready: bool @ 11,
        pub ctrl_error: bool @ 12,
    }
}

#[derive(Debug)]
#[repr(C)]
struct RtRegs {
    mf_index: u32,
    _res1: [u32; 7],
    intregs: [IntReg; 1024],
}

#[derive(Clone, Debug)]
#[repr(C)]
struct IntReg {
    manage: IntManagement,
    moderation: IntModeration,
    event_tbl_size: TableSize,
    _res: u32,
    // rsvdp technically
    table_ptr: u64,
    deque_ptr: DequePtr,
}

#[derive(Debug)]
#[repr(C)]
struct SegmentEntry {
    addr: u64,
    size: u64,
}

bitfield! {
    #[derive(Clone)]
    struct DequePtr(u64): Debug {
        segment_idx: u8 @ 0..=2,
        busy: bool @ 3,
        ptr: u64 @ 4..=63,
    }
}

bitfield! {
    #[derive(Clone)]
    struct IntManagement(u32): Debug {
        pending: bool @ 0,
        enable: bool @ 1,
    }
}

#[derive(Clone, Debug)]
#[repr(C)]
struct IntModeration {
    interval: u16,
    counter: u16,
}

// Written as two seperate things wtf
#[derive(Clone, Debug)]
#[repr(C)]
struct TableSize {
    // size: u16,
    size: u32,
    // _res: u16,
}

#[derive(Clone, Debug)]
#[repr(C)]
struct PortSet {
    status: PortStatus,
    pw_mng: PortManagement,
    link_info: PortLinkInfo,
    extended: PortExtended,
}

bitfield! {
    #[derive(Clone)]
    struct PortStatus(u32): Debug {
        connected: bool @ 0,
        enabled: bool @ 1,
        over_current: bool @ 3,
        reset: bool @ 4,
        link_state: u8 @ 5..=8,
        power: bool @ 9,
        speed: u8 @ 10..=13,
        indicator: u8 @ 14..=15,
        write_stroble: bool @ 16,
        connect_status_change: bool @ 17,
        enabled_change: bool @ 18,
        warm_reset_change: bool @ 19,
        over_curr_change: bool @ 20,
        reset_change: bool @ 21,
        link_state_change: bool @ 22,
        config_error_change: bool @ 23,
        cold_attach_status: bool @ 24,
        wake_on_connect: bool @ 25,
        wake_on_disconnect: bool @ 26,
        wake_on_overcurrent: bool @ 27,
        removable: bool @ 30,
        warm_reset: bool @ 31,
    }
}

bitfield! {
    #[derive(Clone)]
    struct PortManagement(u32): Debug {
        u1_timeout: u8 @ 0..=7,
        u2_timeout: u8 @ 8..=15,
        force_link_pm_accept: bool @ 16,
    }
}

bitfield! {
    #[derive(Clone)]
    struct PortLinkInfo(u32): Debug {
        error_count: u16 @ 0..=15,
        rx_lane_count: u8 @ 16..=19,
        tx_lane_count: u8 @ 20..=23,
    }
}

bitfield! {
    #[derive(Clone)]
    struct PortExtended(u32): Debug {
        soft_err_count: u16 @ 0..=15,
    }
}

pub fn msi_kekw(
    allocator: &mut crate::frame::Allocator,
    device: &mut crate::pcie::Device,
    bar: u64,
    caps: *const HostCaps,
    msi: &MsiCaps,
    opregs: &mut OpRegs,
) {
    

    let trb_page = allocator.allocate().unwrap();
    let segment_table = allocator.allocate().unwrap();

    let segment_entry =
        unsafe { &mut *(crate::phys_offset() + segment_table.as_u64()).as_mut_ptr::<SegmentEntry>() };

    segment_entry.addr = trb_page.as_u64();
    segment_entry.size = 256; // one whole page

    println!("segment_entry: {:?}", segment_entry);

    let capss = unsafe { &*caps };
    // crate::mm::virt::map_page(allocator, x86_64::VirtAddr::new(caps.addr() as u64 + 4096), x86_64::PhysAddr::new())
    let rt_regs = unsafe { &mut *(caps.byte_add(capss.rt_regs_off as usize) as *mut RtRegs) };

    // println!("rt_regs {:?}", rt_regs);
    // let mut size = rt_regs.intregs[0].event_tbl_size.clone();
    // size.size = 1;

    println!("opsegs: {:?}", opregs.clone());
    unsafe {
        addr_of_mut!(rt_regs.intregs[0].event_tbl_size).write_volatile(TableSize { size: 1 });
    }

    rt_regs.intregs[0]
        .deque_ptr
        .set_ptr(segment_entry.addr >> 4);
    rt_regs.intregs[0].table_ptr = segment_table.as_u64();

    unsafe { &mut *((msi as *const MsiCaps) as *mut MsiCaps) }
        .msg_control
        .set_enable(true);

    unsafe {
        addr_of_mut!(opregs.cmd).write_volatile(opregs.cmd.clone().with_interrupter_enable(true));
        addr_of_mut!(rt_regs.intregs[0].manage)
            .write_volatile(rt_regs.intregs[0].manage.clone().with_enable(true));
    }

    unsafe { addr_of_mut!(opregs.cmd).write_volatile(opregs.cmd.clone().with_run(true)) };
    println!("opsegs: {:?}", opregs.clone());

    let portreg = unsafe { (opregs as *mut _ as *mut PortSet).byte_add(0x400) };
    let portregs = unsafe {
        core::slice::from_raw_parts_mut(
            portreg,
            addr_of!(capss.sparams1).read_volatile().max_ports().into(),
        )
    };

    println!("opsegs: {:?}", opregs.clone());
    let trb = (crate::phys_offset() + trb_page.as_u64()).as_ptr::<u8>();
    let trbs = unsafe { core::slice::from_raw_parts(trb, 4096) };
    println!("rt_reg: {:?}", rt_regs.intregs[0].clone());

    let cmd_ring = allocator.allocate().unwrap();

    let doorbellreg =
        (crate::phys_offset() + bar + capss.dorbell_off as u64).as_mut_ptr::<DoorBellReg>();

    let doorbellregs = unsafe { core::slice::from_raw_parts_mut(doorbellreg, 256) };

    crate::mm::virt::map_page(
        allocator,
        crate::phys_offset() + bar + capss.dorbell_off as u64,
        x86_64::PhysAddr::new(bar + capss.dorbell_off as u64),
    );

    let base_arr = (crate::phys_offset() + opregs.device_ctx_ptr).as_mut_ptr::<u64>();
    let base_arr = unsafe {
        core::slice::from_raw_parts_mut(base_arr, capss.sparams1.clone().max_slots().into())
    };

    opregs.cmd_ring = CommandRingCtrl(0)
        .with_ring_ptr(cmd_ring.as_u64() >> 6)
        .with_ring_cycle_state(true);
    let mut cmd_enque = (crate::phys_offset() + cmd_ring.as_u64()).as_mut_ptr::<EnableSlotCmd>();

    let transfer_ring = allocator.allocate().unwrap().as_u64();

    let intreg = &mut rt_regs.intregs[0];
    let mut deque_orig = intreg.deque_ptr.ptr() << 4;
    let mut deque_cur = deque_orig;
    let mut port = 0;
    loop {
        let event = unsafe { &mut *(crate::phys_offset() + (deque_cur)).as_mut_ptr::<Trb>() }.clone();
        println!("event {:?}", event);
        if event.remainder.cycle() {
            deque_cur += 16;

            println!("event {:?}", event);
            let trb_type = event.remainder.trb_type().unwrap();
            if trb_type == TrbType::PortStatusChangeEv {
                let event = unsafe { core::mem::transmute_copy::<_, PortChangedTrb>(&event) };
                port = event.port;
                println!("event actual {:?}", event);

                // Port == port - 1 cuz 1 indexing
                let portset = &mut portregs[event.port as usize - 1];
                let portclone = portset.clone();
                if portclone.status.enabled() || portclone.status.link_state() != 7 {
                    continue;
                }

                // put asserts here lmfao
                // assert_eq!(portregs.status.st)
                unsafe {
                    addr_of_mut!(portset.status)
                        .write_volatile(portset.status.clone().with_reset(true));
                }

                for i in 0..2 {
                    println!("e");
                }

                let enable = EnableSlotCmd {
                    _res1: [0, 0, 0],
                    meta: EnSlotMeta(0)
                        .with_cycle(true)
                        .with_slot_type(0)
                        .with_trb_type(TrbType::EnableSlotCmd),
                };

                let cmd = unsafe { &mut *cmd_enque };
                *cmd = enable;
                unsafe {
                    cmd_enque = cmd_enque.byte_add(16);
                }

                unsafe {
                    addr_of_mut!(doorbellregs[0])
                        .cast::<u32>()
                        .write_volatile(0);
                }

                println!("\nport {:?}", portset.clone());
                println!("\nopregs {:?}", opregs.clone());
                println!("\nintreg {:?}", intreg.clone());
            } else if trb_type == TrbType::CmdCompletionEv {
                let event = unsafe { core::mem::transmute_copy::<_, CmdCompletion>(&event) };
                println!("event actual {:?}", event);
                println!("device_lel {:?}", base_arr[event.slot_id as usize]);

                let cmd =
                    unsafe { (*(crate::phys_offset() + event.cmd_trb_ptr).as_ptr::<Trb>()).clone() };

                let cmd_type = cmd.remainder.trb_type().unwrap();
                if cmd_type == TrbType::EnableSlotCmd {
                    println!("got to it");

                    base_arr[event.slot_id as usize] = allocator.allocate().unwrap().as_u64();
                    let input_ctx_addr = allocator.allocate().unwrap().as_u64();
                    let input_ctx = unsafe {
                        &mut *(crate::phys_offset() + input_ctx_addr).as_mut_ptr::<InputContext>()
                    };

                    input_ctx.ctx.add_flags.set(0, true);
                    input_ctx.ctx.add_flags.set(1, true);
                    let slot = &mut input_ctx.device_ctx.slot;
                    slot.root_hub_port_num = port;
                    slot.part1.set_context_entries(1);

                    let control = &mut input_ctx.device_ctx.eps[0];
                    control.part2.set_endp_type(4);
                    // TODO: portsc map thingy
                    control.part2.set_max_packet_size(64);
                    control.part3.set_deque_ptr(transfer_ring >> 4);
                    control.part3.set_deque_cycle_state(true);
                    control.part2.set_err_count(0);

                    let cmd = SetAddrCmd {
                        input_ctx: input_ctx_addr,
                        _res: 0,
                        meta: SetAddrMeta(0)
                            .with_cycle(true)
                            .with_trb_type(TrbType::AddrDeviceCmd)
                            .with_slot_id(event.slot_id),
                    };

                    let cmd2 = unsafe { &mut *cmd_enque.cast::<SetAddrCmd>() };
                    *cmd2 = cmd;
                    unsafe {
                        cmd_enque = cmd_enque.byte_add(16);
                    }

                    unsafe {
                        addr_of_mut!(doorbellregs[0])
                            .cast::<u32>()
                            .write_volatile(0);
                    }
                } else if cmd_type == TrbType::AddrDeviceCmd {
                    let transfer_ring = (crate::phys_offset() + transfer_ring).as_mut_ptr::<Trb>();
                    let cmd = unsafe { &mut *transfer_ring.cast::<NoOp>() };
                    *cmd = NoOp {
                        ..NoOp {
                            _res1: 0,
                            int: NoOpInt(0),
                            meta: NoOpMeta(0)
                                .with_trb_type(TrbType::NoOp)
                                .with_int_on_completion(true)
                                .with_cycle(true),
                        }
                    };

                    let cmd = unsafe { &mut *transfer_ring.cast::<SetupStage>().add(1) };

                    let data = allocator.allocate().unwrap().as_u64();

                    *cmd = SetupStage {
                        ..SetupStage {
                            request_type: 0b10000000,
                            request: 6,
                            value: 1 << 8,
                            index: 0,
                            length: 4096,
                            int: SSInt(0).with_trb_transfer_len(8),
                            meta: SSMeta(0)
                                .with_cycle(true)
                                .with_trb_type(TrbType::SetupStage)
                                .with_int_on_completion(true)
                                .with_immediate_data(true)
                                .with_transfer_type(3),
                        }
                    };

                    let cmd = unsafe { &mut *transfer_ring.cast::<DataStage>().add(2) };

                    *cmd = DataStage {
                        ..DataStage {
                            data_buf: data,
                            int: DataInt(0).with_trb_len(4096).with_td_size(1),
                            meta: DataMeta(0)
                                .with_chain(true)
                                .with_cycle(true)
                                .with_int_on_completion(true)
                                .with_trb_type(TrbType::DataStage)
                                .with_dir(true),
                        }
                    };

                    let cmd = unsafe { &mut *transfer_ring.cast::<StatusStage>().add(3) };

                    *cmd = StatusStage {
                        ..StatusStage {
                            _res1: 0,
                            int: NoOpInt(0),
                            meta: StatusMeta(0)
                                .with_cycle(true)
                                .with_int_on_completion(true)
                                .with_trb_type(TrbType::StatusStage)
                                .with_dir(true),
                        }
                    };

                    let cmd = unsafe { &mut *transfer_ring.cast::<SetupStage>().add(4) };

                    *cmd = SetupStage {
                        ..SetupStage {
                            request_type: 0x21,
                            request: 0x0B,
                            value: 0,
                            index: 0,
                            length: 0,
                            int: SSInt(0).with_trb_transfer_len(8),
                            meta: SSMeta(0)
                                .with_cycle(true)
                                .with_trb_type(TrbType::SetupStage)
                                .with_int_on_completion(true)
                                .with_transfer_type(0),
                        }
                    };

                    let cmd = unsafe { &mut *transfer_ring.cast::<StatusStage>().add(5) };

                    *cmd = StatusStage {
                        ..StatusStage {
                            _res1: 0,
                            int: NoOpInt(0),
                            meta: StatusMeta(0)
                                .with_cycle(true)
                                .with_int_on_completion(true)
                                .with_trb_type(TrbType::StatusStage)
                                .with_dir(true),
                        }
                    };

                    let cmd = unsafe { &mut *transfer_ring.cast::<SetupStage>().add(6) };

                    let data2 = allocator.allocate().unwrap().as_u64();

                    *cmd = SetupStage {
                        ..SetupStage {
                            request_type: 0xA1,
                            request: 1,
                            value: 0x100,
                            index: 0,
                            length: 8,
                            int: SSInt(0).with_trb_transfer_len(8),
                            meta: SSMeta(0)
                                .with_cycle(true)
                                .with_trb_type(TrbType::SetupStage)
                                .with_int_on_completion(true)
                                .with_immediate_data(true)
                                .with_transfer_type(3),
                        }
                    };

                    let cmd = unsafe { &mut *transfer_ring.cast::<DataStage>().add(7) };

                    *cmd = DataStage {
                        ..DataStage {
                            data_buf: data2,
                            int: DataInt(0).with_trb_len(8).with_td_size(1),
                            meta: DataMeta(0)
                                .with_chain(true)
                                .with_cycle(true)
                                // .with_immediate_data(true)
                                .with_int_on_completion(true)
                                .with_trb_type(TrbType::DataStage)
                                .with_dir(true),
                        }
                    };

                    let cmd = unsafe { &mut *transfer_ring.cast::<StatusStage>().add(8) };

                    *cmd = StatusStage {
                        ..StatusStage {
                            _res1: 0,
                            int: NoOpInt(0),
                            meta: StatusMeta(0)
                                .with_cycle(true)
                                .with_int_on_completion(true)
                                .with_trb_type(TrbType::StatusStage)
                                .with_dir(true),
                        }
                    };

                    doorbellregs[event.slot_id as usize] = DoorBellReg {
                        db_target: 1,
                        _res1: 0,
                        db_task_id: 0,
                    };

                    for i in 0..3 {
                        println!("e");
                    }

                    let dev = unsafe {
                        &*(crate::phys_offset() + base_arr[event.slot_id as usize])
                            .as_ptr::<DeviceContext>()
                    };

                    let data =
                        unsafe { &*(crate::phys_offset() + data).as_ptr::<DeviceDescriptor>() };
                    println!("lel {:?}", data);

                    let data2 = unsafe {
                        core::slice::from_raw_parts((crate::phys_offset() + data2).as_ptr::<u8>(), 8)
                    };
                    println!("kb_data {:?}", data2);
                }
            }
        } else {
            unsafe {
                if deque_cur == deque_orig {
                    continue;
                }
                addr_of_mut!(intreg.deque_ptr).write_volatile(
                    intreg
                        .deque_ptr
                        .clone()
                        .with_busy(false)
                        .with_ptr(deque_cur >> 4),
                );
                deque_orig = deque_cur;
            }
        }
    }
}

#[derive(Debug)]
#[repr(C)]
struct DeviceDescriptor {
    length: u8,
    desc_type: u8,

    release_num: u16,

    class: u8,
    subclass: u8,

    protocol: u8,
    max_packet_size: u8,

    vendor_id: u16,

    product_id: u16,

    dev_release: u16,

    manufacturere_str: u8,
    product_str: u8,

    serial_str: u8,
    conf_num: u8,
}

#[derive(Debug)]
#[repr(C)]
struct SetupStage {
    request_type: u8,
    request: u8,
    value: u16,

    index: u16,
    length: u16,

    int: SSInt,
    meta: SSMeta,
}

bitfield! {
    struct SSMeta(u32): Debug {
        cycle: bool @ 0,
        int_on_completion: bool @ 5,
        immediate_data: bool @ 6,
        trb_type: u8 [try TrbType] @ 10..=15,
        transfer_type: u8 @ 16..=17,
    }
}

#[derive(Debug)]
#[repr(C)]
struct StatusStage {
    _res1: u64,
    int: NoOpInt,
    meta: StatusMeta,
}

#[derive(Debug)]
#[repr(C)]
struct DataStage {
    data_buf: u64,
    int: DataInt,
    meta: DataMeta,
}

bitfield! {
    struct DataMeta(u32): Debug {
        cycle: bool @ 0,
        eval_next_trb: bool @ 1,
        int_on_short: bool @ 2,
        no_snoop: bool @ 3,
        chain: bool @ 4,
        int_on_completion: bool @ 5,
        immediate_data: bool @ 6,
        trb_type: u8 [try TrbType] @ 10..=15,
        dir: bool @ 16,
    }
}

bitfield! {
    struct DataInt(u32): Debug {
        trb_len: u32 @ 0..=16,
        td_size: u8 @ 17..=21,
        int: u16 @ 22..=31,
    }
}

bitfield! {
    struct StatusMeta(u32): Debug {
        cycle: bool @ 0,
        eval_next_trb: bool @ 1,
        chain: bool @ 4,
        int_on_completion: bool @ 5,
        trb_type: u8 [try TrbType] @ 10..=15,
        dir: bool @ 16,
    }
}

bitfield! {
    struct SSInt(u32): Debug {
        trb_transfer_len: u32 @ 0..=16,
        interrupter: u16 @ 22..=31,
    }
}

#[derive(Debug)]
#[repr(C)]
struct InputContext {
    ctx: InputCtx,
    device_ctx: DeviceContext,
}

#[derive(Debug)]
#[repr(C)]
struct InputCtx {
    drop_flags: bitvec::array::BitArray<[u32; 1], Lsb0>,
    add_flags: bitvec::array::BitArray<[u32; 1], Lsb0>,
    _res1: [u32; 5],
    config_value: u8,
    interface_num: u8,
    alternate: u8,
    _res2: u8,
}

#[derive(Debug)]
#[repr(C)]
struct DeviceContext {
    slot: SlotContext,
    eps: [EndPointContext; 31],
}

#[derive(Debug)]
#[repr(C)]
struct EndPointContext {
    part1: EndpPart1,
    part2: EndpPart2,
    part3: EndpPart3,
    average_trb_len: u16,
    max_esit_pl_low: u16,
    _res: [u32; 3],
}

bitfield! {
    struct EndpPart1(u32): Debug {
        state: u8 @ 0..=2,
        mult: u8 @ 8..=9,
        max_p_stream: u8 @ 10..=14,
        linear_stream_array: bool @ 15,
        interval: u8 @ 16..=23,
        max_esit_high: u8 @ 24..=31,
    }
}

bitfield! {
    struct EndpPart2(u32): Debug {
        err_count: u8 @ 1..=2,
        endp_type: u8 @ 3..=5,
        host_initiate_disable: bool @ 7,
        max_burst_size: u8 @ 8..=15,
        max_packet_size: u16 @ 16..=31,
    }
}

bitfield! {
    struct EndpPart3(u64): Debug {
        deque_cycle_state: bool @ 0,
        deque_ptr: u64 @ 4..=63,
    }
}

#[derive(Debug)]
#[repr(C)]
struct SlotContext {
    part1: SlotPart1,
    max_exit_latency: u16,
    root_hub_port_num: u8,
    num_ports: u8,
    part2: SlotPart2,
    part3: SlotPart3,
    _res: [u32; 4],
}

bitfield! {
    struct SlotPart1(u32): Debug {
        route_str: u32 @ 0..=19,
        speed: u8 @ 20..=23,
        multi_tt: bool @ 25,
        hub: bool @ 26,
        context_entries: u8 @ 27..=31,
    }
}

bitfield! {
    struct SlotPart2(u32): Debug {
        parent_hub_slot: u8 @ 0..=7,
        parent_port: u8 @ 8..=15,
        think_time: u8 @ 16..=17,
        interrupter_target: u16 @ 22..=31,
    }
}

bitfield! {
    struct SlotPart3(u32): Debug {
        usb_dev_addr: u8 @ 0..=7,
        slot_state: u8 @ 27..=31,
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
struct DoorBellReg {
    db_target: u8,
    _res1: u8,
    db_task_id: u16,
}

#[derive(Debug, Clone)]
#[repr(C)]
struct CmdCompletion {
    cmd_trb_ptr: u64,
    result: CompletionResult,
    _res: u16,
    vf_id: u8,
    slot_id: u8,
}

bitfield! {
    #[derive(Clone)]
    struct CompletionResult(u32): Debug {
        param: u32 @ 0..=23,
        code: u8 @ 24..=31,
    }
}

#[derive(Debug, Clone)]
#[repr(C)]
struct Trb {
    _res: u64,
    remainder: ETrbRemainder,
}

bitfield! {
    #[derive(Clone)]
    struct ETrbRemainder(u64): Debug {
        cycle: bool @ 32,
        trb_type: u8 [try TrbType] @ 42..=47,
    }
}

#[derive(Debug)]
#[repr(C)]
struct PortChangedTrb {
    _res1: [u8; 3],
    port: u8,
    _res2: [u8; 7],
    completion_code: u8,
    meta: Meta,
}

#[derive(Debug)]
#[repr(C)]
struct EnableSlotCmd {
    _res1: [u32; 3],
    meta: EnSlotMeta,
}

#[derive(Debug)]
#[repr(C)]
struct NoOp {
    _res1: u64,
    int: NoOpInt,
    meta: NoOpMeta,
}

bitfield! {
    struct NoOpInt(u32): Debug {
        int_target: u16 @ 22..=31,
    }
}

bitfield! {
    struct NoOpMeta(u32): Debug {
        cycle: bool @ 0,
        eval_next_trb: bool @ 1,
        chain: bool @ 4,
        int_on_completion: bool @ 5,
        trb_type: u8 [try TrbType] @ 10..=15,
    }
}

#[derive(Debug)]
#[repr(C)]
struct SetAddrCmd {
    input_ctx: u64,
    _res: u32,
    meta: SetAddrMeta,
}

bitfield! {
    struct SetAddrMeta(u32): Debug {
        cycle: bool @ 0,
        do_request: bool @ 9,
        trb_type: u8 [try TrbType] @ 10..=15,
        slot_id: u8 @ 24..=31,
    }
}

bitfield! {
    struct EnSlotMeta(u32): Debug {
        cycle: bool @ 0,
        trb_type: u8 [try TrbType] @ 10..=15,
        slot_type: u8 @ 16..=20,
    }
}

bitfield! {
    struct Meta(u32): Debug {
        cycle: bool @ 0,
        trb_type: u8 [try TrbType] @ 10..=15,
    }
}

#[derive(Clone, Debug, IntoPrimitive, TryFromPrimitive, PartialEq)]
#[repr(u8)]
enum TrbType {
    // Transfer
    Normal = 1,
    SetupStage = 2,
    DataStage = 3,
    StatusStage = 4,
    Isoch = 5,
    Link = 6,
    EventData = 7,
    NoOp = 8,

    // Command
    EnableSlotCmd = 9,
    DisableSlotCmd = 10,
    AddrDeviceCmd = 11,
    ConfigEndpointCmd = 12,
    EvalContextCmd = 13,
    ResetEndpointCmd = 14,
    StopEndpointCmd = 15,
    SetTRDequePtrCmd = 16,
    ResetDeviceCmd = 17,
    ForceEventCmd = 18,
    NegotiateBWCmd = 19,
    LatencyToleranceCmd = 20,
    GetPortBWCmd = 21,
    ForceHeader = 22,
    NoOpCmd = 23,
    GetExtPropertyCmd = 24,
    SetExtPropertyCmd = 25,

    // Event
    TransferEv = 32,
    CmdCompletionEv = 33,
    PortStatusChangeEv = 34,
    BWRequestEv = 35,
    DoorbellEv = 36,
    HostCtrlEv = 37,
    DevNotificationEv = 28,
    MFIndexWrapEv = 49,
}

pub extern "x86-interrupt" fn xhci_int(_: InterruptStackFrame) {
    println!("\n\n\n\nlmao\n\n\n");
    let mut eoi = Msr::new(0x80B);
    unsafe {
        eoi.write(0);
    }
}
