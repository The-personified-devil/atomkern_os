use crate::println;
use arrayvec::ArrayVec;
use core::slice;
use nom::branch::alt;
use nom::bytes::complete::{tag, take};
use nom::combinator::{map, not};
use nom::number::complete::{le_u32, le_u8};
use nom::sequence::{preceded, tuple};
use nom::IResult;

#[derive(Clone, Copy, Debug)]
#[repr(C, packed)]
struct RSDP2 {
    pub signature: [u8; 8],
    pub checksum: u8,
    pub oemid: [u8; 6],
    pub revision: u8,
    pub rsdt_addr: u32,

    pub length: u32,
    pub xsdt_addr: u64,
    pub extended_checksum: u8,
    pub reserved: [u8; 3],
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
struct SystemDescriptionHeader {
    signature: [u8; 4],
    length: u32,
    revision: u8,
    checksum: u8,
    oem_id: [u8; 6],
    oem_table_id: [u8; 8],
    oem_revision: u32,
    creator_id: u32,
    creator_revision: u32,
    data: [u8; 0],
}

#[derive(Clone, Copy, Debug)]
#[repr(C, packed)]
pub struct PcieRef {
    pub addr: u64,
    pub segment_group: u16,
    pub start_bus: u8,
    pub end_bus: u8,
}

fn to_skip(data: u8) -> ParseReturn {
    ParseReturn::Skip(data)
}

#[derive(Clone, Debug)]
#[repr(C)]
struct ApicHeader {
    pub entry_type: u8,
    pub length: u8,
}

#[derive(Debug)]
pub struct X2Apic {
    pub processor_id: u32,
    pub flags: u32,
    pub apic_id: u32,
}

fn to_apic(data: (u8, u8, u32)) -> ParseReturn {
    println!("to_apic");
    ParseReturn::X2Apic(X2Apic {
        processor_id: data.0.into(),
        flags: data.2,
        apic_id: data.1.into(),
    })
}

fn to_x2apic(data: (u32, u32, u32)) -> ParseReturn {
    println!("to_x2apic");
    ParseReturn::X2Apic(X2Apic {
        processor_id: data.2,
        flags: data.1,
        apic_id: data.0,
    })
}

enum ParseReturn {
    // Apic(Apic),
    X2Apic(X2Apic),
    Skip(u8),
}

fn parse(input: &[u8]) -> IResult<&[u8], X2Apic> {
    let entry_type = alt((
        map(
            preceded(
                tuple((tag([9u8, 16]), take(2usize))),
                tuple((le_u32, le_u32, le_u32)),
            ),
            to_x2apic,
        ),
        map(
            preceded(tag([0u8, 8]), tuple((le_u8, le_u8, le_u32))),
            to_apic,
        ),
        map(preceded(not(tag([255u8])), le_u8), to_skip),
    ))(input)?;

    match entry_type {
        // (remaining, ParseReturn::Apic(apic)) => Ok((remaining, apic)),
        (remaining, ParseReturn::X2Apic(apic)) => Ok((remaining, apic)),
        (remaining, ParseReturn::Skip(skip)) => {
            parse(take(skip.checked_sub(2).unwrap_or(0))(remaining)?.0)
        }
    }
}

struct Table {
    data: &'static [u8],
}

pub struct Acpi {
    apic: Table,
    pcie: Table,
}

impl Acpi {
    pub fn parse(rdsp_addr: u64) -> Self {
        let rdsp = unsafe { &*((crate::PHYS_OFFSET + rdsp_addr).as_ptr::<RSDP2>()) };
        println!("{:?}", rdsp);

        let xsdt =
            unsafe { &*(crate::PHYS_OFFSET + rdsp.xsdt_addr).as_ptr::<SystemDescriptionHeader>() };
        println!("{:?}", xsdt);

        let length = unsafe { core::ptr::addr_of!(xsdt.length).read_unaligned() };

        fn check_signature(ptr: *const SystemDescriptionHeader) {
            let header = unsafe { &*ptr };
            let length = unsafe { core::ptr::addr_of!(header.length).read_unaligned() };
            let sum = unsafe { core::slice::from_raw_parts(ptr as *const u8, length as usize) }
                .iter()
                .fold(0_u64, |acc, x| acc + (*x) as u64);
            assert_eq!(sum % 0x100, 0);
        }

        let mut tables = ArrayVec::<_, 100>::new();
        for offset in
            (0..length as usize - core::mem::size_of::<SystemDescriptionHeader>()).step_by(8)
        {
            let addr = unsafe {
                ((core::ptr::addr_of!(xsdt.data).addr() + offset) as *const u64).read_unaligned()
            };
            let table =
                unsafe { &*(crate::PHYS_OFFSET + addr).as_ptr::<SystemDescriptionHeader>() };

            check_signature(table);
            tables.push(table)
        }

        let pcie = tables
            .iter()
            .find(|x| core::str::from_utf8(x.signature.as_slice()).unwrap() == "MCFG")
            .unwrap();

        for table in tables.iter() {
            let signature = core::str::from_utf8(table.signature.as_slice()).unwrap();
            println!("{}:, {:?}", signature, table);
        }

        let mut apic = None;
        let mut pcie = None;
        for table in tables.iter() {
            let data = unsafe {
                slice::from_raw_parts(
                    core::ptr::addr_of!(table.data).cast::<u8>(),
                    (table.length as usize) - core::mem::size_of::<SystemDescriptionHeader>(),
                )
            };

            match &table.signature {
                b"APIC" => apic = Some(Table { data }),
                b"MCFG" => pcie = Some(Table { data }),
                _ => (),
            }
        }

        Acpi {
            apic: apic.unwrap(),
            pcie: pcie.unwrap(),
        }
    }

    pub fn get_apics(&self) -> ArrayVec<X2Apic, 256> {
        let mut apics = ArrayVec::<X2Apic, 256>::new();

        let mut remainder = &self.apic.data[8..];
        while let Ok((remaining, apic)) = parse(remainder).map_err(|x| println!("{:?}", x)) {
            remainder = remaining;
            apics.push(apic);
        }

        apics.iter().for_each(|x| println!("{:?}", x));

        apics
    }

    pub fn get_pcie_configs(&self) -> ArrayVec<PcieRef, 256> {
        let data = &self.pcie.data[8..];

        let mut configs = ArrayVec::<PcieRef, 256>::new();
        let mut data = data;
        while data.len() >= core::mem::size_of::<PcieRef>() {
            let config = unsafe { &*(data.as_ptr() as *const PcieRef) };
            configs.push(config.clone());
            data = &data[core::mem::size_of::<PcieRef>()..];
        }

        println!("configs: {:?}", configs);

        configs
    }
}
