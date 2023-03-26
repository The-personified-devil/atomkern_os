#![feature(int_roundings)]
#![feature(pointer_byte_offsets)]
#![feature(inline_const)]
#![feature(const_for)]
#![feature(const_mut_refs)]

use array_init::array_init;
use bitvec::array::BitArray;
use bitvec::prelude::*;
use core::ptr::null_mut;
use intrusive_collections::{intrusive_adapter, LinkedList, LinkedListLink};
use std::num::NonZeroUsize;
use std::ptr::{addr_of, addr_of_mut};
use std::sync::atomic::*;
use std::cell::RefCell;

// TODO: Make more dynamic, just set for now
const SEGMENT_SIZE: usize = 0x2000000; // 32 MiB
const SLICE_SIZE: usize = 0x10000; // 64 KiB

const SLICES_PER_SEGMENT: usize = SEGMENT_SIZE / SLICE_SIZE;

const SMALL_PAGE_SIZE: usize = SLICE_SIZE;
const MEDIUM_PAGE_SIZE: usize = SLICE_SIZE * 8;

const SMALL_OBJ_MAX_SIZE: usize = SMALL_PAGE_SIZE / 4;
const MEDIUM_OBJ_MAX_SIZE: usize = MEDIUM_PAGE_SIZE / 4;

const META_SLICE_COUNT: usize = core::mem::size_of::<SegmentMeta>().div_ceil(SLICE_SIZE);

const SLICE_MAP_VALUE_COUNT: usize = SLICES_PER_SEGMENT.div_ceil(64);

type SliceMap = BitArray<[u64; SLICE_MAP_VALUE_COUNT], Lsb0>;

// TODO: Resolve assumptions about the size of usize, particularly in relation to block size

// Terminology:
//
// A Block is a unit that can be allocated
// A Slice is a fixed size unit
// A Page is a unit that contains a number of blocks, differs greatly from an os page and is
// represented as one or more slices
// A Segment is a unit that contains a fixed number of slices, so a variable number of pages
// A Heap is a per thread structure that contains everything allocated by that thread

// We could save a bit by not ever storing the metadata pages, but that'd have recursive (if
// limited) effects, and makes calculations more difficult, providing little benefit in terms of
// metadata overhead
struct SegmentMeta {
    // Should this count the pages in use or slices in use
    used: usize,
    used_map: SliceMap,
    slice_meta: [SliceMeta; SLICES_PER_SEGMENT],
}

impl SegmentMeta {
    // Great way to blow up the stack, causing the one thing rust doesn't properly protect against
    // (except my stupidity that is)
    fn new() -> Self {
        // TODO: Should work, but unfuck anyway
        let used_map_init: SliceMap = {
            let mut value: u64 = 0;
            for i in 0..META_SLICE_COUNT {
                value |= 1 << i;
            }

            let mut arr = [0; SLICE_MAP_VALUE_COUNT];
            arr[0] = value;

            BitArray::new(arr)
        };

        Self {
            used: META_SLICE_COUNT,
            used_map: used_map_init,
            slice_meta: [const { SliceMeta::new() }; SLICES_PER_SEGMENT],
        }
    }

    fn is_empty(self: &Self) -> bool {
        // Not necessary, just some additional safety
        self.used == META_SLICE_COUNT && self.used_map.leading_ones() == META_SLICE_COUNT
    }

    // This entire function is highly unsafe, but if nobody fucked with the source code too much it
    // should be valid, but this design aspect obviously undermines quite a bit of rusts safety
    // The total lack of mutable on the params makes this an even bigger safety nightmare,
    // but we need to make it work ¯\_(ツ)_/¯
    fn get_page_data(self: &Self, page: &SliceMeta) -> &mut [Block] {
        let page_addr = addr_of!(*page);
        let arr_addr = addr_of!(self.slice_meta[0]);

        // Ensure abs goes the right way, bla bla
        let idx = unsafe { page_addr.offset_from(arr_addr) }.abs() as usize;

        let start_ptr = unsafe {
            addr_of!(*self)
                .byte_add(idx * SLICE_SIZE)
                .cast::<Block>()
                .cast_mut()
        };

        unsafe { core::slice::from_raw_parts_mut(start_ptr, page_kind_to_size(page.kind)) }
    }
}

// Shitty linked list based on indices, to ease validity checks
struct Block {
    next: usize,
}

#[derive(Clone, Copy, PartialEq, PartialOrd)]
enum PageKind {
    Small,
    Medium,
    Large,
}

// This primarily contains information about a page, but because it's stored for every slice we
// refer to it by the term slice
struct SliceMeta {
    link: LinkedListLink,
    kind: PageKind,
    // Could potentially use a smaller integer for this, but why bother, waste is cool, no?
    // Includes blocks in xthread_free_list
    used: usize,
    // Preferrably use newtype for index
    // Use index as rust does not take kindly to pointers (and this removes a couple points of
    // failure)
    // Should we model this as a regular intrusive singly linked list?
    // Use usize::MAX as special value, as block 0 being free is legitimate
    // Probably turn into intrusive singly linked list, idk
    // Might intrusive ll actually be safer, due to get_segment shenaniganry, prolly not though?
    free_block_list: usize,
    // We can't use an Option here as there is not atomic variant of it, probably for the better
    xthread_free_list: AtomicUsize,
    // Currently usable blocks, may be smaller than actual number of blocks stored in page
    capacity: usize,
    // Easiest just to store here, although conflicts with rusts memory model a bit, but what gives
    block_size: usize,
}

impl SliceMeta {
    const fn new() -> Self {
        Self {
            link: LinkedListLink::new(),
            kind: PageKind::Small,
            used: 0,
            free_block_list: usize::MAX,
            xthread_free_list: AtomicUsize::new(0),
            capacity: 0,
            // Should not matter, but set it to 8 bytes anyway, i.e. one Block data type (ideally)
            block_size: 8,
        }
    }

    fn is_empty(self: &Self) -> bool {
        self.used == 0
    }

    fn merge_xthread_free(self: &mut Self, slice_data: &mut [Block]) {
        let first_block = self.xthread_free_list.swap(usize::MAX, Ordering::Relaxed);

        if first_block == usize::MAX {
            return;
        }

        // Potentially limit amount merged at once, or store the amount of blocks in the
        // xthread_free_list separately, although this would probably cause atomic bugs
        let mut value = first_block;
        let mut last_block: Option<&mut Block> = None;
        while value != usize::MAX {
            self.used += 1;
            // Bounds check, providing safety
            let block = &mut slice_data[value];
            value = block.next;

            last_block = Some(block);
        }

        // Because we exit early if there's block on the xthread_free_list, this will always be
        // Some
        last_block.unwrap().next = self.free_block_list;
        self.free_block_list = first_block;
    }

    fn try_alloc_internal(self: &mut Self, slice_data: &mut [Block]) -> Option<usize> {
        let free = self.free_block_list;
        // Ugh, magic values
        if free == usize::MAX {
            return None;
        }

        let block = &mut slice_data[free];
        self.free_block_list = block.next;
        self.used += 1;

        Some(free)
    }

    fn block_idx_to_ptr(block_idx: usize, slice_data: &mut [Block]) -> *mut u8 {
        addr_of_mut!(slice_data[block_idx]).cast::<u8>()
    }

    // Deduplicate the map lmao
    fn try_alloc(self: &mut Self, slice_data: &mut [Block]) -> Option<*mut u8> {
        if let Some(x) = self.try_alloc_internal(slice_data) {
            return Some(Self::block_idx_to_ptr(x, slice_data));
        }

        if let Some(_) = self.try_extend_free(slice_data) {
            // This should not fail but if it does, nothing goes wrong anyway
            return self
                .try_alloc_internal(slice_data)
                .map(|x| Self::block_idx_to_ptr(x, slice_data));
        }

        self.merge_xthread_free(slice_data);

        self.try_alloc_internal(slice_data)
            .map(|x| Self::block_idx_to_ptr(x, slice_data))
    }

    fn try_extend_free(self: &mut Self, slice_data: &mut [Block]) -> Option<()> {
        if self.capacity >= page_kind_to_size(self.kind) / self.block_size {
            return None;
        }

        // Unmath this maybe
        let block_idx = self.capacity * self.block_size / 8;

        let mut previous = self.free_block_list;
        for (i, block) in slice_data
            [block_idx..(block_idx + 100).min(page_kind_to_size(self.kind) / 8)]
            .iter_mut()
            .enumerate()
        {
            block.next = previous;
            previous = block_idx + i;
        }

        Some(())
    }
}

fn page_kind_to_size(kind: PageKind) -> usize {
    match kind {
        PageKind::Small => SMALL_PAGE_SIZE,
        PageKind::Medium => MEDIUM_PAGE_SIZE,
        PageKind::Large => unreachable!(),
    }
}

const SMALL_SIZE_BIN_COUNT: usize = 128;
const SMALL_WORDSIZE_MAX: usize = SMALL_SIZE_BIN_COUNT;
// TODO: Make dynamic lmfao
const WORD_SIZE: usize = 8;

const BIN_COUNT: usize = 73;

intrusive_adapter!(PageAdapter<'a> = &'a SliceMeta: SliceMeta { link: LinkedListLink });

struct Queue<'a> {
    list: LinkedList<PageAdapter<'a>>,
    size: usize,
}

impl Queue<'_> {
    fn new() -> Self {
        Self {
            list: LinkedList::new(PageAdapter::new()),
            size: 0,
        }
    }
}

// Awful trick copied from mimalloc that relies on segments being 4 MiB aligned, which they should
// be, but we really have no way of verifying that
// I do not have the smallest clue how that livetime makes it work, but if it works, sure, great
fn get_segment<'a>(page: &mut SliceMeta) -> &'a mut SegmentMeta {
    unsafe { &mut *((addr_of_mut!(*page) as usize & !(0x400000)) as *mut SegmentMeta) }
}

struct Heap<'a> {
    small_size_map: [*mut SliceMeta; SMALL_SIZE_BIN_COUNT],
    bin_map: [Queue<'a>; BIN_COUNT],
    full_bin: Queue<'a>,
}

impl Heap<'_> {
    fn new() -> Self {
        Self {
            small_size_map: [null_mut(); SMALL_SIZE_BIN_COUNT],
            bin_map: array_init(|_| Queue::new()),
            full_bin: Queue::new(),
        }
    }

    fn allocate_small(self: &mut Self, wsize: usize) -> Option<*mut u8> {
        let small_size = self.small_size_map[wsize];
        if small_size.is_null() {
            return None;
        }

        let page = unsafe { &mut *small_size };
        let segment = get_segment(page);
        let data = segment.get_page_data(page);

        page.try_alloc(data)
    }

    fn allocate(self: &mut Self, size: usize) -> *mut u8 {
        let wsize = size / WORD_SIZE;
        if wsize <= SMALL_WORDSIZE_MAX {
            // We'll find surely find a way to allocate memory if this doesn't succeed - Duck, 2023 (<- clueless)
            if let Some(addr) = self.allocate_small(wsize) {
                return addr;
            }
        }

        null_mut()
    }

    fn alloc_generic(self: &mut Self, wsize: usize) -> *mut u8 {
        null_mut()
    }

}

struct SegmentTld {
    small_free: Queue<'static>,
    medium_large_free: Queue<'static>,
}

struct Arena {
    block_count: AtomicUsize,
    dirty: [AtomicU64; 16],
    used: [AtomicU64; 16],
    // Has to be 4MiB aligned, due to segment_from_page shenaniganry
    start: AtomicPtr<u8>,
}

const BLOCK_SIZE: usize = SEGMENT_SIZE;

impl Arena {
    // TODO: Already searched bits optimization
    fn try_find_claim(self: &mut Self) -> Option<u64> {
        let value_count = self.block_count.load(Ordering::Relaxed).div_ceil(64);
        for (i, value) in self.used[..value_count].iter().enumerate() {
            let old = value.load(Ordering::Relaxed);
            let leading = old.leading_ones();

            if leading == 64 {
                continue;
            }

            let mask = 0b1 << (64 - leading - 1);
            let new = old | mask;

            if let Ok(bitidx) =
                value.compare_exchange_weak(old, new, Ordering::AcqRel, Ordering::Acquire)
            {
                return Some(i as u64 * 64 + bitidx);
            }
        }

        None
    }

    fn alloc_new_from_os(self: &mut Self) -> Option<u64> {
        // Increment immediately, we don't want another thread trying to allocate the same block
        let old_block_count = self.block_count.fetch_add(1, Ordering::Relaxed);

        // At start of block to be allocated
        let addr = unsafe {
            self.start
                .load(Ordering::Relaxed)
                .byte_add(BLOCK_SIZE * old_block_count)
        };

        // TODO: Actually alloc from os
        None
    }

    fn alloc_new_unclaimed(self: &mut Self) -> Option<u64> {
        let idx = self.alloc_new_from_os()?;

        let value_idx = idx / 64;
        let value = &self.used[value_idx as usize];

        let old = value.load(Ordering::Relaxed);

        let bit_shift = 64 - (idx - value_idx) - 1;
        // Unset bit corresponding to block, marking it as free
        let new = old & !(0b1 << bit_shift);

        value
            .compare_exchange_weak(old, new, Ordering::AcqRel, Ordering::Acquire)
            .ok()
    }

    fn new() -> Self {
        // Storing blocks that the allocator has no knowledge about as used allows us to not have
        // to check whether we're going out of bound of our blocks in the middle of a bitmap value
        Self {
            block_count: AtomicUsize::new(0),
            dirty: [const { AtomicU64::new(!0) }; 16],
            used: [const { AtomicU64::new(!0) }; 16],
            start: AtomicPtr::new(null_mut()),
        }
    }
}

thread_local! {
    pub static TLD: RefCell<*mut Tld<'static>> = RefCell::new(init());
}

fn sys_alloc(addr: u64, size: u64) {
    unsafe {
        do_syscall(2, 0, addr, 0, size);
    }
}

struct Tld<'a> {
    segment: SegmentTld,
    heap: Heap<'a>,
}

impl Tld<'_> {
    fn new() -> Self {
        Self { segment: SegmentTld { small_free: Queue::new(), medium_large_free: Queue::new() }, heap: Heap::new() }
    }
}

fn init() -> *mut Tld<'static> {
    // randomly guess for good measure lmao
    sys_alloc(0xBFFFFFFF, 20);
    let ptr = 0xBFFFFFFF as *mut Tld;
    *unsafe {&mut *ptr} = Tld::new();
    ptr
}
