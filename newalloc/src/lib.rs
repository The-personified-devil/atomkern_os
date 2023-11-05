#![no_std]

#![feature(int_roundings)]
#![feature(pointer_byte_offsets)]
#![feature(inline_const)]
#![feature(const_for)]
#![feature(const_mut_refs)]
#![feature(strict_provenance)]
#![feature(thread_local)]

use array_init::array_init;
use bitvec::array::BitArray;
use bitvec::prelude::*;
use core::ptr::null_mut;
use core::ptr::{addr_of, addr_of_mut};
use core::sync::atomic::*;
use intrusive_collections::{intrusive_adapter, LinkedList, LinkedListLink};
// use mmap_fixed::{MapOption, MemoryMap};
use atomkern_abi::do_syscall;
use cell_family::Cell;
use cell_family::Family;
use cell_family::CellOwner;

pub use cell_family;

// TODO: Extract size to wordsize into function since it's kinda complex actually

// TODO: Make more dynamic, just set for now
const SEGMENT_SIZE: usize = 0x2000000; // 32 MiB
const SLICE_SIZE: usize = 0x10000; // 64 KiB

const SLICES_PER_SEGMENT: usize = SEGMENT_SIZE / SLICE_SIZE;

const SMALL_PAGE_SIZE: usize = SLICE_SIZE;
const MEDIUM_PAGE_SIZE: usize = SLICE_SIZE * 8;

const SMALL_OBJ_MAX_SIZE: usize = SMALL_PAGE_SIZE / 4;
const MEDIUM_OBJ_MAX_SIZE: usize = MEDIUM_PAGE_SIZE / 4;

// dummy type because the size should not vary
cell_family::define!(type Dummy: DummyOwner for DummyCell<T>);
const META_SLICE_COUNT: usize = core::mem::size_of::<SegmentMeta<Dummy>>().div_ceil(SLICE_SIZE);

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
struct SegmentMeta<Key: Family> {
    // Should this count the pages in use or slices in use
    used: usize,
    used_map: SliceMap,
    slice_meta: [SliceMeta<Key>; SLICES_PER_SEGMENT],
}
// First few slices are used by metaata
//
type PageId = usize;
type SegmentId = usize;

impl<Key: Family> SegmentMeta<Key> {
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

    fn is_full(self: &Self) -> bool {
        // Not necessary, just some additional safety
        self.used == SLICES_PER_SEGMENT || self.used_map.leading_ones() == SLICES_PER_SEGMENT
    }

    // This entire function is highly unsafe, but if nobody fucked with the source code too much it
    // should be valid, but this design aspect obviously undermines quite a bit of rusts safety
    // The total lack of mutable on the params makes this an even bigger safety nightmare,
    // but we need to make it work ¯\_(ツ)_/¯
    fn get_page_data(&self, owner: &mut CellOwner<Key>, page: &SliceMeta<Key>) -> &mut [Block] {
        let page_addr = addr_of!(*page);
        let arr_addr = addr_of!(self.slice_meta[0]);

        // Ensure abs "rounds" the right way, bla bla
        let idx = unsafe { page_addr.offset_from(arr_addr) }.abs() as usize;

        let start_ptr = unsafe {
            addr_of!(*self)
                .byte_add(idx * SLICE_SIZE)
                .cast::<Block>()
                .cast_mut()
        };

        unsafe {
            core::slice::from_raw_parts_mut(
                start_ptr,
                page_kind_to_size(page.data.get(owner).kind) / core::mem::size_of::<Block>(),
            )
        }
    }

    fn get_id_data(&self, owner: & CellOwner<Key>, id: PageId) -> &mut [Block] {
        let start_ptr = unsafe {
            addr_of!(*self)
                .byte_add(id * SLICE_SIZE)
                .cast::<Block>()
                .cast_mut()
        };

        let page = &self.slice_meta[id];

        unsafe {
            core::slice::from_raw_parts_mut(
                start_ptr,
                page_kind_to_size(page.data.get(owner).kind) / core::mem::size_of::<Block>(),
            )
        }
    }

    fn get_page(&self, id: PageId) -> &SliceMeta<Key> {
        &self.slice_meta[id]
    }

    fn alloc_page(&mut self, slices: usize) -> Option<PageId> {
        if self.used + slices > SLICES_PER_SEGMENT {
            return None;
        }

        let old = self.used;
        self.used += slices;

        for mut val in &mut self.used_map[old..self.used] {
            *val = true;
        }

        Some(old)
    }
}

// Shitty linked list based on indices, to ease validity checks
struct Block {
    next: usize,
}

#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
enum PageKind {
    Small,
    Medium,
    Large,
}

// cell_family::define!(type SliceMarker: SliceOwner for SliceCell<T>);

#[derive(Debug)]
struct SliceMetaInner {
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

// This primarily contains information about a page, but because it's stored for every slice we
// refer to it by the term slice
// #[derive(Debug)]
struct SliceMeta<Key: Family> {
    link: LinkedListLink,
    data: Cell<Key, SliceMetaInner>,
}

// impl core::fmt::Debug for SliceMeta<Key> {
//     fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
//         f.write_fmt(format_args!("SliceMeta: link: {:?}", self.link))
//     }
// }

impl SliceMetaInner {
    fn is_empty(self: &Self) -> bool {
        self.used == 0
    }

    fn merge_xthread_free(&mut self, slice_data: &mut [Block]) {
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

        let block = &mut slice_data[free * (self.block_size / 8)];
        self.free_block_list = block.next;
        self.used += 1;

        Some(free * (self.block_size / 8))
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
        // TODO: Check for correctness
        if self.capacity >= page_kind_to_size(self.kind) / self.block_size {
            return None;
        }

        let mut previous = self.free_block_list;
        for (i, block) in slice_data
            .iter_mut()
            .step_by(self.block_size / 8)
            .skip(self.capacity)
            .take(100.min(page_kind_to_size(self.kind) / self.block_size))
            .enumerate()
        {
            block.next = previous;
            previous = self.capacity + i;
        }

        self.free_block_list = previous;

        self.capacity = previous + 1;

        Some(())
    }
}

impl<Key: Family> SliceMeta<Key> {
    const fn new() -> Self {
        Self {
            link: LinkedListLink::new(),
            data: Cell::<Key, _>::new(SliceMetaInner {
                kind: PageKind::Small,
                used: 0,
                free_block_list: usize::MAX,
                xthread_free_list: AtomicUsize::new(usize::MAX),
                capacity: 0,
                // Should not matter, but set it to 8 bytes anyway, i.e. one Block data type (ideally)
                block_size: 8,
            }),
        }
    }

    fn is_empty(self: &Self, owner: &mut CellOwner<Key>) -> bool {
        self.data.get(owner).is_empty()
    }

    // Deduplicate the map lmao
    fn try_alloc(self: &Self, owner: &mut CellOwner<Key>, slice_data: &mut [Block]) -> Option<*mut u8> {
        self.data.get_mut(owner).try_alloc(slice_data)
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

intrusive_adapter!(PageAdapter<'a, Key> = &'a SliceMeta<Key>: SliceMeta<Key> { link: LinkedListLink } where Key: Family);

struct Queue<'a, Key: Family> {
    list: LinkedList<PageAdapter<'a, Key>>,
    size: usize,
}

impl<Key: Family> Queue<'_, Key> {
    fn new() -> Self {
        Self {
            list: LinkedList::new(PageAdapter::<Key>::new()),
            size: 0,
        }
    }
}

// Awful trick copied from mimalloc that relies on segments being 4 MiB aligned, which they should
// be, but we really have no way of verifying that
// I do not have the smallest clue how that livetime makes it work, but if it works, sure, great
fn get_segment<'a, Key: Family>(page: &SliceMeta<Key>) -> &'a mut SegmentMeta<Key> {
    unsafe { &mut *(((addr_of!(*page) as usize) & !(0x400000 - 1)) as *mut SegmentMeta<Key>) }
}

// TODO: Abstract over custom marker types to enable multiple instances
pub struct Heap<'a, Key: Family> {
    // TODO: Just store reference, we have interior mutability anyway
    small_size_map: [*const SliceMeta<Key>; SMALL_SIZE_BIN_COUNT],
    bin_map: [Queue<'a, Key>; BIN_COUNT],
    // full_bin: Queue<'a>,

    // small_free: Queue<'static>,
    // medium_large_free: Queue<'static>,
    small_free_bm: FreeBm,
    medium_large_free_bm: FreeBm,

    owner: CellOwner<Key>,
}

impl<Key: Family> Heap<'_, Key> {
    pub fn new() -> Self {
        Self {
            small_size_map: [null_mut(); SMALL_SIZE_BIN_COUNT],
            bin_map: array_init(|_| Queue::new()),
            // full_bin: Queue::new(),

            // small_free: Queue::new(),
            // medium_large_free: Queue::new(),
            small_free_bm: FreeBm::new([0u64; 64]),
            medium_large_free_bm: FreeBm::new([0u64; 64]),

            owner: CellOwner::<Key>::new(),
        }
    }

    // TODO: Turn into distributor function only
    pub fn allocate(&mut self, size: usize) -> *mut u8 {
        let wsize = size_to_wsize(size);

        if wsize < SMALL_WORDSIZE_MAX {
            if let Some(addr) = self.allocate_small(wsize) {
                return addr;
            }
        }

        if size > MEDIUM_OBJ_MAX_SIZE {
            // TODO: Implement large object handling as early return
            //       => Can't move it to Arena because the user should not have direct access to
            //       arena and should also not have to check for the object size themselves
            // return null_mut();
            unimplemented!();
        }

        let bin = size_to_bin(size);

        // TODO: Extract to other function OR bring allocate_small back in?
        if let Some(page) = self.bin_map[bin as usize].list.front().get() {
            let segment = get_segment(page);
            let data = segment.get_page_data(&mut self.owner, page);

            if let Some(alloc) = page.try_alloc(&mut self.owner, data) {
                // println!("Got to get something from the big used page cache");
                return alloc;
            }
        }

        self.bin_map[bin as usize].list.pop_front();

        let (segment_id, page_id) = self.get_page(size);

        let segment = ARENA.get_segment_mut(segment_id);

        let page = segment.get_page(page_id);
        let data = segment.get_id_data(&self.owner, page_id);

        let bin_wsize = E[bin as usize];
        // println!("SegmentId {segment_id} PageId {page_id}");
        self.bin_map[bin as usize].list.push_back(page);

        if wsize < SMALL_WORDSIZE_MAX {
            // TODO: Clean up lmao + Refactor to big PageId
            let prev_bin_wsize = E[bin as usize - 1];
            // println!("prev_bin_wsize {prev_bin_wsize}, bin_wsize {bin_wsize}");

            for slot in &mut self.small_size_map
                [prev_bin_wsize + if prev_bin_wsize == 0 { 0 } else { 1 }..(bin_wsize + 1).min(128)]
            {
                *slot = addr_of!(*page);
            }
        }

        page.try_alloc(&mut self.owner, data).unwrap()
    }

    fn get_page(&mut self, size: usize) -> (SegmentId, PageId) {
        // let wsize = size_to_wsize(size);

        // TODO: Extract into sane function
        let kind = if size < SMALL_OBJ_MAX_SIZE {
            PageKind::Small
        } else {
            PageKind::Medium
        };

        let segment_id;
        let segment;

        if let Some(index) = (kind == PageKind::Small)
            .then(|| self.small_free_bm.first_one())
            .flatten()
        {
            segment_id = index;

            let ptr = (0x3EC0000000 + SEGMENT_SIZE * index) as *mut SegmentMeta<Key>;
            segment = unsafe { ptr.as_mut().unwrap() };

            // TODO: Exactly this is problematic it causes a bunch of all over the place bs and
            // just creates bugs
            if segment.used >= 511 {
                *self.small_free_bm.get_mut(index).unwrap() = false;
            }
        } else if let Some(index) = self.medium_large_free_bm.first_one() {
            segment_id = index;

            let ptr = (0x3EC0000000 + SEGMENT_SIZE * index) as *mut SegmentMeta<Key>;
            segment = unsafe { ptr.as_mut().unwrap() };

            // TODO: This always loses the entire 8 slices even if only 1 ends up getting
            // used
            // println!("used {}", segment.used);
            if segment.used + 8 > 504 {
                *self.medium_large_free_bm.get_mut(index).unwrap() = false;
            }
        } else {
            let ptr = ARENA.alloc() as *mut SegmentMeta<Key>;
            segment = unsafe { ptr.as_mut().unwrap() };
            *segment = SegmentMeta::new();

            segment_id = ARENA.ptr_to_id(segment);

            *self.medium_large_free_bm.get_mut(segment_id).unwrap() = true;
        }
        // TODO: Don't do stupid predictive calculation just ask the segment afterwards and then
        // directly set the data

        let page_slices = match kind {
            PageKind::Small => 1,
            PageKind::Medium => 8,
            _ => unreachable!(),
        };

        let page_id = segment.alloc_page(page_slices).unwrap();
        let page = segment.get_page(page_id);

        let bin_wsize = E[size_to_bin(size) as usize];

        let inner = page.data.get_mut(&mut self.owner);

        inner.block_size = bin_wsize * 8;
        inner.kind = kind;

        (segment_id, page_id)
    }

    fn allocate_small(&mut self, wsize: usize) -> Option<*mut u8> {
        // TODO: Should it start at index 0 (-> see small test in `allocate()`)
        let small_size = self.small_size_map[wsize];
        if small_size.is_null() {
            return None;
        }

        let page = unsafe { &*small_size };
        let segment = get_segment(page);
        let data = segment.get_page_data(&mut self.owner, page);

        page.try_alloc(&mut self.owner, data)
    }
}

type FreeBm = BitArray<[u64; 64], Lsb0>;

struct Arena {
    block_count: AtomicUsize,
    // Msb0
    dirty: [AtomicU64; 64],
    used: [AtomicU64; 64],
    // Has to be 4MiB aligned, due to segment_from_page shenaniganry
    start: AtomicPtr<u8>,
}

const BLOCK_SIZE: usize = SEGMENT_SIZE;

impl Arena {
    fn alloc(&self) -> *mut u8 {
        if let Some(index) = self.try_find_claim() {
            // println!("{}", index);
            return unsafe {
                self.start
                    .load(Ordering::Relaxed)
                    .byte_add(BLOCK_SIZE * (index as usize))
            };
        }

        if let Some(_) = self.alloc_new_unclaimed() {
            let index = self.try_find_claim().unwrap();
            return unsafe {
                self.start
                    .load(Ordering::Relaxed)
                    .byte_add(BLOCK_SIZE * index as usize)
            };
        }

        unreachable!();
    }

    // TODO: Already searched bits optimization
    fn try_find_claim(&self) -> Option<u64> {
        let value_count = self.block_count.load(Ordering::Relaxed).div_ceil(64);
        for (i, value) in self.used[..value_count].iter().enumerate() {
            let old = value.load(Ordering::Relaxed);
            let leading = old.leading_ones();

            if leading == 64 {
                continue;
            }

            let mask = 0b1 << (64 - leading - 1);
            let new = old | mask;

            if let Ok(_) =
                value.compare_exchange_weak(old, new, Ordering::AcqRel, Ordering::Acquire)
            {
                // println!("i {} bitidx {}", i, leading);
                return Some(i as u64 * 64 + leading as u64);
            }
        }

        None
    }

    fn alloc_new_from_os(&self) -> Option<u64> {
        // Increment immediately, we don't want another thread trying to allocate the same block
        let old_block_count = self.block_count.fetch_add(1, Ordering::Relaxed);

        // At start of block to be allocated
        let addr = unsafe {
            self.start
                .load(Ordering::Relaxed)
                .byte_add(BLOCK_SIZE * old_block_count)
        };

        sys_alloc(addr as u64, SEGMENT_SIZE as u64);
        Some(old_block_count as u64)
    }

    // TODO: bit inefficient having to redo it after
    fn alloc_new_unclaimed(&self) -> Option<u64> {
        let idx = self.alloc_new_from_os()?;

        let value_idx = idx / 64;
        let value = &self.used[value_idx as usize];

        let old = value.load(Ordering::Relaxed);

        let bit_shift = 64 - 1 - (idx - value_idx * 64);
        // Unset bit corresponding to block, marking it as free
        let new = old & !(0b1 << bit_shift);

        value
            .compare_exchange_weak(old, new, Ordering::AcqRel, Ordering::Acquire)
            .unwrap();

        Some(idx)
    }

    const fn new() -> Self {
        // Storing blocks that the allocator has no knowledge about as used allows us to not have
        // to check whether we're going out of bound of our blocks in the middle of a bitmap value
        Self {
            block_count: AtomicUsize::new(0),
            dirty: [const { AtomicU64::new(!0) }; 64],
            used: [const { AtomicU64::new(!0) }; 64],
            start: AtomicPtr::new(0x3EC0000000 as *mut u8),
        }
    }

    // TODO: Ensure that SegmentMeta size != Segment size never happens again
    fn ptr_to_id<Key: Family>(&self, ptr: *const SegmentMeta<Key>) -> SegmentId {
        unsafe { ptr.byte_offset_from(self.start.load(Ordering::Relaxed)) }.unsigned_abs()
            / SEGMENT_SIZE
    }

    // TODO: This is unsafe with the prerequisite of the Segment being owned by the same thread
    // that is calling this
    fn get_segment_mut<Key: Family>(&self, id: SegmentId) -> &mut SegmentMeta<Key> {
        unsafe {
            self.start
                .load(Ordering::Relaxed)
                .byte_add(id * SEGMENT_SIZE)
                .cast::<SegmentMeta<Key>>()
                .as_mut()
                .unwrap()
        }
    }
}

// thread_local! {
//     // TODO: Make not use fixed addr allocs
//     pub static TLD: RefCell<*mut Tld<'static>> = RefCell::new(init());
// }

// #[thread_local]
// pub static TLD: Tld = Tld::new();

fn sys_alloc(addr: u64, size: u64) {
    // std::mem::forget(
    //     MemoryMap::new(
    //         size as usize,
    //         &[
    //             MapOption::MapAddr(addr as *mut u8),
    //             MapOption::MapWritable,
    //             MapOption::MapReadable,
    //         ],
    //     )
    //     .unwrap(),
    // );
    unsafe {
        do_syscall(2, 0, addr, 0, size.div_ceil(0x1000));
    }
}

// pub struct Tld<'a> {
//     pub heap: Heap<'a>,
// }

static ARENA: Arena = Arena::new();

// impl Tld<'_> {
//     fn new() -> Self {
//         Self { heap: Heap::new() }
//     }
// }

const E: [usize; 75] = [
    0, // Should not be relevant, could be anything
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8, /* 8 */
    10,
    12,
    14,
    16,
    20,
    24,
    28,
    32, /* 16 */
    40,
    48,
    56,
    64,
    80,
    96,
    112,
    128, /* 24 */
    160,
    192,
    224,
    256,
    320,
    384,
    448,
    512, /* 32 */
    640,
    768,
    896,
    1024,
    1280,
    1536,
    1792,
    2048, /* 40 */
    2560,
    3072,
    3584,
    4096,
    5120,
    6144,
    7168,
    8192, /* 48 */
    10240,
    12288,
    14336,
    16384,
    20480,
    24576,
    28672,
    32768, /* 56 */
    40960,
    49152,
    57344,
    65536,
    81920,
    98304,
    114688,
    131072, /* 64 */
    163840,
    196608,
    229376,
    262144,
    327680,
    393216,
    458752,
    524288,                      /* 72 */
    MEDIUM_OBJ_MAX_SIZE / 8 + 1, /* 655360, Huge queue */
    MEDIUM_OBJ_MAX_SIZE / 8 + 2, /* Full queue */
];

fn size_to_bin(size: usize) -> u8 {
    let wsize = size_to_wsize(size);

    let bin: u8;

    if wsize <= 1 {
        bin = 1;
    } else if wsize <= 8 {
        bin = wsize as u8;
    } else if wsize > MEDIUM_OBJ_MAX_SIZE {
        bin = 77 as u8;
    } else {
        let wsize = wsize - 1;
        let b = WORD_SIZE as u8 * 8 - 1 - wsize.leading_zeros() as u8;

        // and use the top 3 bits to determine the bin (~12.5% worst internal fragmentation).
        // - adjust with 3 because we use do not round the first 8 sizes
        //   which each get an exact bin
        bin = ((b << 2) + (((wsize >> (b - 2)) as u8) & 0x3)) - 3;
    }

    bin
}

fn size_to_wsize(size: usize) -> usize {
    size.div_ceil(WORD_SIZE)
}
