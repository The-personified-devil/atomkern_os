#![feature(int_roundings)]
#![feature(pointer_byte_offsets)]
#![feature(inline_const)]

use bitvec::prelude::*;
use core::ptr::null_mut;
use std::ptr::addr_of_mut;
use std::sync::atomic::*;

fn main() {
    println!("Hello, world!");
}

const MI_SEGMENT_BIN_MAX: usize = 35; // 35 == mi_segment_bin(MI_SLICES_PER_SEGMENT)
                                      // #ifndef MI_MAX_ALIGN_SIZE
const MI_MAX_ALIGN_SIZE: usize = 16; // sizeof(max_align_t)
                                     // #endif

const MI_BIN_FULL: usize = MI_BIN_HUGE + 1;
const SIZE_COUNT: usize = 128;
const SLICES_PER_SEGMENT: usize = 127;
struct Queue {
    first: *mut Page,
    last: *mut Page,
    block_size: usize,
}

const SEGMENT_SIZE: usize = 0x2000000; // 32 MiB
const SLICE_SIZE: usize = 0x10000; // 64 KiB

const SLICES_PER_SEGMENT: usize = SEGMENT_SIZE / SLICE_SIZE;

const SMALL_PAGE_SIZE: usize = SLICE_SIZE;
const MEDIUM_PAGE_SIZE: usize = SLICE_SIZE * 8;

struct Segment {

}


struct Segment {
    memid: usize,
    mem_is_large: bool,
    mem_alignment: usize,
    mem_align_offset: usize,

    next: *mut Segment,

    abandoned: usize,
    abandoned_visits: usize,
    used: usize,
    segment_slices: usize,
    segment_info_slices: usize,
    kind: SegmentKind,
    slice_entries: usize,
    thread_id: AtomicUsize,
    slices: [Page; SLICES_PER_SEGMENT],
}

struct Page {
    slice_count: u32,
    slice_offset: u32,
    is_reset: bool,
    is_committed: bool,
    is_zero_init: bool,
    capacity: usize,
    reserved: usize,
    free: *mut Block,
    local_free: *mut Block,
    retire_expire: u8,
    used: u32,
    is_zero: bool,
    block_size: u32,
    heap: AtomicPtr<Heap>,
    // xthread_free:
    next: *mut Page,
    prev: *mut Page,
}

enum PageKind {
    Small,
    Medium,
    Large,
}

struct Page2 {
    kind: PageKind,

}

struct Block {
    next: *mut Block,
}

pub struct Heap {
    pages_free_direct: [*mut Page; SIZE_COUNT + 1],
    arena_id: ArenaId,
    pages: [Queue; 73 + 1],
    page_count: usize,
    page_retired_min: usize,
    page_retired_max: usize,
    tld: *mut Tlb,
}

struct Tlb {
    heap: *mut Heap,
    os_tld: OsTld,
    segments: SegmentsTld,
}

thread_local! {
    static TLB: Option<Tlb> = None;
}

pub fn malloc_small(heap: &mut Heap, n: usize) -> *mut u8 {
    let page = unsafe { &mut *heap.pages_free_direct[n.div_ceil(8)] };

    let block_ptr = page.free;
    // TODO: Check whether 0, fall back to expensive alloc

    // This is a take from queue, reimpl as such
    let block = unsafe { &mut *block_ptr };

    page.free = block.next;
    page.used += 1;

    block_ptr.cast::<u8>()
}

const MI_SEGMENT_SLICE_SHIFT: usize = 13 + 3; // 64KiB  (32KiB on 32-bit)

const MI_SEGMENT_SHIFT: usize = 9 + MI_SEGMENT_SLICE_SHIFT; // 32MiB

const MI_SMALL_PAGE_SHIFT: usize = MI_SEGMENT_SLICE_SHIFT; // 64KiB
const MI_MEDIUM_PAGE_SHIFT: usize = 3 + MI_SMALL_PAGE_SHIFT; // 512KiB

const MI_INTPTR_SIZE: usize = 8;

// Derived constants
const MI_SEGMENT_SIZE: usize = 1 << MI_SEGMENT_SHIFT;
const MI_SEGMENT_ALIGN: usize = MI_SEGMENT_SIZE;
const MI_SEGMENT_MASK: usize = MI_SEGMENT_ALIGN - 1;
const MI_SEGMENT_SLICE_SIZE: usize = 1 << MI_SEGMENT_SLICE_SHIFT;
const MI_SLICES_PER_SEGMENT: usize = MI_SEGMENT_SIZE / MI_SEGMENT_SLICE_SIZE; // 1024

const MI_SMALL_PAGE_SIZE: usize = 1 << MI_SMALL_PAGE_SHIFT;
const MI_MEDIUM_PAGE_SIZE: usize = 1 << MI_MEDIUM_PAGE_SHIFT;

const MI_SMALL_OBJ_SIZE_MAX: usize = MI_SMALL_PAGE_SIZE / 4; // 8KiB on 64-bit
const MI_MEDIUM_OBJ_SIZE_MAX: usize = MI_MEDIUM_PAGE_SIZE / 4; // 128KiB on 64-bit
const MI_MEDIUM_OBJ_WSIZE_MAX: usize = MI_MEDIUM_OBJ_SIZE_MAX / MI_INTPTR_SIZE;
const MI_LARGE_OBJ_SIZE_MAX: usize = MI_SEGMENT_SIZE / 2; // 32MiB on 64-bit
const MI_LARGE_OBJ_WSIZE_MAX: usize = MI_LARGE_OBJ_SIZE_MAX / MI_INTPTR_SIZE;

// Maximum number of size classes. (spaced exponentially in 12.5% increments)
const MI_BIN_HUGE: usize = 73;
const MB: usize = 0x100000;
const MI_GiB: usize = MB * 1024;
const MEDIUM_BLOCK_SIZE: usize = 1;

const MI_MAX_SLICE_OFFSET: usize = (MI_ALIGNMENT_MAX / MI_SEGMENT_SLICE_SIZE) - 1;

// Used as a special value to encode block sizes in 32 bits.
const MI_HUGE_BLOCK_SIZE: u32 = 2 * MI_GiB as u32;

// blocks up to this size are always allocated aligned
const MI_MAX_ALIGN_GUARANTEE: usize = 8 * MI_MAX_ALIGN_SIZE;

// Alignments over MI_ALIGNMENT_MAX are allocated in dedicated huge page segments
const MI_ALIGNMENT_MAX: usize = MI_SEGMENT_SIZE >> 1;

fn retire_page(page: *mut Page) {
    let page = unsafe { &mut *page };
    let heap = unsafe { &mut *page.heap.load(Ordering::Relaxed) };
    // TODO: Impl later idgaf
}

fn heap_delayed_free(heap: &mut Heap) {}

fn free_block(page: *mut Page, block_ptr: *mut Block, local: bool) {
    let page = unsafe { &mut *page };
    let block = unsafe { &mut *block_ptr };
    block.next = block_ptr;
    page.local_free = block;
    page.used -= 1;
}

fn mi_heap_malloc(heap: *mut Heap, size: usize) {
    // small enough
    if true {
    } else {
        malloc_generic(heap, size);
    }
}

fn malloc_generic(heap: *mut Heap, size: usize) {
    // free_thread_stuff
}

fn find_page(heap: &mut Heap, n: usize) -> *mut Page {
    if n > MI_MEDIUM_OBJ_SIZE_MAX {
        // check for bigger than ptrdiff_max
        // large alloc
        return null_mut();
    } else {
        return find_free_page(heap, n);
    }
}

fn find_free_page(heap: &mut Heap, n: usize) -> &mut Page {
    let page_queue = heap.pages[n.div_ceil(8)];
    let page_ptr = page_queue.first;

    if !page_ptr.is_null() {
        let page = unsafe { &mut *page_ptr };

        page_free_collect(page);

        if page_immediately_available(page) {
            // page.retire_expire = 0;
            return page;
        }
    }

    unsafe { &mut *page_queue_find_free(heap, &mut page_queue) }
}

fn page_queue_find_free(heap: &mut Heap, queue: &mut Queue) -> *mut Page {
    let count: usize = 0;
    let mut page_ptr = queue.first;
    while !page_ptr.is_null() {
        count += 1;

        let page = unsafe { &mut *page_ptr };
        let next_ptr = page.next;

        page_free_collect(page);

        if page_immediately_available(page) {
            break;
        }

        if page.capacity < page.reserved {
            page_extend_free(heap, page);
            break;
        }
        page_ptr = next_ptr;
    }

    if page_ptr.is_null() {
        heap_collect_retired(heap);
        page_ptr = page_fresh(heap, queue);

        if page_ptr.is_null() {
            page_ptr = page_queue_find_free(heap, queue);
        }
    } else {
        unsafe { &mut *page_ptr }.retire_expire = 0;
    }
    return page_ptr;
}

fn page_fresh(heap: &mut Heap, queue: &mut Queue) -> *mut Page {
    let page = page_fresh_alloc(heap, queue, queue.block_size, 0);
    if page.is_null() {
        return null_mut();
    }
    return page;
}

fn page_fresh_alloc(
    heap: &mut Heap,
    queue: &mut Queue,
    block_size: usize,
    alignment: usize,
) -> *mut Page {
    let tld = unsafe { &mut *heap.tld };
    let page = segment_page_alloc(
        heap,
        queue,
        block_size,
        alignment,
        &mut tld.segments,
        &mut tld.os_tld,
    );
    if page.is_null() {
        // this may be out-of-memory, or an abandoned page was reclaimed (and in our queue)
        return null_mut();
    }

    // fresh page found, initialize it
    return page;
}

// static mi_page_t* mi_page_fresh_alloc(mi_heap_t* heap, mi_page_queue_t* pq, size_t block_size, size_t page_alignment) {
//   const size_t full_block_size = ((pq == NULL || mi_page_queue_is_huge(pq)) ? mi_page_block_size(page) : block_size); // see also: mi_segment_huge_page_alloc
//   if (pq != NULL) { mi_page_queue_push(heap, pq, page); }
// }
//
//
fn segment_page_alloc(
    heap: &mut Heap,
    queue: &mut Queue,
    block_size: usize,
    alignment: usize,
    tld: &mut SegmentsTld,
    os_tld: &mut OsTld,
) -> *mut Page {
    let page: *mut Page;
    if alignment > MI_ALIGNMENT_MAX {
        if alignment < MI_SEGMENT_SIZE {
            alignment = MI_SEGMENT_SIZE;
            page = mi_segment_huge_page_alloc(block_size, alignment, heap.arena_id, tld, os_tld);
        }
    } else if block_size <= MI_SMALL_OBJ_SIZE_MAX {
        page = mi_segments_page_alloc(heap, PageKind::Small, block_size, block_size, tld, os_tld);
    } else if block_size <= MI_MEDIUM_OBJ_SIZE_MAX {
        page = mi_segments_page_alloc(
            heap,
            PageKind::Medium,
            MI_MEDIUM_PAGE_SIZE,
            block_size,
            tld,
            os_tld,
        );
    } else if block_size <= MI_LARGE_OBJ_SIZE_MAX {
        page = mi_segments_page_alloc(heap, PageKind::Large, block_size, block_size, tld, os_tld);
    } else {
        page = mi_segment_huge_page_alloc(block_size, alignment, heap.arena_id, tld, os_tld);
    }
    return page;
}

struct OsTld {
    region_idx: usize,
}

fn mi_align_up(sz: usize, alignment: usize) -> usize {
    let mask = alignment - 1;
    if (alignment & mask) == 0 {
        // power of two
        return (sz + mask) & !mask;
    } else {
        return (sz + mask) / alignment * alignment;
    }
}

fn mi_segment_os_alloc(
    required: usize,
    alignment: usize,
    req_arena_id: ArenaId,
    segment_slices: usize,
    pre_size: usize,
    info_slices: usize,
    tld: &mut SegmentsTld,
    os_tld: &mut OsTld,
) -> (*mut Segment, usize, usize, usize, bool) {
    let mut alignment2 = MI_SEGMENT_ALIGN;
    let mut align_offset = 0;
    let SEGMENT_SIZE = segment_slices * MI_SEGMENT_SLICE_SIZE;

    if alignment > 0 {
        alignment2 = alignment;

        let info_size = info_slices * MI_SEGMENT_SLICE_SIZE;
        align_offset = mi_align_up(info_size, MI_SEGMENT_ALIGN);

        let extra = align_offset - info_size;
        let (segment_slices, pre_size, info_slices) = mi_segment_calculate_slices(required + extra);
    }

    let mut segment: *mut Segment;
    let mut large = false;
    let mut is_zero = false;
    let mut memid = 0;
    if alignment == 0 {
        (segment, large, is_zero, memid) = mi_segment_cache_pop(SEGMENT_SIZE, req_arena_id, os_tld);
    }
    if segment.is_null() {
        segment =
            mi_arena_alloc_aligned(SEGMENT_SIZE, alignment2, align_offset, req_arena_id, os_tld);
        if segment.is_null() {
            return (null_mut(), 0, 0, 0, false);
        }
    }

    let ssegment = unsafe { &mut *segment };
    ssegment.memid = memid;
    ssegment.mem_is_large = mem_large;
    ssegment.mem_alignment = alignment2;
    ssegment.mem_align_offset = align_offset;

    mi_segments_track_size(SEGMENT_SIZE as isize, tld);
    // _mi_segment_map_allocated_at(segment);

    (segment, segment_slices, pre_size, info_slices, is_zero)
}

fn mi_segment_cache_pop(
    size: usize,
    req_arena_id: ArenaId,
    os_tld: &mut OsTld,
) -> (*mut Segment, bool, bool, usize) {
    if size != MI_SEGMENT_SIZE {
        return (null_mut(), false, false, 0);
    }
    // pretend we don't have any cache, even though this is what's gonna kill us, not any of the os
    // shenanigans
    (null_mut(), false, false, 0)

    // no numa lmfao
    // let start_field = 0;
    // let mut bitidx:usize = 0
}

fn mi_arena_alloc_aligned(
    size: usize,
    alignment: usize,
    align_offset: usize,
    req_arena_id: ArenaId,
    os_tld: &mut OsTld,
) -> (*mut Segment, bool, bool, usize) {
    let memid = MI_MEMID_0S;
    let is_zero = false;
    let large = false;

    if size >= MI_ARENA_MIN_OBJ_SIZE && alignment <= MI_SEGMENT_ALIGN && align_offset == 0 {
        let p = mi_arena_allocate(size, alignment, req_arena_id, memid, os_tld);
        if !p.is_null() {
            return p;
        }

        let is_zero = true;
        let memid = MI_MEMID_OS;
        let p = _mi_os_alloc_aligned_offset(size, alignment, align_offset);

        return p;
    }
}

struct Arena {
    block_count: AtomicUsize,
    dirty: [AtomicU64; 16],
    used: [AtomicU64; 16],
    // Has to be 4MiB aligned, due to segment_from_page shenaniganry
    start: AtomicPtr<u8>,
}

const BLOCK_SIZE: usize = MI_SEGMENT_SIZE;

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

        value.compare_exchange_weak(old, new, Ordering::AcqRel, Ordering::Acquire).ok()
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

fn os_alloc_pages_at(addr: *mut u8, page_count: usize) {}
// void* _mi_arena_alloc_aligned(size_t size, size_t alignment, size_t align_offset, bool* commit, bool* large, bool* is_pinned, bool* is_zero,
//                               mi_arena_id_t req_arena_id, size_t* memid, mi_os_tld_t* tld)
// {
//   *memid   = MI_MEMID_OS;
//   *is_zero = false;
//   *is_pinned = false;

//   bool default_large = false;
//   if (large == NULL) large = &default_large;   // ensure `large != NULL`
//   const int numa_node = _mi_os_numa_node(tld); // current numa node

//   // try to allocate in an arena if the alignment is small enough and the object is not too small (as for heap meta data)
//   if (size >= MI_ARENA_MIN_OBJ_SIZE && alignment <= MI_SEGMENT_ALIGN && align_offset == 0) {
//     void* p = mi_arena_allocate(numa_node, size, alignment, commit, large, is_pinned, is_zero, req_arena_id, memid, tld);
//     if (p != NULL) return p;
//   }

//   // finally, fall back to the OS
//   if (mi_option_is_enabled(mi_option_limit_os_alloc) || req_arena_id != _mi_arena_id_none()) {
//     errno = ENOMEM;
//     return NULL;
//   }
//   *is_zero = true;
//   *memid   = MI_MEMID_OS;
//   void* p = _mi_os_alloc_aligned_offset(size, alignment, align_offset, *commit, large, tld->stats);
//   if (p != NULL) { *is_pinned = *large; }
//   return p;
// }

// mi_decl_noinline static void* mi_segment_cache_pop_ex(
//                               bool all_suitable,
//                               size_t size, mi_commit_mask_t* commit_mask,
//                               mi_commit_mask_t* decommit_mask, bool* large, bool* is_pinned, bool* is_zero,
//                               mi_arena_id_t _req_arena_id, size_t* memid, mi_os_tld_t* tld)
// {
//   // only segment blocks
//   if (size != MI_SEGMENT_SIZE) return NULL;

//   // numa node determines start field
//   const int numa_node = _mi_os_numa_node(tld);
//   size_t start_field = 0;
//   if (numa_node > 0) {
//     start_field = (MI_CACHE_FIELDS / _mi_os_numa_node_count())*numa_node;
//     if (start_field >= MI_CACHE_FIELDS) start_field = 0;
//   }

//   // find an available slot
//   mi_bitmap_index_t bitidx = 0;
//   bool claimed = false;
//   mi_arena_id_t req_arena_id = _req_arena_id;
//   mi_bitmap_pred_fun_t pred_fun = (all_suitable ? NULL : &mi_segment_cache_is_suitable);  // cannot pass NULL as the arena may be exclusive itself; todo: do not put exclusive arenas in the cache?

//   if (*large) {  // large allowed?
//     claimed = _mi_bitmap_try_find_from_claim_pred(cache_available_large, MI_CACHE_FIELDS, start_field, 1, pred_fun, &req_arena_id, &bitidx);
//     if (claimed) *large = true;
//   }
//   if (!claimed) {
//     claimed = _mi_bitmap_try_find_from_claim_pred (cache_available, MI_CACHE_FIELDS, start_field, 1, pred_fun, &req_arena_id, &bitidx);
//     if (claimed) *large = false;
//   }

//   if (!claimed) return NULL;

//   // found a slot
//   mi_cache_slot_t* slot = &cache[mi_bitmap_index_bit(bitidx)];
//   void* p = slot->p;
//   *memid = slot->memid;
//   *is_pinned = slot->is_pinned;
//   *is_zero = false;
//   *commit_mask = slot->commit_mask;
//   *decommit_mask = slot->decommit_mask;
//   slot->p = NULL;
//   mi_atomic_storei64_release(&slot->expire,(mi_msecs_t)0);
//
//   // mark the slot as free again
//   mi_assert_internal(_mi_bitmap_is_claimed(cache_inuse, MI_CACHE_FIELDS, 1, bitidx));
//   _mi_bitmap_unclaim(cache_inuse, MI_CACHE_FIELDS, 1, bitidx);
//   return p;
// #endif
// }

// void _mi_segment_map_allocated_at(const mi_segment_t* segment) {
//   size_t bitidx;
//   size_t index = mi_segment_map_index_of(segment, &bitidx);
//   mi_assert_internal(index <= MI_SEGMENT_MAP_WSIZE);
//   if (index==MI_SEGMENT_MAP_WSIZE) return;
//   uintptr_t mask = mi_atomic_load_relaxed(&mi_segment_map[index]);
//   uintptr_t newmask;
//   do {
//     newmask = (mask | ((uintptr_t)1 << bitidx));
//   } while (!mi_atomic_cas_weak_release(&mi_segment_map[index], &mask, newmask));
// }

// static mi_segment_t* mi_segment_os_alloc( size_t required, size_t page_alignment, bool eager_delay, mi_arena_id_t req_arena_id,
//                                           size_t* psegment_slices, size_t* ppre_size, size_t* pinfo_slices,
//                                           mi_commit_mask_t* pcommit_mask, mi_commit_mask_t* pdecommit_mask,
//                                           bool* is_zero, bool* pcommit, mi_segments_tld_t* tld, mi_os_tld_t* os_tld)

// {
//   // Allocate the segment from the OS
//   bool mem_large = (!eager_delay && (MI_SECURE==0)); // only allow large OS pages once we are no longer lazy
//   bool is_pinned = false;
//   size_t memid = 0;
//   size_t align_offset = 0;
//   size_t alignment = MI_SEGMENT_ALIGN;
//   const size_t segment_size = (*psegment_slices) * MI_SEGMENT_SLICE_SIZE;
//
//   mi_segment_t* segment = NULL;

//   // get from cache?
//   if (page_alignment == 0) {
//     segment = (mi_segment_t*)_mi_segment_cache_pop(segment_size, pcommit_mask, pdecommit_mask, &mem_large, &is_pinned, is_zero, req_arena_id, &memid, os_tld);
//   }
//
//   // get from OS
//     segment = (mi_segment_t*)_mi_arena_alloc_aligned(segment_size, alignment, align_offset, pcommit, &mem_large, &is_pinned, is_zero, req_arena_id, &memid, os_tld);

// }
//

fn mi_segment_calculate_slices(required: usize) -> (usize, usize, usize) {
    // get proper value later
    let page_size = 4096;
    let i_size = mi_align_up(core::mem::size_of::<Segment>(), page_size);

    let pre_size = i_size;
    // seems rather stupid without guard pages lmao
    let i_size = mi_align_up(i_size, MI_SEGMENT_SLICE_SIZE);
    let info_slices = i_size / MI_SEGMENT_SLICE_SIZE;
    let SEGMENT_SIZE = if required == 0 {
        MI_SEGMENT_SLICE_SIZE
    } else {
        mi_align_up(required + i_size, MI_SEGMENT_SLICE_SIZE)
    };

    (SEGMENT_SIZE / MI_SEGMENT_SLICE_SIZE, pre_size, info_slices)
}

// static size_t mi_segment_calculate_slices(size_t required, size_t* pre_size, size_t* info_slices) {
//   size_t page_size = _mi_os_page_size();
//   size_t isize     = _mi_align_up(sizeof(mi_segment_t), page_size);
//   size_t guardsize = 0;
//
//   if (pre_size != NULL) *pre_size = isize;
//   isize = _mi_align_up(isize + guardsize, MI_SEGMENT_SLICE_SIZE);
//   if (info_slices != NULL) *info_slices = isize / MI_SEGMENT_SLICE_SIZE;
//   size_t segment_size = (required==0 ? MI_SEGMENT_SIZE : _mi_align_up( required + isize + guardsize, MI_SEGMENT_SLICE_SIZE) );
//   mi_assert_internal(segment_size % MI_SEGMENT_SLICE_SIZE == 0);
//   return (segment_size / MI_SEGMENT_SLICE_SIZE);
// }
//


fn mi_segment_alloc(
    required: usize,
    alignment: usize,
    req_arena_id: ArenaId,
    tld: &mut SegmentsTld,
    os_tld: &mut OsTld,
) -> (*mut Segment, *mut Page) {
    let (segment_slices, pre_size, info_slices) = mi_segment_calculate_slices(required);
    // eager commit logic

    let is_zero = false;

    // Allocate the segment from the OS
    let (segment, segment_slices, pre_size, info_slices, is_zero) = mi_segment_os_alloc(
        required,
        alignment,
        req_arena_id,
        segment_slices,
        pre_size,
        info_slices,
        tld,
        os_tld,
    );
    if segment.is_null() {
        return (null_mut(), null_mut());
    }
    //  skip zeroing
    //  skip decommit

    let segment_slices = segment_slices.min(MI_SLICES_PER_SEGMENT);
    let ssegment = unsafe { &mut *segment };
    ssegment.segment_slices = segment_slices;
    ssegment.segment_info_slices = info_slices;
    ssegment.thread_id = mi_thread_id();
    ssegment.slice_entries = slice.entries;

    ssegment.kind = if required == 0 {
        SegmentKind::Normal
    } else {
        SegmentKind::Huge
    };

    mi_segment_span_allocate(ssegment, 0, info_slices, tld);
    // internal slice no count towards use
    //
    ssegment.used = 0;
    if ssegment.kind == SegmentKind::Normal {
        mi_segment_span_free(ssegment, info_slices, segment_slices - info_slices, tld);
    } else {
        return (
            segment,
            mi_segment_span_allocate(ssegment, info_slices, segment_slices - info_slices, tld),
        );
    }
    (segment, null_mut())
}

fn mi_segment_huge_page_alloc(
    size: usize,
    alignment: usize,
    req_arena_id: ArenaId,
    tld: &mut SegmentsTld,
    os_tld: &mut OsTld,
) -> *mut Page {
    let (segment, page_ptr) = mi_segment_alloc(size, alignment, req_arena_id, tld, os_tld);
    if segment.is_null() || page_ptr.is_null() {
        return null_mut();
    }
    let segment = unsafe { &mut *segment };
    let (start, psize) = page_start_from_segment(segment, page_ptr);

    let page = unsafe { &mut *page_ptr };
    page.block_size = (psize as u32).min(MI_HUGE_BLOCK_SIZE);

    // skip decommit
    return page_ptr;
}

fn mi_segments_page_alloc(
    heap: &mut Heap,
    page_kind: PageKind,
    required: usize,
    block_size: usize,
    tld: &mut SegmentsTld,
    os_tld: &mut OsTld,
) -> *mut Page {
    let page_size = mi_align_up(
        required,
        if required > MI_MEDIUM_PAGE_SIZE {
            MI_MEDIUM_PAGE_SIZE
        } else {
            MI_SEGMENT_SIZE
        },
    );
    let slices_needed = page_size / MI_SEGMENT_SLICE_SIZE;
    let page = mi_segments_page_find_and_allocate(slices_needed, heap.arena_id, tld);

    if page.is_null() {
        if mi_segment_reclaim_or_alloc(heap, slices_needed, heap.arena_id, tld, os_tld).is_null() {
            // no free page, allocate a new segment and try again
            return null_mut();
        } else {
            return mi_segments_page_alloc(heap, page_kind, required, block_size, tld, os_tld);
        }
    }

    return page;
}

fn mi_segment_reclaim_or_alloc(
    heap: &mut Heap,
    needed_slice: usize,
    block_size: usize,
    tld: &mut SegmentsTld,
    os_tld: &mut OsTld,
) -> *mut Segment {
    let (segment, reclaimed) = mi_segment_try_reclaim(heap, needed_slice, block_size, tld);
    if reclaimed {
        return null_mut();
    } else if !segment.is_null() {
        return segment;
    }

    mi_segment_alloc(0, 0, heap.arena_id, tld, os_tld).0
}

fn mi_segment_try_reclaim(
    heap: &mut Heap,
    needed_slice: usize,
    block_size: usize,
    tld: &mut SegmentsTld,
) -> (*mut Segment, bool) {
    let mut max_tries = mi_option_max_segment_reclaim;
    let segment: *mut Segment;
    while {
        max_tries -= 1;
        max_tries
    } > 0
        && {
            segment = mi_abandoned_pop();
            !segment.is_null()
        }
    {
        let ssegment = unsafe { &mut *segment };
        ssegment.abandoned_visits += 1;
        let is_suitable = memid_is_suitable(heap, ssegment.memid);
        let has_page = mi_segment_check_free(ssegment, needed_slice, block_size, tld);

        if ssegment.used == 0 {
            mi_segment_reclaim(ssegment, heap, 0, tld);
        } else if has_page && is_suitable {
            return mi_segment_reclaim(ssegment, heap, block_size, tld);
        } else if ssegment.abandoned_visits > 3 && is_suitable {
            mi_segment_reclaim(ssegment, heap, 0, tld);
        } else {
            // decommit maybe
            mi_abandoned_visited_push(segment);
        }
    }
    return (null_mut(), false);
}

fn mi_segment_check_free(
    segment: &mut Segment,
    slices_needed: usize,
    block_size: usize,
    tld: &mut SegmentsTld,
) -> bool {
    let has_page = false;
    let (mut slice, end) = mi_slices_start_iterate(segment);
    while slice < end {
        let sslice = unsafe { &mut *slice };
        if mi_slice_is_used(sslice) {
            page_free_collect(sslice);
            if page_all_free(sslice) {
                segment.abandoned -= 1;
                slice = mi_segment_page_clear(sslice, tld);
                let sslice = unsafe { &mut *slice };
                if sslice.slice_count as usize >= slices_needed {
                    has_page = true;
                }
            } else {
                if sslice.block_size as usize == block_size && mi_page_has_any_available(sslice) {
                    has_page = true
                }
            }
        } else {
            if sslice.slice_count as usize >= slices_needed {
                has_page = true;
            }
        }
        slice = unsafe { slice.add(sslice.slice_count as usize) };
    }
    return has_page;
}

fn mi_page_has_any_available(page: &mut Page) -> bool {
    (page.used as usize) < page.reserved //||!mi_page_thread_free(page).is_null()
}

fn mi_page_queue(heap: &mut Heap, size: usize) -> &mut Queue {
    &mut heap.pages[mi_bin(size) as usize]
}

// static inline mi_page_queue_t* mi_page_queue(const mi_heap_t* heap, size_t size) {
//   return &((mi_heap_t*)heap)->pages[_mi_bin(size)];
// }

fn _mi_page_reclaim(heap: &mut Heap, page: &mut Page) {
    let pq = mi_page_queue(heap, mi_page_block_size(page));
}

fn mi_page_queue_push(heap: &mut Heap, queue: &mut Queue, page: &mut Page) {
    let page_addr = addr_of_mut!(*page);
    mi_page_set_in_full(page, mi_page_queue_is_full(queue));
    page.next = queue.first;
    page.prev = null_mut();
    if !queue.first.is_null() {
        unsafe { &mut *queue.first }.prev = page_addr;
        queue.first = page_addr;
    } else {
        queue.first = page_addr;
        queue.last = page_addr;
    }
    heap_queue_first_update(heap, queue);
    heap.page_count += 1;
}

// void _mi_page_reclaim(mi_heap_t* heap, mi_page_t* page) {
//   // TODO: push on full queue immediately if it is full?
//   mi_page_queue_t* pq = mi_page_queue(heap, mi_page_block_size(page));
//   mi_page_queue_push(heap, pq, page);
//   mi_assert_expensive(_mi_page_is_valid(page));
// }

fn mi_segment_reclaim(
    segment: &mut Segment,
    heap: &mut Heap,
    requested_block_size: usize,
    tld: &mut SegmentsTld,
) -> (*mut Segment, bool) {
    let mut right_page_reclaimed = false;
    segment.thread_id = mi_thread_id();

    segment.abandoned_visits = 0;
    mi_segments_track_size(mi_segment_size(segment).try_into().unwrap(), tld);

    let (mut slice, end) = mi_slices_start_iterate(segment);
    while slice < end {
        let sslice = unsafe { &mut *slice };
        if mi_slice_is_used(sslice) {
            segment.abandoned -= 1;

            mi_page_set_heap(sslice, heap);

            // TODO: Make thread safe over here lmfao
            // _mi_page_use_delayed_free(page, MI_USE_DELAYED_FREE, true);

            page_free_collect(sslice);
            if page_all_free(sslice) {
                slice = mi_segment_page_clear(sslice, tld);
            } else {
                _mi_page_reclaim(heap, sslice);
                if requested_block_size == sslice.block_size as usize
                    && mi_page_has_any_available(sslice)
                {
                    right_page_reclaimed = true;
                }
            }
        } else {
            slice = mi_segment_span_free_coalesce(sslice, tld);
        }
        slice = unsafe { slice.add(sslice.slice_count as usize) };
    }
    if segment.used == 0 {
        mi_segment_free(segment, tld);
        return (null_mut(), right_page_reclaimed);
    } else {
        return (segment, right_page_reclaimed);
    }
}

fn mi_segment_free(segment: &mut Segment, tld: &mut SegmentsTld) {
    let mut slice = addr_of_mut!(segment.slices[0]);
    let end = mi_segment_slices_end(segment);
    let mut page_count = 0;
    while slice < end {
        let page = unsafe { &mut *slice };
        if page.block_size == 0 && segment.kind != SegmentKind::Huge {
            mi_segment_span_remove_from_queue(page, tld);
        }
        page_count += 1;
        slice = unsafe { slice.add(page.slice_count as usize) };
    }
    // mi_os_segment_free(segment, tld);
}

fn mi_page_block_size(page: &mut Page) -> usize {
    let bsize = page.block_size;
    if bsize < MI_HUGE_BLOCK_SIZE {
        return bsize as usize;
    } else {
        return page_start_from_segment(get_segment(page), page).1;
    }
}

const mi_option_page_reset: bool = false;

fn mi_segment_page_clear(page: &mut Page, tld: &mut SegmentsTld) -> *mut Page {
    let segment = get_segment(page);
    let inuse = page.capacity * mi_page_block_size(page);
    if !page.is_reset && mi_option_page_reset {
        let (start, psize) = page_start_from_segment(segment, page);
        page.is_reset = true;
        // _mi_os_reset(start, psize);
    }

    *page = Page {
        slice_count: page.slice_count,
        slice_offset: page.slice_offset,
        is_reset: page.is_reset,
        is_committed: page.is_committed,
        is_zero_init: false,
        capacity: 0,
        reserved: 0,
        free: null_mut(),
        local_free: null_mut(),
        retire_expire: 0,
        used: 0,
        is_zero: false,
        block_size: 1,
        heap: null_mut::<Heap>().into(),
        next: null_mut(),
        prev: null_mut(),
    };
    let slice = mi_segment_span_free_coalesce(page, tld);
    segment.used -= 1;
    return slice;
}
// void _mi_page_use_delayed_free(mi_page_t* page, mi_delayed_t delay, bool override_never) {
//   while (!_mi_page_try_use_delayed_free(page, delay, override_never)) {
//     mi_atomic_yield();
//   }

fn mi_page_set_heap(page: &mut Page, heap: &mut Heap) {
    page.heap.store(addr_of_mut!(*heap), Ordering::Relaxed);
}

fn mi_slice_is_used(slice: &Page) -> bool {
    slice.block_size > 0
}

fn mi_slices_start_iterate(segment: &mut Segment) -> (*mut Page, *mut Page) {
    let slice = addr_of_mut!(segment.slices[0]);
    let end = mi_segment_slices_end(segment);
    (
        unsafe { slice.add(unsafe { &*slice }.slice_count as usize) },
        end,
    )
}

fn mi_segment_size(segment: &mut Segment) -> usize {
    segment.segment_slices * MI_SEGMENT_SLICE_SIZE
}

fn mi_segments_track_size(SEGMENT_SIZE: isize, tld: &mut SegmentsTld) {
    tld.count += (SEGMENT_SIZE >= 0) as usize;
    tld.peak_count = tld.peak_count.max(tld.count);

    tld.current_size = (tld.current_size as isize + SEGMENT_SIZE) as usize;
    tld.peak_size = tld.peak_size.max(tld.current_size);
}

// static mi_segment_t* mi_segment_reclaim(mi_segment_t* segment, mi_heap_t* heap, size_t requested_block_size, bool* right_page_reclaimed, mi_segments_tld_t* tld) {
//   // for all slices
//   const mi_slice_t* end;
//   mi_slice_t* slice = mi_slices_start_iterate(segment, &end);
//   while (slice < end) {
//       // the span is free, add it to our page queues
//       slice = mi_segment_span_free_coalesce(slice, tld); // set slice again due to coalesceing
//     }
//     mi_assert_internal(slice->slice_count>0 && slice->slice_offset==0);
//     slice = slice + slice->slice_count;
//   }

//   if (segment->used == 0) {  // due to page_clear
//     mi_segment_free(segment, false, tld);
//     return NULL;
//   }
//   else {
//     return segment;
//   }
// }

const mi_option_max_segment_reclaim: usize = 0;

fn mi_segments_page_find_and_allocate(
    slice_count: usize,
    arena_id: ArenaId,
    tld: &mut SegmentsTld,
) -> *mut Page {
    let span_queue = mi_span_queue_for(slice_count, tld);
    let mut span_queue_addr = addr_of_mut!(*span_queue);
    slice_count = slice_count.max(1);
    while span_queue_addr <= addr_of_mut!(tld.spans[MI_SEGMENT_BIN_MAX]) {
        let slice_ptr = span_queue.first;
        while !slice_ptr.is_null() {
            let slice = unsafe { &mut *slice_ptr };

            if slice.slice_count as usize >= slice_count {
                let segment = get_segment(slice);

                if mi_arena_memid_is_suitable(segment.memid, arena_id) {
                    mi_span_queue_delete(&mut span_queue, slice);

                    if slice.slice_count as usize > slice_count {
                        mi_segment_slice_split(segment, slice, slice_count, tld);

                        let page = mi_segment_span_allocate(
                            segment,
                            mi_slice_index(slice),
                            slice.slice_count as usize,
                            tld,
                        );

                        if page.is_null() {
                            mi_segment_span_free_coalesce(slice, tld);
                            return null_mut();
                        }
                        return page;
                    }
                }
            }

            slice_ptr = slice.next;
        }
        span_queue_addr = unsafe { span_queue_addr.add(1) };
    }

    return null_mut();
}

fn mi_segment_span_free_coalesce(slice: &mut Page, tld: &mut SegmentsTld) -> *mut Page {
    let segment = get_segment(slice);
    let is_abandoned = mi_segment_is_abandoned(segment);
    if segment.kind == SegmentKind::Huge {
        slice.block_size = 0;
        return slice;
    }
    let slice_addr = addr_of_mut!(*slice);

    let mut slice_count = slice.slice_count as usize;
    let next = unsafe { slice_addr.add(slice_count as usize) };
    let nnext = unsafe { &mut *next };
    if next < mi_segment_slices_end(segment) && nnext.block_size == 0 {
        slice_count += nnext.slice_count as usize;
        if !is_abandoned {
            mi_segment_span_remove_from_queue(nnext, tld);
        }
    }
    if slice_addr > addr_of_mut!(segment.slices[0]) {
        let prev_addr = mi_slice_first(unsafe { slice_addr.sub(1) });
        let prev = unsafe { &mut *prev_addr };
        if prev.block_size == 0 {
            slice_count += prev.slice_count as usize;
            if !is_abandoned {
                mi_segment_span_remove_from_queue(prev, tld);
            }
            slice_addr = prev_addr;
        }
    }

    let slice = unsafe { &mut *slice_addr };
    mi_segment_span_free(segment, mi_slice_index(slice), slice_count as usize, tld);
    return slice_addr;
}
fn mi_segment_slices_end(segment: &mut Segment) -> *mut Page {
    unsafe { addr_of_mut!(segment.slices[segment.slice_entries - 1]).add(1) }
}

fn mi_segment_span_remove_from_queue(slice: &mut Page, tld: &mut SegmentsTld) {
    let sq = mi_span_queue_for(slice.slice_count as usize, tld);
    mi_span_queue_delete(sq, slice);
}

fn mi_slice_first(slice_addr: *mut Page) -> *mut Page {
    let slice = unsafe { &mut *slice_addr };
    unsafe { slice_addr.byte_sub(slice.slice_offset as usize) }
}

fn mi_segment_span_allocate(
    segment: &mut Segment,
    slice_index: usize,
    slice_count: usize,
    tld: &mut SegmentsTld,
) -> *mut Page {
    let slice = &mut segment.slices[slice_index];

    slice.slice_offset = 0;
    slice.slice_count = slice_count as u32;

    let bsize = slice_count * MI_SEGMENT_SLICE_SIZE;
    slice.block_size = (bsize as u32).min(MI_HUGE_BLOCK_SIZE);
    let page = slice;

    let extra = slice_count - 1;
    extra = extra.min(MI_MAX_SLICE_OFFSET);
    if slice_index + extra >= segment.slice_entries {
        extra = segment.slice_entries - slice_index - 1;
    }
    let slice_addr = addr_of_mut!(*slice);
    for i in 1..=extra {
        let slice_next = unsafe { &mut *slice_addr.add(i) };
        slice_next.slice_offset = (core::mem::size_of::<Page>() * i) as u32;
        slice_next.slice_count = 0;
        slice_next.block_size = 1;
    }

    let last_addr = unsafe { slice_addr.add(slice_count - 1) };
    let end_addr = mi_segment_slices_end(segment);
    if last_addr > end_addr {
        last_addr = end_addr;
    }
    if last_addr > slice_addr {
        let last = unsafe { &mut *last_addr };
        last.slice_offset = core::mem::size_of::<Page>() as u32
            * (unsafe { last_addr.offset_from(slice_addr) as u32 });
        last.slice_count = 0;
        last.block_size = 1;
    }

    let ppage = unsafe { &mut *page };
    ppage.is_reset = false;
    ppage.is_committed = true;
    segment.used += 1;

    return page;
}

fn mi_segment_slice_split(
    segment: &mut Segment,
    slice: &mut Page,
    slice_count: usize,
    tld: &mut SegmentsTld,
) {
    if slice.slice_count as usize <= slice_count {
        return;
    }
    let next_index = mi_slice_index(slice) + slice_count;
    let next_count = slice.slice_count as usize - slice_count;
    mi_segment_span_free(segment, next_index, next_count, tld);
    slice.slice_count = slice_count as u32;
}

fn mi_slice_index(slice: &mut Page) -> usize {
    let segment = get_segment(slice);
    (unsafe { addr_of_mut!(*slice).offset_from(addr_of_mut!(segment.slices[0])) }) as usize
}

#[derive(PartialEq, Eq)]
enum SegmentKind {
    Normal,
    Huge,
}

fn mi_segment_is_abandoned(segment: &mut Segment) -> bool {
    segment.thread_id.load(Ordering::Relaxed) == 0
}

fn mi_segment_span_free(
    segment: &mut Segment,
    slice_index: usize,
    slice_count: usize,
    tld: &mut SegmentsTld,
) {
    let sq = if segment.kind == SegmentKind::Huge || mi_segment_is_abandoned(segment) {
        null_mut()
    } else {
        mi_span_queue_for(slice_count, tld)
    };
    slice_count = slice_count.max(1);
    let slice = &mut segment.slices[slice_index];
    slice.slice_count = slice_count as u32;
    slice.slice_offset = 0;
    if slice_count > 1 {
        let last = &mut segment.slices[slice_index + slice_count - 1];
        last.slice_count = 0;
        last.slice_offset = core::mem::size_of::<Page>() as u32 * (slice_count as u32 - 1);
        last.block_size = 0;
    }

    // and push it on the free page queue (if it was not a huge page)
    if !sq.is_null() {
        mi_span_queue_push(unsafe { &mut *sq }, slice);
    } else {
        // mark huge page as free anyways
        slice.block_size = 0;
    }
}

fn mi_span_queue_push(sq: &mut SpanQueue, slice: &mut Page) {
    let slice_addr = addr_of_mut!(*slice);
    slice.prev = null_mut();
    slice.next = sq.first;
    sq.first = slice_addr;

    if !slice.next.is_null() {
        unsafe { &mut *slice.next }.prev = slice_addr;
    } else {
        sq.last = slice_addr;
    }

    slice.block_size = 0;
}

fn mi_span_queue_delete(sq: &mut SpanQueue, slice: &mut Page) {
    let slice_addr = addr_of_mut!(*slice);
    if !slice.prev.is_null() {
        unsafe { &mut *slice.prev }.next = slice.next;
    }
    if slice_addr == sq.first {
        sq.first = slice.next;
    }
    if !slice.next.is_null() {
        unsafe { &mut *slice.next }.prev = slice.prev;
    }
    if slice_addr == sq.last {
        sq.last = slice.prev;
    }

    slice.prev = null_mut();
    slice.next = null_mut();
    slice.block_size = 1;
}

type ArenaId = usize;

fn mi_arena_memid_is_suitable(arena_memid: usize, arena_id: ArenaId) -> bool {
    let id: ArenaId = arena_memid & 0x7F;
    let exclusive: bool = (arena_memid & 0x80) != 0;

    mi_arena_id_is_suitable(id, exclusive, arena_id)
}

fn mi_arena_id_none() -> ArenaId {
    0
}

fn mi_arena_id_is_suitable(arena_id: ArenaId, is_exclusive: bool, req_arena_id: ArenaId) -> bool {
    (!is_exclusive && req_arena_id == mi_arena_id_none()) || arena_id == req_arena_id
}

struct SpanQueue {
    first: *mut Page,
    last: *mut Page,
    slice_count: usize,
}

struct SegmentsTld {
    spans: [SpanQueue; MI_SEGMENT_BIN_MAX + 1],
    count: usize,
    peak_count: usize,
    current_size: usize,
    peak_size: usize,
}

fn mi_slice_bin(slice_count: usize) -> usize {
    if slice_count <= 1 {
        return slice_count;
    }
    slice_count -= 1;

    let s = MI_INTPTR_SIZE * 8 - 1 - slice_count.leading_zeros() as usize;
    if s <= 2 {
        return slice_count + 1;
    }

    let bin: usize = ((s << 2) | ((slice_count >> (s - 2)) & 0x03)) - 4;

    return bin;
}

fn mi_span_queue_for(slice_count: usize, segmets_tld: &mut SegmentsTld) -> &mut SpanQueue {
    let bin = mi_slice_bin(slice_count);

    &mut segmets_tld.spans[bin]
}

fn heap_collect_retired(heap: &mut Heap) {
    let min = MI_BIN_FULL;
    let max = 0;
    for bin in heap.page_retired_min..=heap.page_retired_max {
        let page_queue = heap.pages[bin];
        let page_ptr = page_queue.first;
        if !page_ptr.is_null() {
            let page = unsafe { &mut *page_ptr };
            if page.retire_expire != 0 {
                if page_all_free(page) {
                    page.retire_expire -= 1;
                    if page.retire_expire == 0 {
                        page_free(heap, &mut page_queue, page);
                    } else {
                        if bin < min {
                            min = bin;
                        }
                        if bin > max {
                            max = bin;
                        }
                    }
                } else {
                    page.retire_expire = 0;
                }
            }
        }
    }
    heap.page_retired_min = min;
    heap.page_retired_max = min;
}

fn page_all_free(page: &mut Page) -> bool {
    page.used == 0
}

fn page_free(heap: &mut Heap, queue: &mut Queue, page: &mut Page) {

    //   mi_page_set_has_aligned(page, false);

    //   // remove from the page list
    //   // (no need to do _mi_heap_delayed_free first as all blocks are already free)
    //   mi_page_queue_remove(pq, page);

    //   // and free it
    //   mi_page_set_heap(page,NULL);
    //   mi_segments_tld_t* segments_tld = &heap->tld->segments;
    //   _mi_segment_page_free(page, force, segments_tld);
    // }
}

fn page_queue_remove(heap: &mut Heap, queue: &mut Queue, page_ptr: *mut Page) {
    let page = unsafe { &mut *page_ptr };

    if !page.prev.is_null() {
        let prev_page = unsafe { &mut *(page.prev) };
        prev_page.next = page.next;
    }

    if !page.next.is_null() {
        let next_page = unsafe { &mut *(page.next) };
        next_page.prev = page.prev;
    }
    if page_ptr == queue.last {
        queue.last = page.prev;
    }
    if page_ptr == queue.first {
        queue.first = page.next;
        heap_queue_first_update(heap, queue);
    }

    heap.page_count -= 1;
    page.prev = null_mut();
    page.next = null_mut();

    mi_page_set_in_full(page, false);
}

const MI_ALIGN4W: bool = false;
const MI_ALIGN2W: bool = false;

fn wsize_from_size(size: usize) -> usize {
    size / MI_INTPTR_SIZE
}

fn mi_bin(size: usize) -> u8 {
    let wsize = wsize_from_size(size);

    let bin: u8;

    if wsize <= 1 {
        bin = 1;
    } else if (MI_ALIGN4W && wsize <= 4) || (MI_ALIGN2W && size <= 8) {
        bin = ((wsize + 1) & !1) as u8; // Align by 2 words
    } else if wsize <= 8 {
        bin = wsize as u8;
    } else if wsize > MI_MEDIUM_OBJ_SIZE_MAX {
        bin = MI_BIN_HUGE as u8;
    } else {
        if MI_ALIGN4W {
            wsize = (wsize + 3) & !3; // round to 4 words
        }
        wsize -= 1;
        let b = MI_INTPTR_SIZE as u8 * 8 - 1 - wsize.leading_zeros() as u8;

        // and use the top 3 bits to determine the bin (~12.5% worst internal fragmentation).
        // - adjust with 3 because we use do not round the first 8 sizes
        //   which each get an exact bin
        bin = ((b << 2) + ((wsize >> (b - 2)) as u8) & 0x3) - 3;
    }

    bin
}

fn heap_queue_first_update(heap: &mut Heap, queue: &mut Queue) {
    let size = queue.block_size;

    let empty: Page = Page {
        is_committed: false,
        is_zero_init: false,
        is_reset: false,
        slice_count: 0,
        slice_offset: 0,
        capacity: 0,
        reserved: 0,
        retire_expire: 0,
        free: null_mut(),
        local_free: null_mut(),
        used: 0,
        is_zero: false,
        block_size: 0,
        heap: null_mut::<Heap>().into(),
        next: null_mut(),
        prev: null_mut(),
    };

    let mut page = queue.first;
    if queue.first.is_null() {
        page = addr_of_mut!(empty);
    }

    let pages_free = &mut heap.pages_free_direct;
    let idx = wsize_from_size(size);
    if pages_free[idx] == page {
        return;
    }

    let mut start;
    if idx <= 0 {
        start = 0;
    } else {
        let bin = mi_bin(size);
        let prev_queue_ptr = unsafe { addr_of_mut!(*queue).sub(1) };
        while bin == mi_bin(unsafe { &mut *prev_queue_ptr }.block_size)
            && prev_queue_ptr > addr_of_mut!(heap.pages[0])
        {
            prev_queue_ptr = unsafe { prev_queue_ptr.sub(1) };
        }
        start = 1 + wsize_from_size(unsafe { &mut *prev_queue_ptr }.block_size);

        let start = start.min(idx);

        for i in start..=idx {
            pages_free[i] = page;
        }
    }
}

fn page_to_full(page_addr: *mut Page, heap: &mut Heap, from: &mut Queue, to: &mut Queue) {
    let page = unsafe { &mut *page_addr };
    // check if already in full, but why????
    if !page.prev.is_null() {
        let prev_page = unsafe { &mut *(page.prev) };
        prev_page.next = page.next;
    }

    if !page.next.is_null() {
        let next_page = unsafe { &mut *(page.next) };
        next_page.prev = page.prev;
    }
    if page_addr == from.last {
        from.last = page.prev;
    }
    if page_addr == from.first {
        from.first = page.next;
        heap_queue_first_update(heap, from)
    }
    page.prev = to.last;
    page.next = null_mut();

    if !to.last.is_null() {
        unsafe { &mut *to.last }.next = page_addr;
        to.last = page;
    } else {
        to.first = page_addr;
        to.last = page_addr;
        heap_queue_first_update(heap, to);
    }

    mi_page_set_in_full(page, mi_page_queue_is_full(to));
}

fn mi_page_set_in_full(page: &mut Page, in_full: bool) {
    page.flags.x.in_full = in_full;
}

fn mi_page_queue_is_full(pq: &Queue) -> bool {
    pq.block_size == MI_MEDIUM_OBJ_SIZE_MAX + 2 * core::mem::size_of::<*mut u8>()
}

// fn mi_page_is_in_full(page: &Page) -> bool {
//   page.flags.x.in_full
// }

fn mi_page_queue_enqueue_from(
    page_addr: *mut Page,
    heap: &mut Heap,
    from: &mut Queue,
    to: &mut Queue,
) {
    let page = unsafe { &mut *page_addr };
    // check if already in full, but why????
    if !page.prev.is_null() {
        let prev_page = unsafe { &mut *(page.prev) };
        prev_page.next = page.next;
    }

    if !page.next.is_null() {
        let next_page = unsafe { &mut *(page.next) };
        next_page.prev = page.prev;
    }
    if page_addr == from.last {
        from.last = page.prev;
    }
    if page_addr == from.first {
        from.first = page.next;
        heap_queue_first_update(heap, from)
    }
    page.prev = to.last;
    page.next = null_mut();

    if !to.last.is_null() {
        unsafe { &mut *to.last }.next = page_addr;
        to.last = page;
    } else {
        to.first = page_addr;
        to.last = page_addr;
        heap_queue_first_update(heap, to);
    }
}

fn mi_page_to_full(heap: &mut Heap, page: &mut Page, queue: &mut Queue) {
    // if mi_page_is_in_full() return;
    mi_page_queue_enqueue_from(
        addr_of_mut!(*page),
        heap,
        &mut heap.pages[MI_BIN_FULL],
        queue,
    );
    page_free_collect(page);
}

const MI_MAX_EXTEND_SIZE: usize = 4 * 1024; // heuristic, one OS page seems to work well.
const MI_MIN_EXTEND: usize = 4;

fn page_extend_free(heap: &mut Heap, page: &mut Page) {
    if !page.free.is_null() {
        return;
    }
    if page.capacity >= page.reserved {
        return;
    }

    let stuff = page_start_from_segment(get_segment(page), addr_of_mut!(*page));
    let page_size = stuff.1;

    let bsize = if page.block_size < MI_HUGE_BLOCK_SIZE {
        page.block_size as usize
    } else {
        page_size
    };

    let mut extend = page.reserved - page.capacity;

    let mut max_extend = if bsize >= MI_MAX_EXTEND_SIZE {
        MI_MIN_EXTEND
    } else {
        MI_MAX_EXTEND_SIZE / bsize
    };
    max_extend = max_extend.max(MI_MIN_EXTEND);
    extend = extend.min(max_extend);

    // we saved out on  secure extend, hope that doesn't fuck us over
    page_free_list_extend(page, bsize, extend, stuff.0);

    page.capacity += extend;

    //   // extension into zero initialized memory preserves the zero'd free list
    if !page.is_zero_init {
        page.is_zero = false;
    }
}

fn page_free_list_extend(page: &mut Page, block_size: usize, extend: usize, page_area: *mut u8) {
    let first_free_block = unsafe {
        page_area
            .byte_add(page.capacity * block_size)
            .cast::<Block>()
    };
    // let's hope this isn't off by something, too small would be fine too

    for i in 0..extend - page.capacity {
        let block = unsafe { &mut *first_free_block.add(i) };
        block.next = unsafe { first_free_block.add(i + 1) };
    }

    let last = unsafe { &mut *first_free_block.add(extend - page.capacity) };

    // usually null
    last.next = page.free;
    page.free = first_free_block;
}

fn get_segment(page: &mut Page) -> &mut Segment {
    unsafe { &mut *((addr_of_mut!(*page) as usize & !(4 * MB)) as *mut Segment) }
}

fn page_start_from_segment(segment: &mut Segment, p: *mut Page) -> (*mut u8, usize) {
    let idx = unsafe { p.offset_from(addr_of_mut!(segment.slices[0])) } as usize;
    let page = unsafe { &mut *p };
    let psize = page.slice_count as usize * MI_SEGMENT_SLICE_SIZE;
    // potentially unalign but idgaf

    (
        unsafe {
            addr_of_mut!(*segment)
                .byte_add(idx * MI_SEGMENT_SLICE_SIZE)
                .cast::<u8>()
        },
        psize,
    )
}

fn page_free_collect(page: &mut Page) {
    page_thread_free_collect(page);

    if !page.local_free.is_null() {
        if page.free.is_null() {
            page.free = page.local_free;
            page.local_free = null_mut();
            page.is_zero = false
        }
    }
}

fn page_thread_free_collect(page: &mut Page) {
    todo!();
}

fn page_immediately_available(page: &mut Page) -> bool {
    !page.free.is_null()
}

pub fn free(p: *mut u8) {
    // Null check
    let segment_ptr = (p as usize & !(4 * MB)) as *mut Segment;
    let segment = unsafe { &mut *segment_ptr };

    let page = unsafe {
        &mut segment.slices[p.offset_from(segment_ptr as *const u8) as usize >> /* segment->page_shift*/ 16]
    };

    let block_addr = p.cast::<Block>();
    let block = unsafe { &mut *block_addr };

    // if (thread_id() == segment->thread_id) { // local free

    block.next = page.local_free;
    page.local_free = block_addr;
    page.used -= 1;
    if page.used == 0 {}
    // if (page->used - page->thread_freed == 0) page_free(page);

    // }
    // else { // non-local free
    // atomic_push( &page->thread_free, block);
    // atomic_incr( &page->thread_freed );
    // }
    // }
}
