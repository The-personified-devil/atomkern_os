#[derive(Debug)]
#[repr(C)]
pub struct StartParams {
    pub tls: Tls,
    pub tid: super::ThreadId,
    pub thread_start: u64,
}

#[derive(Debug, Clone, Copy, Default)]
#[repr(C)]
pub struct Tls {
    pub image_addr: u64,
    pub image_size: u64,
    pub size: u64,
}
