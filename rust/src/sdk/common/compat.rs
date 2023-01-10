extern "C" {
    fn memcmp(s1: *const u8, s2: *const u8, count: usize) -> i32;
}

#[no_mangle]
fn bcmp (s1: *const u8, s2: *const u8, count: usize) -> i32
{
    unsafe { return memcmp(s1, s2, count); }
}