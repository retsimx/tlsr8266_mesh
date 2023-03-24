use core::ptr::null_mut;

#[no_mangle] // required by light_ll.c
pub unsafe fn memcpy4(mut d: *mut u8, mut s: *const u8, mut length: u32) {
    let mut dst = d as *mut u32;
    let mut src = s as *mut u32;
    let mut len = length >> 2;
    loop {
        let fresh16 = len;
        len -= 1;
        if fresh16 == 0 {
            break;
        }
        let fresh17 = src;
        src = src.offset(1);
        let fresh18 = dst;
        dst = dst.offset(1);
        *fresh18 = *fresh17;
    }
}

#[no_mangle]
pub unsafe fn memset4(mut dest: *mut u8, mut val: u32, mut len: u32) -> *mut u8 {
    let mut p = dest as *mut u32;
    len = len >> 2;
    let mut i = 0;
    while i < len {
        let fresh23 = p;
        p = p.offset(1);
        *fresh23 = val;
        i += 1;
    }
    return dest;
}

#[no_mangle] // required by light_ll.c
pub unsafe fn zeromem4(mut data: *mut u8, mut len: u32) {
    memset4(data, 0, len);
}
