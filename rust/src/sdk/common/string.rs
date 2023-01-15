use std::ptr::null_mut;

#[no_mangle]
pub unsafe fn strcpy(mut dst0: *mut u8, mut src0: *const u8) -> *mut u8 {
    let mut s = dst0;
    loop {
        let fresh0 = src0;
        src0 = src0.offset(1);
        let fresh1 = dst0;
        dst0 = dst0.offset(1);
        *fresh1 = *fresh0;
        if *fresh1 == 0 {
            break;
        }
    }
    return s;
}

#[no_mangle]
pub unsafe fn strchr(mut s: *const u8, mut c: u8) -> *mut u8 {
    loop {
        if *s == c {
            return s as *mut u8;
        }
        let fresh2 = s;
        s = s.offset(1);
        if *fresh2 == 0 {
            break;
        }
    }
    return null_mut();
}

#[no_mangle]
pub unsafe fn memcmp(mut m1: *const u8, mut m2: *const u8, mut n: u32) -> i32 {
    let mut s1 = m1;
    let mut s2 = m2;
    loop {
        let fresh3 = n;
        n = n.wrapping_sub(1);
        if fresh3 == 0 {
            break;
        }
        if *s1 != *s2 {
            return *s1 as i32 - *s2 as i32;
        }
        s1 = s1.offset(1);
        s2 = s2.offset(1);
    }
    return 0;
}

#[no_mangle]
pub unsafe fn memchr(mut src_void: *const u8, mut c: i32, mut length: u32) -> *mut u8 {
    let mut src = src_void;
    loop {
        let fresh4 = length;
        length -= 1;
        if fresh4 == 0 {
            break;
        }
        if *src == c as u8 {
            return src as *mut u8;
        }
        src = src.offset(1);
    }

    return null_mut();
}

#[no_mangle]
pub unsafe fn memmove(mut dest: *mut u8, mut src: *const u8, mut n: u32) -> *mut u8 {
    let mut d = dest;
    let mut s = src;
    loop {
        let fresh5 = n;
        n = n.wrapping_sub(1);
        if fresh5 == 0 {
            break;
        }
        let fresh6 = s;
        s = s.offset(1);
        let fresh7 = d;
        d = d.offset(1);
        *fresh7 = *fresh6;
    }
    return dest;
}

#[no_mangle]
pub unsafe fn bcopy(mut src: *mut u8, mut dest: *mut u8, mut len: u32) {
    if dest < src {
        loop {
            let fresh8 = len;
            len = len.wrapping_sub(1);
            if fresh8 == 0 {
                break;
            }
            let fresh9 = src;
            src = src.offset(1);
            let fresh10 = dest;
            dest = dest.offset(1);
            *fresh10 = *fresh9;
        }
    } else {
        let mut lasts = src.offset(len.wrapping_sub(1) as isize);
        let mut lastd = dest.offset(len.wrapping_sub(1) as isize);
        loop {
            let fresh11 = len;
            len = len.wrapping_sub(1);
            if fresh11 == 0 {
                break;
            }
            let fresh12 = lasts;
            lasts = lasts.offset(-1);
            let fresh13 = lastd;
            lastd = lastd.offset(-1);
            *fresh13 = *fresh12;
        }
    };
}

#[no_mangle]
pub unsafe fn memset(mut dest: *mut u8, mut val: i32, mut len: u32) -> *mut u8 {
    let mut ptr = dest;
    loop {
        let fresh14 = len;
        len = len.wrapping_sub(1);
        if fresh14 == 0 {
            break;
        }
        let fresh15 = ptr;
        ptr = ptr.offset(1);
        *fresh15 = val as u8;
    }
    return dest;
}

#[no_mangle]
pub unsafe fn memcpy(mut out: *mut u8, mut in_0: *const u8, mut length: u32) -> *mut u8 {
    bcopy(in_0 as *mut u8, out, length);
    return out;
}

#[no_mangle]
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
pub unsafe fn strlen(mut str: *const u8) -> u32 {
    let mut len = 0;

    if !str.is_null() {
        while *str != 0 {
            len += 1;
            str = str.offset(1);
        }
    }

    return len;
}

#[no_mangle]
pub unsafe fn strcmp(mut first_string: *const u8, mut second_string: *const u8) -> i32 {
    while *first_string == *second_string {
        if *first_string == 0 {
            return 0;
        }
        first_string = first_string.offset(1);
        second_string = second_string.offset(1);
    }
    if (*first_string as i8 - *second_string as i8) < 0 {
        return -1;
    }
    return 1;
}

#[no_mangle]
pub unsafe fn strncpy(mut s: *mut u8, mut t: *const u8, mut n: u32) -> *mut u8 {
    let mut p: *mut u8 = s;
    let mut i: u32 = 0;
    if s.is_null() {
        return s;
    }
    while !t.is_null() && i < n {
        let fresh19 = t;
        t = t.offset(1);
        let fresh20 = s;
        s = s.offset(1);
        *fresh20 = *fresh19;
        i += 1;
    }
    if t.is_null() {
        loop {
            let fresh21 = s;
            s = s.offset(1);
            *fresh21 = 0;
            let fresh22 = i;
            i += 1;
            if !(fresh22 < n) {
                break;
            }
        }
    }
    return p;
}

#[no_mangle]
pub unsafe fn ismemzero4(mut data: *mut u8, mut len: u32) -> i32 {
    let mut p = data as *mut u32;
    len = len >> 2;
    let mut i = 0;
    while i < len {
        if *p != 0 {
            return 0;
        }
        p = p.offset(1);
        i += 1;
    }
    return 1;
}

#[no_mangle]
pub unsafe fn ismemf4(mut data: *mut u8, mut len: u32) -> i32 {
    let mut p = data as *mut u32;
    len = len >> 2;
    let mut i = 0;
    while i < len {
        if *p != 0xffffffff {
            return 0;
        }
        p = p.offset(1);
        i += 1;
    }
    return 1;
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

#[no_mangle]
pub unsafe fn zeromem4(mut data: *mut u8, mut len: u32) {
    memset4(data, 0, len);
}
