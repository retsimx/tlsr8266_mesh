pub const ONES_32 : u32 =  0xffffffff;

#[macro_export]
macro_rules! BIT {
    ( $x:expr ) => {
        1 << $x
    };
}

#[macro_export]
macro_rules! BIT_MASK_LEN {
    ( $x:expr ) => {
        BIT!($x) - 1
    };
}

// bits range: BITS_RNG(4, 5)  0b000111110000,  start from 4, length = 5
#[macro_export]
macro_rules! BIT_RNG {
    ( $s:expr, $e:expr ) => {
        BIT_MASK_LEN!($e-$s+1) << $s
    };
}

#[macro_export]
macro_rules! BIT_LOW_BIT {
    ($y:expr) => (
    if (($y as u32) & BIT ! (0) != 0){0} else {if (($y as u32) & BIT ! (1) != 0){1} else {if (($y as u32) & BIT ! (2) != 0){2} else {if (($y as u32) & BIT ! (3) != 0){3} else {
    if (($y as u32) & BIT ! (4) != 0){4} else {if (($y as u32) & BIT ! (5) != 0){5} else {if (($y as u32) & BIT ! (6) != 0){6} else {if (($y as u32) & BIT ! (7) != 0){7} else {
    if (($y as u32) & BIT ! (8) != 0){8} else {if (($y as u32) & BIT ! (9) != 0){9} else {if (($y as u32) & BIT ! (10) != 0){10} else {if (($y as u32) & BIT ! (11) != 0){11} else {
    if (($y as u32) & BIT ! (12) != 0){12} else {if (($y as u32) & BIT ! (13) != 0){13} else {if (($y as u32) & BIT ! (14) != 0){14} else {if (($y as u32) & BIT ! (15) != 0){15} else {
    if (($y as u32) & BIT ! (16) != 0){16} else {if (($y as u32) & BIT ! (17) != 0){17} else {if (($y as u32) & BIT ! (18) != 0){18} else {if (($y as u32) & BIT ! (19) != 0){19} else {
    if (($y as u32) & BIT ! (20) != 0){20} else {if (($y as u32) & BIT ! (21) != 0){21} else {if (($y as u32) & BIT ! (22) != 0){22} else {if (($y as u32) & BIT ! (23) != 0){23} else {
    if (($y as u32) & BIT ! (24) != 0){24} else {if (($y as u32) & BIT ! (25) != 0){25} else {if (($y as u32) & BIT ! (26) != 0){26} else {if (($y as u32) & BIT ! (27) != 0){27} else {
    if (($y as u32) & BIT ! (28) != 0){28} else {if (($y as u32) & BIT ! (29) != 0){29} else {if (($y as u32) & BIT ! (30) != 0){30} else {if (($y as u32) & BIT ! (31) != 0){31} else {32
    }}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}
    )
}

#[macro_export]
macro_rules! MASK_VAL {
    ($m:expr, $v:expr) => (
        (($v << BIT_LOW_BIT!($m)) & $m)
    );

    ($m:expr, $v:expr $(, $extra:expr)*) => (
        (($v << BIT_LOW_BIT!($m)) & $m) | MASK_VAL!($($extra),*)
    )
}

#[macro_export]
macro_rules! BM_MASK_VAL {
    ($x:expr, $mask:expr, $v:expr) => (
        ((($x) & !($mask)) | ($v))
    )
}

#[macro_export]
macro_rules! BM_SET_MASK_FLD {
    ($x:expr, $mask:expr, $v:expr) => (
        (($x) = BM_MASK_VAL!($x,$mask,$v))
    )
}

#[macro_export]
macro_rules! SET_FLD_V {
    ($x:expr, $m:expr, $v:expr) => (
        BM_SET_MASK_FLD!($x, $m, MASK_VAL!($m,$v))
    );
    
    ($x:expr, $m1:expr, $v1:expr, $m2:expr, $v2:expr) => (
        BM_SET_MASK_FLD!($x, $m1|$m2, MASK_VAL!($m1,$v1)| MASK_VAL!($m2,$v2))
    );
    
    ($x:expr, $m1:expr, $v1:expr, $m2:expr, $v2:expr, $m3:expr, $v3:expr) => (
        BM_SET_MASK_FLD!($x, $m1|$m2|$m3, MASK_VAL!($m1,$v1)| MASK_VAL!($m2,$v2)| MASK_VAL!($m3,$v3))
    );
    
    ($x:expr, $m1:expr, $v1:expr, $m2:expr, $v2:expr, $m3:expr, $v3:expr, $m4:expr, $v4:expr) => (
        BM_SET_MASK_FLD!($x, $m1|$m2|$m3|$m4, MASK_VAL!($m1,$v1)| MASK_VAL!($m2,$v2)| MASK_VAL!($m3,$v3)| MASK_VAL!($m4,$v4))
    );
    
    ($x:expr, $m1:expr, $v1:expr, $m2:expr, $v2:expr, $m3:expr, $v3:expr, $m4:expr, $v4:expr, $m5:expr, $v5:expr) => (
        BM_SET_MASK_FLD!($x, $m1|$m2|$m3|$m4|$m5, MASK_VAL!($m1,$v1)| MASK_VAL!($m2,$v2)| MASK_VAL!($m3,$v3)| MASK_VAL!($m4,$v4)| MASK_VAL!($m5,$v5))
    );
    
    ($x:expr, $m1:expr, $v1:expr, $m2:expr, $v2:expr, $m3:expr, $v3:expr, $m4:expr, $v4:expr, $m5:expr, $v5:expr, $m6:expr, $v6:expr) => (
        BM_SET_MASK_FLD!($x, $m1|$m2|$m3|$m4|$m5|$m6, MASK_VAL!($m1,$v1)| MASK_VAL!($m2,$v2)| MASK_VAL!($m3,$v3)| MASK_VAL!($m4,$v4)| MASK_VAL!($m5,$v5)| MASK_VAL!($m6,$v6))
    );
}

#[macro_export]
macro_rules! SET_FLD {
    ($x:expr, $mask:expr) => (
        BM_SET!($x, $mask)
    )
}

#[macro_export]
macro_rules! BM_SET {
    ($x:expr, $mask:expr) => (
        (($x) |= ($mask))
    )
}

#[macro_export]
macro_rules! CLR_FLD {
    ($x:expr, $mask:expr) => (
        BM_CLR!($x, $mask)
    )
}

#[macro_export]
macro_rules! BM_CLR {
    ($x:expr, $mask:expr) => (
        (($x) &= !($mask))
    )
}