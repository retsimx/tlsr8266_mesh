#[macro_export]
macro_rules! pub_mut {
    ( $name:ident, $typ:ty, $val:expr ) => {
        #[no_mangle]
        static mut $name: $typ = $val;

        paste::paste! {
            #[allow(dead_code)]
            pub fn [<get_ $name>]() -> &'static mut $typ {
                unsafe {
                    return &mut $name;
                }
            }

            #[allow(dead_code)]
            pub fn [<set_ $name>](value: $typ) {
                unsafe {
                    $name = value;
                }
            }

            #[allow(dead_code)]
            pub fn [<get_ $name _addr>]() -> *const $typ {
                unsafe {
                    use core::ptr::addr_of;
                    return addr_of!($name);
                }
            }
        }
    };

    ( $name:ident, $typ:ty ) => {
        extern "C" {
            static mut $name: $typ;
        }

        paste::paste! {
            #[allow(dead_code)]
            pub fn [<get_ $name>]() -> &'static mut $typ {
                unsafe {
                    return &mut $name;
                }
            }

            #[allow(dead_code)]
            pub fn [<set_ $name>](value: $typ) {
                unsafe {
                    $name = value;
                }
            }

            #[allow(dead_code)]
            pub fn [<get_ $name _addr>]() -> *const $typ {
                unsafe {
                    use core::ptr::addr_of;
                    return addr_of!($name);
                }
            }
        }
    };
}

#[macro_export]
macro_rules! pub_static {
    ( $name:ident, $typ:ty, $val:expr ) => {
        #[no_mangle]
        static $name: $typ = $val;

        paste::paste! {
            #[allow(dead_code)]
            pub fn [<get_ $name>]() -> &'static $typ {
                unsafe {
                    return &$name;
                }
            }

            #[allow(dead_code)]
            pub fn [<get_ $name _addr>]() -> *const $typ {
                unsafe {
                    use core::ptr::addr_of;
                    return addr_of!($name);
                }
            }
        }
    };

    ( $name:ident, $typ:ty ) => {
        extern "C" {
            static $name: $typ;
        }

        paste::paste! {
            #[allow(dead_code)]
            pub fn [<get_ $name>]() -> &'static $typ {
                unsafe {
                    return &$name;
                }
            }

            #[allow(dead_code)]
            pub fn [<get_ $name _addr>]() -> *const $typ {
                unsafe {
                    use core::ptr::addr_of;
                    return addr_of!($name);
                }
            }
        }
    };
}

#[macro_export]
macro_rules! pub_mut_no_move {
    ( $name:ident, $typ:ty, $val:expr ) => {
        #[no_mangle]
        static mut $name: $typ = $val;

        paste::paste! {
            #[allow(dead_code)]
            pub fn [<get_ $name>]() -> &'static mut $typ {
                unsafe {
                    return &mut $name;
                }
            }

            #[allow(dead_code)]
            pub fn [<set_ $name>](value: $typ) {
                unsafe {
                    $name = value;
                }
            }

            #[allow(dead_code)]
            pub fn [<get_ $name _addr>]() -> *const $typ {
                unsafe {
                    use core::ptr::addr_of;
                    return addr_of!($name);
                }
            }
        }
    };
}

#[macro_export]
macro_rules! no_mangle_fn_def {
    ( $name:ident ) => {
        #[no_mangle]
        extern "C" fn $name() {}

        paste::paste! {
            pub fn [<_ $name>]() {
                unsafe {
                    $name();
                }
            }
        }
    };
}

#[macro_export]
macro_rules! no_mangle_fn {
    ( $name:ident ) => {
        #[no_mangle]
        extern "C" { fn $name(); }

        paste::paste! {
            pub fn [<_ $name>]() {
                unsafe {
                    $name();
                }
            }
        }
    };

    ( $name:ident, $rettype:ty) => {
        #[no_mangle]
        extern "C" { fn $name() -> $rettype; }

        paste::paste! {
            pub fn [<_ $name>]() -> $rettype {
                unsafe {
                    return $name();
                }
            }
        }
    };

    ( $name:ident, $rettype:ty, $i1:ident: $t1:ty) => {
        #[no_mangle]
        extern "C" { fn $name($i1: $t1) -> $rettype; }

        paste::paste! {
            pub fn [<_ $name>]($i1: $t1) -> $rettype {
                unsafe {
                    return $name($i1);
                }
            }
        }
    };

    ( $name:ident, $i1:ident: $t1:ty) => {
        #[no_mangle]
        extern "C" { fn $name($i1: $t1); }

        paste::paste! {
            pub fn [<_ $name>]($i1: $t1) {
                unsafe {
                    $name($i1);
                }
            }
        }
    };

    ( $name:ident, $rettype:ty, $i1:ident: $t1:ty, $i2:ident: $t2:ty) => {
        #[no_mangle]
        extern "C" { fn $name($i1: $t1, $i2: $t2) -> $rettype; }

        paste::paste! {
            pub fn [<_ $name>]($i1: $t1, $i2: $t2) -> $rettype {
                unsafe {
                    return $name($i1, $i2);
                }
            }
        }
    };

    ( $name:ident, $i1:ident: $t1:ty, $i2:ident: $t2:ty) => {
        #[no_mangle]
        extern "C" { fn $name($i1: $t1, $i2: $t2); }

        paste::paste! {
            pub fn [<_ $name>]($i1: $t1, $i2: $t2) {
                unsafe {
                    $name($i1, $i2);
                }
            }
        }
    };

    ( $name:ident, $rettype:ty, $i1:ident: $t1:ty, $i2:ident: $t2:ty, $i3:ident: $t3:ty) => {
        #[no_mangle]
        extern "C" { fn $name($i1: $t1, $i2: $t2, $i3: $t3) -> $rettype; }

        paste::paste! {
            pub fn [<_ $name>]($i1: $t1, $i2: $t2, $i3: $t3) -> $rettype {
                unsafe {
                    return $name($i1, $i2, $i3);
                }
            }
        }
    };

    ( $name:ident, $i1:ident: $t1:ty, $i2:ident: $t2:ty, $i3:ident: $t3:ty) => {
        #[no_mangle]
        extern "C" { fn $name($i1: $t1, $i2: $t2, $i3: $t3); }

        paste::paste! {
            pub fn [<_ $name>]($i1: $t1, $i2: $t2, $i3: $t3) {
                unsafe {
                    $name($i1, $i2, $i3);
                }
            }
        }
    };

    ( $name:ident, $rettype:ty, $i1:ident: $t1:ty, $i2:ident: $t2:ty, $i3:ident: $t3:ty, $i4:ident: $t4:ty) => {
        #[no_mangle]
        extern "C" { fn $name($i1: $t1, $i2: $t2, $i3: $t3, $i4: $t4) -> $rettype; }

        paste::paste! {
            pub fn [<_ $name>]($i1: $t1, $i2: $t2, $i3: $t3, $i4: $t4) -> $rettype {
                unsafe {
                    return $name($i1, $i2, $i3, $i4);
                }
            }
        }
    };

    ( $name:ident, $i1:ident: $t1:ty, $i2:ident: $t2:ty, $i3:ident: $t3:ty, $i4:ident: $t4:ty) => {
        #[no_mangle]
        extern "C" { fn $name($i1: $t1, $i2: $t2, $i3: $t3, $i4: $t4); }

        paste::paste! {
            pub fn [<_ $name>]($i1: $t1, $i2: $t2, $i3: $t3, $i4: $t4) {
                unsafe {
                    $name($i1, $i2, $i3, $i4);
                }
            }
        }
    };

    ( $name:ident, $rettype:ty, $i1:ident: $t1:ty, $i2:ident: $t2:ty, $i3:ident: $t3:ty, $i4:ident: $t4:ty, $i5:ident: $t5:ty) => {
        #[no_mangle]
        extern "C" { fn $name($i1: $t1, $i2: $t2, $i3: $t3, $i4: $t4, $i5: $t5) -> $rettype; }

        paste::paste! {
            pub fn [<_ $name>]($i1: $t1, $i2: $t2, $i3: $t3, $i4: $t4, $i5: $t5) -> $rettype {
                unsafe {
                    return $name($i1, $i2, $i3, $i4, $i5);
                }
            }
        }
    };

    ( $name:ident, $i1:ident: $t1:ty, $i2:ident: $t2:ty, $i3:ident: $t3:ty, $i4:ident: $t4:ty, $i5:ident: $t5:ty) => {
        #[no_mangle]
        extern "C" { fn $name($i1: $t1, $i2: $t2, $i3: $t3, $i4: $t4, $i5: $t5); }

        paste::paste! {
            pub fn [<_ $name>]($i1: $t1, $i2: $t2, $i3: $t3, $i4: $t4, $i5: $t5) {
                unsafe {
                    $name($i1, $i2, $i3, $i4, $i5);
                }
            }
        }
    };

    ( $name:ident, $rettype:ty, $i1:ident: $t1:ty, $i2:ident: $t2:ty, $i3:ident: $t3:ty, $i4:ident: $t4:ty, $i5:ident: $t5:ty, $i6:ident: $t6:ty) => {
        #[no_mangle]
        extern "C" { fn $name($i1: $t1, $i2: $t2, $i3: $t3, $i4: $t4, $i5: $t5, $i6: $t6) -> $rettype; }

        paste::paste! {
            pub fn [<_ $name>]($i1: $t1, $i2: $t2, $i3: $t3, $i4: $t4, $i5: $t5, $i6: $t6) -> $rettype {
                unsafe {
                    return $name($i1, $i2, $i3, $i4, $i5, $i6);
                }
            }
        }
    };

    ( $name:ident, $i1:ident: $t1:ty, $i2:ident: $t2:ty, $i3:ident: $t3:ty, $i4:ident: $t4:ty, $i5:ident: $t5:ty, $i6:ident: $t6:ty) => {
        #[no_mangle]
        extern "C" { fn $name($i1: $t1, $i2: $t2, $i3: $t3, $i4: $t4, $i5: $t5, $i6: $t6); }

        paste::paste! {
            pub fn [<_ $name>]($i1: $t1, $i2: $t2, $i3: $t3, $i4: $t4, $i5: $t5, $i6: $t6) {
                unsafe {
                    $name($i1, $i2, $i3, $i4, $i5, $i6);
                }
            }
        }
    };
}
