#[macro_export]
macro_rules! pub_mut {
    ( $name:ident, $typ:ty, $val:expr ) => {
        paste::paste! {
            #[repr(C, align(4))]
            struct [<$name _type>] ($typ);

            #[allow(non_upper_case_globals)]
            static mut $name: [<$name _type>] = [<$name _type>] {0: $val};

            #[allow(dead_code)]
            pub fn [<get_ $name>]() -> &'static mut $typ {
                unsafe {
                    return &mut $name.0;
                }
            }

            #[allow(dead_code)]
            pub fn [<set_ $name>](value: $typ) {
                unsafe {
                    $name.0 = value;
                }
            }

            #[allow(dead_code)]
            pub fn [<get_ $name _addr>]() -> *mut $typ {
                unsafe {
                    use core::ptr::addr_of_mut;
                    return addr_of_mut!($name.0);
                }
            }
        }
    };
}

#[macro_export]
macro_rules! const_assert {
    ($($tt:tt)*) => {
        const _: () = assert!($($tt)*);
    }
}