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
macro_rules! pub_static {
    ( $name:ident, $typ:ty, $val:expr ) => {
        paste::paste! {
            #[repr(C, align(4))]
            struct [<$name _type>] ($typ);

            #[allow(non_upper_case_globals)]
            static $name: [<$name _type>] = [<$name _type>] {0: $val};

            #[allow(dead_code)]
            pub fn [<get_ $name>]() -> &'static $typ {
                unsafe {
                    return &$name.0;
                }
            }

            #[allow(dead_code)]
            pub fn [<get_ $name _addr>]() -> *const $typ {
                unsafe {
                    use core::ptr::addr_of;
                    return addr_of!($name.0);
                }
            }
        }
    };
}

pub const unsafe fn transmute<From, To>(from: From) -> To {
    union Transmute<From, To> {
        from: core::mem::ManuallyDrop<From>,
        to: core::mem::ManuallyDrop<To>,
    }

    core::mem::ManuallyDrop::into_inner(Transmute { from: core::mem::ManuallyDrop::new(from) }.to)
}

pub const unsafe fn concat<First, Second, Out>(a: &[u8], b: &[u8]) -> Out
    where
        First: Copy,
        Second: Copy,
        Out: Copy,
{
    #[repr(C)]
    #[derive(Copy, Clone)]
    struct Both<A, B>(A, B);

    let arr: Both<First, Second> = Both(
        *transmute::<_, *const First>(a.as_ptr()),
        *transmute::<_, *const Second>(b.as_ptr()),
    );

    transmute(arr)
}

#[macro_export]
macro_rules! const_concat {
    () => {
        ""
    };
    ($a:expr) => {
        $a
    };
    ($a:expr, $b:expr) => {{
        let bytes: [u8; 16] = unsafe {
            crate::sdk::common::macros::concat::<
                [u8; $a.len()],
                [u8; $b.len()],
                [u8; $a.len() + $b.len()],
            >($a, $b)
        };

        unsafe { crate::sdk::common::macros::transmute::<_, [u8; 16]>(bytes) }
    }};
    ($a:expr, $($rest:expr),*) => {{
        const TAIL: &str = const_concat!($($rest),*);
        const_concat!($a, TAIL)
    }};
    ($a:expr, $($rest:expr),*,) => {
        const_concat!($a, $($rest),*)
    };
}
