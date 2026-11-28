//! Generic async loop bodies and concrete task-wrapper macro.

use crate::api::Product;

/// Generic mesh receive loop body; composition selects the product monomorphically.
pub async fn platform_mesh_rx_loop<P: Product>() {
    loop {
        core::future::pending::<()>().await;
    }
}

/// Generic heartbeat loop body; composition selects the product monomorphically.
pub async fn platform_heartbeat_loop<P: Product>() {
    loop {
        core::future::pending::<()>().await;
    }
}

/// Stamp concrete Embassy wrappers in a scoped module for a product ZST.
///
/// This macro is intentionally module-scoped so multiple products can expose
/// independent wrappers in the same binary without a runtime registry.
#[macro_export]
macro_rules! define_platform_tasks {
    ($mod_name:ident, $product:ty) => {
        pub mod $mod_name {
            use super::*;

            #[embassy_executor::task]
            pub async fn mesh_rx() {
                $crate::tasks::platform_mesh_rx_loop::<$product>().await;
            }

            #[embassy_executor::task]
            pub async fn heartbeat() {
                $crate::tasks::platform_heartbeat_loop::<$product>().await;
            }
        }
    };
}
