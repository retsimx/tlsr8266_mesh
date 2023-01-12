extern crate core;

use app::App;

use config::*;
use ota::handle_ota_update__attribute_ram_code;

mod sdk;
mod main_light;
mod common;
mod vendor_light;
mod ota;
mod config;
mod app;
mod math_wrapper;

pub static APP: App = App::default();

#[no_mangle]
pub fn main_entrypoint() -> () {
    // Must happen first
    handle_ota_update__attribute_ram_code();

    // Configure the system
    APP.init();

    // Run the application
    APP.run();
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}