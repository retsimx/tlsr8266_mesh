use app::App;
use ota::handle_ota_update__attribute_ram_code;

mod sdk;
mod main_light;
mod common;
mod vendor_light;
mod app;
mod config;
mod ota;

pub static APP: App = App::default();

#[no_mangle]
pub unsafe fn main_entrypoint() {
    // Must happen first
    handle_ota_update__attribute_ram_code();

    // Configure the system
    APP.init();

    // Run the application
    APP.run();
}
