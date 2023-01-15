use app::App;
use ota::OtaManager;

mod app;
mod common;
mod config;
mod main_light;
mod mesh;
mod ota;
mod sdk;
mod vendor_light;

static mut APP: App = App::default();

pub fn app() -> &'static mut App {
    return unsafe { &mut APP };
}

#[no_mangle]
pub fn main_entrypoint() {
    // Must happen first
    OtaManager::handle_ota_update__attribute_ram_code();

    // Configure the system
    app().init();

    // Run the application
    app().run();
}
