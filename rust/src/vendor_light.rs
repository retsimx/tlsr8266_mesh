use core::cell::RefCell;
use crate::sdk::light::{AdvPrivate};
use core::mem::size_of;
use core::ptr::addr_of;
use core::slice;
use crate::sdk::common::compat::array4_to_int;
use crate::sdk::rf_drv::rf_link_slave_set_adv_private_data;
use crate::sdk::ble_app::rf_drv_8266::get_mac_id;
use crate::state::State;

pub fn vendor_set_adv_data(state: &RefCell<State>) {
    // config adv data
    state.borrow_mut().adv_pri_data.mac_address = array4_to_int(get_mac_id());

    rf_link_slave_set_adv_private_data(
        unsafe {
            slice::from_raw_parts(
                addr_of!(state.borrow().adv_pri_data) as *const u8,
                size_of::<AdvPrivate>() as usize,
            )
        }
    );

    // Light mode CCT = 0x02
    state.borrow_mut().adv_rsp_pri_data.product_uuid = 0x02;

    // config adv rsp data
    state.borrow_mut().adv_rsp_pri_data.mac_address = array4_to_int(get_mac_id());

    // set rsv to 0..16
    state.borrow_mut().adv_rsp_pri_data
        .rsv
        .iter_mut()
        .enumerate()
        .for_each(|(i, x)| *x = i as u8);
}
