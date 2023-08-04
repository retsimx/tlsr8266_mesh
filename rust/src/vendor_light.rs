use core::mem::size_of;
use core::ptr::addr_of;
use core::slice;

use crate::sdk::common::compat::array4_to_int;
use crate::sdk::light::AdvPrivate;
use crate::sdk::rf_drv::rf_link_slave_set_adv_private_data;
use crate::state::{ADV_PRI_DATA, ADV_RSP_PRI_DATA, MAC_ID};

pub fn vendor_set_adv_data() {
    // config adv data
    let mut adv_pri_data = ADV_PRI_DATA.lock();

    adv_pri_data.mac_address = array4_to_int(&*MAC_ID.lock());

    let tmp = *adv_pri_data;
    rf_link_slave_set_adv_private_data(
        unsafe {
            slice::from_raw_parts(
                addr_of!(tmp) as *const u8,
                size_of::<AdvPrivate>(),
            )
        }
    );

    let mut adv_rsp_pri_data = ADV_RSP_PRI_DATA.lock();

    // Light mode CCT = 0x02
    adv_rsp_pri_data.product_uuid = 0x02;

    // config adv rsp data
    adv_rsp_pri_data.mac_address = array4_to_int(&*MAC_ID.lock());

    // set rsv to 0..16
    adv_rsp_pri_data
        .rsv
        .iter_mut()
        .enumerate()
        .for_each(|(i, x)| *x = i as u8);
}
