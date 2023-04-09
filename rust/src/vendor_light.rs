use crate::config::VENDOR_ID;
use crate::pub_mut_no_move;
use crate::sdk::light::{get_slave_p_mac, ll_adv_private_t, ll_adv_rsp_private_t};
use core::mem::size_of;
use core::ptr::addr_of;
use core::slice;
use crate::sdk::rf_drv::rf_link_slave_set_adv_private_data;

pub_mut_no_move!(
    adv_pri_data,
    ll_adv_private_t,
    ll_adv_private_t {
        ManufactureID: VENDOR_ID,
        MeshProductUUID: VENDOR_ID,
        MacAddress: 0
    }
);

pub_mut_no_move!(
    adv_rsp_pri_data,
    ll_adv_rsp_private_t,
    ll_adv_rsp_private_t {
        ManufactureID: VENDOR_ID,
        MeshProductUUID: VENDOR_ID,
        MacAddress: 0,
        ProductUUID: 0x1234,
        status: 0x01,
        DeviceAddress: 0,
        rsv: [0; 16]
    }
);

pub fn vendor_set_adv_data() {
    // config adv data
    get_adv_pri_data().MacAddress = unsafe { *(*get_slave_p_mac() as *const u32) };
    rf_link_slave_set_adv_private_data(
        unsafe {
            slice::from_raw_parts(
                get_adv_pri_data_addr() as *const u8,
                size_of::<ll_adv_private_t>() as usize,
            )
        }
    );

    // Light mode CCT = 0x02
    get_adv_rsp_pri_data().ProductUUID = 0x02;

    // config adv rsp data
    get_adv_rsp_pri_data().MacAddress = unsafe { *(*get_slave_p_mac() as *const u32) };

    // set rsv to 0..16
    get_adv_rsp_pri_data()
        .rsv
        .iter_mut()
        .enumerate()
        .for_each(|(i, x)| *x = i as u8);
}
