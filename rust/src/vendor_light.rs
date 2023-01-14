use std::mem::size_of;
use sdk::light::{get_slave_p_mac, ll_adv_private_t, ll_adv_rsp_private_t};
use std::ptr::addr_of;
use config::VENDOR_ID;
use pub_mut_no_move;
use sdk::rf_drv::_rf_link_slave_set_adv_private_data;

pub_mut_no_move!(adv_pri_data, ll_adv_private_t, ll_adv_private_t {
    ManufactureID: VENDOR_ID,
    MeshProductUUID: VENDOR_ID,
    MacAddress: 0
});

pub_mut_no_move!(adv_rsp_pri_data, ll_adv_rsp_private_t, ll_adv_rsp_private_t {
    ManufactureID: VENDOR_ID,
    MeshProductUUID: VENDOR_ID,
    MacAddress: 0,
    ProductUUID: 0x1234,
    status: 0x01,
    DeviceAddress: [0; 2],
    rsv: [0; 16]
});

pub unsafe fn vendor_set_adv_data() {
    // config adv data
    adv_pri_data.MacAddress = *(*get_slave_p_mac() as *const u32);
    _rf_link_slave_set_adv_private_data(addr_of!(adv_pri_data) as *const u8, size_of::<ll_adv_private_t>() as u8);

    // Light mode CCT = 0x02
    adv_rsp_pri_data.ProductUUID = 0x02;

    // config adv rsp data
    adv_rsp_pri_data.MacAddress = *(*get_slave_p_mac() as *const u32);
    for i in 0..16
    {
        adv_rsp_pri_data.rsv[i] = i as u8;
    }
}