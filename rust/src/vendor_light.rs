use crate::config::VENDOR_ID;
use crate::{pub_mut};
use crate::sdk::light::{get_slave_p_mac, AdvPrivate, AdvRspPrivate};
use core::mem::size_of;
use core::slice;
use crate::sdk::rf_drv::rf_link_slave_set_adv_private_data;

pub_mut!(
    adv_pri_data,
    AdvPrivate,
    AdvPrivate {
        manufacture_id: VENDOR_ID,
        mesh_product_uuid: VENDOR_ID,
        mac_address: 0
    }
);

pub_mut!(
    adv_rsp_pri_data,
    AdvRspPrivate,
    AdvRspPrivate {
        manufacture_id: VENDOR_ID,
        mesh_product_uuid: VENDOR_ID,
        mac_address: 0,
        product_uuid: 0x1234,
        status: 0x01,
        device_address: 0,
        rsv: [0; 16]
    }
);

pub fn vendor_set_adv_data() {
    // config adv data
    get_adv_pri_data().mac_address = unsafe { *(*get_slave_p_mac() as *const u32) };
    rf_link_slave_set_adv_private_data(
        unsafe {
            slice::from_raw_parts(
                get_adv_pri_data_addr() as *const u8,
                size_of::<AdvPrivate>() as usize,
            )
        }
    );

    // Light mode CCT = 0x02
    get_adv_rsp_pri_data().product_uuid = 0x02;

    // config adv rsp data
    get_adv_rsp_pri_data().mac_address = unsafe { *(*get_slave_p_mac() as *const u32) };

    // set rsv to 0..16
    get_adv_rsp_pri_data()
        .rsv
        .iter_mut()
        .enumerate()
        .for_each(|(i, x)| *x = i as u8);
}
