use no_mangle_fn;
use sdk::light::ll_packet_l2cap_data_t;
use sdk::mcu::register::write_reg32;

no_mangle_fn!(rf_set_power_level_index, level: u32);
no_mangle_fn!(rf_link_slave_pairing_enable, en: u32);
no_mangle_fn!(rf_link_slave_set_buffer, p: *mut [u32; 9], n: u8);
no_mangle_fn!(rf_link_set_max_bridge, num: u32);
no_mangle_fn!(rf_link_slave_init, interval: u32);
no_mangle_fn!(rf_link_get_op_para, u32, p: *const ll_packet_l2cap_data_t, op: *mut u8, op_len: &u8, para: *mut u8, para_len: &u8, mesh_flag: u8);
no_mangle_fn!(rf_link_del_group, bool, group: u16);
no_mangle_fn!(rf_link_add_group, bool, group: u16);
no_mangle_fn!(rf_link_add_dev_addr, bool, deviceaddress: u16);
no_mangle_fn!(rf_link_slave_set_adv_private_data, pdata: *const u8, data_len: u8);

pub enum RF_POWER {
    RF_POWER_8dBm	= 0,
    RF_POWER_4dBm	= 1,
    RF_POWER_0dBm	= 2,
    RF_POWER_m4dBm	= 3,
    RF_POWER_m10dBm	= 4,
    RF_POWER_m14dBm	= 5,
    RF_POWER_m20dBm	= 6,
    RF_POWER_m24dBm	= 8,
    RF_POWER_m28dBm	= 9,
    RF_POWER_m30dBm	= 10,
    RF_POWER_m37dBm	= 11,
    RF_POWER_OFF	= 16,
}

pub unsafe fn rf_set_ble_access_code(p: *const u8)
{
	write_reg32 (0x800408, *p.offset(3) as u32 | (*p.offset(2) as u32) << 8 | (*p.offset(1) as u32) << 16 | (*p.offset(0) as u32) << 24);
}