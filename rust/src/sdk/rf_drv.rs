extern "C" {
    pub fn rf_set_power_level_index (level: u32);
    pub fn rf_link_slave_pairing_enable(en: u32);
    pub fn rf_link_slave_set_buffer(p: *mut [u32; 9], n: u8);
    pub fn rf_link_set_max_bridge(num: u32);
    pub fn rf_link_slave_init(interval: u32);
}

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