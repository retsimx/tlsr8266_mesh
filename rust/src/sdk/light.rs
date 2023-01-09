// Stuff from the libble library

extern "C" {
    pub fn light_set_tick_per_us(tick: u32);

    pub fn mesh_security_enable(en: bool);
    pub fn mesh_get_fw_version();

    pub fn register_mesh_ota_master_ui(p: fn(*const u8));

    pub fn setSppUUID(p_service_uuid: *const u8, p_data_s2c_uuid: *const u8, p_data_c2s_uuid: *const u8, p_data_ota_uuid: *const u8, p_data_pair_uuid: *const u8);

    pub fn vendor_id_init(vendor_id: u16);

    pub fn is_receive_ota_window() -> bool;
    pub fn rf_link_slave_proc();

    pub fn light_sw_reboot();

    pub fn encode_password(pd: *mut u8);
    pub fn decode_password(pd: *mut u8);

    pub static mut user_data: [u8; 16];
    pub static mut user_data_len: u8;
    pub static mut pair_config_valid_flag: u8;

    pub static mut pair_config_mesh_name: [u8; 17];
    pub static mut pair_config_mesh_pwd: [u8; 17];
    pub static mut pair_config_mesh_ltk: [u8; 17];

    pub static mut p_adv_pri_data: *const ll_adv_private_t;
    pub static mut p_adv_rsp_data: *const ll_adv_rsp_private_t;
    pub static mut adv_private_data_len: u8;

    pub static mut online_status_timeout: u32;

    pub static mut security_enable: u8;
    pub static mut pair_login_ok: u8;
    pub static mut slave_first_connected_tick: u32;
}

pub static PMW_MAX_TICK_BASE	 : u16 =           255;   // must 255
pub static PMW_MAX_TICK_MULTI	 : u16 =           209;   // 209: freq = 600.4Hz
pub static PMW_MAX_TICK_1		 : u16 =           PMW_MAX_TICK_BASE*PMW_MAX_TICK_MULTI;  // must less or equal than (255*256)
pub static PMW_MAX_TICK		     : u16 =           PMW_MAX_TICK_1;

pub static BRIDGE_MAX_CNT: u32 = 0;
pub static IRQ_TIMER1_ENABLE: bool = true;
pub static IRQ_TIME1_INTERVAL: u8 = 10;

pub static ONLINE_STATUS_TIMEOUT: u32 = 3000;

pub static AUTH_TIME: u32 = 60;

#[repr(C, packed)]
pub struct ll_adv_private_t {
    ManufactureID: u16,  // must vendor id to follow spec
    MeshProductUUID: u16,
    MacAddress: u32// low 4 byte
}

#[repr(C, packed)]
pub struct ll_adv_rsp_private_t{
    ManufactureID: u16,  // must vendor id to follow spec
    MeshProductUUID: u16,
    MacAddress: u32,// low 4 byte
    ProductUUID: u16,
    status: u8,
    DeviceAddress: [u8; 2],
    rsv: [u8; 16]
}