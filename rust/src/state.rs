use core::cell::RefCell;
use core::mem::size_of;
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicU8, AtomicUsize, Ordering};

use embassy_sync::blocking_mutex::CriticalSectionMutex;
use heapless::Deque;

use crate::config::VENDOR_ID;
use crate::mesh::{MESH_NODE_ST_PAR_LEN, mesh_node_st_t, mesh_node_st_val_t};
use crate::sdk::light::{AdvPrivate, AdvRspPrivate, BLT_FIFO_TX_PACKET_COUNT, BUFF_RESPONSE_PACKET_COUNT, CFG_SECTOR_ADR_CALIBRATION_CODE, IrqHandlerStatus, LIGHT_RX_BUFF_COUNT, LightRxBuff, MAX_GROUP_NUM, MESH_NODE_MASK_LEN, MESH_NODE_MAX_NUM, MeshPkt, OtaState, PacketAttCmd, PacketAttData, PacketAttErrRsp, PacketAttMtu, PacketAttReadRsp, PacketAttValue, PacketAttWrite, PacketAttWriteRsp, PacketCtrlUnknown, PacketFeatureRsp, PacketLlInit, PacketScanRsp, PacketVersionInd, PairState, PktBuf, RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS, RfPacketAdvIndModuleT, StatusRecord};

#[repr(align(4))]
pub struct State {
    pub blt_tx_fifo: [[u8; 48]; BLT_FIFO_TX_PACKET_COUNT],
    pub blt_tx_wptr: usize,
    pub conn_update_successed: bool,
    pub buff_response: [PacketAttData; BUFF_RESPONSE_PACKET_COUNT],
    pub conn_update_cnt: usize,
    pub set_uuid_flag: bool,
    pub max_mesh_name_len: usize,
    pub led_event_pending: u32,
    pub led_count: u32,
    pub led_ton: u32,
    pub led_toff: u32,
    pub led_sel: u32,
    pub led_tick: u32,
    pub led_no: u32,
    pub led_is_on: u32,

    pub get_mac_en: bool,
    pub mesh_pair_enable: bool,
    pub mesh_node_st: [mesh_node_st_t; MESH_NODE_MAX_NUM],

    pub adv_pri_data: AdvPrivate,
    pub adv_rsp_pri_data: AdvRspPrivate,

    pub ble_ll_channel_num: usize,
    pub ble_ll_last_unmapped_ch: usize,
    pub ble_ll_channel_table: [u8; 40],

    pub pkt_version_ind: PacketVersionInd,
    pub rf_pkt_unknown_response: PacketCtrlUnknown,
    pub pkt_feature_rsp: PacketFeatureRsp,
    pub pkt_mtu_rsp: PacketAttMtu,
    pub pkt_err_rsp: PacketAttErrRsp,
    pub rf_packet_att_rsp: PacketAttReadRsp,
    pub pkt_write_rsp: PacketAttWriteRsp,
    pub att_service_discover_tick: u32,
    pub slave_link_time_out: u32,

    pub slave_timing_update: u32,
    pub slave_instant_next: u16,
    pub slave_chn_map: [u8; 5],
    pub slave_interval_old: u32,
    pub ble_conn_timeout: u32,
    pub ble_conn_interval: u32,
    pub ble_conn_offset: u32,
    pub add_tx_packet_rsp_failed: u32,
    pub g_vendor_id: u16,
    pub p_st_handler: IrqHandlerStatus,
    pub pkt_light_report: PacketAttCmd,

    pub adr_reset_cnt_idx: u32,
    pub reset_cnt: u8,
    pub clear_st: u8,
    pub reset_check_time: u32,

    // These are filled at startup from values in config.rs
    pub pair_config_mesh_name: [u8; 16],
    pub pair_config_mesh_pwd: [u8; 16],
    pub pair_config_mesh_ltk: [u8; 16],

    pub not_need_login: bool,
    pub slave_first_connected_tick: u32,

    pub device_node_sn: u8,
    pub dev_grp_next_pos: u16,

    pub group_address: [u16; MAX_GROUP_NUM as usize],

    pub adr_flash_cfg_idx: i32,

    pub slave_read_status_busy: u8,

    pub pair_setting_flag: PairState,

    pub cur_ota_flash_addr: u32,
    pub rf_slave_ota_finished_flag: OtaState,
    pub rf_slave_ota_terminate_flag: bool,

    pub rf_slave_ota_timeout_s: u16,
    pub set_mesh_info_expired_flag: bool,
    pub set_mesh_info_time: u32,

    pub pair_ivm: [u8; 8],

    pub pair_config_pwd_encode_sk: [u8; 17],
    pub pair_ivs: [u8; 8],

    /////////////// adv par define ///////////////////////////////////////////////////
    pub adv_interval2listen_interval: u16,
    // unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);
    pub online_status_interval2listen_interval: u16,
    // unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);

    pub flash_sector_calibration: u32,
    pub slave_status_record: [StatusRecord; MESH_NODE_MAX_NUM],
    pub slave_status_record_size: u16,
    pub rc_pkt_buf: Deque<PktBuf, 5>,

    pub dev_address_next_pos: u16,
    pub need_update_connect_para: bool,
    pub update_interval_user_max: u16,
    pub update_interval_user_min: u16,
    pub update_timeout_user: u32,
    pub update_interval_flag: u16,
    pub update_interval_time: bool,
    pub slave_data_valid: u32,
    pub t_bridge_cmd: u32,
    pub st_brige_no: u32,
    pub app_cmd_time: u32,
    pub slave_status_buffer_wptr: usize,
    pub slave_status_buffer_rptr: usize,
    pub slave_stat_sno: [u8; 3],
    pub slave_read_status_unicast_flag: u8,
    pub slave_timing_adjust_enable: bool,
    pub slave_tick_brx: u32,
    pub slave_window_offset: u32,
    pub slave_instant: u16,
    pub slave_status_tick: u8,
    pub slave_link_cmd: u8,
    pub rcv_pkt_ttc: u8,
    pub org_ttl: u8,
    pub slave_read_status_response: bool,
    pub SLAVE_SNO: [u8; 3],
    pub slave_status_record_idx: usize,
    pub notify_req_mask_idx: u8,
    pub adv_flag: bool,
    pub online_st_flag: bool,
    pub slave_read_status_busy_time: u32,

    pub slave_listen_interval: u32,
    pub slave_adv_enable: bool,
    pub slave_connection_enable: bool,
    pub mac_id: [u8; 6],
    pub adv_data: [u8; 3],
    pub user_data_len: u8,
    pub user_data: [u8; 16],

    pub rf_tx_mode: u8,
    pub rfhw_tx_power: u8,

    pub pkt_adv: RfPacketAdvIndModuleT,
    pub pkt_scan_rsp: PacketScanRsp,
    pub pkt_light_data: PacketAttCmd,
    pub pkt_light_status: PacketAttCmd,
    pub pkt_read_rsp: PacketAttReadRsp,
    pub pkt_light_adv_status: PacketAttWrite,
    pub pkt_mesh_user_cmd_buf: MeshPkt,
    pub pkt_init: PacketLlInit,
}

pub static STATE: CriticalSectionMutex<RefCell<State>> = CriticalSectionMutex::new(RefCell::new(State {
    blt_tx_fifo: [[0; 48]; 8],
    blt_tx_wptr: 0,
    conn_update_successed: false,
    buff_response: [PacketAttData {
        dma_len: 0,
        _type: 0,
        rf_len: 0,
        l2cap: 0,
        chanid: 0,
        att: 0,
        hl: 0,
        hh: 0,
        dat: [0; 23],
    }; 48],
    conn_update_cnt: 0,
    set_uuid_flag: false,
    max_mesh_name_len: 16,
    led_event_pending: 0,
    led_count: 0,
    led_ton: 0,
    led_toff: 0,
    led_sel: 0,
    led_tick: 0,
    led_no: 0,
    led_is_on: 0,

    get_mac_en: false,
    mesh_pair_enable: false,
    mesh_node_st: [mesh_node_st_t {
        tick: 0,
        val: mesh_node_st_val_t {
            dev_adr: 0,
            sn: 0,
            par: [0; MESH_NODE_ST_PAR_LEN as usize],
        },
    }; MESH_NODE_MAX_NUM],

    adv_pri_data: AdvPrivate {
        manufacture_id: VENDOR_ID,
        mesh_product_uuid: VENDOR_ID,
        mac_address: 0,
    },
    adv_rsp_pri_data: AdvRspPrivate {
        manufacture_id: VENDOR_ID,
        mesh_product_uuid: VENDOR_ID,
        mac_address: 0,
        product_uuid: 0x1234,
        status: 0x01,
        device_address: 0,
        rsv: [0; 16],
    },

    ble_ll_channel_num: 0,
    ble_ll_last_unmapped_ch: 0,
    ble_ll_channel_table: [0; 40],

    pkt_version_ind: PacketVersionInd {
        dma_len: 8,
        _type: 3,
        rf_len: 6,
        opcode: 0x0c,
        main_ver: 0x08,
        vendor: VENDOR_ID,
        sub_ver: 0x08,
    },
    rf_pkt_unknown_response: PacketCtrlUnknown {
        dma_len: 0x04,
        _type: 0x03,
        rf_len: 0x02,
        opcode: 0x07,
        data: [0],
    },
    pkt_feature_rsp: PacketFeatureRsp {
        dma_len: 0x0b,
        _type: 0x3,
        rf_len: 0x09,
        opcode: 0x09,
        data: [1, 0, 0, 0, 0, 0, 0, 0],
    },
    pkt_mtu_rsp: PacketAttMtu {
        dma_len: 0x09,
        _type: 2,
        rf_len: 0x07,
        l2cap_len: 0x03,
        chan_id: 0x04,
        opcode: 0x03,
        mtu: [0x17, 0x00],
    },
    pkt_err_rsp: PacketAttErrRsp {
        dma_len: 0x0b,
        _type: 0x02,
        rf_len: 0x09,
        l2cap_len: 0x05,
        chan_id: 0x04,
        opcode: 0x01,
        err_opcode: 0,
        err_handle: 0,
        err_reason: 0x0a,
    },
    rf_packet_att_rsp: PacketAttReadRsp {
        dma_len: 0,
        _type: 0,
        rf_len: 0,
        l2cap_len: 0,
        chan_id: 0,
        opcode: 0,
        value: [0; 22],
    },
    pkt_write_rsp: PacketAttWriteRsp {
        dma_len: 0x07,
        _type: 2,
        rf_len: 0x05,
        l2cap_len: 0x01,
        chan_id: 0x04,
        opcode: 0x13,
    },
    att_service_discover_tick: 0,
    slave_link_time_out: 0,

    slave_timing_update: 0,
    slave_instant_next: 0,
    slave_chn_map: [0; 5],
    slave_interval_old: 0,
    ble_conn_timeout: 0,
    ble_conn_interval: 0,
    ble_conn_offset: 0,
    add_tx_packet_rsp_failed: 0,
    g_vendor_id: 0x211,
    p_st_handler: IrqHandlerStatus::None,
    pkt_light_report: PacketAttCmd {
        dma_len: 0x1D,
        _type: 2,
        rf_len: 0x1B,
        l2cap_len: 0x17,
        chan_id: 4,
        opcode: 0x1B,
        handle: 0x12,
        handle1: 0,
        value: PacketAttValue {
            sno: [0; 3],
            src: [0; 2],
            dst: [0; 2],
            val: [0xdc, 0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        },
    },

    adr_reset_cnt_idx: 0,
    reset_cnt: 0,
    clear_st: 3,
    reset_check_time: 0,

    pair_config_mesh_name: [0; 16],
    pair_config_mesh_pwd: [0; 16],
    pair_config_mesh_ltk: [0; 16],

    not_need_login: false,
    slave_first_connected_tick: 0,

    device_node_sn: 1,
    dev_grp_next_pos: 0,

    group_address: [0; MAX_GROUP_NUM as usize],

    adr_flash_cfg_idx: 0,

    slave_read_status_busy: 0,

    pair_setting_flag: PairState::PairSetted,

    cur_ota_flash_addr: 0,
    rf_slave_ota_finished_flag: OtaState::Continue,
    rf_slave_ota_terminate_flag: false,

    rf_slave_ota_timeout_s: RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS,
    set_mesh_info_expired_flag: false,
    set_mesh_info_time: 0,

    pair_ivm: [0, 0, 0, 0, 1, 0, 0, 0],

    pair_config_pwd_encode_sk: [0; 17],
    pair_ivs: [0; 8],

    adv_interval2listen_interval: 4,
    online_status_interval2listen_interval: 8,

    flash_sector_calibration: CFG_SECTOR_ADR_CALIBRATION_CODE,

    slave_status_record: [StatusRecord {
        adr: [0],
        alarm_id: 0,
    }; MESH_NODE_MAX_NUM],

    slave_status_record_size: size_of::<[StatusRecord; MESH_NODE_MAX_NUM]>() as u16,
    rc_pkt_buf: Deque::new(),

    dev_address_next_pos: 0,
    need_update_connect_para: false,
    update_interval_user_max: 0,
    update_interval_user_min: 0,
    update_timeout_user: 0,
    update_interval_flag: 0,
    update_interval_time: false,
    slave_data_valid: 0,
    t_bridge_cmd: 0,
    st_brige_no: 0,
    app_cmd_time: 0,
    slave_status_buffer_wptr: 0,
    slave_status_buffer_rptr: 0,
    slave_stat_sno: [0; 3],
    slave_read_status_unicast_flag: 0,
    slave_timing_adjust_enable: false,
    slave_tick_brx: 0,
    slave_window_offset: 0,
    slave_instant: 0,
    slave_status_tick: 0,
    slave_link_cmd: 0,
    rcv_pkt_ttc: 0,
    org_ttl: 0,
    slave_read_status_response: false,
    SLAVE_SNO: [0; 3],
    slave_status_record_idx: 0,
    notify_req_mask_idx: 0,
    adv_flag: true,
    online_st_flag: true,
    slave_read_status_busy_time: 0,

    slave_listen_interval: 0,
    slave_adv_enable: false,
    slave_connection_enable: false,
    mac_id: [0; 6],
    adv_data: [2, 1, 5],
    user_data_len: 0,
    user_data: [0; 16],

    rf_tx_mode: 0,
    rfhw_tx_power: 0x40,

    pkt_adv: RfPacketAdvIndModuleT {
        dma_len: 0x27,
        _type: 0,
        rf_len: 0x25,
        adv_a: [0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5],
        data: [0; 31],
    },
    pkt_scan_rsp: PacketScanRsp {
        dma_len: 0x27,
        _type: 0x4,
        rf_len: 0x25,
        adv_a: [0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5],
        data: [0; 31],
    },
    pkt_light_data: PacketAttCmd {
        dma_len: 0x27,
        _type: 2,
        rf_len: 0x25,
        l2cap_len: 0xCCDD,
        chan_id: 0,
        opcode: 0,
        handle: 0,
        handle1: 0,
        value: PacketAttValue {
            sno: [0; 3],
            src: [0; 2],
            dst: [0; 2],
            val: [0; 23],
        },
    },
    pkt_light_status: PacketAttCmd {
        dma_len: 0x27,
        _type: 2,
        rf_len: 0x25,
        l2cap_len: 0x21,
        chan_id: 0,
        opcode: 0,
        handle: 0,
        handle1: 0,
        value: PacketAttValue {
            sno: [0; 3],
            src: [0; 2],
            dst: [0; 2],
            val: [0; 23],
        },
    },
    pkt_read_rsp: PacketAttReadRsp {
        dma_len: 0x1d,
        _type: 2,
        rf_len: 0x1b,
        l2cap_len: 0x17,
        chan_id: 0x4,
        opcode: 0xb,
        value: [0; 22],
    },
    pkt_light_adv_status: PacketAttWrite {
        dma_len: 0x27,
        rtype: 2,
        rf_len: 0x25,
        l2cap_len: 0x21,
        chan_id: 0xffff,
        opcode: 0,
        handle: 0,
        handle1: 0,
        value: [0; 30],
    },
    pkt_mesh_user_cmd_buf: MeshPkt {
        dma_len: 0,
        _type: 0,
        rf_len: 0,
        l2cap_len: 0,
        chan_id: 0,
        src_tx: 0,
        handle1: 0,
        sno: [0; 3],
        src_adr: 0,
        dst_adr: 0,
        op: 0,
        vendor_id: 0,
        par: [0; 10],
        internal_par1: [0; 5],
        ttl: 0,
        internal_par2: [0; 4],
        no_use: [0; 4],
    },
    pkt_init: PacketLlInit {
        dma_len: 0x24,
        _type: 0x5,
        rf_len: 0x22,
        scan_a: [0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5],
        adv_a: [0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5],
        aa: [0xaa, 0x55, 0x55, 0xaa],
        crcinit: [0x55, 0x55, 0x55],
        wsize: 2,
        woffset: 0x1f,
        interval: 0x20,
        latency: 0,
        timeout: 0x48,
        chm: [0xff, 0xff, 0xff, 0xff, 0x1f],
        hop: 0xac,
    },
}));

pub static PAIR_LTK: CriticalSectionMutex<RefCell<[u8; 16]>> = CriticalSectionMutex::new(RefCell::new([0; 16]));
pub static PAIR_SK: CriticalSectionMutex<RefCell<[u8; 16]>> = CriticalSectionMutex::new(RefCell::new([0; 16]));
pub static PAIR_WORK: CriticalSectionMutex<RefCell<[u8; 16]>> = CriticalSectionMutex::new(RefCell::new([0; 16]));
pub static PAIR_NN: CriticalSectionMutex<RefCell<[u8; 16]>> = CriticalSectionMutex::new(RefCell::new([0; 16]));
pub static PAIR_PASS: CriticalSectionMutex<RefCell<[u8; 16]>> = CriticalSectionMutex::new(RefCell::new([0; 16]));
pub static PAIR_LTK_MESH: CriticalSectionMutex<RefCell<[u8; 16]>> = CriticalSectionMutex::new(RefCell::new([0; 16]));
pub static PAIR_SK_COPY: CriticalSectionMutex<RefCell<[u8; 16]>> = CriticalSectionMutex::new(RefCell::new([0; 16]));

pub static LIGHT_RX_BUFF: CriticalSectionMutex<RefCell<[LightRxBuff; LIGHT_RX_BUFF_COUNT]>> = CriticalSectionMutex::new(RefCell::new(
    [
        LightRxBuff {
            dma_len: 0,
            unk1: [0; 3],
            rssi: 0,
            unk2: [0; 3],
            rx_time: 0,
            sno: [0; 3],
            unk3: [0; 5],
            mac: [0; 4],
            unk4: [0; 40],
        }; LIGHT_RX_BUFF_COUNT
    ]
));

pub static MESH_NODE_MASK: CriticalSectionMutex<RefCell<[u32; MESH_NODE_MASK_LEN]>> = CriticalSectionMutex::new(RefCell::new([0; ((MESH_NODE_MAX_NUM + 31) >> 5)]));
pub static PAIR_RANDS: CriticalSectionMutex<RefCell<[u8; 8]>> = CriticalSectionMutex::new(RefCell::new([0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7]));
pub static PAIR_RANDM: CriticalSectionMutex<RefCell<[u8; 8]>> = CriticalSectionMutex::new(RefCell::new([0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7]));

pub static BLE_PAIR_ST: AtomicU8 = AtomicU8::new(0);
pub static PAIR_LOGIN_OK: AtomicBool = AtomicBool::new(false);
pub static PAIR_ENC_ENABLE: AtomicBool = AtomicBool::new(false);

pub static LIGHT_RX_WPTR: AtomicUsize = AtomicUsize::new(0);

pub static DEVICE_ADDRESS: AtomicU16 = AtomicU16::new(0);
pub static PAIR_AC: AtomicU32 = AtomicU32::new(0);
pub static PAIR_READ_PENDING: AtomicBool = AtomicBool::new(false);

pub static RF_TP_BASE: AtomicU32 = AtomicU32::new(0x1D);
pub static RF_TP_GAIN: AtomicU32 = AtomicU32::new(0xC);

pub static T_SCAN_RSP_INTVL: AtomicU32 = AtomicU32::new(0x92);

pub static SLAVE_LINK_STATE: AtomicU8 = AtomicU8::new(0);
pub static RCV_PKT_TIME: AtomicU32 = AtomicU32::new(0);

pub static SECURITY_ENABLE: AtomicBool = AtomicBool::new(false);

pub static ST_LISTEN_NO: AtomicU32 = AtomicU32::new(0);
pub static LIGHT_CONN_SN_MASTER: AtomicU16 = AtomicU16::new(0);

pub static SLAVE_CONNECTED_TICK: AtomicU32 = AtomicU32::new(0);
pub static SLAVE_LINK_CONNECTED: AtomicBool = AtomicBool::new(false);
pub static SLAVE_LINK_INTERVAL: AtomicU32 = AtomicU32::new(0x9c400);
pub static SLAVE_WINDOW_SIZE: AtomicU32 = AtomicU32::new(0);
pub static SLAVE_WINDOW_SIZE_UPDATE: AtomicU32 = AtomicU32::new(0);
pub static SLAVE_TIMING_UPDATE2_FLAG: AtomicBool = AtomicBool::new(false);
pub static SLAVE_TIMING_UPDATE2_OK_TIME: AtomicU32 = AtomicU32::new(0);
pub static SLAVE_NEXT_CONNECT_TICK: AtomicU32 = AtomicU32::new(0);
pub static RF_SLAVE_OTA_BUSY_MESH: AtomicBool = AtomicBool::new(false);
pub static RF_SLAVE_OTA_BUSY: AtomicBool = AtomicBool::new(false);

pub static MESH_NODE_MAX: AtomicU8 = AtomicU8::new(0);
pub static MESH_NODE_REPORT_ENABLE: AtomicBool = AtomicBool::new(false);

pub trait SimplifyLS<T> {
    fn get(&self) -> T;
    fn set(&self, val: T);
}

impl SimplifyLS<bool> for AtomicBool {
    fn get(&self) -> bool {
        self.load(Ordering::Relaxed)
    }

    fn set(&self, val: bool) {
        self.store(val, Ordering::Relaxed);
    }
}

impl SimplifyLS<u8> for AtomicU8 {
    fn get(&self) -> u8 {
        self.load(Ordering::Relaxed)
    }

    fn set(&self, val: u8) {
        self.store(val, Ordering::Relaxed);
    }
}

impl SimplifyLS<u16> for AtomicU16 {
    fn get(&self) -> u16 {
        self.load(Ordering::Relaxed)
    }

    fn set(&self, val: u16) {
        self.store(val, Ordering::Relaxed);
    }
}

impl SimplifyLS<u32> for AtomicU32 {
    fn get(&self) -> u32 {
        self.load(Ordering::Relaxed)
    }

    fn set(&self, val: u32) {
        self.store(val, Ordering::Relaxed);
    }
}

impl SimplifyLS<usize> for AtomicUsize {
    fn get(&self) -> usize {
        self.load(Ordering::Relaxed)
    }

    fn set(&self, val: usize) {
        self.store(val, Ordering::Relaxed);
    }
}