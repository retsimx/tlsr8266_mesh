use core::cell::RefCell;
use embassy_sync::blocking_mutex::{CriticalSectionMutex};
use crate::config::VENDOR_ID;
use crate::mesh::{MESH_NODE_ST_PAR_LEN, mesh_node_st_t, mesh_node_st_val_t};
use crate::sdk::light::{AdvPrivate, AdvRspPrivate, BLT_FIFO_TX_PACKET_COUNT, BUFF_RESPONSE_PACKET_COUNT, LightRxBuff, MESH_NODE_MASK_LEN, MESH_NODE_MAX_NUM, MeshPkt, PacketAttData};

#[repr(align(4))]
pub struct State {
    pub light_rx_wptr: usize,
    pub light_rx_buff: [LightRxBuff; 4],
    pub blt_tx_fifo: [[u8; 48]; BLT_FIFO_TX_PACKET_COUNT],
    pub blt_tx_wptr: usize,
    pub conn_update_successed: bool,
    pub buff_response: [PacketAttData; BUFF_RESPONSE_PACKET_COUNT],
    pub pair_rands: [u8; 8],
    pub pair_randm: [u8; 8],
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

    pub mesh_node_mask: [u32; MESH_NODE_MASK_LEN],
    pub get_mac_en: bool,
    pub mesh_pair_enable: bool,
    pub mesh_node_st: [mesh_node_st_t; MESH_NODE_MAX_NUM],
    pub pkt_user_cmd: MeshPkt,

    pub adv_pri_data: AdvPrivate,
    pub adv_rsp_pri_data: AdvRspPrivate
}

pub static STATE: CriticalSectionMutex<RefCell<State>> = CriticalSectionMutex::new(RefCell::new(State {
    light_rx_wptr: 0,
    light_rx_buff: [LightRxBuff{
        dma_len: 0,
        unk1: [0; 3],
        rssi: 0,
        unk2: [0; 3],
        rx_time: 0,
        sno: [0; 3],
        unk3: [0; 5],
        mac: [0; 4],
        unk4: [0; 40]
    }; 4],
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
        dat: [0; 23]
    }; 48],
    pair_rands: [0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7],
    pair_randm: [0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7],
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

    mesh_node_mask: [0; ((MESH_NODE_MAX_NUM + 31) >> 5) as usize],
    get_mac_en: false,
    mesh_pair_enable: false,
    mesh_node_st: [mesh_node_st_t {
        tick: 0,
        val: mesh_node_st_val_t {
            dev_adr: 0,
            sn: 0,
            par: [0; MESH_NODE_ST_PAR_LEN as usize],
        }
    }; MESH_NODE_MAX_NUM],

    pkt_user_cmd: MeshPkt {
        dma_len: 0x27,
        _type: 2,
        rf_len: 0x25,
        l2cap_len: 0xCCDD,
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
        no_use: [0; 4]
    },

    adv_pri_data: AdvPrivate {
        manufacture_id: VENDOR_ID,
        mesh_product_uuid: VENDOR_ID,
        mac_address: 0
    },
    adv_rsp_pri_data: AdvRspPrivate {
        manufacture_id: VENDOR_ID,
        mesh_product_uuid: VENDOR_ID,
        mac_address: 0,
        product_uuid: 0x1234,
        status: 0x01,
        device_address: 0,
        rsv: [0; 16]
    }
}));