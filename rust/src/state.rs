use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU16, AtomicU32, AtomicU8, AtomicUsize, Ordering};

use heapless::Deque;

use crate::config::VENDOR_ID;
use crate::embassy::sync::mutex::CriticalSectionMutex;
use crate::mesh::{MESH_NODE_ST_PAR_LEN, mesh_node_st_t, mesh_node_st_val_t};
use crate::sdk::light::{*};
use crate::sdk::packet_types::{*};

pub static BUFF_RESPONSE: CriticalSectionMutex<[Packet; BUFF_RESPONSE_PACKET_COUNT]> = CriticalSectionMutex::new(
    [
        Packet {
            att_data: PacketAttData {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 0,
                    rf_len: 0,
                    l2cap_len: 0,
                    chan_id: 0,
                },
                att: 0,
                hl: 0,
                hh: 0,
                dat: [0; 23],
            }
        };
        BUFF_RESPONSE_PACKET_COUNT
    ]
);

pub static MESH_NODE_ST: CriticalSectionMutex<[mesh_node_st_t; MESH_NODE_MAX_NUM]> = CriticalSectionMutex::new(
    [
        mesh_node_st_t {
            tick: 0,
            val: mesh_node_st_val_t {
                dev_adr: 0,
                sn: 0,
                par: [0; MESH_NODE_ST_PAR_LEN],
            },
        };
        MESH_NODE_MAX_NUM
    ]
);

pub static ADV_PRI_DATA: CriticalSectionMutex<AdvPrivate> = CriticalSectionMutex::new(
    AdvPrivate {
        manufacture_id: VENDOR_ID,
        mesh_product_uuid: VENDOR_ID,
        mac_address: 0,
    }
);

pub static ADV_RSP_PRI_DATA: CriticalSectionMutex<AdvRspPrivate> = CriticalSectionMutex::new(
    AdvRspPrivate {
        manufacture_id: VENDOR_ID,
        mesh_product_uuid: VENDOR_ID,
        mac_address: 0,
        product_uuid: 0x1234,
        status: 0x01,
        device_address: 0,
        rsv: [0; 16],
    }
);

pub static BLE_LL_CHANNEL_TABLE: CriticalSectionMutex<[u8; 40]> = CriticalSectionMutex::new([0; 40]);

pub static SLAVE_CHN_MAP: CriticalSectionMutex<[u8; 5]> = CriticalSectionMutex::new([0; 5]);
pub static P_ST_HANDLER: CriticalSectionMutex<IrqHandlerStatus> = CriticalSectionMutex::new(IrqHandlerStatus::None);

pub static PAIR_CONFIG_MESH_NAME: CriticalSectionMutex<[u8; 16]> = CriticalSectionMutex::new([0; 16]);
pub static PAIR_CONFIG_MESH_PWD: CriticalSectionMutex<[u8; 16]> = CriticalSectionMutex::new([0; 16]);
pub static PAIR_CONFIG_MESH_LTK: CriticalSectionMutex<[u8; 16]> = CriticalSectionMutex::new([0; 16]);
pub static GROUP_ADDRESS: CriticalSectionMutex<[u16; MAX_GROUP_NUM as usize]> = CriticalSectionMutex::new([0; MAX_GROUP_NUM as usize]);
pub static PAIR_SETTING_FLAG: CriticalSectionMutex<ePairState> = CriticalSectionMutex::new(ePairState::PairSetted);
pub static RF_SLAVE_OTA_FINISHED_FLAG: CriticalSectionMutex<OtaState> = CriticalSectionMutex::new(OtaState::Continue);
pub static PAIR_IVM: CriticalSectionMutex<[u8; 8]> = CriticalSectionMutex::new([0, 0, 0, 0, 1, 0, 0, 0]);
pub static PAIR_CONFIG_PWD_ENCODE_SK: CriticalSectionMutex<[u8; 17]> = CriticalSectionMutex::new([0; 17]);
pub static PAIR_IVS: CriticalSectionMutex<[u8; 8]> = CriticalSectionMutex::new([0; 8]);
pub static SLAVE_STATUS_RECORD: CriticalSectionMutex<[StatusRecord; MESH_NODE_MAX_NUM]> = CriticalSectionMutex::new(
    [
        StatusRecord {
            adr: [0],
            alarm_id: 0,
        };
        MESH_NODE_MAX_NUM
    ]
);
pub static RC_PKT_BUF: CriticalSectionMutex<Deque<PktBuf, 5>> = CriticalSectionMutex::new(Deque::new());
pub static SLAVE_STAT_SNO: CriticalSectionMutex<[u8; 3]> = CriticalSectionMutex::new([0; 3]);
pub static SLAVE_SNO: CriticalSectionMutex<[u8; 3]> = CriticalSectionMutex::new([0; 3]);
pub static MAC_ID: CriticalSectionMutex<[u8; 6]> = CriticalSectionMutex::new([0; 6]);
pub static ADV_DATA: CriticalSectionMutex<[u8; 3]> = CriticalSectionMutex::new([2, 1, 5]);
pub static PKT_ADV: CriticalSectionMutex<Packet> = CriticalSectionMutex::new(
    Packet {
        adv_ind_module: RfPacketAdvIndModuleT {
            dma_len: 0x27,
            _type: 0,
            rf_len: 0x25,
            adv_a: [0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5],
            data: [0; 31],
        }
    }
);

pub static PKT_LIGHT_DATA: CriticalSectionMutex<Packet> = CriticalSectionMutex::new(
    Packet {
        att_cmd: PacketAttCmd {
            head: PacketL2capHead {
                dma_len: 0x27,
                _type: 2,
                rf_len: 0x25,
                l2cap_len: 0xCCDD,
                chan_id: 0,
            },
            opcode: 0,
            handle: 0,
            handle1: 0,
            value: PacketAttValue {
                sno: [0; 3],
                src: [0; 2],
                dst: [0; 2],
                val: [0; 23],
            },
        }
    }
);
pub static PKT_LIGHT_STATUS: CriticalSectionMutex<Packet> = CriticalSectionMutex::new(
    Packet {
        att_cmd: PacketAttCmd {
            head: PacketL2capHead {
                dma_len: 0x27,
                _type: 2,
                rf_len: 0x25,
                l2cap_len: 0x21,
                chan_id: 0,
            },
            opcode: 0,
            handle: 0,
            handle1: 0,
            value: PacketAttValue {
                sno: [0; 3],
                src: [0; 2],
                dst: [0; 2],
                val: [0; 23],
            },
        }
    }
);

pub static PKT_INIT: CriticalSectionMutex<Packet> = CriticalSectionMutex::new(
    Packet {
        ll_init: PacketLlInit {
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
        }
    }
);

pub struct PairState {
    pub pair_ltk: [u8; 16],
    pub pair_sk: [u8; 16],
    pub pair_work: [u8; 16],
    pub pair_nn: [u8; 16],
    pub pair_pass: [u8; 16],
    pub pair_ltk_mesh: [u8; 16],
    pub pair_sk_copy: [u8; 16],
    pub pair_rands: [u8; 8],
    pub pair_randm: [u8; 8],
}

pub static PAIR_STATE: CriticalSectionMutex<PairState> = CriticalSectionMutex::new(
    PairState {
        pair_ltk: [0; 16],
        pair_sk: [0; 16],
        pair_work: [0; 16],
        pair_nn: [0; 16],
        pair_pass: [0; 16],
        pair_ltk_mesh: [0; 16],
        pair_sk_copy: [0; 16],
        pair_rands: [0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7],
        pair_randm: [0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7],
    }
);

pub static LIGHT_RX_BUFF: CriticalSectionMutex<[LightRxBuff; LIGHT_RX_BUFF_COUNT]> = CriticalSectionMutex::new(
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
);

// This needs to be forced in to .data otherwise the DMA module will try to fetch from flash, which
// doesn't work
#[link_section = ".data"]
pub static PKT_EMPTY: Packet = Packet {
    head: PacketL2capHead {
        dma_len: 2,
        _type: 1,
        rf_len: 0,
        l2cap_len: 0,
        chan_id: 0,
    }
};

#[link_section = ".data"]
pub static PKT_TERMINATE: Packet = Packet {
    ctrl_unknown: PacketCtrlUnknown {
        dma_len: 4,
        _type: 3,
        rf_len: 2,
        opcode: 2,
        data: [0x13],
    }
};

#[link_section = ".data"]
pub static PKT_ERR_RSP: Packet = Packet {
    att_err_rsp: PacketAttErrRsp {
        head: PacketL2capHead {
            dma_len: 0x0b,
            _type: 0x02,
            rf_len: 0x09,
            l2cap_len: 0x05,
            chan_id: 0x04,
        },
        opcode: 0x01,
        err_opcode: 0,
        err_handle: 0,
        err_reason: 0x0a,
    }
};

pub static MESH_PAIR_ENABLE: AtomicBool = AtomicBool::new(false);

pub static MESH_NODE_MASK: CriticalSectionMutex<[u32; MESH_NODE_MASK_LEN]> = CriticalSectionMutex::new([0; MESH_NODE_MASK_LEN]);
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

pub static CONN_UPDATE_SUCCESSED: AtomicBool = AtomicBool::new(false);
pub static CONN_UPDATE_CNT: AtomicUsize = AtomicUsize::new(0);
pub static SET_UUID_FLAG: AtomicBool = AtomicBool::new(false);
pub static MAX_MESH_NAME_LEN: AtomicUsize = AtomicUsize::new(16);
pub static LED_EVENT_PENDING: AtomicU32 = AtomicU32::new(0);
pub static LED_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LED_TON: AtomicU32 = AtomicU32::new(0);
pub static LED_TOFF: AtomicU32 = AtomicU32::new(0);
pub static LED_SEL: AtomicU32 = AtomicU32::new(0);
pub static LED_TICK: AtomicU32 = AtomicU32::new(0);
pub static LED_NO: AtomicU32 = AtomicU32::new(0);
pub static LED_IS_ON: AtomicU32 = AtomicU32::new(0);
pub static GET_MAC_EN: AtomicBool = AtomicBool::new(false);
pub static BLE_LL_CHANNEL_NUM: AtomicUsize = AtomicUsize::new(0);
pub static BLE_LL_LAST_UNMAPPED_CH: AtomicUsize = AtomicUsize::new(0);
pub static ATT_SERVICE_DISCOVER_TICK: AtomicU32 = AtomicU32::new(0);
pub static SLAVE_LINK_TIME_OUT: AtomicU32 = AtomicU32::new(0);
pub static SLAVE_TIMING_UPDATE: AtomicU32 = AtomicU32::new(0);
pub static SLAVE_INSTANT_NEXT: AtomicU16 = AtomicU16::new(0);
pub static SLAVE_INTERVAL_OLD: AtomicU32 = AtomicU32::new(0);
pub static BLE_CONN_TIMEOUT: AtomicU32 = AtomicU32::new(0);
pub static BLE_CONN_INTERVAL: AtomicU32 = AtomicU32::new(0);
pub static BLE_CONN_OFFSET: AtomicU32 = AtomicU32::new(0);
pub static ADR_RESET_CNT_IDX: AtomicU32 = AtomicU32::new(0);
pub static RESET_CNT: AtomicU8 = AtomicU8::new(0);
pub static CLEAR_ST: AtomicU8 = AtomicU8::new(3);
pub static RESET_CHECK_TIME: AtomicU32 = AtomicU32::new(0);
pub static SLAVE_FIRST_CONNECTED_TICK: AtomicU32 = AtomicU32::new(0);
pub static DEVICE_NODE_SN: AtomicU8 = AtomicU8::new(1);
pub static DEV_GRP_NEXT_POS: AtomicU16 = AtomicU16::new(0);
pub static ADR_FLASH_CFG_IDX: AtomicI32 = AtomicI32::new(0);
pub static SLAVE_READ_STATUS_BUSY: AtomicU8 = AtomicU8::new(0);
pub static CUR_OTA_FLASH_ADDR: AtomicU32 = AtomicU32::new(0);
pub static RF_SLAVE_OTA_TERMINATE_FLAG: AtomicBool = AtomicBool::new(false);
pub static RF_SLAVE_OTA_TIMEOUT_S: AtomicU16 = AtomicU16::new(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);
pub static DEV_ADDRESS_NEXT_POS: AtomicU16 = AtomicU16::new(0);
pub static NEED_UPDATE_CONNECT_PARA: AtomicBool = AtomicBool::new(false);
pub static UPDATE_INTERVAL_USER_MAX: AtomicU16 = AtomicU16::new(0);
pub static UPDATE_INTERVAL_USER_MIN: AtomicU16 = AtomicU16::new(0);
pub static SLAVE_DATA_VALID: AtomicU32 = AtomicU32::new(0);
pub static T_BRIDGE_CMD: AtomicU32 = AtomicU32::new(0);
pub static ST_BRIGE_NO: AtomicU32 = AtomicU32::new(0);
pub static APP_CMD_TIME: AtomicU32 = AtomicU32::new(0);
pub static SLAVE_STATUS_BUFFER_WPTR: AtomicUsize = AtomicUsize::new(0);
pub static SLAVE_STATUS_BUFFER_RPTR: AtomicUsize = AtomicUsize::new(0);
pub static SLAVE_READ_STATUS_UNICAST_FLAG: AtomicU8 = AtomicU8::new(0);
pub static SLAVE_TIMING_ADJUST_ENABLE: AtomicBool = AtomicBool::new(false);
pub static SLAVE_TICK_BRX: AtomicU32 = AtomicU32::new(0);
pub static SLAVE_WINDOW_OFFSET: AtomicU32 = AtomicU32::new(0);
pub static SLAVE_INSTANT: AtomicU16 = AtomicU16::new(0);
pub static SLAVE_STATUS_TICK: AtomicU8 = AtomicU8::new(0);
pub static SLAVE_LINK_CMD: AtomicU8 = AtomicU8::new(0);
pub static RCV_PKT_TTC: AtomicU8 = AtomicU8::new(0);
pub static ORG_TTL: AtomicU8 = AtomicU8::new(0);
pub static SLAVE_STATUS_RECORD_IDX: AtomicUsize = AtomicUsize::new(0);
pub static NOTIFY_REQ_MASK_IDX: AtomicU8 = AtomicU8::new(0);
pub static ADV_FLAG: AtomicBool = AtomicBool::new(true);
pub static ONLINE_ST_FLAG: AtomicBool = AtomicBool::new(true);
pub static SLAVE_READ_STATUS_BUSY_TIME: AtomicU32 = AtomicU32::new(0);
pub static SLAVE_LISTEN_INTERVAL: AtomicU32 = AtomicU32::new(0);
pub static SLAVE_ADV_ENABLE: AtomicBool = AtomicBool::new(false);
pub static SLAVE_CONNECTION_ENABLE: AtomicBool = AtomicBool::new(false);

pub trait SimplifyLS<T> {
    fn get(&self) -> T;
    fn set(&self, val: T);
    fn inc(&self);
    fn dec(&self);
}

impl SimplifyLS<bool> for AtomicBool {
    fn get(&self) -> bool {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: bool) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) { panic!("Not valid for bool"); }
    fn dec(&self) { panic!("Not valid for bool") }
}

impl SimplifyLS<u8> for AtomicU8 {
    fn get(&self) -> u8 {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: u8) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) { self.set(self.get() + 1); }
    fn dec(&self) { self.set(self.get() - 1); }
}

impl SimplifyLS<u16> for AtomicU16 {
    fn get(&self) -> u16 {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: u16) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) { self.set(self.get() + 1); }
    fn dec(&self) { self.set(self.get() - 1); }
}

impl SimplifyLS<u32> for AtomicU32 {
    fn get(&self) -> u32 {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: u32) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) { self.set(self.get() + 1); }
    fn dec(&self) { self.set(self.get() - 1); }
}

impl SimplifyLS<usize> for AtomicUsize {
    fn get(&self) -> usize {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: usize) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) { self.set(self.get() + 1); }
    fn dec(&self) { self.set(self.get() - 1); }
}

impl SimplifyLS<i32> for AtomicI32 {
    fn get(&self) -> i32 {
        self.load(Ordering::Relaxed)
    }
    fn set(&self, val: i32) {
        self.store(val, Ordering::Relaxed);
    }
    fn inc(&self) { self.set(self.get() + 1); }
    fn dec(&self) { self.set(self.get() - 1); }
}