use core::mem;
use crate::const_assert;
use crate::sdk::light::{AdvRspPrivate, PKT_CMD_LEN};

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct RfPacketAdvIndModuleT {
    pub dma_len: u32,       // 0    //won't be a fixed number as previous, should adjust with the mouse package number

    pub _type: u8,			// 4	//RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,			// 5	//LEN(6)_RFU(2)
    pub adv_a: [u8; 6],		// 6	//adv address
    pub data: [u8; 31]		// 12	//0-31 byte
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct PacketAttValue {
    pub sno: [u8; 3],
    pub src: [u8; 2],
    pub dst: [u8; 2],
    pub val: [u8; 23], // op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
    // get status req: params[0]=tick  mac-app[2-3]=src-mac1...
    // get status rsp: mac-app[0]=ttc  mac-app[1]=hop-count
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct PacketL2capHead {
    pub dma_len: u32,   // 0         //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,		// 4		//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
    pub rf_len: u8,		// 5		//LEN(5)_RFU(3)
    pub l2cap_len: u16,  // 6        // 0x17
    pub chan_id: u16,	// 8			//0x04,
}

#[derive(Clone, Copy)]
#[repr(C, align(4))]
pub struct PacketL2capData {
    pub l2cap_len: u16,
    pub chan_id: u16,
    pub opcode: u8,
    pub handle: u8,
    pub handle1: u8,
    pub value: [u8; 30],
}

#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct PacketLlData {
    pub head: PacketL2capHead, // 0
    pub att: u8,		// 10		//0x12 for master; 0x1b for slave// as ttl when relay
    pub hl: u8,			// 11		// assigned by master
    pub hh: u8,			// 12		//
    pub sno: u8,        // 13
    pub nid: u8,        // 14
    pub ttc: u8,        // 15
    pub group: u16,     // 16
    pub sid: [u16; 2],
    pub cmd: [u8; PKT_CMD_LEN]
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct AppCmdValue {
    pub sno: [u8; 3],    // 0    13
    pub src: u16,    // 3    16
    pub dst: u16,    // 5    18
    pub op: u8,          // 7    20
    pub vendor_id: u16,  // 8    21
    pub par: [u8; 10],   // 10   23
}

#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct PacketLlApp {
    pub head: PacketL2capHead, // 0
    pub opcode: u8,     // 10
    pub handle: u8,     // 11
    pub handle1: u8,    // 12
    pub value: AppCmdValue, // 13
    pub rsv: [u8; 10],
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PacketAttReadRsp {
    pub head: PacketL2capHead, // 0
    pub opcode: u8,     // 10
    pub value: [u8; 22] // 11
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PacketLlInit {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,				//RA(1)_TA(1)_RFU(2)_TYPE(4): connect request PDU
    pub rf_len: u8,				//LEN(6)_RFU(2)
    pub scan_a: [u8; 6],		// scan address
    pub adv_a: [u8; 6],			// adv address
    pub aa: [u8; 4],			// access code
    pub crcinit: [u8; 3],
    pub wsize: u8,
    pub woffset: u16,
    pub interval: u16,
    pub latency: u16,
    pub timeout: u16,
    pub chm: [u8; 5],
    pub hop: u8,				//sca(3)_hop(5)
}

#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct PktL2capSigConnParaUpRsp {
    pub head: PacketL2capHead, // 0
    pub code: u8,
    pub id: u8,
    pub data_len: u16,
    pub result: u16,
}

#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct PacketAttWrite {
    pub head: PacketL2capHead, // 0
    pub opcode: u8,
    pub handle: u8,
    pub handle1: u8,
    pub value: [u8; 30], //sno[3],src[2],dst[2],op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
}

#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct PacketAttCmd {
    pub head: PacketL2capHead, // 0
    pub opcode: u8,     // 10
    pub handle: u8,     // 11
    pub handle1: u8,    // 12
    pub value: PacketAttValue //sno[3],src[2],dst[2],op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
}

#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct PacketAttData {
    pub head: PacketL2capHead, // 0

    pub att: u8,
    //0x12 for master; 0x1b for slave// as ttl when relay
    pub hl: u8,
    // assigned by master
    pub hh: u8, //

    pub dat: [u8; 23],
}

#[derive(Clone, Copy, Default)]
#[repr(C, align(4))]
pub struct MeshPkt {
    pub head: PacketL2capHead, // 0
    pub src_tx: u16,            // 10
    pub handle1: u8,            // 12 for flag
    pub sno: [u8; 3],           // 13
    pub src_adr: u16,           // 16
    pub dst_adr: u16,           // 18
    pub op: u8,                 // 20
    pub vendor_id: u16,         // 21
    pub par: [u8; 10],          // 23
    pub internal_par1: [u8; 5], // 33
    pub ttl: u8,                // 38
    pub internal_par2: [u8; 4], // 39
    pub no_use: [u8; 4]         // 43 size must 48, when is set to be rf tx address.
}

const_assert!(mem::size_of::<MeshPkt>() == 48);

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PacketAttMtu {
    pub head: PacketL2capHead, // 0
    pub opcode: u8,
    pub mtu: [u8; 2]
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PacketAttErrRsp {
    pub head: PacketL2capHead, // 0
    pub opcode: u8,
    pub err_opcode: u8,
    pub err_handle: u16,
    pub err_reason: u8
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PacketLlDataRsp {
    pub head: PacketL2capHead, // 0

    pub att: u8,				//0x12 for master; 0x1b for slave// as ttl when relay
    pub hl: u8,				// assigned by master
    pub hh: u8,				//
    pub init: u8,
    pub dat: [u8; 14]
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PacketVersionInd {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,				//RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,				//LEN(6)_RFU(2)
    pub opcode: u8,
    pub main_ver: u8,
    pub vendor: u16,
    pub sub_ver: u16
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PacketFeatureRsp {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,				//RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,				//LEN(6)_RFU(2)
    pub opcode: u8,
    pub data: [u8; 8]
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PacketCtrlUnknown {
    pub dma_len: u32,            //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,				//RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,				//LEN(6)_RFU(2)
    pub opcode: u8,
    pub data: [u8; 1]
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PacketLlWriteRsp {
    pub head: PacketL2capHead, // 0

    pub	op: u16
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PacketAttWriteRsp {
    pub head: PacketL2capHead, // 0
    pub opcode: u8
}

#[derive(Clone, Copy, Debug)]
#[repr(C, packed)]
pub struct PktBuf {
    pub op: u8,
    pub sno: [u8; 3],
    pub notify_ok_flag: bool,
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct ScanRspData {
    pub handle: u16,
    pub data: AdvRspPrivate
}

#[derive(Clone, Copy)]
#[repr(C, align(4))]
pub struct PacketScanRsp {
    pub dma_len: u32,       // 0     //won't be a fixed number as previous, should adjust with the mouse package number

    pub _type: u8,          // 4				//RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,			// 5	//LEN(6)_RFU(2)
    pub adv_a: [u8; 6],		// 6	//adv address
    pub data: ScanRspData		// 12	//0-31 byte
}

#[derive(Clone, Copy)]
#[repr(C, align(4))]
pub union Packet {
    pub head: PacketL2capHead,
    pub att_write: PacketAttWrite,
    pub mesh: MeshPkt,
    pub att_cmd: PacketAttCmd,
    pub ll_data: PacketLlData,
    pub ll_app: PacketLlApp,
    pub sig_conn_para_up_rsp: PktL2capSigConnParaUpRsp,
    pub att_data: PacketAttData,
    pub version_ind: PacketVersionInd,
    pub ctrl_unknown: PacketCtrlUnknown,
    pub feature_rsp: PacketFeatureRsp,
    pub att_mtu: PacketAttMtu,
    pub att_err_rsp: PacketAttErrRsp,
    pub att_read_rsp: PacketAttReadRsp,
    pub att_write_rsp: PacketAttWriteRsp,
    pub adv_ind_module: RfPacketAdvIndModuleT,
    pub scan_rsp: PacketScanRsp,
    pub ll_init: PacketLlInit,
    pub l2cap_data: PacketL2capData
}

impl Packet {
    pub fn head(&self) -> &PacketL2capHead {
        unsafe { &self.head }
    }

    pub fn head_mut(&mut self) -> &mut PacketL2capHead {
        unsafe { &mut self.head }
    }

    pub fn att_write(&self) -> &PacketAttWrite {
        unsafe { &self.att_write }
    }

    pub fn att_write_mut(&mut self) -> &mut PacketAttWrite {
        unsafe { &mut self.att_write }
    }

    pub fn mesh(&self) -> &MeshPkt {
        unsafe { &self.mesh }
    }

    pub fn mesh_mut(&mut self) -> &mut MeshPkt {
        unsafe { &mut self.mesh }
    }

    pub fn att_cmd(&self) -> &PacketAttCmd {
        unsafe { &self.att_cmd }
    }

    pub fn att_cmd_mut(&mut self) -> &mut PacketAttCmd {
        unsafe { &mut self.att_cmd }
    }

    pub fn ll_data(&self) -> &PacketLlData {
        unsafe { &self.ll_data }
    }

    pub fn ll_data_mut(&mut self) -> &mut PacketLlData {
        unsafe { &mut self.ll_data }
    }

    pub fn ll_app(&self) -> &PacketLlApp {
        unsafe { &self.ll_app }
    }

    pub fn ll_app_mut(&mut self) -> &mut PacketLlApp {
        unsafe { &mut self.ll_app }
    }

    pub fn sig_conn_para_up_rsp(&self) -> &PktL2capSigConnParaUpRsp {
        unsafe { &self.sig_conn_para_up_rsp }
    }

    pub fn sig_conn_para_up_rsp_mut(&mut self) -> &mut PktL2capSigConnParaUpRsp {
        unsafe { &mut self.sig_conn_para_up_rsp }
    }

    pub fn att_data(&self) -> &PacketAttData {
        unsafe { &self.att_data }
    }

    pub fn att_data_mut(&mut self) -> &mut PacketAttData {
        unsafe { &mut self.att_data }
    }

    pub fn version_ind(&self) -> &PacketVersionInd {
        unsafe { &self.version_ind }
    }

    pub fn version_ind_mut(&mut self) -> &mut PacketVersionInd {
        unsafe { &mut self.version_ind }
    }

    pub fn ctrl_unknown(&self) -> &PacketCtrlUnknown {
        unsafe { &self.ctrl_unknown }
    }

    pub fn ctrl_unknown_mut(&mut self) -> &mut PacketCtrlUnknown {
        unsafe { &mut self.ctrl_unknown }
    }

    pub fn feature_rsp(&self) -> &PacketFeatureRsp {
        unsafe { &self.feature_rsp }
    }

    pub fn feature_rsp_mut(&mut self) -> &mut PacketFeatureRsp {
        unsafe { &mut self.feature_rsp }
    }

    pub fn att_mtu(&self) -> &PacketAttMtu {
        unsafe { &self.att_mtu }
    }

    pub fn att_mtu_mut(&mut self) -> &mut PacketAttMtu {
        unsafe { &mut self.att_mtu }
    }

    pub fn att_err_rsp(&self) -> &PacketAttErrRsp {
        unsafe { &self.att_err_rsp }
    }

    pub fn att_err_rsp_mut(&mut self) -> &mut PacketAttErrRsp {
        unsafe { &mut self.att_err_rsp }
    }

    pub fn att_read_rsp(&self) -> &PacketAttReadRsp {
        unsafe { &self.att_read_rsp }
    }

    pub fn att_read_rsp_mut(&mut self) -> &mut PacketAttReadRsp {
        unsafe { &mut self.att_read_rsp }
    }

    pub fn att_write_rsp(&self) -> &PacketAttWriteRsp {
        unsafe { &self.att_write_rsp }
    }

    pub fn att_write_rsp_mut(&mut self) -> &mut PacketAttWriteRsp {
        unsafe { &mut self.att_write_rsp }
    }

    pub fn adv_ind_module(&self) -> &RfPacketAdvIndModuleT {
        unsafe { &self.adv_ind_module }
    }

    pub fn adv_ind_module_mut(&mut self) -> &mut RfPacketAdvIndModuleT {
        unsafe { &mut self.adv_ind_module }
    }

    pub fn scan_rsp(&self) -> &PacketScanRsp {
        unsafe { &self.scan_rsp }
    }

    pub fn scan_rsp_mut(&mut self) -> &mut PacketScanRsp {
        unsafe { &mut self.scan_rsp }
    }

    pub fn ll_init(&self) -> &PacketLlInit {
        unsafe { &self.ll_init }
    }

    pub fn ll_init_mut(&mut self) -> &mut PacketLlInit {
        unsafe { &mut self.ll_init }
    }

    pub fn l2cap_data(&self) -> &PacketL2capData {
        unsafe { &self.l2cap_data }
    }

    pub fn l2cap_data_mut(&mut self) -> &mut PacketL2capData {
        unsafe { &mut self.l2cap_data }
    }
}

const_assert!(mem::size_of::<Packet>() == 48);