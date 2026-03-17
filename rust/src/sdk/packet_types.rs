use crate::sdk::light::{AdvRspPrivate, PKT_CMD_LEN};
use core::mem;

#[macro_export]
macro_rules! const_assert {
    ($($tt:tt)*) => {
        const _: () = assert!($($tt)*);
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct RfPacketAdvIndModuleT {
    pub dma_len: u32, // 0    //won't be a fixed number as previous, should adjust with the mouse package number

    pub _type: u8,      // 4	//RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,     // 5	//LEN(6)_RFU(2)
    pub adv_a: [u8; 6], // 6	//adv address
    pub data: [u8; 31], // 12	//0-31 byte
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
    pub dma_len: u32, // 0         //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,    // 4		//RFU(3)_MD(1)_SN(1)_NESN(1)-LLID(2)
    pub rf_len: u8,   // 5		//LEN(5)_RFU(3)
    pub l2cap_len: u16, // 6        // 0x17
    pub chan_id: u16, // 8			//0x04,
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct PacketL2capData {
    pub l2cap_len: u16,
    pub chan_id: u16,
    pub opcode: u8,
    pub handle: u8,
    pub handle1: u8,
    pub value: [u8; 30],
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct PacketLlData {
    pub head: PacketL2capHead, // 0
    pub att: u8,               // 10		//0x12 for master; 0x1b for slave// as ttl when relay
    pub hl: u8,                // 11		// assigned by master
    pub hh: u8,                // 12		//
    pub sno: u8,               // 13
    pub nid: u8,               // 14
    pub ttc: u8,               // 15
    pub group: u16,            // 16
    pub sid: [u16; 2],
    pub cmd: [u8; PKT_CMD_LEN],
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct AppCmdValue {
    pub sno: [u8; 3],   // 0    13
    pub src: u16,       // 3    16
    pub dst: u16,       // 5    18
    pub op: u8,         // 7    20
    pub vendor_id: u16, // 8    21
    pub par: [u8; 10],  // 10   23
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct PacketLlApp {
    pub head: PacketL2capHead, // 0
    pub opcode: u8,            // 10
    pub handle: u8,            // 11
    pub handle1: u8,           // 12
    pub value: AppCmdValue,    // 13
    pub rsv: [u8; 10],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PacketAttReadRsp {
    pub head: PacketL2capHead, // 0
    pub opcode: u8,            // 10
    pub value: [u8; 22],       // 11
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PacketLlInit {
    pub dma_len: u32, //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,    //RA(1)_TA(1)_RFU(2)_TYPE(4): connect request PDU
    pub rf_len: u8,   //LEN(6)_RFU(2)
    pub scan_a: [u8; 6], // scan address
    pub adv_a: [u8; 6], // adv address
    pub aa: [u8; 4],  // access code
    pub crcinit: [u8; 3],
    pub wsize: u8,
    pub woffset: u16,
    pub interval: u16,
    pub latency: u16,
    pub timeout: u16,
    pub chm: [u8; 5],
    pub hop: u8, //sca(3)_hop(5)
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct PktL2capSigConnParaUpRsp {
    pub head: PacketL2capHead, // 0
    pub code: u8,
    pub id: u8,
    pub data_len: u16,
    pub result: u16,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct PacketAttWrite {
    pub head: PacketL2capHead, // 0
    pub opcode: u8,
    pub handle: u8,
    pub handle1: u8,
    pub value: PacketAttValue, //sno[3],src[2],dst[2],op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct PacketAttRawValue {
    pub head: PacketL2capHead, // 0
    pub opcode: u8,
    pub handle: u8,
    pub handle1: u8,
    pub value: [u8; 30],
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct PacketAttCmd {
    pub head: PacketL2capHead, // 0
    pub opcode: u8,            // 10
    pub handle: u8,            // 11
    pub handle1: u8,           // 12
    pub value: PacketAttValue, //sno[3],src[2],dst[2],op[1~3],params[0~10],mac-app[5],ttl[1],mac-net[4]
}

#[repr(C, packed)]
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
    pub head: PacketL2capHead,  // 0
    pub src_tx: u16,            // 10
    pub handle1: u8,            // 12 for flag
    pub sno: [u8; 3],           // 13
    pub src_adr: u16,           // 16
    pub dst_adr: u16,           // 18
    pub op: u8,                 // 20
    pub vendor_id: u16,         // 22 (1 byte padding after op for u16 alignment)
    pub par: [u8; 10],          // 24
    pub internal_par1: [u8; 5], // 34
    pub ttl: u8,                // 39
    pub internal_par2: [u8; 4], // 40
    pub no_use: [u8; 4],        // 44 size must 48, when is set to be rf tx address.
}

const_assert!(mem::size_of::<MeshPkt>() == 48);

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PacketAttMtu {
    pub head: PacketL2capHead, // 0
    pub opcode: u8,
    pub mtu: [u8; 2],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PacketAttErrRsp {
    pub head: PacketL2capHead, // 0
    pub opcode: u8,
    pub err_opcode: u8,
    pub err_handle: u16,
    pub err_reason: u8,
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PacketLlDataRsp {
    pub head: PacketL2capHead, // 0

    pub att: u8, //0x12 for master; 0x1b for slave// as ttl when relay
    pub hl: u8,  // assigned by master
    pub hh: u8,  //
    pub init: u8,
    pub dat: [u8; 14],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PacketVersionInd {
    pub dma_len: u32, //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,    //RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,   //LEN(6)_RFU(2)
    pub opcode: u8,
    pub main_ver: u8,
    pub vendor: u16,
    pub sub_ver: u16,
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PacketFeatureRsp {
    pub dma_len: u32, //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,    //RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,   //LEN(6)_RFU(2)
    pub opcode: u8,
    pub data: [u8; 8],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PacketCtrlUnknown {
    pub dma_len: u32, //won't be a fixed number as previous, should adjust with the mouse package number
    pub _type: u8,    //RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,   //LEN(6)_RFU(2)
    pub opcode: u8,
    pub data: [u8; 1],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PacketLlWriteRsp {
    pub head: PacketL2capHead, // 0
    pub op: u16,
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PacketAttWriteRsp {
    pub head: PacketL2capHead, // 0
    pub opcode: u8,
}

#[derive(Clone, Copy, Debug)]
#[repr(C, packed)]
pub struct PktBuf {
    pub op: u8,
    pub sno: [u8; 3],
    pub notify_ok_flag: bool,
}

/// BLE Channel Map Update packet structure for type 3 control packets.
///
/// This packet type handles BLE connection channel map updates, which occur when
/// the central device needs to change the frequency hopping sequence used in the connection.
/// The 5-byte channel map data starts 1 byte into the l2cap_len field (at the high byte)
/// and extends 4 more bytes beyond it, as defined by the BLE specification.
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PacketChannelMapUpdate {
    pub dma_len: u32,             // 0 - DMA length field
    pub _type: u8,                // 4 - Packet type (should be 3 for channel map update)
    pub rf_len: u8,               // 5 - RF packet length
    pub l2cap_len_low: u8,        // 6 - Low byte of L2CAP length (should be 1)
    pub channel_map: [u8; 5],     // 7 - 5-byte channel map starting at l2cap_len high byte
    pub remaining_data: [u8; 36], // 12 - Rest of packet data including timing information
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct ScanRspData {
    pub handle: u16,
    pub data: AdvRspPrivate,
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct PacketScanRsp {
    pub dma_len: u32, // 0     //won't be a fixed number as previous, should adjust with the mouse package number

    pub _type: u8,         // 4				//RA(1)_TA(1)_RFU(2)_TYPE(4)
    pub rf_len: u8,        // 5	//LEN(6)_RFU(2)
    pub adv_a: [u8; 6],    // 6	//adv address
    pub data: ScanRspData, // 12	//0-31 byte
}

#[derive(Clone, Copy)]
#[repr(C, align(4))]
pub union Packet {
    pub head: PacketL2capHead,
    pub att_write: PacketAttWrite,
    pub mesh: MeshPkt,
    pub att_cmd: PacketAttCmd,
    pub att_val: PacketAttRawValue,
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
    pub channel_map_update: PacketChannelMapUpdate,
    pub adv_ind_module: RfPacketAdvIndModuleT,
    pub scan_rsp: PacketScanRsp,
    pub ll_init: PacketLlInit,
    pub l2cap_data: PacketL2capData,
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

    pub fn att_val(&self) -> &PacketAttRawValue {
        unsafe { &self.att_val }
    }

    pub fn att_val_mut(&mut self) -> &mut PacketAttRawValue {
        unsafe { &mut self.att_val }
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

    pub fn channel_map_update(&self) -> &PacketChannelMapUpdate {
        unsafe { &self.channel_map_update }
    }

    pub fn channel_map_update_mut(&mut self) -> &mut PacketChannelMapUpdate {
        unsafe { &mut self.channel_map_update }
    }
}

const_assert!(mem::size_of::<Packet>() == 48);

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use core::mem;

    /// Tests that MeshPkt struct size is exactly 48 bytes as required
    #[test]
    fn test_mesh_pkt_size() {
        assert_eq!(mem::size_of::<MeshPkt>(), 48);
    }

    /// Tests that Packet union size is exactly 48 bytes as required
    #[test]
    fn test_packet_union_size() {
        assert_eq!(mem::size_of::<Packet>(), 48);
    }

    /// Tests sizes of individual packet structures
    #[test]
    fn test_packet_structure_sizes() {
        // Test basic packet head size
        assert_eq!(mem::size_of::<PacketL2capHead>(), 10);

        // Test various packet types (packed sizes)
        assert_eq!(mem::size_of::<PacketAttWrite>(), 43);
        assert_eq!(mem::size_of::<PacketAttCmd>(), 43);
        assert_eq!(mem::size_of::<PacketLlData>(), 33);
        assert_eq!(mem::size_of::<PacketLlApp>(), 43);
        assert_eq!(mem::size_of::<MeshPkt>(), 48);

        // Test smaller structures
        assert_eq!(mem::size_of::<PacketAttValue>(), 30);
        assert_eq!(mem::size_of::<AppCmdValue>(), 20);
        assert_eq!(mem::size_of::<RfPacketAdvIndModuleT>(), 43);
    }

    /// Tests that Packet union accessor methods work correctly
    #[test]
    fn test_packet_union_accessors() {
        let mut packet = Packet {
            head: PacketL2capHead {
                dma_len: 100,
                _type: 0x01,
                rf_len: 0x02,
                l2cap_len: 0x0304,
                chan_id: 0x0506,
            },
        };

        // Test head accessor - copy unaligned fields to primitives
        let dma_len = packet.head().dma_len;
        let l2cap_len = packet.head().l2cap_len;
        let chan_id = packet.head().chan_id;
        assert_eq!(dma_len, 100);
        assert_eq!(packet.head()._type, 0x01);
        assert_eq!(packet.head().rf_len, 0x02);
        assert_eq!(l2cap_len, 0x0304);
        assert_eq!(chan_id, 0x0506);

        // Test head_mut accessor
        packet.head_mut().dma_len = 200;
        let dma_len_after = packet.head().dma_len;
        assert_eq!(dma_len_after, 200);
    }

    /// Tests PacketAttWrite union field access
    #[test]
    fn test_packet_att_write_access() {
        let mut packet = Packet {
            att_write: PacketAttWrite {
                head: PacketL2capHead {
                    dma_len: 48,
                    _type: 0x02,
                    rf_len: 0x20,
                    l2cap_len: 0x0027,
                    chan_id: 0x0004,
                },
                opcode: 0x12,
                handle: 0x01,
                handle1: 0x00,
                value: PacketAttValue {
                    sno: [0x01, 0x02, 0x03],
                    src: [0x04, 0x05],
                    dst: [0x06, 0x07],
                    val: [
                        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13,
                        0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E,
                    ],
                },
            },
        };

        // Test att_write accessor
        let att_write = packet.att_write();
        let opcode = att_write.opcode;
        assert_eq!(opcode, 0x12);
        assert_eq!(att_write.handle, 0x01);
        // Copy to temporary for comparison due to packed struct alignment
        let sno_copy = att_write.value.sno;
        assert_eq!(sno_copy, [0x01, 0x02, 0x03]);

        // Test att_write_mut accessor
        packet.att_write_mut().opcode = 0x13;
        let opcode_after = packet.att_write().opcode;
        assert_eq!(opcode_after, 0x13);
    }

    /// Tests MeshPkt union field access
    #[test]
    fn test_packet_mesh_access() {
        let mut packet = Packet {
            mesh: MeshPkt {
                head: PacketL2capHead {
                    dma_len: 48,
                    _type: 0x02,
                    rf_len: 0x20,
                    l2cap_len: 0x0027,
                    chan_id: 0x0004,
                },
                src_tx: 0x0102,
                handle1: 0x03,
                sno: [0x04, 0x05, 0x06],
                src_adr: 0x0708,
                dst_adr: 0x090A,
                op: 0x0B,
                vendor_id: 0x0C0D,
                par: [0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17],
                internal_par1: [0x18, 0x19, 0x1A, 0x1B, 0x1C],
                ttl: 0x1D,
                internal_par2: [0x1E, 0x1F, 0x20, 0x21],
                no_use: [0x22, 0x23, 0x24, 0x25],
            },
        };

        // Test mesh accessor - copy unaligned fields to primitives
        let mesh = packet.mesh();
        let src_tx = mesh.src_tx;
        let src_adr = mesh.src_adr;
        let dst_adr = mesh.dst_adr;
        let vendor_id = mesh.vendor_id;
        assert_eq!(src_tx, 0x0102);
        assert_eq!(mesh.handle1, 0x03);
        // Copy to temporary for comparison due to packed struct alignment
        let sno_copy = mesh.sno;
        assert_eq!(sno_copy, [0x04, 0x05, 0x06]);
        assert_eq!(src_adr, 0x0708);
        assert_eq!(dst_adr, 0x090A);
        assert_eq!(mesh.op, 0x0B);
        assert_eq!(vendor_id, 0x0C0D);
        assert_eq!(mesh.ttl, 0x1D);

        // Test mesh_mut accessor
        packet.mesh_mut().src_tx = 0x0203;
        let src_tx_after = packet.mesh().src_tx;
        assert_eq!(src_tx_after, 0x0203);
    }

    /// Tests advertising packet accessors
    #[test]
    fn test_packet_adv_access() {
        let mut packet = Packet {
            adv_ind_module: RfPacketAdvIndModuleT {
                dma_len: 43,
                _type: 0x00,
                rf_len: 0x20,
                adv_a: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06],
                data: [
                    0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13,
                    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20,
                    0x21, 0x22, 0x23, 0x24, 0x25,
                ],
            },
        };

        // Test adv_ind_module accessor
        let adv = packet.adv_ind_module();
        let dma_len = adv.dma_len;
        assert_eq!(dma_len, 43);
        assert_eq!(adv._type, 0x00);
        assert_eq!(adv.rf_len, 0x20);
        // Copy to temporary for comparison due to packed struct alignment
        let adv_a_copy = adv.adv_a;
        assert_eq!(adv_a_copy, [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
        assert_eq!(adv.data.len(), 31);

        // Test adv_ind_module_mut accessor
        packet.adv_ind_module_mut().dma_len = 50;
        let dma_len_after = packet.adv_ind_module().dma_len;
        assert_eq!(dma_len_after, 50);
    }

    /// Tests scan response packet accessors
    #[test]
    fn test_packet_scan_rsp_access() {
        let mut packet = Packet {
            scan_rsp: PacketScanRsp {
                dma_len: 40,
                _type: 0x04,
                rf_len: 0x1F,
                adv_a: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06],
                data: ScanRspData {
                    handle: 0x0708,
                    data: AdvRspPrivate::default(),
                },
            },
        };

        // Test scan_rsp accessor
        let scan = packet.scan_rsp();
        let dma_len = scan.dma_len;
        let data_handle = scan.data.handle;
        assert_eq!(dma_len, 40);
        assert_eq!(scan._type, 0x04);
        assert_eq!(scan.rf_len, 0x1F);
        // Copy to temporary for comparison due to packed struct alignment
        let adv_a_copy = scan.adv_a;
        assert_eq!(adv_a_copy, [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
        assert_eq!(data_handle, 0x0708);

        // Test scan_rsp_mut accessor
        packet.scan_rsp_mut().dma_len = 45;
        let dma_len_after = packet.scan_rsp().dma_len;
        assert_eq!(dma_len_after, 45);
    }

    /// Tests that structs implement required traits
    #[test]
    fn test_struct_traits() {
        // Test Clone trait
        let pkt1 = RfPacketAdvIndModuleT::default();
        let pkt2 = pkt1.clone();
        let dma_len1 = pkt1.dma_len;
        let dma_len2 = pkt2.dma_len;
        assert_eq!(dma_len1, dma_len2);

        // Test Copy trait (should be bitwise copy)
        let pkt3 = pkt1;
        let pkt4 = pkt1; // Should work if Copy is implemented
        let dma_len3 = pkt3.dma_len;
        let dma_len4 = pkt4.dma_len;
        assert_eq!(dma_len3, dma_len4);

        // Test Default trait
        let default_mesh = MeshPkt::default();
        let head_copy = default_mesh.head;
        let dma_len = head_copy.dma_len;
        assert_eq!(dma_len, 0);
        let src_tx = default_mesh.src_tx;
        let op = default_mesh.op;
        assert_eq!(src_tx, 0);
        assert_eq!(op, 0);
    }

    /// Tests packet structure field alignments and offsets
    #[test]
    fn test_packet_field_offsets() {
        // Test that fields are at expected positions in packed structs
        let mesh = MeshPkt::default();

        // Since these are packed structs, we need to be careful about alignment
        // but we can test that the structs can be created and accessed

        // Test MeshPkt field access
        assert_eq!(mem::offset_of!(MeshPkt, head), 0);
        assert_eq!(mem::offset_of!(MeshPkt, src_tx), 10);
        assert_eq!(mem::offset_of!(MeshPkt, handle1), 12);
        assert_eq!(mem::offset_of!(MeshPkt, sno), 13);
        assert_eq!(mem::offset_of!(MeshPkt, src_adr), 16);
        assert_eq!(mem::offset_of!(MeshPkt, dst_adr), 18);
        assert_eq!(mem::offset_of!(MeshPkt, op), 20);
        assert_eq!(mem::offset_of!(MeshPkt, vendor_id), 22);
        assert_eq!(mem::offset_of!(MeshPkt, par), 24);
        assert_eq!(mem::offset_of!(MeshPkt, internal_par1), 34);
        assert_eq!(mem::offset_of!(MeshPkt, ttl), 39);
        assert_eq!(mem::offset_of!(MeshPkt, internal_par2), 40);
        assert_eq!(mem::offset_of!(MeshPkt, no_use), 44);
    }

    /// Tests that MeshPkt broadcast encryption fields are at correct absolute byte offsets.
    /// pair_dec_packet_mesh/pair_enc_packet_mesh for broadcast packets (chan_id == 0xffff) use:
    ///   - IV: 8 bytes starting from head.rf_len (offset 5)
    ///   - MIC: 2 bytes at internal_par2[1] (must be offset 41)
    ///   - Data: 0x1c bytes starting from sno (offset 13, so data covers bytes 13..41)
    /// The MIC must immediately follow the data region (offset 41) for AES-CCM to work correctly.
    /// MeshPkt is repr(C, align(4)): the 1-byte padding after op (offset 21) means internal_par2
    /// is at offset 40 and internal_par2[1] is at offset 41.
    #[test]
    fn test_meshpkt_broadcast_encryption_field_offsets() {
        let pkt = MeshPkt::default();
        let base = core::ptr::addr_of!(pkt) as usize;

        // IV source: rf_len in head (offset 5 within PacketL2capHead)
        let rf_len_offset = unsafe { core::ptr::addr_of!(pkt.head.rf_len) as usize - base };
        assert_eq!(
            rf_len_offset, 5,
            "rf_len must be at offset 5 for broadcast IV"
        );

        // Encrypted data region: 0x1c (28) bytes starting from sno
        let sno_offset = mem::offset_of!(MeshPkt, sno);
        assert_eq!(
            sno_offset, 13,
            "sno must be at offset 13 for broadcast data"
        );

        // MIC location: internal_par2[1] - must be at offset 41 (sno + 0x1c = 13 + 28)
        let internal_par2_offset = mem::offset_of!(MeshPkt, internal_par2);
        assert_eq!(
            internal_par2_offset, 40,
            "internal_par2 must be at offset 40"
        );
        let mic_offset = internal_par2_offset + 1; // internal_par2[1]
        assert_eq!(
            mic_offset,
            sno_offset + 0x1c,
            "broadcast MIC (internal_par2[1]) must immediately follow the 0x1c-byte data region"
        );
    }

    /// Tests PktBuf structure
    #[test]
    fn test_pkt_buf_structure() {
        let pkt_buf = PktBuf {
            op: 0x01,
            sno: [0x02, 0x03, 0x04],
            notify_ok_flag: true,
        };

        let op = pkt_buf.op;
        assert_eq!(op, 0x01);
        // Copy to temporary for comparison due to packed struct alignment
        let sno_copy = pkt_buf.sno;
        assert_eq!(sno_copy, [0x02, 0x03, 0x04]);
        assert_eq!(pkt_buf.notify_ok_flag, true);

        // Test Debug trait (PktBuf has Debug)
        let debug_str = format!("{:?}", pkt_buf);
        assert!(debug_str.contains("PktBuf"));
    }

    /// Tests Channel Map Update packet structure
    #[test]
    fn test_channel_map_update_structure() {
        let channel_update = PacketChannelMapUpdate {
            dma_len: 50,
            _type: 0x03,
            rf_len: 0x16,
            l2cap_len_low: 0x01,
            channel_map: [0xFF, 0xFF, 0xFF, 0xFF, 0x1F],
            remaining_data: [0; 36],
        };

        let dma_len = channel_update.dma_len;
        let l2cap_len_low = channel_update.l2cap_len_low;
        assert_eq!(dma_len, 50);
        assert_eq!(channel_update._type, 0x03);
        assert_eq!(channel_update.rf_len, 0x16);
        assert_eq!(l2cap_len_low, 0x01);
        // Copy to temporary for comparison due to packed struct alignment
        let channel_map_copy = channel_update.channel_map;
        assert_eq!(channel_map_copy, [0xFF, 0xFF, 0xFF, 0xFF, 0x1F]);
    }

    /// Tests that all union variants can be safely accessed
    #[test]
    fn test_union_variant_access() {
        // Create a packet and test that all accessors work without panicking
        let mut packet = Packet {
            head: PacketL2capHead::default(),
        };

        // These should all be safe to call (even if they return garbage data)
        let _ = packet.head();
        let _ = packet.att_write();
        let _ = packet.mesh();
        let _ = packet.att_cmd();
        let _ = packet.att_val();
        let _ = packet.ll_data();
        let _ = packet.ll_app();
        let _ = packet.sig_conn_para_up_rsp();
        let _ = packet.att_data();
        let _ = packet.version_ind();
        let _ = packet.ctrl_unknown();
        let _ = packet.feature_rsp();
        let _ = packet.att_mtu();
        let _ = packet.att_err_rsp();
        let _ = packet.att_read_rsp();
        let _ = packet.att_write_rsp();
        let _ = packet.channel_map_update();
        let _ = packet.adv_ind_module();
        let _ = packet.scan_rsp();
        let _ = packet.ll_init();
        let _ = packet.l2cap_data();

        // Test mutable accessors too
        let _ = packet.head_mut();
        let _ = packet.att_write_mut();
        let _ = packet.mesh_mut();
        let _ = packet.att_cmd_mut();
        let _ = packet.att_val_mut();
        let _ = packet.ll_data_mut();
        let _ = packet.ll_app_mut();
        let _ = packet.sig_conn_para_up_rsp_mut();
        let _ = packet.att_data_mut();
        let _ = packet.version_ind_mut();
        let _ = packet.ctrl_unknown_mut();
        let _ = packet.feature_rsp_mut();
        let _ = packet.att_mtu_mut();
        let _ = packet.att_err_rsp_mut();
        let _ = packet.att_read_rsp_mut();
        let _ = packet.att_write_rsp_mut();
        let _ = packet.channel_map_update_mut();
        let _ = packet.adv_ind_module_mut();
        let _ = packet.scan_rsp_mut();
        let _ = packet.ll_init_mut();
        let _ = packet.l2cap_data_mut();
    }
}
