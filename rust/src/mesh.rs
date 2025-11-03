use core::ptr::addr_of;

use bytemuck::{Pod, Zeroable};
use heapless::{Deque, Vec};

use crate::common::{access_code, mesh_node_init, pair_load_key, SYS_CHN_LISTEN};
use crate::config::{FLASH_ADR_PAIRING, VENDOR_ID};
use crate::embassy::time_driver::clock_time64;
use crate::embassy::yield_now::yield_now;
use crate::main_light::{light_slave_tx_command, rf_link_light_event_callback};
use crate::sdk::ble_app::ble_ll_pair::{pair_enc_packet_mesh, pair_save_key};
use crate::sdk::ble_app::light_ll::mesh_management::rf_link_match_group_mac;
use crate::sdk::ble_app::light_ll::packet_processing::{
    is_add_packet_buf_ready, rf_link_add_tx_packet, rf_link_rc_data,
};
use crate::sdk::ble_app::rf_drv_8266::{
    rf_set_ble_access_code, rf_set_ble_channel, rf_set_ble_crc_adv, rf_set_rxmode,
    rf_set_tx_rx_off, rf_start_srx2tx,
};
use crate::sdk::drivers::flash::{flash_write_page};
use crate::sdk::light::*;
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed, sleep_us, CLOCK_SYS_CLOCK_1US};
use crate::sdk::mcu::register::{
    read_reg_rnd_number, read_reg_system_tick, write_reg_rf_irq_status, FLD_RF_IRQ_MASK,
};
use crate::sdk::packet_types::{Packet, PacketAttCmd, PacketAttValue, PacketL2capHead};
use crate::state::*;
use crate::uart_manager::light_mesh_rx_cb;
use crate::{app, uprintln, BIT};

pub const MESH_PAIR_CMD_INTERVAL: u32 = 500;

//unit: s
pub const MESH_PAIR_TIMEOUT: u32 = 10;

pub const MESH_NODE_ST_VAL_LEN: usize = 4;
// MIN: 4,   MAX: 10
pub const MESH_NODE_ST_PAR_LEN: usize = MESH_NODE_ST_VAL_LEN - 2;

#[derive(Clone, Copy, PartialEq, Debug)]
enum MeshPairState {
    MeshPairName1 = 0,
    MeshPairName2,
    MeshPairPwd1,
    MeshPairPwd2,
    MeshPairLtk1,
    MeshPairLtk2,
    MeshPairEffectDelay,
    MeshPairEffect,
    MeshPairDefaultMesh,
}

impl TryFrom<u8> for MeshPairState {
    type Error = ();

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            x if x == MeshPairState::MeshPairName1 as u8 => Ok(MeshPairState::MeshPairName1),
            x if x == MeshPairState::MeshPairName2 as u8 => Ok(MeshPairState::MeshPairName2),
            x if x == MeshPairState::MeshPairPwd1 as u8 => Ok(MeshPairState::MeshPairPwd1),
            x if x == MeshPairState::MeshPairPwd2 as u8 => Ok(MeshPairState::MeshPairPwd2),
            x if x == MeshPairState::MeshPairLtk1 as u8 => Ok(MeshPairState::MeshPairLtk1),
            x if x == MeshPairState::MeshPairLtk2 as u8 => Ok(MeshPairState::MeshPairLtk2),
            x if x == MeshPairState::MeshPairEffectDelay as u8 => {
                Ok(MeshPairState::MeshPairEffectDelay)
            }
            x if x == MeshPairState::MeshPairEffect as u8 => Ok(MeshPairState::MeshPairEffect),
            x if x == MeshPairState::MeshPairDefaultMesh as u8 => {
                Ok(MeshPairState::MeshPairDefaultMesh)
            }
            _ => Err(()),
        }
    }
}

#[derive(Clone, Copy, Debug)]
#[repr(C, packed)]
pub struct MeshNodeStValT {
    pub dev_adr: u8,
    // don't change include type
    pub sn: u8,
    // don't change include type
    pub par: [u8; MESH_NODE_ST_PAR_LEN], //lumen-rsv,
}

// SAFETY: This struct is safe to use with bytemuck because:
// 1. It's #[repr(C, packed)] so it has a stable memory layout
// 2. All fields are u8 which are safe to transmute
// 3. The struct size is fixed and known at compile time
unsafe impl Pod for MeshNodeStValT {}
unsafe impl Zeroable for MeshNodeStValT {}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct MeshNodeStT {
    pub tick: u16,
    // don't change include type
    pub val: MeshNodeStValT,
}

struct SendPkt {
    pub delay: u64,
    pub pkt: Packet,
    pub send_count: u8,
}

#[cfg_attr(test, mry::mry)]
pub struct MeshManager {
    mesh_pair_start_time: u32,
    default_mesh_time: u32,
    default_mesh_effect_delay_ref: u32,
    new_mesh_name: [u8; 16],
    new_mesh_pwd: [u8; 16],
    new_mesh_ltk: [u8; 16],
    default_mesh_time_ref: u32,
    effect_new_mesh: u8,
    effect_new_mesh_delay_time: u32,
    mesh_pair_cmd_interval: u32,
    mesh_pair_timeout: u32,
    mesh_pair_time: u32,
    mesh_pair_state: MeshPairState,
    pkt_send_buf: Vec<SendPkt, 6>,
    pkt_rcv_buf: Deque<Packet, 6>,
}

#[cfg_attr(test, mry::mry(skip_fns(default_const)))]
impl MeshManager {
    #[cfg(not(test))]
    pub const fn default_const() -> Self {
        Self {
            mesh_pair_start_time: 0,
            default_mesh_time: 0,
            default_mesh_effect_delay_ref: 0,
            new_mesh_name: [0; 16],
            new_mesh_pwd: [0; 16],
            new_mesh_ltk: [0; 16],
            default_mesh_time_ref: 0,
            effect_new_mesh: 0,
            effect_new_mesh_delay_time: 0,
            mesh_pair_cmd_interval: 0,
            mesh_pair_timeout: 0,
            mesh_pair_time: 0,
            mesh_pair_state: MeshPairState::MeshPairName1,
            pkt_send_buf: Vec::new(),
            pkt_rcv_buf: Deque::new(),
        }
    }

    #[cfg(test)]
    pub fn default() -> Self {
        Self {
            mesh_pair_start_time: 0,
            default_mesh_time: 0,
            default_mesh_effect_delay_ref: 0,
            new_mesh_name: [0; 16],
            new_mesh_pwd: [0; 16],
            new_mesh_ltk: [0; 16],
            default_mesh_time_ref: 0,
            effect_new_mesh: 0,
            effect_new_mesh_delay_time: 0,
            mesh_pair_cmd_interval: 0,
            mesh_pair_timeout: 0,
            mesh_pair_time: 0,
            mesh_pair_state: MeshPairState::MeshPairName1,
            pkt_send_buf: Vec::new(),
            pkt_rcv_buf: Deque::new(),
            mry: Default::default(),
        }
    }

    pub fn send_mesh_message(
        &mut self,
        data: &Vec<u8, 13>,
        destination: u16,
        retransmit_count: u8,
        send_ack: bool,
    ) -> [u8; 3] {
        // Sends a message to the mesh and returns the SNO of the sent message
        // data: 13 bytes of data usually in form [op, vendor_id_hi, vendor_id_lo, params...]
        // destination: The address of the destination
        // retransmit_count: How many times the packet should be rebroadcast at each node
        // send_ack: If the destination device should send an ack message back after it's handled the message
        let pkt = light_slave_tx_command(data, destination, retransmit_count, send_ack);
        let (group_match, device_match) = rf_link_match_group_mac(&pkt);
        if group_match || device_match {
            app().mesh_manager.add_rcv_mesh_msg(&pkt);
            if !device_match {
                self.add_send_mesh_msg(
                    &pkt,
                    0,
                    pkt.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
                );
            }
        } else {
            self.add_send_mesh_msg(
                &pkt,
                0,
                pkt.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
            );
        }

        pkt.mesh().sno
    }

    pub fn mesh_pair_init(&mut self) {
        MESH_PAIR_ENABLE.set(true);
        self.mesh_pair_cmd_interval = MESH_PAIR_CMD_INTERVAL;
        self.mesh_pair_timeout = MESH_PAIR_TIMEOUT;
    }

    pub fn mesh_pair_proc_effect(&mut self) {
        if self.effect_new_mesh != 0
            || (self.effect_new_mesh_delay_time != 0
                && (clock_time_exceed(
                    self.effect_new_mesh_delay_time,
                    self.mesh_pair_cmd_interval * 1000,
                )))
        {
            self.save_effect_new_mesh();
            self.effect_new_mesh = 0;
            self.effect_new_mesh_delay_time = 0;
        }
    }

    /// Marks device address configuration as successfully validated.
    /// Called when LGT_CMD_CONFIG_DEV_ADDR is processed and device address is accepted.
    /// Sets the validation flag to false, allowing mesh credentials to be saved.
    pub fn mesh_device_address_validation_completed(&mut self) {
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false); // validation completed successfully
        if MESH_PAIR_ENABLE.get() {
            // Write to byte [1] of the header to clear the device address validation pending flag
            // The header is stored at FLASH_CONFIGURATION_INDEX + 0x00, and byte [1] holds the validation state
            let mut data: [u8; 1] = [0];
            flash_write_page(
                (FLASH_ADR_PAIRING as i32 + FLASH_CONFIGURATION_INDEX.get() + 1) as u32,
                1,
                data.as_mut_ptr(),
            );
        }
    }

    pub fn mesh_pair_cb(&mut self, params: &Vec<u8, 10>) {
        if self.default_mesh_time_ref != 0 {
            self.default_mesh_time_ref = clock_time() | 1;
        }
        let cmd = MeshPairState::try_from(params[0]);
        if cmd.is_err() {
            return;
        }

        match cmd.unwrap() {
            MeshPairState::MeshPairName1 => {
                self.mesh_pair_start_time = clock_time() | 1;
                self.new_mesh_name[0..8].copy_from_slice(&params[1..9]);
            }
            MeshPairState::MeshPairName2 => {
                self.new_mesh_name[8..16].copy_from_slice(&params[1..9])
            }
            MeshPairState::MeshPairPwd1 => self.new_mesh_pwd[0..8].copy_from_slice(&params[1..9]),
            MeshPairState::MeshPairPwd2 => self.new_mesh_pwd[8..16].copy_from_slice(&params[1..9]),
            MeshPairState::MeshPairLtk1 => self.new_mesh_ltk[0..8].copy_from_slice(&params[1..9]),
            MeshPairState::MeshPairLtk2 => self.new_mesh_ltk[8..16].copy_from_slice(&params[1..9]),
            MeshPairState::MeshPairEffectDelay => {
                self.effect_new_mesh_delay_time = clock_time() | 1;
                if self.default_mesh_time_ref != 0 {
                    /* Keep default_mesh_time_ref non-zero */
                    self.default_mesh_time = self.mesh_pair_cmd_interval * 2;
                }
            }
            MeshPairState::MeshPairEffect => self.effect_new_mesh = 1,
            MeshPairState::MeshPairDefaultMesh => {
                self.default_mesh_effect_delay_ref = clock_time() | 1;
                self.default_mesh_time = params[1] as u32 * 1000;
            }
        }
    }

    fn mesh_cmd_notify(&self, op: u8, p: &Vec<u8, 10>, dev_adr: u16) -> i32 {
        let mut err = -1;
        if BLE_PERIPHERAL_CONNECTION_ACTIVE.get() && PAIR_LOGIN_OK.get() {
            if p.len() > 10 {
                //max length of par is 10
                return -1;
            }

            let mut pkt_notify = PacketAttCmd {
                head: PacketL2capHead {
                    dma_len: 0x1d,   // dma_len
                    _type: 0x02,     // type
                    rf_len: 0x1b,    // rf_len
                    l2cap_len: 0x17, // u16
                    chan_id: 0x04,   // chanid
                },
                opcode: 0x1b, // notify
                handle: 0x12,
                handle1: 0x00, // status handler
                value: PacketAttValue {
                    sno: [op | 0xc0, (VENDOR_ID & 0xff) as u8, (VENDOR_ID >> 8) as u8],
                    src: [(dev_adr & 0xFF) as u8, (dev_adr >> 8) as u8], // todo: Should this actually be dst?
                    dst: [0; 2],
                    val: [0; 23],
                },
            };

            pkt_notify.value.val[3..3 + p.len()].copy_from_slice(&p[0..p.len()]);

            if is_add_packet_buf_ready()
                && rf_link_add_tx_packet(&Packet {
                    att_cmd: pkt_notify,
                })
            {
                err = 0;
            }
        }

        err
    }

    fn mesh_pair_complete_notify(&self) -> i32 {
        let par = [CMD_NOTIFY_MESH_PAIR_END];
        self.mesh_cmd_notify(
            LGT_CMD_MESH_CMD_NOTIFY,
            &Vec::from_slice(&par).unwrap(),
            DEVICE_ADDRESS.get(),
        )
    }

    fn safe_effect_new_mesh_finish(&mut self) {
        self.new_mesh_name = [0; 16];
        self.new_mesh_pwd = [0; 16];
        self.new_mesh_ltk = [0; 16];
        self.mesh_pair_start_time = 0;
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
    }

    fn save_effect_new_mesh(&mut self) {
        // Only skip saving if either:
        // 1. We're in a default mesh timeout period, OR
        // 2. Device address validation is still pending (not yet completed)
        if self.default_mesh_time_ref != 0 || MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get() {
            self.mesh_pair_complete_notify();
            sleep_us(1000);
            /* Switch to normal mesh */
            pair_load_key();
            self.default_mesh_time_ref = 0;

            mesh_node_init();
            app().light_manager.device_status_update();
            self.safe_effect_new_mesh_finish();
            return;
        }

        {
            let mut pair_state = PAIR_STATE.lock();

            if self.effect_new_mesh == 0 {
                pair_state.pair_nn.copy_from_slice(&self.new_mesh_name);
                pair_state.pair_pass.copy_from_slice(&self.new_mesh_pwd);
                pair_state.pair_ltk.copy_from_slice(&self.new_mesh_ltk);
            } else {
                let pair_ltk_mesh = pair_state.pair_ltk_mesh;
                pair_state.pair_ltk.copy_from_slice(&pair_ltk_mesh);
            }
        }

        self.mesh_pair_complete_notify();

        // make sure not receive legacy mesh data from now on
        critical_section::with(|_| {
            pair_save_key();
            rf_set_ble_access_code(PAIR_AC.get()); // use new access code at once.
            rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO); // clear online status :mesh_node_init()
            sleep_us(1000);
            write_reg_rf_irq_status(FLD_RF_IRQ_MASK::IRQ_RX.bits()); // clear current rx in buffer
        });

        self.safe_effect_new_mesh_finish();
    }

    fn switch_to_default_mesh(&mut self, delay_s: u8) {
        self.default_mesh_time_ref = clock_time() | 1;
        self.default_mesh_time = delay_s as u32 * 1000;

        /* Only change AC and LTK */
        PAIR_AC.set(access_code(
            &*PAIR_CONFIG_MESH_NAME.lock(),
            &*PAIR_CONFIG_MESH_PWD.lock(),
        ));
        PAIR_STATE
            .lock()
            .pair_ltk
            .copy_from_slice(&*PAIR_CONFIG_MESH_LTK.lock());
    }

    fn get_online_node_cnt(&mut self) -> u8 {
        let mesh_node_st = MESH_NODE_ST.lock();

        let mut cnt = 0;
        for i in 0..MESH_NODE_MAX.get() {
            if mesh_node_st[i as usize].tick != 0 {
                cnt += 1;
            }
        }

        cnt
    }

    pub fn mesh_pair_proc(&mut self) {
        let mut dst_addr = 0xFFFF;
        let mut op_para: [u8; 16] = [0; 16];

        if self.default_mesh_effect_delay_ref != 0
            && clock_time_exceed(
                self.default_mesh_effect_delay_ref,
                MESH_PAIR_CMD_INTERVAL * 1000,
            )
        {
            self.default_mesh_effect_delay_ref = 0;

            if self.default_mesh_time == 0x00 {
                pair_load_key();
                self.default_mesh_time_ref = 0;
            } else {
                self.switch_to_default_mesh((self.default_mesh_time / 1000) as u8);
                self.default_mesh_time_ref = clock_time() | 1;
            }
        } else if self.default_mesh_time_ref != 0
            && clock_time_exceed(self.default_mesh_time_ref, self.default_mesh_time * 1000)
        {
            /* Switch to normal mesh */
            if self.default_mesh_time == 255000 {
                self.default_mesh_time_ref = clock_time() | 1;
            } else {
                pair_load_key();
                self.default_mesh_time_ref = 0;
            }
        }
        if self.mesh_pair_start_time != 0
            && clock_time_exceed(
                self.mesh_pair_start_time,
                self.mesh_pair_timeout * 1000 * 1000,
            )
        {
            //mesh pair time out
            pair_load_key();
            *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
            rf_link_light_event_callback(LGT_CMD_MESH_PAIR_TIMEOUT);
            return;
        }

        // TODO REMOVE THIS FALSE
        if false
            && *PAIR_SETTING_FLAG.lock() == ePairState::PairSetMeshTxStart
            && self.mesh_pair_state == MeshPairState::MeshPairName1
            && self.get_online_node_cnt() == 1
        {
            op_para[0] = LGT_CMD_MESH_PAIR;
            op_para[3] = MeshPairState::MeshPairEffect as u8;
            dst_addr = 0x0000; // there is noly one device in mesh,just effect itself.
            self.mesh_pair_state = MeshPairState::MeshPairName1;
            self.mesh_pair_start_time = 0;
            *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
        } else if *PAIR_SETTING_FLAG.lock() as u8 >= ePairState::PairSetMeshTxStart as u8
            && clock_time_exceed(self.mesh_pair_time, self.mesh_pair_cmd_interval * 1000)
        {
            self.mesh_pair_time = clock_time();
            if *PAIR_SETTING_FLAG.lock() == ePairState::PairSetMeshTxStart {
                let pair_state = PAIR_STATE.lock();

                op_para[0] = LGT_CMD_MESH_PAIR;
                op_para[3] = self.mesh_pair_state as u8;
                match self.mesh_pair_state {
                    MeshPairState::MeshPairName1 => {
                        // send mesh name [0-7]
                        op_para[4..4 + 8].copy_from_slice(&pair_state.pair_nn[0..8]);
                        self.mesh_pair_state = MeshPairState::MeshPairName2;
                    }
                    MeshPairState::MeshPairName2 => {
                        // send mesh name [8-15]
                        op_para[4..4 + 8].copy_from_slice(&pair_state.pair_nn[8..16]);
                        self.mesh_pair_state = MeshPairState::MeshPairPwd1;
                    }
                    MeshPairState::MeshPairPwd1 => {
                        // send mesh pwd [0-7]
                        op_para[4..4 + 8].copy_from_slice(&pair_state.pair_pass[0..8]);
                        self.mesh_pair_state = MeshPairState::MeshPairPwd2;
                    }
                    MeshPairState::MeshPairPwd2 => {
                        // send mesh pwd [8-15]
                        op_para[4..4 + 8].copy_from_slice(&pair_state.pair_pass[8..16]);
                        self.mesh_pair_state = MeshPairState::MeshPairLtk1;
                    }
                    MeshPairState::MeshPairLtk1 => {
                        // send mesh ltk [0-7]
                        op_para[4..4 + 8].copy_from_slice(&pair_state.pair_ltk_mesh[0..8]);
                        self.mesh_pair_state = MeshPairState::MeshPairLtk2;
                    }
                    MeshPairState::MeshPairLtk2 => {
                        // send mesh ltk [8-15]
                        op_para[4..4 + 8].copy_from_slice(&pair_state.pair_ltk_mesh[8..16]);
                        self.mesh_pair_state = MeshPairState::MeshPairName1;
                        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshTxDone;
                    }
                    _ => {
                        self.mesh_pair_state = MeshPairState::MeshPairName1;
                        self.mesh_pair_start_time = 0;
                        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
                        return;
                    }
                }
            } else if *PAIR_SETTING_FLAG.lock() == ePairState::PairSetMeshTxDone {
                // get mesh nodes' confirm value
                op_para[0] = 0;
                op_para[3] = 0x10; // bridge cnt
                op_para[4] = PAR_READ_MESH_PAIR_CONFIRM;
                *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshRxDone;
            } else if *PAIR_SETTING_FLAG.lock() == ePairState::PairSetMeshRxDone {
                //send cmd to switch to new mesh
                op_para[0] = LGT_CMD_MESH_PAIR;
                op_para[3] = MeshPairState::MeshPairEffectDelay as u8;
                self.mesh_pair_state = MeshPairState::MeshPairName1;
                self.mesh_pair_start_time = 0;
                *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
            }
        } else {
            return;
        }

        let pkt = light_slave_tx_command(
            &Vec::from_slice(&op_para[..13]).unwrap(),
            dst_addr,
            0,
            false,
        );
        let (group_match, device_match) = rf_link_match_group_mac(&pkt);
        if group_match || device_match {
            app().mesh_manager.add_rcv_mesh_msg(&pkt);
            if !device_match {
                self.add_send_mesh_msg(
                    &pkt,
                    0,
                    pkt.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
                );
            }
        } else {
            self.add_send_mesh_msg(
                &pkt,
                0,
                pkt.mesh().internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT],
            );
        }
    }

    pub fn mesh_node_buf_init(&self) {
        MESH_NODE_ST.lock().fill(MeshNodeStT {
            tick: 0,
            val: MeshNodeStValT {
                dev_adr: 0,
                sn: 0,
                par: [0; MESH_NODE_ST_PAR_LEN],
            },
        });

        app().light_manager.device_status_update();
    }

    pub fn mesh_security_enable(&self, enable: bool) {
        SECURITY_ENABLE.set(enable);
    }

    pub fn add_send_mesh_msg(&mut self, packet: &Packet, delay: u64, send_count: u8) {
        // If we sent the message, and we are sending the message back to ourselves - just report the packet over uart
        if packet.ll_app().value.src == DEVICE_ADDRESS.get()
            && packet.ll_app().value.dst == DEVICE_ADDRESS.get()
        {
            light_mesh_rx_cb(packet);
        } else if self
            .pkt_send_buf
            .push(SendPkt {
                delay,
                pkt: *packet,
                send_count,
            })
            .is_err()
        {
            uprintln!("pkt send buf is full, dropping packet...");
        }
    }

    /// Process a single iteration of the mesh message sending task
    async fn send_mesh_msg_iteration(&mut self) {
        yield_now().await;

        let result = critical_section::with(|_| {
            let found = self
                .pkt_send_buf
                .iter()
                .enumerate()
                .find(|(_, elem)| clock_time64() > elem.delay);

            let (index, _) = found?;

            let pkt = self.pkt_send_buf.swap_remove(index);

            Some(pkt)
        });

        if result.is_none() {
            return;
        }

        let result = result.unwrap();
        let mut pkt = result.pkt;

        pkt.mesh_mut().src_tx = DEVICE_ADDRESS.get();
        pkt.mesh_mut().handle1 = 0;

        // Encrypt the packet if required
        if SECURITY_ENABLE.get() {
            pkt.head_mut()._type |= BIT!(7);
            pair_enc_packet_mesh(&mut pkt);
        }

        // Pick a random start channel
        let start_chan_idx = (read_reg_system_tick() as u16 ^ read_reg_rnd_number()) as usize;

        // Send the packet on each channel
        for channel_index in start_chan_idx..start_chan_idx + SYS_CHN_LISTEN.len() {
            // This block needs to operate in a critical section
            critical_section::with(|_| {
                // Configure the BLE mode
                rf_set_tx_rx_off();
                rf_set_ble_access_code(PAIR_AC.get());
                rf_set_ble_crc_adv();

                rf_set_ble_channel(SYS_CHN_LISTEN[channel_index % SYS_CHN_LISTEN.len()]);
                rf_start_srx2tx(
                    addr_of!(pkt) as u32,
                    read_reg_system_tick() + CLOCK_SYS_CLOCK_1US * 30,
                );
            });

            #[cfg(not(test))]
            Timer::after(Duration::from_micros(600)).await;

            #[cfg(test)]
            {
                // In test builds, skip the timer delay
                yield_now().await;
            }
        }

        rf_set_rxmode();

        if result.send_count != 0 {
            // Random delay to avoid congestion between 0us and 8ms
            let delay =
                8000 - (((read_reg_system_tick() as u16 ^ read_reg_rnd_number()) % 16) * 500);

            self.add_send_mesh_msg(
                &result.pkt,
                clock_time64() + (delay as u64 * CLOCK_SYS_CLOCK_1US as u64),
                result.send_count - 1,
            );
        }
    }

    pub async fn send_mesh_msg_task(&mut self) {
        #[cfg(not(test))]
        loop {
            self.send_mesh_msg_iteration().await;
        }

        #[cfg(test)]
        {
            // In test builds, just run one iteration to allow testing
            self.send_mesh_msg_iteration().await;
        }
    }

    pub fn add_rcv_mesh_msg(&mut self, packet: &Packet) {
        if self.pkt_rcv_buf.push_back(*packet).is_err() {
            uprintln!("pkt rcv buf is full, dropping packet...");
        }
    }

    /// Process a single iteration of the mesh message receiving task
    async fn rcv_mesh_msg_iteration(&mut self) {
        yield_now().await;

        if !self.pkt_rcv_buf.is_empty() {
            let mut result = critical_section::with(|_| self.pkt_rcv_buf.pop_front().unwrap());

            rf_link_rc_data(&mut result);
        }
    }

    pub async fn rcv_mesh_msg_task(&mut self) {
        #[cfg(not(test))]
        loop {
            self.rcv_mesh_msg_iteration().await;
        }

        #[cfg(test)]
        {
            // In test builds, just run one iteration to allow testing
            self.rcv_mesh_msg_iteration().await;
        }
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::embassy::time_driver::mock_clock_time64;
    use crate::sdk::ble_app::ble_ll_pair::mock_pair_save_key;
    use crate::sdk::ble_app::light_ll::mesh_management::mock_rf_link_match_group_mac;
    use crate::sdk::ble_app::light_ll::packet_processing::{
        mock_is_add_packet_buf_ready, mock_rf_link_add_tx_packet, mock_rf_link_rc_data,
    };
    use crate::sdk::ble_app::rf_drv_8266::{
        mock_rf_set_ble_access_code, mock_rf_set_ble_channel, mock_rf_set_ble_crc_adv,
        mock_rf_set_rxmode, mock_rf_set_tx_rx_off, mock_rf_start_srx2tx,
    };
    use crate::sdk::drivers::flash::{mock_flash_read_page, mock_flash_write_page};
    use crate::sdk::light::{ePairState, LGT_CMD_MESH_PAIR_TIMEOUT, LGT_CMD_SET_MESH_INFO};
    use crate::sdk::mcu::clock::{mock_clock_time, mock_clock_time_exceed, mock_sleep_us};
    use crate::sdk::mcu::register::{
        mock_read_reg_rnd_number, mock_read_reg_system_tick, mock_write_reg_rf_irq_status,
    };
    use crate::{app_mocker, mock_app_mocker, App};
    use heapless::Vec;
    use mry::Any;

    use crate::common::{mock_access_code, mock_mesh_node_init, mock_pair_load_key};
    use crate::main_light::{mock_light_slave_tx_command, mock_rf_link_light_event_callback};
    use crate::uart_manager::mock_light_mesh_rx_cb;

    /// Helper function to create a test packet with mesh data
    fn create_test_packet(src: u16, dst: u16, sno: [u8; 3]) -> Packet {
        Packet {
            att_cmd: PacketAttCmd {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 0,
                    rf_len: 0,
                    l2cap_len: 0,
                    chan_id: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: PacketAttValue {
                    sno,
                    src: src.to_le_bytes(),
                    dst: dst.to_le_bytes(),
                    val: [0; 23],
                },
            },
        }
    }

    // --- TryFrom Tests ---

    /// Tests MeshPairState::try_from with all valid enum values
    #[test]
    fn test_mesh_pair_state_try_from_valid() {
        assert_eq!(
            MeshPairState::try_from(0).unwrap(),
            MeshPairState::MeshPairName1
        );
        assert_eq!(
            MeshPairState::try_from(1).unwrap(),
            MeshPairState::MeshPairName2
        );
        assert_eq!(
            MeshPairState::try_from(2).unwrap(),
            MeshPairState::MeshPairPwd1
        );
        assert_eq!(
            MeshPairState::try_from(3).unwrap(),
            MeshPairState::MeshPairPwd2
        );
        assert_eq!(
            MeshPairState::try_from(4).unwrap(),
            MeshPairState::MeshPairLtk1
        );
        assert_eq!(
            MeshPairState::try_from(5).unwrap(),
            MeshPairState::MeshPairLtk2
        );
        assert_eq!(
            MeshPairState::try_from(6).unwrap(),
            MeshPairState::MeshPairEffectDelay
        );
        assert_eq!(
            MeshPairState::try_from(7).unwrap(),
            MeshPairState::MeshPairEffect
        );
        assert_eq!(
            MeshPairState::try_from(8).unwrap(),
            MeshPairState::MeshPairDefaultMesh
        );
    }

    /// Tests MeshPairState::try_from with invalid value
    #[test]
    fn test_mesh_pair_state_try_from_invalid() {
        assert!(MeshPairState::try_from(9).is_err());
        assert!(MeshPairState::try_from(255).is_err());
    }

    // --- send_mesh_message Tests ---

    /// Tests send_mesh_message with group match
    #[test]
    #[mry::lock(light_slave_tx_command, rf_link_match_group_mac, app_mocker)]
    fn test_send_mesh_message_group_match() {
        // Setup
        let mut app = App::default();
        let mesh_manager = MeshManager::default();
        app.mesh_manager = mesh_manager;
        mock_app_mocker().returns(&mut app);

        let test_data: Vec<u8, 13> = Vec::from_slice(&[1, 2, 3, 4, 5]).unwrap();
        let test_pkt = create_test_packet(0x1234, 0xFFFF, [10, 20, 30]);

        mock_light_slave_tx_command(Any, 0xFFFF, 3, true).returns(test_pkt);
        mock_rf_link_match_group_mac(Any).returns((true, false)); // group match, not device match

        // Execute
        let result = app
            .mesh_manager
            .send_mesh_message(&test_data, 0xFFFF, 3, true);

        // Verify
        assert_eq!(result, [10, 20, 30]);
        mock_light_slave_tx_command(Any, 0xFFFF, 3, true).assert_called(1);
        mock_rf_link_match_group_mac(Any).assert_called(1);
    }

    /// Tests send_mesh_message with device match
    #[test]
    #[mry::lock(light_slave_tx_command, rf_link_match_group_mac, app_mocker)]
    fn test_send_mesh_message_device_match() {
        // Setup
        let mut app = App::default();
        let mesh_manager = MeshManager::default();
        app.mesh_manager = mesh_manager;
        mock_app_mocker().returns(&mut app);

        let test_data: Vec<u8, 13> = Vec::from_slice(&[1, 2, 3]).unwrap();
        let test_pkt = create_test_packet(0x0001, 0x0001, [5, 6, 7]);

        mock_light_slave_tx_command(Any, 0x0001, 0, false).returns(test_pkt);
        mock_rf_link_match_group_mac(Any).returns((false, true)); // device match

        // Execute
        let result = app
            .mesh_manager
            .send_mesh_message(&test_data, 0x0001, 0, false);

        // Verify - device match means no retransmission
        assert_eq!(result, [5, 6, 7]);
        mock_rf_link_match_group_mac(Any).assert_called(1);
    }

    /// Tests send_mesh_message with no match
    #[test]
    #[mry::lock(light_slave_tx_command, rf_link_match_group_mac, app_mocker)]
    fn test_send_mesh_message_no_match() {
        // Setup
        let mut app = App::default();
        let mesh_manager = MeshManager::default();
        app.mesh_manager = mesh_manager;
        mock_app_mocker().returns(&mut app);

        let test_data: Vec<u8, 13> = Vec::from_slice(&[1, 2, 3]).unwrap();
        let test_pkt = create_test_packet(0x0002, 0x0003, [8, 9, 10]);

        mock_light_slave_tx_command(Any, 0x0003, 2, false).returns(test_pkt);
        mock_rf_link_match_group_mac(Any).returns((false, false)); // no match

        // Execute
        let result = app
            .mesh_manager
            .send_mesh_message(&test_data, 0x0003, 2, false);

        // Verify
        assert_eq!(result, [8, 9, 10]);
        mock_rf_link_match_group_mac(Any).assert_called(1);
    }

    // --- mesh_pair_init Tests ---

    /// Tests mesh_pair_init sets proper values
    #[test]
    fn test_mesh_pair_init() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        MESH_PAIR_ENABLE.set(false);

        // Execute
        mesh_manager.mesh_pair_init();

        // Verify
        assert!(MESH_PAIR_ENABLE.get());
        assert_eq!(mesh_manager.mesh_pair_cmd_interval, MESH_PAIR_CMD_INTERVAL);
        assert_eq!(mesh_manager.mesh_pair_timeout, MESH_PAIR_TIMEOUT);
    }

    // --- mesh_pair_proc_effect Tests ---

    /// Tests mesh_pair_proc_effect with effect_new_mesh set
    #[test]
    #[mry::lock(
        sleep_us,
        pair_save_key,
        rf_set_ble_access_code,
        rf_link_light_event_callback,
        write_reg_rf_irq_status,
        is_add_packet_buf_ready,
        rf_link_add_tx_packet,
        app_mocker
    )]
    fn test_mesh_pair_proc_effect_with_effect_new_mesh() {
        // Setup
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);
        MESH_PAIR_ENABLE.set(false);
        PAIR_AC.set(0x12345678);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x0001);

        let mut app = App::default();
        let mut mesh_manager = MeshManager::default();
        mesh_manager.effect_new_mesh = 1;
        app.mesh_manager = mesh_manager;
        app.light_manager.mock_device_status_update().returns(());
        mock_app_mocker().returns(&mut app);

        mock_pair_save_key().returns(());
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO).returns(());
        mock_sleep_us(1000).returns(());
        mock_write_reg_rf_irq_status(Any).returns(());
        mock_is_add_packet_buf_ready().returns(true);
        mock_rf_link_add_tx_packet(Any).returns(true);

        // Execute
        app.mesh_manager.mesh_pair_proc_effect();

        // Verify
        assert_eq!(app.mesh_manager.effect_new_mesh, 0);
        assert_eq!(app.mesh_manager.effect_new_mesh_delay_time, 0);
        mock_pair_save_key().assert_called(1);
    }

    /// Tests mesh_pair_proc_effect with delay time exceeded
    #[test]
    #[mry::lock(
        clock_time,
        clock_time_exceed,
        pair_load_key,
        pair_save_key,
        is_add_packet_buf_ready,
        rf_link_add_tx_packet,
        rf_set_ble_access_code,
        rf_link_light_event_callback,
        write_reg_rf_irq_status
    )]
    fn test_mesh_pair_proc_effect_with_delay_exceeded() {
        // Setup
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);
        MESH_PAIR_ENABLE.set(false);
        PAIR_AC.set(0x12345678);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x0001);

        let mut mesh_manager = MeshManager::default();
        mesh_manager.effect_new_mesh = 0;
        mesh_manager.effect_new_mesh_delay_time = 1000;
        mesh_manager.mesh_pair_cmd_interval = 500;

        mock_clock_time().returns(0);
        mock_clock_time_exceed(Any, Any).returns(true); // For sleep_us
        mock_pair_load_key().returns(());
        mock_pair_save_key().returns(());
        mock_is_add_packet_buf_ready().returns(true);
        mock_rf_link_add_tx_packet(Any).returns(true);
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_link_light_event_callback(Any).returns(());
        mock_write_reg_rf_irq_status(Any).returns(());

        // Execute
        mesh_manager.mesh_pair_proc_effect();

        // Verify
        assert_eq!(mesh_manager.effect_new_mesh_delay_time, 0);
        mock_clock_time_exceed(Any, Any).assert_called(2);
    }

    /// Tests mesh_pair_proc_effect with no effect needed
    #[test]
    #[mry::lock(clock_time_exceed)]
    fn test_mesh_pair_proc_effect_no_effect() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        mesh_manager.effect_new_mesh = 0;
        mesh_manager.effect_new_mesh_delay_time = 1000;
        mesh_manager.mesh_pair_cmd_interval = 500;

        mock_clock_time_exceed(1000, 500 * 1000).returns(false);

        // Execute
        mesh_manager.mesh_pair_proc_effect();

        // Verify - nothing should change
        assert_eq!(mesh_manager.effect_new_mesh_delay_time, 1000);
        mock_clock_time_exceed(1000, 500 * 1000).assert_called(1);
    }

    // --- mesh_device_address_validation_completed Tests ---

    /// Tests mesh_device_address_validation_completed with mesh pair enabled
    #[test]
    #[mry::lock(flash_write_page)]
    fn test_mesh_device_address_validation_completed_pair_enabled() {
        // Setup
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(true);
        MESH_PAIR_ENABLE.set(true);
        FLASH_CONFIGURATION_INDEX.set(10);

        let mut mesh_manager = MeshManager::default();
        mock_flash_write_page((FLASH_ADR_PAIRING as i32 + 10 + 1) as u32, 1, Any).returns(());

        // Execute
        mesh_manager.mesh_device_address_validation_completed();

        // Verify
        assert!(!MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get());
        mock_flash_write_page((FLASH_ADR_PAIRING as i32 + 10 + 1) as u32, 1, Any).assert_called(1);
    }

    /// Tests mesh_device_address_validation_completed without mesh pair
    #[test]
    fn test_mesh_device_address_validation_completed_pair_disabled() {
        // Setup
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(true);
        MESH_PAIR_ENABLE.set(false);

        let mut mesh_manager = MeshManager::default();

        // Execute
        mesh_manager.mesh_device_address_validation_completed();

        // Verify
        assert!(!MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get());
    }

    // --- mesh_pair_cb Tests ---

    /// Tests mesh_pair_cb with MeshPairName1
    #[test]
    #[mry::lock(clock_time)]
    fn test_mesh_pair_cb_name1() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        let params: Vec<u8, 10> = Vec::from_slice(&[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]).unwrap();
        mock_clock_time().returns(1000);

        // Execute
        mesh_manager.mesh_pair_cb(&params);

        // Verify
        assert_eq!(mesh_manager.mesh_pair_start_time, 1001);
        assert_eq!(&mesh_manager.new_mesh_name[0..8], &[1, 2, 3, 4, 5, 6, 7, 8]);
    }

    /// Tests mesh_pair_cb with MeshPairName2
    #[test]
    fn test_mesh_pair_cb_name2() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        let params: Vec<u8, 10> = Vec::from_slice(&[1, 11, 12, 13, 14, 15, 16, 17, 18]).unwrap();

        // Execute
        mesh_manager.mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &mesh_manager.new_mesh_name[8..16],
            &[11, 12, 13, 14, 15, 16, 17, 18]
        );
    }

    /// Tests mesh_pair_cb with MeshPairPwd1
    #[test]
    fn test_mesh_pair_cb_pwd1() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        let params: Vec<u8, 10> = Vec::from_slice(&[2, 21, 22, 23, 24, 25, 26, 27, 28]).unwrap();

        // Execute
        mesh_manager.mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &mesh_manager.new_mesh_pwd[0..8],
            &[21, 22, 23, 24, 25, 26, 27, 28]
        );
    }

    /// Tests mesh_pair_cb with MeshPairPwd2
    #[test]
    fn test_mesh_pair_cb_pwd2() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        let params: Vec<u8, 10> = Vec::from_slice(&[3, 31, 32, 33, 34, 35, 36, 37, 38]).unwrap();

        // Execute
        mesh_manager.mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &mesh_manager.new_mesh_pwd[8..16],
            &[31, 32, 33, 34, 35, 36, 37, 38]
        );
    }

    /// Tests mesh_pair_cb with MeshPairLtk1
    #[test]
    fn test_mesh_pair_cb_ltk1() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        let params: Vec<u8, 10> = Vec::from_slice(&[4, 41, 42, 43, 44, 45, 46, 47, 48]).unwrap();

        // Execute
        mesh_manager.mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &mesh_manager.new_mesh_ltk[0..8],
            &[41, 42, 43, 44, 45, 46, 47, 48]
        );
    }

    /// Tests mesh_pair_cb with MeshPairLtk2
    #[test]
    fn test_mesh_pair_cb_ltk2() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        let params: Vec<u8, 10> = Vec::from_slice(&[5, 51, 52, 53, 54, 55, 56, 57, 58]).unwrap();

        // Execute
        mesh_manager.mesh_pair_cb(&params);

        // Verify
        assert_eq!(
            &mesh_manager.new_mesh_ltk[8..16],
            &[51, 52, 53, 54, 55, 56, 57, 58]
        );
    }

    /// Tests mesh_pair_cb with MeshPairEffectDelay
    #[test]
    #[mry::lock(clock_time)]
    fn test_mesh_pair_cb_effect_delay() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        mesh_manager.default_mesh_time_ref = 500;
        mesh_manager.mesh_pair_cmd_interval = 250;
        let params: Vec<u8, 10> = Vec::from_slice(&[6, 0, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        mock_clock_time().returns(2000);

        // Execute
        mesh_manager.mesh_pair_cb(&params);

        // Verify
        assert_eq!(mesh_manager.effect_new_mesh_delay_time, 2001);
        assert_eq!(mesh_manager.default_mesh_time, 500); // 250 * 2
    }

    /// Tests mesh_pair_cb with MeshPairEffect
    #[test]
    fn test_mesh_pair_cb_effect() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        let params: Vec<u8, 10> = Vec::from_slice(&[7, 0, 0, 0, 0, 0, 0, 0, 0]).unwrap();

        // Execute
        mesh_manager.mesh_pair_cb(&params);

        // Verify
        assert_eq!(mesh_manager.effect_new_mesh, 1);
    }

    /// Tests mesh_pair_cb with MeshPairDefaultMesh
    #[test]
    #[mry::lock(clock_time)]
    fn test_mesh_pair_cb_default_mesh() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        let params: Vec<u8, 10> = Vec::from_slice(&[8, 5, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        mock_clock_time().returns(3000);

        // Execute
        mesh_manager.mesh_pair_cb(&params);

        // Verify
        assert_eq!(mesh_manager.default_mesh_effect_delay_ref, 3001);
        assert_eq!(mesh_manager.default_mesh_time, 5000); // 5 * 1000
    }

    /// Tests mesh_pair_cb with invalid command
    #[test]
    fn test_mesh_pair_cb_invalid_command() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        let params: Vec<u8, 10> = Vec::from_slice(&[99, 0, 0, 0, 0, 0, 0, 0, 0]).unwrap();

        // Execute - should return early without panic
        mesh_manager.mesh_pair_cb(&params);

        // Verify - no changes should occur
        assert_eq!(mesh_manager.mesh_pair_start_time, 0);
    }

    /// Tests mesh_pair_cb updates default_mesh_time_ref when it's set
    #[test]
    #[mry::lock(clock_time)]
    fn test_mesh_pair_cb_updates_default_mesh_time_ref() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        mesh_manager.default_mesh_time_ref = 100;
        let params: Vec<u8, 10> = Vec::from_slice(&[1, 1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        mock_clock_time().returns(5000);

        // Execute
        mesh_manager.mesh_pair_cb(&params);

        // Verify
        assert_eq!(mesh_manager.default_mesh_time_ref, 5001);
    }

    // --- mesh_cmd_notify Tests ---

    /// Tests mesh_cmd_notify when BLE is not connected
    #[test]
    fn test_mesh_cmd_notify_ble_not_connected() {
        // Setup
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(false);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x1234);

        let mesh_manager = MeshManager::default();
        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3]).unwrap();

        // Execute
        let result = mesh_manager.mesh_cmd_notify(0x10, &params, 0x1234);

        // Verify
        assert_eq!(result, -1);
    }

    /// Tests mesh_cmd_notify when pair login is not ok
    #[test]
    fn test_mesh_cmd_notify_pair_login_not_ok() {
        // Setup
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(false);
        DEVICE_ADDRESS.set(0x1234);

        let mesh_manager = MeshManager::default();
        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3]).unwrap();

        // Execute
        let result = mesh_manager.mesh_cmd_notify(0x10, &params, 0x1234);

        // Verify
        assert_eq!(result, -1);
    }

    /// Tests mesh_cmd_notify with params too long
    /// Note: Vec<u8, 10> can't actually exceed 10 elements, but this documents the behavior
    #[test]
    #[mry::lock(is_add_packet_buf_ready)]
    fn test_mesh_cmd_notify_params_too_long() {
        // Setup
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x1234);

        let mesh_manager = MeshManager::default();
        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]).unwrap();
        // The function checks p.len() > 10, which is impossible with Vec<u8, 10>
        // This test documents that the capacity constraint prevents overlong params

        mock_is_add_packet_buf_ready().returns(false);

        // Execute
        let _result = mesh_manager.mesh_cmd_notify(0x10, &params, 0x1234);

        // With Vec<u8, 10>, the length can never exceed 10, so the early return is unreachable
    }

    /// Tests mesh_cmd_notify with successful notification
    #[test]
    #[mry::lock(is_add_packet_buf_ready, rf_link_add_tx_packet)]
    fn test_mesh_cmd_notify_success() {
        // Setup
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x1234);

        let mesh_manager = MeshManager::default();
        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3]).unwrap();

        mock_is_add_packet_buf_ready().returns(true);
        mock_rf_link_add_tx_packet(Any).returns(true);

        // Execute
        let result = mesh_manager.mesh_cmd_notify(0x10, &params, 0x1234);

        // Verify
        assert_eq!(result, 0);
        mock_is_add_packet_buf_ready().assert_called(1);
        mock_rf_link_add_tx_packet(Any).assert_called(1);
    }

    /// Tests mesh_cmd_notify when buffer is not ready
    #[test]
    #[mry::lock(is_add_packet_buf_ready)]
    fn test_mesh_cmd_notify_buffer_not_ready() {
        // Setup
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x1234);

        let mesh_manager = MeshManager::default();
        let params: Vec<u8, 10> = Vec::from_slice(&[1, 2, 3]).unwrap();

        mock_is_add_packet_buf_ready().returns(false);

        // Execute
        let result = mesh_manager.mesh_cmd_notify(0x10, &params, 0x1234);

        // Verify
        assert_eq!(result, -1);
        mock_is_add_packet_buf_ready().assert_called(1);
    }

    // --- mesh_pair_complete_notify Tests ---

    /// Tests mesh_pair_complete_notify
    #[test]
    #[mry::lock(is_add_packet_buf_ready, rf_link_add_tx_packet)]
    fn test_mesh_pair_complete_notify() {
        // Setup
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x5678);

        let mesh_manager = MeshManager::default();

        mock_is_add_packet_buf_ready().returns(true);
        mock_rf_link_add_tx_packet(Any).returns(true);

        // Execute
        let result = mesh_manager.mesh_pair_complete_notify();

        // Verify
        assert_eq!(result, 0);
        mock_is_add_packet_buf_ready().assert_called(1);
        mock_rf_link_add_tx_packet(Any).assert_called(1);
    }

    // --- safe_effect_new_mesh_finish Tests ---

    /// Tests safe_effect_new_mesh_finish clears state
    #[test]
    fn test_safe_effect_new_mesh_finish() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        mesh_manager.new_mesh_name = [1; 16];
        mesh_manager.new_mesh_pwd = [2; 16];
        mesh_manager.new_mesh_ltk = [3; 16];
        mesh_manager.mesh_pair_start_time = 1000;
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetting;

        // Execute
        mesh_manager.safe_effect_new_mesh_finish();

        // Verify
        assert_eq!(mesh_manager.new_mesh_name, [0; 16]);
        assert_eq!(mesh_manager.new_mesh_pwd, [0; 16]);
        assert_eq!(mesh_manager.new_mesh_ltk, [0; 16]);
        assert_eq!(mesh_manager.mesh_pair_start_time, 0);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), ePairState::PairSetted);
    }

    // --- save_effect_new_mesh Tests ---

    /// Tests save_effect_new_mesh with default mesh time ref set
    #[test]
    #[mry::lock(
        is_add_packet_buf_ready,
        rf_link_add_tx_packet,
        sleep_us,
        pair_load_key,
        mesh_node_init,
        app_mocker
    )]
    fn test_save_effect_new_mesh_default_mesh_active() {
        // Setup
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x0001);

        let mut app = App::default();
        let mut mesh_manager = MeshManager::default();
        mesh_manager.default_mesh_time_ref = 1000; // Set to non-zero
        app.mesh_manager = mesh_manager;
        app.light_manager.mock_device_status_update().returns(());
        mock_app_mocker().returns(&mut app);

        mock_is_add_packet_buf_ready().returns(true);
        mock_rf_link_add_tx_packet(Any).returns(true);
        mock_sleep_us(1000).returns(());
        mock_pair_load_key().returns(());
        mock_mesh_node_init().returns(());

        // Execute
        app.mesh_manager.save_effect_new_mesh();

        // Verify
        assert_eq!(app.mesh_manager.default_mesh_time_ref, 0);
        assert_eq!(app.mesh_manager.mesh_pair_start_time, 0);
        mock_pair_load_key().assert_called(1);
        mock_mesh_node_init().assert_called(1);
    }

    /// Tests save_effect_new_mesh with validation pending
    #[test]
    #[mry::lock(
        is_add_packet_buf_ready,
        rf_link_add_tx_packet,
        sleep_us,
        pair_load_key,
        mesh_node_init,
        app_mocker
    )]
    fn test_save_effect_new_mesh_validation_pending() {
        // Setup
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(true);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x0001);

        let mut app = App::default();
        let mut mesh_manager = MeshManager::default();
        mesh_manager.default_mesh_time_ref = 0;
        app.mesh_manager = mesh_manager;
        app.light_manager.mock_device_status_update().returns(());
        mock_app_mocker().returns(&mut app);

        mock_is_add_packet_buf_ready().returns(true);
        mock_rf_link_add_tx_packet(Any).returns(true);
        mock_sleep_us(1000).returns(());
        mock_pair_load_key().returns(());
        mock_mesh_node_init().returns(());

        // Execute
        app.mesh_manager.save_effect_new_mesh();

        // Verify
        mock_pair_load_key().assert_called(1);
        mock_mesh_node_init().assert_called(1);
    }

    /// Tests save_effect_new_mesh normal save path
    #[test]
    #[mry::lock(
        is_add_packet_buf_ready,
        rf_link_add_tx_packet,
        pair_save_key,
        rf_set_ble_access_code,
        rf_link_light_event_callback,
        sleep_us,
        write_reg_rf_irq_status
    )]
    fn test_save_effect_new_mesh_normal_save() {
        // Setup
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.set(false);
        BLE_PERIPHERAL_CONNECTION_ACTIVE.set(true);
        PAIR_LOGIN_OK.set(true);
        DEVICE_ADDRESS.set(0x0001);
        PAIR_AC.set(0xABCDEF12);

        let mut mesh_manager = MeshManager::default();
        mesh_manager.default_mesh_time_ref = 0;
        mesh_manager.effect_new_mesh = 0;
        mesh_manager.new_mesh_name = [1; 16];
        mesh_manager.new_mesh_pwd = [2; 16];
        mesh_manager.new_mesh_ltk = [3; 16];

        mock_is_add_packet_buf_ready().returns(true);
        mock_rf_link_add_tx_packet(Any).returns(true);
        mock_pair_save_key().returns(());
        mock_rf_set_ble_access_code(0xABCDEF12).returns(());
        mock_rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO).returns(());
        mock_sleep_us(1000).returns(());
        mock_write_reg_rf_irq_status(Any).returns(());

        // Execute
        mesh_manager.save_effect_new_mesh();

        // Verify
        assert_eq!(mesh_manager.mesh_pair_start_time, 0);
        mock_pair_save_key().assert_called(1);
        mock_rf_set_ble_access_code(0xABCDEF12).assert_called(1);
        mock_rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO).assert_called(1);
    }

    // --- switch_to_default_mesh Tests ---

    /// Tests switch_to_default_mesh
    #[test]
    #[mry::lock(clock_time, access_code)]
    fn test_switch_to_default_mesh() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        let test_name = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
        let test_pwd = [
            17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
        ];
        let test_ltk = [
            33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
        ];

        *PAIR_CONFIG_MESH_NAME.lock() = test_name;
        *PAIR_CONFIG_MESH_PWD.lock() = test_pwd;
        *PAIR_CONFIG_MESH_LTK.lock() = test_ltk;

        mock_clock_time().returns(5000);
        mock_access_code(Any, Any).returns(0xDEADBEEF);

        // Execute
        mesh_manager.switch_to_default_mesh(10);

        // Verify
        assert_eq!(mesh_manager.default_mesh_time_ref, 5001);
        assert_eq!(mesh_manager.default_mesh_time, 10000);
        assert_eq!(PAIR_AC.get(), 0xDEADBEEF);
        assert_eq!(PAIR_STATE.lock().pair_ltk, test_ltk);
        mock_access_code(Any, Any).assert_called(1);
    }

    // --- get_online_node_cnt Tests ---

    /// Tests get_online_node_cnt with no online nodes
    #[test]
    fn test_get_online_node_cnt_zero() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        MESH_NODE_MAX.set(5);

        let mut mesh_node_st = MESH_NODE_ST.lock();
        for i in 0..5 {
            mesh_node_st[i] = MeshNodeStT {
                tick: 0,
                val: MeshNodeStValT {
                    dev_adr: 0,
                    sn: 0,
                    par: [0; MESH_NODE_ST_PAR_LEN],
                },
            };
        }
        drop(mesh_node_st);

        // Execute
        let count = mesh_manager.get_online_node_cnt();

        // Verify
        assert_eq!(count, 0);
    }

    /// Tests get_online_node_cnt with some online nodes
    #[test]
    fn test_get_online_node_cnt_some_online() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        MESH_NODE_MAX.set(5);

        let mut mesh_node_st = MESH_NODE_ST.lock();
        mesh_node_st[0].tick = 100;
        mesh_node_st[1].tick = 0;
        mesh_node_st[2].tick = 200;
        mesh_node_st[3].tick = 0;
        mesh_node_st[4].tick = 300;
        drop(mesh_node_st);

        // Execute
        let count = mesh_manager.get_online_node_cnt();

        // Verify
        assert_eq!(count, 3);
    }

    // --- mesh_pair_proc Tests ---

    /// Tests mesh_pair_proc with timeout
    #[test]
    #[mry::lock(clock_time_exceed, pair_load_key, rf_link_light_event_callback)]
    fn test_mesh_pair_proc_timeout() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        mesh_manager.mesh_pair_start_time = 1000;
        mesh_manager.mesh_pair_timeout = 10;
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetting;

        mock_clock_time_exceed(1000, 10 * 1000 * 1000).returns(true);
        mock_pair_load_key().returns(());
        mock_rf_link_light_event_callback(LGT_CMD_MESH_PAIR_TIMEOUT).returns(());

        // Execute
        mesh_manager.mesh_pair_proc();

        // Verify
        assert_eq!(*PAIR_SETTING_FLAG.lock(), ePairState::PairSetted);
        mock_pair_load_key().assert_called(1);
        mock_rf_link_light_event_callback(LGT_CMD_MESH_PAIR_TIMEOUT).assert_called(1);
    }

    /// Tests mesh_pair_proc default mesh effect delay
    #[test]
    #[mry::lock(clock_time_exceed, pair_load_key)]
    fn test_mesh_pair_proc_default_mesh_effect_delay() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        mesh_manager.default_mesh_effect_delay_ref = 1000;
        mesh_manager.default_mesh_time = 0;
        mesh_manager.mesh_pair_start_time = 0;
        mesh_manager.default_mesh_time_ref = 0; // Ensure we don't skip the condition

        mock_clock_time_exceed(1000, MESH_PAIR_CMD_INTERVAL * 1000).returns(true);
        mock_pair_load_key().returns(());

        // Execute
        mesh_manager.mesh_pair_proc();

        // Verify
        assert_eq!(mesh_manager.default_mesh_effect_delay_ref, 0);
        assert_eq!(mesh_manager.default_mesh_time_ref, 0);
        mock_pair_load_key().assert_called(1);
    }

    /// Tests mesh_pair_proc default mesh effect delay with switch
    #[test]
    #[mry::lock(clock_time_exceed, clock_time, access_code)]
    fn test_mesh_pair_proc_default_mesh_effect_delay_with_switch() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        mesh_manager.default_mesh_effect_delay_ref = 1000;
        mesh_manager.default_mesh_time = 5000;
        mesh_manager.mesh_pair_start_time = 0;

        mock_clock_time_exceed(1000, MESH_PAIR_CMD_INTERVAL * 1000).returns(true);
        mock_clock_time().returns(2000);
        mock_access_code(Any, Any).returns(0x11223344);

        // Execute
        mesh_manager.mesh_pair_proc();

        // Verify
        assert_eq!(mesh_manager.default_mesh_effect_delay_ref, 0);
        assert_eq!(mesh_manager.default_mesh_time_ref, 2001);
        mock_access_code(Any, Any).assert_called(1);
    }

    /// Tests mesh_pair_proc default mesh timeout
    #[test]
    #[mry::lock(clock_time_exceed, pair_load_key)]
    fn test_mesh_pair_proc_default_mesh_timeout() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        mesh_manager.default_mesh_effect_delay_ref = 0;
        mesh_manager.default_mesh_time_ref = 1000;
        mesh_manager.default_mesh_time = 5000;
        mesh_manager.mesh_pair_start_time = 0;

        mock_clock_time_exceed(Any, Any).returns_with(|ref_time: u32, span: u32| -> bool {
            ref_time == 1000 && span == 5000 * 1000
        });
        mock_pair_load_key().returns(());

        // Execute
        mesh_manager.mesh_pair_proc();

        // Verify
        assert_eq!(mesh_manager.default_mesh_time_ref, 0);
        mock_pair_load_key().assert_called(1);
    }

    /// Tests mesh_pair_proc default mesh timeout with 255s delay
    #[test]
    #[mry::lock(clock_time_exceed, clock_time)]
    fn test_mesh_pair_proc_default_mesh_timeout_255s() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        mesh_manager.default_mesh_effect_delay_ref = 0;
        mesh_manager.default_mesh_time_ref = 1000;
        mesh_manager.default_mesh_time = 255000;
        mesh_manager.mesh_pair_start_time = 0;

        mock_clock_time_exceed(Any, Any).returns_with(|ref_time: u32, span: u32| -> bool {
            ref_time == 1000 && span == 255000 * 1000
        });
        mock_clock_time().returns(3000);

        // Execute
        mesh_manager.mesh_pair_proc();

        // Verify - should reset ref time, not clear it
        assert_eq!(mesh_manager.default_mesh_time_ref, 3001);
    }

    /// Tests mesh_pair_proc state machine - Name1
    #[test]
    #[mry::lock(
        clock_time,
        clock_time_exceed,
        light_slave_tx_command,
        rf_link_match_group_mac,
        app_mocker
    )]
    fn test_mesh_pair_proc_state_name1() {
        // Setup
        let mut app = App::default();
        let mut mesh_manager = MeshManager::default();
        mesh_manager.mesh_pair_state = MeshPairState::MeshPairName1;
        mesh_manager.mesh_pair_time = 1000;
        mesh_manager.mesh_pair_cmd_interval = 500;
        app.mesh_manager = mesh_manager;
        mock_app_mocker().returns(&mut app);

        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshTxStart;
        PAIR_STATE.lock().pair_nn = [1; 16];

        mock_clock_time_exceed(Any, Any)
            .returns_with(|ref_time: u32, _span: u32| -> bool { ref_time == 1000 });
        mock_clock_time().returns(2000);
        let test_pkt = create_test_packet(0x0001, 0xFFFF, [1, 2, 3]);
        mock_light_slave_tx_command(Any, Any, Any, Any).returns(test_pkt);
        mock_rf_link_match_group_mac(Any).returns((false, false));

        // Execute
        app.mesh_manager.mesh_pair_proc();

        // Verify - should advance to Name2
        assert_eq!(
            app.mesh_manager.mesh_pair_state,
            MeshPairState::MeshPairName2
        );
    }

    /// Tests mesh_pair_proc state machine - transitions through all states
    #[test]
    #[mry::lock(
        clock_time,
        clock_time_exceed,
        light_slave_tx_command,
        rf_link_match_group_mac,
        app_mocker
    )]
    fn test_mesh_pair_proc_state_transitions() {
        // Setup
        let mut app = App::default();
        let mut mesh_manager = MeshManager::default();
        mesh_manager.mesh_pair_time = 1000;
        mesh_manager.mesh_pair_cmd_interval = 500;

        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshTxStart;
        PAIR_STATE.lock().pair_nn = [1; 16];
        PAIR_STATE.lock().pair_pass = [2; 16];
        PAIR_STATE.lock().pair_ltk_mesh = [3; 16];

        mock_clock_time_exceed(Any, Any).returns(true); // For other clock_time_exceed calls
        mock_clock_time().returns(2000);
        let test_pkt = create_test_packet(0x0001, 0xFFFF, [1, 2, 3]);
        mock_light_slave_tx_command(Any, Any, Any, Any).returns(test_pkt);
        mock_rf_link_match_group_mac(Any).returns((false, false));

        // Test Name1 -> Name2
        app.mesh_manager = mesh_manager;
        app.mesh_manager.mesh_pair_state = MeshPairState::MeshPairName1;
        mock_app_mocker().returns(&mut app);
        app.mesh_manager.mesh_pair_proc();
        assert_eq!(
            app.mesh_manager.mesh_pair_state,
            MeshPairState::MeshPairName2
        );

        // Test Name2 -> Pwd1
        app.mesh_manager.mesh_pair_state = MeshPairState::MeshPairName2;
        app.mesh_manager.mesh_pair_proc();
        assert_eq!(
            app.mesh_manager.mesh_pair_state,
            MeshPairState::MeshPairPwd1
        );

        // Test Pwd1 -> Pwd2
        app.mesh_manager.mesh_pair_state = MeshPairState::MeshPairPwd1;
        app.mesh_manager.mesh_pair_proc();
        assert_eq!(
            app.mesh_manager.mesh_pair_state,
            MeshPairState::MeshPairPwd2
        );

        // Test Pwd2 -> Ltk1
        app.mesh_manager.mesh_pair_state = MeshPairState::MeshPairPwd2;
        app.mesh_manager.mesh_pair_proc();
        assert_eq!(
            app.mesh_manager.mesh_pair_state,
            MeshPairState::MeshPairLtk1
        );

        // Test Ltk1 -> Ltk2
        app.mesh_manager.mesh_pair_state = MeshPairState::MeshPairLtk1;
        app.mesh_manager.mesh_pair_proc();
        assert_eq!(
            app.mesh_manager.mesh_pair_state,
            MeshPairState::MeshPairLtk2
        );

        // Test Ltk2 -> Name1 and state change to TxDone
        app.mesh_manager.mesh_pair_state = MeshPairState::MeshPairLtk2;
        app.mesh_manager.mesh_pair_proc();
        assert_eq!(
            app.mesh_manager.mesh_pair_state,
            MeshPairState::MeshPairName1
        );
        assert_eq!(*PAIR_SETTING_FLAG.lock(), ePairState::PairSetMeshTxDone);
    }

    /// Tests mesh_pair_proc PairSetMeshTxDone state
    #[test]
    #[mry::lock(
        clock_time,
        clock_time_exceed,
        light_slave_tx_command,
        rf_link_match_group_mac,
        app_mocker
    )]
    fn test_mesh_pair_proc_tx_done_state() {
        // Setup
        let mut app = App::default();
        let mut mesh_manager = MeshManager::default();
        mesh_manager.mesh_pair_time = 1000;
        mesh_manager.mesh_pair_cmd_interval = 500;
        app.mesh_manager = mesh_manager;
        mock_app_mocker().returns(&mut app);

        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshTxDone;

        mock_clock_time_exceed(1000, 500 * 1000).returns(true);
        mock_clock_time().returns(2000);
        let test_pkt = create_test_packet(0x0001, 0xFFFF, [1, 2, 3]);
        mock_light_slave_tx_command(Any, Any, Any, Any).returns(test_pkt);
        mock_rf_link_match_group_mac(Any).returns((false, false));

        // Execute
        app.mesh_manager.mesh_pair_proc();

        // Verify - should advance to RxDone state
        assert_eq!(*PAIR_SETTING_FLAG.lock(), ePairState::PairSetMeshRxDone);
    }

    /// Tests mesh_pair_proc PairSetMeshRxDone state
    #[test]
    #[mry::lock(
        clock_time,
        clock_time_exceed,
        light_slave_tx_command,
        rf_link_match_group_mac,
        app_mocker,
        flash_read_page
    )]
    fn test_mesh_pair_proc_rx_done_state() {
        // Setup
        let mut app = App::default();
        let mut mesh_manager = MeshManager::default();
        mesh_manager.mesh_pair_time = 1000;
        mesh_manager.mesh_pair_cmd_interval = 500;
        mesh_manager.mesh_pair_timeout = 60; // Timeout in seconds - won't overflow when multiplied
        mesh_manager.mesh_pair_start_time = 5000;
        app.mesh_manager = mesh_manager;
        mock_app_mocker().returns(&mut app);

        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetMeshRxDone;

        // Mock specific clock_time_exceed calls
        // Don't trigger timeout: clock_time_exceed(5000, 60 * 1000 * 1000)
        mock_clock_time_exceed(5000, 60 * 1000 * 1000).returns(false);
        // Trigger interval check: clock_time_exceed(1000, 500 * 1000)
        mock_clock_time_exceed(1000, 500 * 1000).returns(true);
        mock_clock_time().returns(6000);
        mock_flash_read_page(Any, Any, Any).returns(());
        let test_pkt = create_test_packet(0x0001, 0xFFFF, [1, 2, 3]);
        mock_light_slave_tx_command(Any, Any, Any, Any).returns(test_pkt);
        mock_rf_link_match_group_mac(Any).returns((false, false));

        // Execute
        app.mesh_manager.mesh_pair_proc();

        // Verify - should reset state and clear start time
        assert_eq!(
            app.mesh_manager.mesh_pair_state,
            MeshPairState::MeshPairName1
        );
        // mesh_pair_start_time should be cleared when transitioning from PairSetMeshRxDone
        assert_eq!(app.mesh_manager.mesh_pair_start_time, 0);
        assert_eq!(*PAIR_SETTING_FLAG.lock(), ePairState::PairSetted);
    }

    // --- mesh_node_buf_init Tests ---

    /// Tests mesh_node_buf_init
    #[test]
    #[mry::lock(app_mocker)]
    fn test_mesh_node_buf_init() {
        // Setup
        MESH_NODE_MAX.set(3);
        let mut mesh_node_st = MESH_NODE_ST.lock();
        mesh_node_st[0].tick = 100;
        mesh_node_st[1].tick = 200;
        mesh_node_st[2].tick = 300;
        drop(mesh_node_st);

        let mut app = App::default();
        app.light_manager.mock_device_status_update().returns(());
        mock_app_mocker().returns(&mut app);

        let mesh_manager = MeshManager::default();

        // Execute
        mesh_manager.mesh_node_buf_init();

        // Verify
        let mesh_node_st = MESH_NODE_ST.lock();
        for i in 0..3 {
            // Use local variable to avoid unaligned reference to packed field
            let tick = mesh_node_st[i].tick;
            assert_eq!(tick, 0);
        }
        drop(mesh_node_st);

        app.light_manager
            .mock_device_status_update()
            .assert_called(1);
    }

    // --- mesh_security_enable Tests ---

    /// Tests mesh_security_enable with true
    #[test]
    fn test_mesh_security_enable_true() {
        // Setup
        SECURITY_ENABLE.set(false);
        let mesh_manager = MeshManager::default();

        // Execute
        mesh_manager.mesh_security_enable(true);

        // Verify
        assert!(SECURITY_ENABLE.get());
    }

    /// Tests mesh_security_enable with false
    #[test]
    fn test_mesh_security_enable_false() {
        // Setup
        SECURITY_ENABLE.set(true);
        let mesh_manager = MeshManager::default();

        // Execute
        mesh_manager.mesh_security_enable(false);

        // Verify
        assert!(!SECURITY_ENABLE.get());
    }

    // --- add_send_mesh_msg Tests ---

    /// Tests add_send_mesh_msg loopback
    #[test]
    #[mry::lock(light_mesh_rx_cb)]
    fn test_add_send_mesh_msg_loopback() {
        // Setup
        DEVICE_ADDRESS.set(0x1234);
        let mut mesh_manager = MeshManager::default();
        let test_pkt = create_test_packet(0x1234, 0x1234, [1, 2, 3]);

        mock_light_mesh_rx_cb(Any).returns(());

        // Execute
        mesh_manager.add_send_mesh_msg(&test_pkt, 0, 0);

        // Verify - packet should be processed immediately via UART callback
        mock_light_mesh_rx_cb(Any).assert_called(1);
        assert_eq!(mesh_manager.pkt_send_buf.len(), 0);
    }

    /// Tests add_send_mesh_msg normal add
    #[test]
    fn test_add_send_mesh_msg_normal() {
        // Setup
        DEVICE_ADDRESS.set(0x1234);
        let mut mesh_manager = MeshManager::default();
        let test_pkt = create_test_packet(0x5678, 0xABCD, [1, 2, 3]);

        // Execute
        mesh_manager.add_send_mesh_msg(&test_pkt, 1000, 3);

        // Verify
        assert_eq!(mesh_manager.pkt_send_buf.len(), 1);
        assert_eq!(mesh_manager.pkt_send_buf[0].delay, 1000);
        assert_eq!(mesh_manager.pkt_send_buf[0].send_count, 3);
    }

    /// Tests add_send_mesh_msg buffer full
    #[test]
    fn test_add_send_mesh_msg_buffer_full() {
        // Setup
        DEVICE_ADDRESS.set(0x1234);
        let mut mesh_manager = MeshManager::default();
        let test_pkt = create_test_packet(0x5678, 0xABCD, [1, 2, 3]);

        // Fill the buffer (capacity is 6)
        for i in 0..6 {
            let pkt = create_test_packet(i as u16, 0xFFFF, [i as u8, 0, 0]);
            mesh_manager.add_send_mesh_msg(&pkt, i as u64, 0);
        }

        // Execute - try to add one more
        mesh_manager.add_send_mesh_msg(&test_pkt, 1000, 3);

        // Verify - buffer should still be at max capacity
        assert_eq!(mesh_manager.pkt_send_buf.len(), 6);
    }

    // --- add_rcv_mesh_msg Tests ---

    /// Tests add_rcv_mesh_msg normal add
    #[test]
    fn test_add_rcv_mesh_msg_normal() {
        // Setup
        let mut mesh_manager = MeshManager::default();
        let test_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);

        // Execute
        mesh_manager.add_rcv_mesh_msg(&test_pkt);

        // Verify
        assert_eq!(mesh_manager.pkt_rcv_buf.len(), 1);
    }

    /// Tests add_rcv_mesh_msg buffer full
    #[test]
    fn test_add_rcv_mesh_msg_buffer_full() {
        // Setup
        let mut mesh_manager = MeshManager::default();

        // Fill the buffer (capacity is 6)
        for i in 0..6 {
            let pkt = create_test_packet(i as u16, 0xFFFF, [i as u8, 0, 0]);
            mesh_manager.add_rcv_mesh_msg(&pkt);
        }

        // Execute - try to add one more
        let test_pkt = create_test_packet(0x9999, 0x8888, [9, 9, 9]);
        mesh_manager.add_rcv_mesh_msg(&test_pkt);

        // Verify - buffer should still be at max capacity
        assert_eq!(mesh_manager.pkt_rcv_buf.len(), 6);
    }

    /// Tests send_mesh_msg_iteration with no packets to send
    #[test]
    fn test_send_mesh_msg_iteration_no_packets() {
        use futures::executor::block_on;

        // Setup
        let mut mesh_manager = MeshManager::default();
        // pkt_send_buf is empty by default

        // Execute - should return immediately with no error
        block_on(mesh_manager.send_mesh_msg_iteration());

        // Verify - should still be empty
        assert_eq!(mesh_manager.pkt_send_buf.len(), 0);
    }

    /// Tests send_mesh_msg_iteration with ready packet
    #[test]
    #[mry::lock(
        read_reg_system_tick,
        read_reg_rnd_number,
        rf_set_tx_rx_off,
        rf_set_ble_access_code,
        rf_set_ble_crc_adv,
        rf_set_ble_channel,
        rf_start_srx2tx,
        rf_set_rxmode,
        clock_time64
    )]
    fn test_send_mesh_msg_iteration_with_ready_packet() {
        use futures::executor::block_on;

        // Setup
        let mut mesh_manager = MeshManager::default();
        let test_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);

        // Ensure security is disabled to avoid encryption overflow
        SECURITY_ENABLE.set(false);

        // Add a packet that's ready to send (delay is in the past)
        mesh_manager.add_send_mesh_msg(&test_pkt, 0, 0); // delay=0 means ready to send immediately

        assert_eq!(mesh_manager.pkt_send_buf.len(), 1);

        // Mock the required functions
        mock_read_reg_system_tick().returns(1000);
        mock_read_reg_rnd_number().returns(0x5555);
        mock_clock_time64().returns(1); // Return time > 0 so packet is ready
        mock_rf_set_tx_rx_off().returns(());
        mock_rf_set_ble_access_code(Any).returns(());
        mock_rf_set_ble_crc_adv().returns(());
        mock_rf_set_ble_channel(Any).returns(());
        mock_rf_start_srx2tx(Any, Any).returns(());
        mock_rf_set_rxmode().returns(());

        // Execute
        block_on(mesh_manager.send_mesh_msg_iteration());

        // Verify - packet should be processed and removed from buffer
        assert_eq!(mesh_manager.pkt_send_buf.len(), 0);
        mock_rf_set_tx_rx_off().assert_called(SYS_CHN_LISTEN.len());
    }

    // --- rcv_mesh_msg_iteration Tests ---

    /// Tests rcv_mesh_msg_iteration with no packets to receive
    #[test]
    fn test_rcv_mesh_msg_iteration_no_packets() {
        use futures::executor::block_on;

        // Setup
        let mut mesh_manager = MeshManager::default();
        // pkt_rcv_buf is empty by default

        // Execute - should return immediately with no error
        block_on(mesh_manager.rcv_mesh_msg_iteration());

        // Verify - should still be empty
        assert_eq!(mesh_manager.pkt_rcv_buf.len(), 0);
    }

    /// Tests rcv_mesh_msg_iteration with packet to receive
    #[test]
    #[mry::lock(rf_link_rc_data)]
    fn test_rcv_mesh_msg_iteration_with_packet() {
        use futures::executor::block_on;

        // Setup
        let mut mesh_manager = MeshManager::default();
        let test_pkt = create_test_packet(0x1234, 0x5678, [1, 2, 3]);

        // Add a packet to receive
        mesh_manager.add_rcv_mesh_msg(&test_pkt);
        assert_eq!(mesh_manager.pkt_rcv_buf.len(), 1);

        // Mock the rf_link_rc_data function
        mock_rf_link_rc_data(Any).returns(());

        // Execute
        block_on(mesh_manager.rcv_mesh_msg_iteration());

        // Verify - packet should be processed and removed from buffer
        assert_eq!(mesh_manager.pkt_rcv_buf.len(), 0);
        mock_rf_link_rc_data(Any).assert_called(1);
    }
}
