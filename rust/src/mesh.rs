use core::ptr::addr_of;

use embassy_time::{Duration, Timer};
use heapless::{Deque, Vec};

use crate::{app, BIT, uprintln};
use crate::common::{access_code, mesh_node_init, pair_load_key, SYS_CHN_LISTEN};
use crate::config::{FLASH_ADR_PAIRING, VENDOR_ID};
use crate::embassy::time_driver::clock_time64;
use crate::embassy::yield_now::yield_now;
use crate::main_light::{light_slave_tx_command, rf_link_light_event_callback};
use crate::sdk::ble_app::ble_ll_pair::{pair_enc_packet_mesh, pair_save_key};
use crate::sdk::ble_app::light_ll::{is_add_packet_buf_ready, rf_link_add_tx_packet, rf_link_match_group_mac, rf_link_rc_data};
use crate::sdk::ble_app::rf_drv_8266::{rf_set_ble_access_code, rf_set_ble_channel, rf_set_ble_crc_adv, rf_set_rxmode, rf_set_tx_rx_off, rf_start_srx2tx};
use crate::sdk::drivers::flash::flash_write_page;
use crate::sdk::light::*;
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, clock_time, clock_time_exceed, sleep_us};
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use crate::sdk::mcu::register::{FLD_RF_IRQ_MASK, read_reg_rnd_number, read_reg_system_tick, write_reg_rf_irq_status};
use crate::sdk::packet_types::{Packet, PacketAttCmd, PacketAttValue, PacketL2capHead};
use crate::state::{*};

pub const MESH_PAIR_CMD_INTERVAL: u32 = 500;

//unit: s
pub const MESH_PAIR_TIMEOUT: u32 = 10;

pub const MESH_NODE_ST_VAL_LEN: usize = 4;
// MIN: 4,   MAX: 10
pub const MESH_NODE_ST_PAR_LEN: usize = MESH_NODE_ST_VAL_LEN - 2;

#[derive(Clone, Copy, PartialEq)]
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
            x if x == MeshPairState::MeshPairEffectDelay as u8 => Ok(MeshPairState::MeshPairEffectDelay),
            x if x == MeshPairState::MeshPairEffect as u8 => Ok(MeshPairState::MeshPairEffect),
            x if x == MeshPairState::MeshPairDefaultMesh as u8 => Ok(MeshPairState::MeshPairDefaultMesh),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Copy, Debug)]
#[repr(C, packed)]
pub struct mesh_node_st_val_t {
    pub dev_adr: u8,
    // don't change include type
    pub sn: u8,
    // don't change include type
    pub par: [u8; MESH_NODE_ST_PAR_LEN], //lumen-rsv,
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct mesh_node_st_t {
    pub tick: u16,
    // don't change include type
    pub val: mesh_node_st_val_t,
}

struct SendPkt {
    pub delay: u64,
    pub pkt: Packet,
}

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
    pkt_send_buf: Vec<SendPkt, 10>,
    pkt_rcv_buf: Deque<Packet, 10>,
}

impl MeshManager {
    pub const fn default() -> Self {
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

    pub fn send_mesh_message(&mut self, data: &[u8; 13], destination: u16) -> [u8; 3] {
        let pkt = light_slave_tx_command(data, destination);
        let (group_match, device_match) = rf_link_match_group_mac(&pkt);
        if group_match || device_match {
            app().mesh_manager.add_rcv_mesh_msg(&pkt);
            if !device_match {
                self.add_send_mesh_msg(&pkt, 0);
            }
        } else {
            self.add_send_mesh_msg(&pkt, 0);
        }

        return pkt.mesh().sno;
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

    pub fn mesh_pair_proc_get_mac_flag(&mut self) {
        GET_MAC_EN.set(false); // set success
        if MESH_PAIR_ENABLE.get() {
            let mut data: [u8; 1] = [0];
            flash_write_page(
                (FLASH_ADR_PAIRING as i32 + ADR_FLASH_CFG_IDX.get() + 1) as u32,
                1,
                data.as_mut_ptr(),
            );
        }
    }

    pub fn mesh_pair_cb(&mut self, params: &[u8]) {
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
            MeshPairState::MeshPairName2 => self.new_mesh_name[8..16].copy_from_slice(&params[1..9]),
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

    fn mesh_cmd_notify(&self, op: u8, p: &[u8], dev_adr: u16) -> i32 {
        let mut err = -1;
        if SLAVE_LINK_CONNECTED.get() && PAIR_LOGIN_OK.get() {
            if p.len() > 10 {
                //max length of par is 10
                return -1;
            }

            let mut pkt_notify = PacketAttCmd {
                head: PacketL2capHead {
                    dma_len: 0x1d,  // dma_len
                    _type: 0x02,    // type
                    rf_len: 0x1b,   // rf_len
                    l2cap_len: 0x17, // u16
                    chan_id: 0x04,   // chanid
                },
                opcode: 0x1b,   // notify
                handle: 0x12,
                handle1: 0x00, // status handler
                value: PacketAttValue {
                    sno: [op | 0xc0, (VENDOR_ID & 0xff) as u8, (VENDOR_ID >> 8) as u8],
                    src: [(dev_adr & 0xFF) as u8, (dev_adr >> 8) as u8],    // todo: Should this actually be dst?
                    dst: [0; 2],
                    val: [0; 23],
                },
            };

            pkt_notify.value.val[3..3 + p.len()].copy_from_slice(&p[0..p.len()]);

            if is_add_packet_buf_ready() && rf_link_add_tx_packet(&Packet { att_cmd: pkt_notify }) {
                err = 0;
            }
        }

        err
    }

    fn mesh_pair_complete_notify(&self) -> i32 {
        let par = [CMD_NOTIFY_MESH_PAIR_END];
        self.mesh_cmd_notify(LGT_CMD_MESH_CMD_NOTIFY, &par, DEVICE_ADDRESS.get())
    }

    fn safe_effect_new_mesh_finish(&mut self) {
        self.new_mesh_name = [0; 16];
        self.new_mesh_pwd = [0; 16];
        self.new_mesh_ltk = [0; 16];
        self.mesh_pair_start_time = 0;
        *PAIR_SETTING_FLAG.lock() = ePairState::PairSetted;
    }

    fn save_effect_new_mesh(&mut self) {
        if self.default_mesh_time_ref != 0 || GET_MAC_EN.get() {
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
        let r = irq_disable();
        pair_save_key();
        rf_set_ble_access_code(PAIR_AC.get()); // use new access code at once.
        rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO); // clear online status :mesh_node_init()
        sleep_us(1000);
        write_reg_rf_irq_status(FLD_RF_IRQ_MASK::IRQ_RX as u16); // clear current rx in buffer
        irq_restore(r);

        self.safe_effect_new_mesh_finish();
    }

    fn switch_to_default_mesh(&mut self, delay_s: u8) {
        self.default_mesh_time_ref = clock_time() | 1;
        self.default_mesh_time = delay_s as u32 * 1000;

        /* Only change AC and LTK */
        uprintln!("pairac c");
        PAIR_AC.set(access_code(
            &*PAIR_CONFIG_MESH_NAME.lock(),
            &*PAIR_CONFIG_MESH_PWD.lock(),
        ));
        PAIR_STATE.lock().pair_ltk.copy_from_slice(&*PAIR_CONFIG_MESH_LTK.lock());
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

        if *PAIR_SETTING_FLAG.lock() == ePairState::PairSetMeshTxStart
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

        let pkt = light_slave_tx_command(&op_para, dst_addr);
        let (group_match, device_match) = rf_link_match_group_mac(&pkt);
        if group_match || device_match {
            app().mesh_manager.add_rcv_mesh_msg(&pkt);
            if !device_match {
                self.add_send_mesh_msg(&pkt, 0);
            }
        } else {
            self.add_send_mesh_msg(&pkt, 0);
        }
    }

    pub fn mesh_node_buf_init(&self) {
        MESH_NODE_ST.lock().fill(mesh_node_st_t {
            tick: 0,
            val: mesh_node_st_val_t {
                dev_adr: 0,
                sn: 0,
                par: [0; MESH_NODE_ST_PAR_LEN],
            },
        });

        app().light_manager.device_status_update();
    }

    pub fn mesh_security_enable(&self, enable: bool)
    {
        SECURITY_ENABLE.set(enable);
    }

    pub fn add_send_mesh_msg(&mut self, packet: &Packet, delay: u64) {
        if self.pkt_send_buf.push(
            SendPkt {
                delay,
                pkt: *packet,
            }
        ).is_err() {
            uprintln!("pkt send buf is full, dropping packet...");
        }
    }

    pub async fn send_mesh_msg_task(&mut self) {
        loop {
            let result = critical_section::with(|_| {
                let found = self.pkt_send_buf.iter().enumerate().filter(|(_, elem)| elem.delay < clock_time64()).last();

                let (index, _) = found?;

                let pkt = self.pkt_send_buf.swap_remove(index);

                Some(pkt)
            });

            if result.is_none() {
                yield_now().await;
                continue;
            }

            let mut result = result.unwrap();

            result.pkt.mesh_mut().src_tx = DEVICE_ADDRESS.get();
            result.pkt.mesh_mut().handle1 = 0;

            // Encrypt the packet if required
            if SECURITY_ENABLE.get()
            {
                result.pkt.head_mut()._type |= BIT!(7);
                pair_enc_packet_mesh(&mut result.pkt);
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
                    rf_start_srx2tx(addr_of!(result.pkt) as u32, read_reg_system_tick() + CLOCK_SYS_CLOCK_1US * 30);
                });

                Timer::after(Duration::from_micros(600)).await;
            }

            rf_set_rxmode();
        }
    }

    pub fn add_rcv_mesh_msg(&mut self, packet: &Packet) {
        if self.pkt_rcv_buf.push_back(*packet).is_err() {
            uprintln!("pkt rcv buf is full, dropping packet...");
        }
    }

    pub async fn rcv_mesh_msg_task(&mut self) {
        loop {
            while self.pkt_rcv_buf.is_empty() {
                yield_now().await;
            }

            let mut result = critical_section::with(|_| {
                self.pkt_rcv_buf.pop_front().unwrap()
            });

            rf_link_rc_data(&mut result);
        }
    }
}
