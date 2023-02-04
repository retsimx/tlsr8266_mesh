use crate::config::{get_flash_adr_pairing, VENDOR_ID};
use crate::main_light::{
    light_slave_tx_command,
    rf_link_light_event_callback,
};
use crate::sdk::drivers::flash::flash_write_page;
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed, sleep_us};
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use crate::sdk::mcu::register::{write_reg_rf_irq_status, FLD_RF_IRQ_MASK};
use crate::sdk::rf_drv::rf_set_ble_access_code;
use std::ptr::addr_of;
use crate::{app, BIT};
use crate::mesh::wrappers::*;
use crate::sdk::light::*;

pub const MESH_PAIR_CMD_INTERVAL: u32 = 500;

//unit: s
pub const MESH_PAIR_TIMEOUT: u32 = 10;
//unit: ms
pub const MESH_PAIR_NOTIFY_TIMEOUT: u32 = 2500;

pub const MESH_NODE_ST_VAL_LEN: u8 = 4; // MIN: 4,   MAX: 10
pub const MESH_NODE_ST_PAR_LEN: u8 = MESH_NODE_ST_VAL_LEN - 2;

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

#[derive(PartialEq)]
#[allow(dead_code)]
pub enum GatewayStatus {
    GatewayStatusNormal = 0, /* Normal gateway role */
    GatewayStatusNodeRole,   /* As node role, when pushed button */

    GatewayStatusTempDefaltMesh, /* In default mesh temporary */
    GatewayStatusSwitchToDefaultMesh,
    GatewayStatusScanUnprovDev,       /* Scanning unpair device status */
    GatewayStatusCfgUnproDev,         /* Only provision device */
    GatewayStatusCfgCurNetwork,       /* Change current network's information */
    GatewayStatusAddConfiguredDevice, /* Add configured device */
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct mesh_node_st_val_t {
    pub dev_adr: u8,                              // don't change include type
    pub sn: u8,                                   // don't change include type
    pub par: [u8; MESH_NODE_ST_PAR_LEN as usize], //lumen-rsv,
}

#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct mesh_node_st_t {
    pub tick: u16, // don't change include type
    val: mesh_node_st_val_t,
}

pub struct MeshManager {
    mesh_pair_start_time: u32,
    default_mesh_time: u32,
    default_mesh_effect_delay_ref: u32, /* When receive change to default mesh command, shall delay at least 500ms */
    mesh_pair_start_notify_time: u32,
    mesh_pair_retry_cnt: u8,
    mesh_pair_notify_rsp_mask: [u8; 32],
    new_mesh_name: [u8; 16],
    new_mesh_pwd: [u8; 16],
    new_mesh_ltk: [u8; 16],
    default_mesh_time_ref: u32,
    effect_new_mesh: u8,
    effect_new_mesh_delay_time: u32,
    mesh_pair_cmd_interval: u32,
    mesh_pair_timeout: u32,
    mesh_pair_checksum: [u8; 8],
    mesh_pair_retry_max: u8,
    mesh_pair_time: u32,
    mesh_pair_state: MeshPairState,
    gateway_status: GatewayStatus,
}

impl MeshManager {
    pub const fn default() -> Self {
        Self {
            mesh_pair_start_time: 0,
            default_mesh_time: 0,
            default_mesh_effect_delay_ref: 0, /* When receive change to default mesh command, shall delay at least 500ms */
            mesh_pair_start_notify_time: 0,
            mesh_pair_retry_cnt: 0,
            mesh_pair_notify_rsp_mask: [0; 32],
            new_mesh_name: [0; 16],
            new_mesh_pwd: [0; 16],
            new_mesh_ltk: [0; 16],
            default_mesh_time_ref: 0,
            effect_new_mesh: 0,
            effect_new_mesh_delay_time: 0,
            mesh_pair_cmd_interval: 0,
            mesh_pair_timeout: 0,
            mesh_pair_checksum: [0; 8],
            mesh_pair_retry_max: 3,
            mesh_pair_time: 0,
            mesh_pair_state: MeshPairState::MeshPairName1,
            gateway_status: GatewayStatus::GatewayStatusNormal,
        }
    }

    pub fn mesh_pair_init(&mut self) {
        set_mesh_pair_enable(true);
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

    pub fn get_mesh_pair_checksum_fn(&self, idx: u8) -> u8 {
        let i = (idx % 8) as usize;
        return (self.new_mesh_name[i] ^ self.new_mesh_name[i + 8])
            ^ (self.new_mesh_pwd[i] ^ self.new_mesh_pwd[i + 8])
            ^ (self.new_mesh_ltk[i] ^ self.new_mesh_ltk[i + 8]);
    }

    pub fn mesh_pair_proc_get_mac_flag(&mut self) {
        set_get_mac_en(false); // set success
        if *get_mesh_pair_enable() {
            let mut data: [u8; 1] = [0];
            flash_write_page(
                *get_flash_adr_pairing() + *get_adr_flash_cfg_idx() + 1,
                1,
                data.as_mut_ptr(),
            );
            set_get_mac_en(if data[0] == 1 { true } else { false });
        }
    }

    pub fn mesh_pair_cb(&mut self, params: &[u8]) {
        if self.default_mesh_time_ref != 0 {
            self.default_mesh_time_ref = clock_time() | 1;
        }
        let cmd = params[0];
        if cmd == MeshPairState::MeshPairName1 as u8 {
            self.mesh_pair_start_time = clock_time() | 1;
            self.new_mesh_name[0..8].copy_from_slice(&params[1..9]);
        } else if cmd == MeshPairState::MeshPairName2 as u8 {
            self.new_mesh_name[8..16].copy_from_slice(&params[1..9]);
        } else if cmd == MeshPairState::MeshPairPwd1 as u8 {
            self.new_mesh_pwd[0..8].copy_from_slice(&params[1..9]);
        } else if cmd == MeshPairState::MeshPairPwd2 as u8 {
            self.new_mesh_pwd[8..16].copy_from_slice(&params[1..9]);
        } else if cmd == MeshPairState::MeshPairLtk1 as u8 {
            self.new_mesh_ltk[0..8].copy_from_slice(&params[1..9]);
        } else if cmd == MeshPairState::MeshPairLtk2 as u8 {
            self.new_mesh_ltk[8..16].copy_from_slice(&params[1..9]);
        } else if cmd == MeshPairState::MeshPairEffectDelay as u8 {
            self.effect_new_mesh_delay_time = clock_time() | 1;
            if self.default_mesh_time_ref != 0 {
                /* Keep default_mesh_time_ref non-zero */
                self.default_mesh_time = self.mesh_pair_cmd_interval * 2;
            }
        } else if cmd == MeshPairState::MeshPairEffect as u8 {
            self.effect_new_mesh = 1;
        } else if cmd == MeshPairState::MeshPairDefaultMesh as u8 {
            self.default_mesh_effect_delay_ref = clock_time() | 1;
            self.default_mesh_time = params[1] as u32 * 1000;
        }
    }

    fn mesh_cmd_notify(&self, op: u8, p: &[u8], dev_adr: u16) -> i32 {
        let mut err = -1;
        if *get_slave_link_connected() && *get_pair_login_ok() {
            if p.len() > 10 {
                //max length of par is 10
                return -1;
            }

            let mut pkt_notify = rf_packet_att_cmd_t {
                dma_len: 0x1d,  // dma_len
                _type: 0x02,    // type
                rf_len: 0x1b,   // rf_len
                l2capLen: 0x17, // u16
                chanId: 0x04,   // chanid
                opcode: 0x1b,   // notify
                handle: 0x12,
                handle1: 0x00, // status handler
                value: [0; 30],
            };

            pkt_notify.value[8] = (VENDOR_ID & 0xff) as u8;
            pkt_notify.value[8] = (VENDOR_ID >> 8) as u8;

            pkt_notify.value[3] = (dev_adr & 0xFF) as u8;
            pkt_notify.value[4] = (dev_adr >> 8) as u8;
            pkt_notify.value[7] = op | 0xc0;

            pkt_notify.value[10..10 + p.len()].copy_from_slice(&p[0..p.len()]);

            let r = irq_disable();
            if _is_add_packet_buf_ready() {
                if _rf_link_add_tx_packet(addr_of!(pkt_notify) as *const u8) {
                    err = 0;
                }
            }
            irq_restore(r);
        }

        return err;
    }

    fn mesh_pair_complete_notify(&self) -> i32 {
        let par = [CMD_NOTIFY_MESH_PAIR_END];
        return self.mesh_cmd_notify(LGT_CMD_MESH_CMD_NOTIFY, &par, *get_device_address());
    }

    fn safe_effect_new_mesh_finish(&mut self) {
        self.new_mesh_name = [0; 16];
        self.new_mesh_pwd = [0; 16];
        self.new_mesh_ltk = [0; 16];
        self.mesh_pair_start_notify_time = 0;
        self.mesh_pair_retry_cnt = 0;
        self.mesh_pair_start_time = 0;
        self.mesh_pair_notify_rsp_mask = [0; 32];
        set_pair_setting_flag(PairState::PairSetted);
    }

    fn save_effect_new_mesh(&mut self) {
        if self.default_mesh_time_ref != 0 || *get_get_mac_en() {
            self.mesh_pair_complete_notify();
            sleep_us(1000);
            /* Switch to normal mesh */
            _pair_load_key();
            self.default_mesh_time_ref = 0;

            _mesh_node_init();
            app().light_manager.device_status_update();
            self.safe_effect_new_mesh_finish();
            return;
        }

        if self.effect_new_mesh == 0 {
            get_pair_nn()[0..16].copy_from_slice(&self.new_mesh_name[0..16]);
            get_pair_pass()[0..16].copy_from_slice(&self.new_mesh_pwd[0..16]);
            get_pair_ltk()[0..16].copy_from_slice(&self.new_mesh_ltk[0..16]);
        } else {
            get_pair_ltk()[0..16].copy_from_slice(&get_pair_ltk_mesh()[0..16]);
        }

        self.mesh_pair_complete_notify();

        // make sure not receive legacy mesh data from now on
        let r = irq_disable();
        _pair_save_key();
        rf_set_ble_access_code(get_pair_ac_addr() as *const u8); // use new access code at once.
        rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO); // clear online status :mesh_node_init()
        sleep_us(1000);
        write_reg_rf_irq_status(FLD_RF_IRQ_MASK::IRQ_RX as u16); // clear current rx in buffer
        irq_restore(r);

        self.safe_effect_new_mesh_finish();
    }

    pub fn mesh_pair_notify_refresh(&mut self, p: &rf_packet_att_cmd_t) -> u8 {
        if self.mesh_pair_checksum[0..8] == p.value[12..12 + 8] {
            // mesh pair : success one device, clear the mask flag
            self.mesh_pair_notify_rsp_mask[(p.value[10] / 8) as usize] &= !(BIT!(p.value[10] % 8));
        }

        return 1; // if return 2, then the notify rsp will not report to master.
    }

    fn switch_to_default_mesh(&mut self, delay_s: u8) {
        self.default_mesh_time_ref = clock_time() | 1;
        self.default_mesh_time = delay_s as u32 * 1000;

        /* Only change AC and LTK */
        set_pair_ac(_access_code(
            get_pair_config_mesh_name().as_ptr(),
            get_pair_config_mesh_pwd().as_ptr(),
        ));
        get_pair_ltk()[0..16].copy_from_slice(&get_pair_config_mesh_ltk()[0..16]);
    }

    fn get_online_node_cnt(&mut self) -> u8 {
        let mut cnt = 0;
        for i in 0..*get_mesh_node_max() {
            if get_mesh_node_st()[i as usize].tick != 0 {
                cnt += 1;
                if i > 0 {
                    self.mesh_pair_notify_rsp_mask
                        [(get_mesh_node_st()[i as usize].val.dev_adr / 8) as usize] |=
                        BIT!(get_mesh_node_st()[i as usize].val.dev_adr % 8);
                }
            }
        }

        if self.gateway_status == GatewayStatus::GatewayStatusCfgUnproDev
            || self.gateway_status == GatewayStatus::GatewayStatusAddConfiguredDevice
        {
            /* If provisioning device to network, shall at least return two device */
            if cnt < 2 {
                return 2;
            }
        }

        return cnt;
    }

    fn mesh_pair_proc(&mut self) {
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
                _pair_load_key();
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
                _pair_load_key();
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
            _pair_load_key();
            set_pair_setting_flag(PairState::PairSetted);
            rf_link_light_event_callback(LGT_CMD_MESH_PAIR_TIMEOUT);
            return;
        }

        if *get_pair_setting_flag() == PairState::PairSetMeshTxStart
            && self.mesh_pair_state == MeshPairState::MeshPairName1
            && self.get_online_node_cnt() == 1
        {
            op_para[0] = LGT_CMD_MESH_PAIR;
            op_para[3] = MeshPairState::MeshPairEffect as u8;
            dst_addr = 0x0000; // there is noly one device in mesh,just effect itself.
            self.mesh_pair_state = MeshPairState::MeshPairName1;
            self.mesh_pair_start_notify_time = 0;
            self.mesh_pair_retry_cnt = 0;
            self.mesh_pair_start_time = 0;
            self.mesh_pair_notify_rsp_mask = [0; 32];
            set_pair_setting_flag(PairState::PairSetted);
        } else if *get_pair_setting_flag() as u8 >= PairState::PairSetMeshTxStart as u8
            && clock_time_exceed(self.mesh_pair_time, self.mesh_pair_cmd_interval * 1000)
        {
            self.mesh_pair_time = clock_time();
            if *get_pair_setting_flag() == PairState::PairSetMeshTxStart {
                op_para[0] = LGT_CMD_MESH_PAIR;
                op_para[3] = self.mesh_pair_state as u8;
                if self.mesh_pair_state == MeshPairState::MeshPairName1 {
                    // send mesh name [0-7]
                    op_para[4..4 + 8].copy_from_slice(&get_pair_nn()[0..8]);
                    self.mesh_pair_state = MeshPairState::MeshPairName2;
                } else if self.mesh_pair_state == MeshPairState::MeshPairName2 {
                    // send mesh name [8-15]
                    op_para[4..4 + 8].copy_from_slice(&get_pair_nn()[8..16]);
                    self.mesh_pair_state = MeshPairState::MeshPairPwd1;
                } else if self.mesh_pair_state == MeshPairState::MeshPairPwd1 {
                    // send mesh pwd [0-7]
                    op_para[4..4 + 8].copy_from_slice(&get_pair_pass()[0..8]);
                    self.mesh_pair_state = MeshPairState::MeshPairPwd2;
                } else if self.mesh_pair_state == MeshPairState::MeshPairPwd2 {
                    // send mesh pwd [8-15]
                    op_para[4..4 + 8].copy_from_slice(&get_pair_pass()[8..16]);
                    self.mesh_pair_state = MeshPairState::MeshPairLtk1;
                } else if self.mesh_pair_state == MeshPairState::MeshPairLtk1 {
                    // send mesh ltk [0-7]
                    op_para[4..4 + 8].copy_from_slice(&get_pair_ltk_mesh()[0..8]);
                    self.mesh_pair_state = MeshPairState::MeshPairLtk2;
                } else if self.mesh_pair_state == MeshPairState::MeshPairLtk2 {
                    // send mesh ltk [8-15]
                    op_para[4..4 + 8].copy_from_slice(&get_pair_ltk_mesh()[8..16]);
                    self.mesh_pair_state = MeshPairState::MeshPairName1;
                    set_pair_setting_flag(PairState::PairSetMeshTxDone);
                } else {
                    self.mesh_pair_state = MeshPairState::MeshPairName1;
                    self.mesh_pair_start_notify_time = 0;
                    self.mesh_pair_retry_cnt = 0;
                    self.mesh_pair_start_time = 0;
                    self.mesh_pair_notify_rsp_mask = [0; 32];
                    set_pair_setting_flag(PairState::PairSetted);
                    return;
                }
            } else if *get_pair_setting_flag() == PairState::PairSetMeshTxDone {
                // get mesh nodes' confirm value
                //rf_link_slave_read_status_start();
                op_para[0] = LGT_CMD_MESH_OTA_READ;
                op_para[3] = 0x10; // bridge cnt
                op_para[4] = PAR_READ_MESH_PAIR_CONFIRM;
                set_pair_setting_flag(PairState::PairSetMeshRxDone);
                self.mesh_pair_start_notify_time = clock_time() | 0;
                for i in 0..self.mesh_pair_checksum.len() {
                    self.mesh_pair_checksum[i] = self.get_mesh_pair_checksum_fn(i as u8);
                }
            } else if *get_pair_setting_flag() == PairState::PairSetMeshRxDone {
                let mut effect_flag = self.mesh_pair_notify_rsp_mask == [0; 32];
                if !effect_flag
                    && clock_time_exceed(self.mesh_pair_start_time, MESH_PAIR_NOTIFY_TIMEOUT * 1000)
                {
                    if self.mesh_pair_retry_cnt < self.mesh_pair_retry_max {
                        self.mesh_pair_start_time = clock_time() | 1;
                        set_pair_setting_flag(PairState::PairSetMeshTxStart);
                        self.mesh_pair_state = MeshPairState::MeshPairName1;
                    } else {
                        // retry timeout, effect or cancel?? effect now
                        effect_flag = true;
                    }
                    self.mesh_pair_retry_cnt += 1;
                }
                if effect_flag {
                    //send cmd to switch to new mesh
                    op_para[0] = LGT_CMD_MESH_PAIR;
                    op_para[3] = MeshPairState::MeshPairEffectDelay as u8;
                    self.mesh_pair_state = MeshPairState::MeshPairName1;
                    self.mesh_pair_start_notify_time = 0;
                    self.mesh_pair_retry_cnt = 0;
                    self.mesh_pair_start_time = 0;
                    self.mesh_pair_notify_rsp_mask = [0; 32];
                    set_pair_setting_flag(PairState::PairSetted);
                }
            }
        } else {
            return;
        }

        light_slave_tx_command(&op_para, dst_addr);
    }

    pub fn mesh_node_buf_init(&self) {
        // todo: Do we need this? It causes drama for now
        // [mesh_node_st_t {
        //     tick: 0,
        //     val: mesh_node_st_val_t {
        //         dev_adr: 0,
        //         sn: 0,
        //         par: [0; MESH_NODE_ST_PAR_LEN as usize],
        //     },
        // }; MESH_NODE_MAX_NUM as usize];

        app().light_manager.device_status_update();
    }
}

pub mod wrappers {
    use crate::mesh::{mesh_node_st_t, mesh_node_st_val_t, MESH_NODE_ST_PAR_LEN, MESH_NODE_ST_VAL_LEN};
    use crate::sdk::light::*;
    use std::mem::size_of;
    use crate::{app, pub_mut};

    // BEGIN SHIT LIGHT_LL HAX
    pub_mut!(
        mesh_node_mask,
        [u32; ((MESH_NODE_MAX_NUM + 31) >> 5) as usize],
        [0; ((MESH_NODE_MAX_NUM + 31) >> 5) as usize]
    );
    pub_mut!(mesh_node_max_num, u16, MESH_NODE_MAX_NUM);
    pub_mut!(mesh_node_st_val_len, u8, MESH_NODE_ST_VAL_LEN);
    pub_mut!(mesh_node_st_par_len, u8, MESH_NODE_ST_PAR_LEN);
    pub_mut!(mesh_node_st_len, u8, size_of::<mesh_node_st_t>() as u8);
    // END SHIT LIGHT_LL HAX

    pub_mut!(get_mac_en, bool, false);
    pub_mut!(mesh_pair_enable, bool, false);
    pub_mut!(
        mesh_node_st,
        [mesh_node_st_t; MESH_NODE_MAX_NUM as usize],
        [mesh_node_st_t {
            tick: 0,
            val: mesh_node_st_val_t {
                dev_adr: 0,
                sn: 0,
                par: [0; MESH_NODE_ST_PAR_LEN as usize],
            }
        }; MESH_NODE_MAX_NUM as usize]
    );

    #[no_mangle] // Required by light_ll rf_link_slave_add_status_ll
    pub fn mesh_pair_notify_refresh(p: *const rf_packet_att_cmd_t) -> u8 {
        let p = &(unsafe { *p });

        return app().mesh_manager.mesh_pair_notify_refresh(p);
    }

    #[no_mangle] // required by light_ll
    fn mesh_pair_proc() {
        app().mesh_manager.mesh_pair_proc();
    }

    #[no_mangle] // required by light_ll
    fn mesh_node_buf_init() {
        app().mesh_manager.mesh_node_buf_init();
    }

    #[no_mangle] // required by light_ll
    fn light_slave_tx_command_callback(p: *const u8) {
        _rf_link_data_callback(p);
    }
}
