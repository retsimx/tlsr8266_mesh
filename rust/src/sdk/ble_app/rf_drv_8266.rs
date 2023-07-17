use core::cmp::min;
use core::ops::{Deref, DerefMut};
use core::ptr::{addr_of, addr_of_mut};
use core::slice;

use crate::{BIT, uprintln, uprintln_fast};
use crate::common::{dev_addr_with_mac_flag, mesh_node_init, pair_load_key, retrieve_dev_grp_address};
use crate::config::{FLASH_ADR_MAC, FLASH_ADR_PAIRING, MESH_PWD, OUT_OF_MESH, PAIR_VALID_FLAG};
use crate::main_light::{rf_link_data_callback, rf_link_response_callback};
use crate::sdk::ble_app::ble_ll_pair::pair_dec_packet;
use crate::sdk::ble_app::light_ll::{rf_link_get_op_para, rf_link_is_notify_req, rf_link_match_group_mac, rf_link_slave_add_status, rf_link_slave_read_status_par_init, rf_link_slave_read_status_stop};
use crate::sdk::common::compat::{array4_to_int, load_tbl_cmd_set, TBLCMDSET};
use crate::sdk::common::crc::crc16;
use crate::sdk::drivers::flash::{flash_read_page, flash_write_page};
use crate::sdk::light::{*};
use crate::sdk::mcu::analog::{analog_read, analog_write};
use crate::sdk::mcu::clock::CLOCK_SYS_CLOCK_1US;
use crate::sdk::mcu::crypto::encode_password;
use crate::sdk::mcu::irq_i::irq_disable;
use crate::sdk::mcu::random::rand;
use crate::sdk::mcu::register::{FLD_RF_IRQ_MASK, read_reg_irq_mask, read_reg_rf_mode, read_reg_system_tick, read_reg_system_tick_mode, write_reg16, write_reg32, write_reg8, write_reg_dma2_addr, write_reg_dma2_ctrl, write_reg_dma3_addr, write_reg_dma_chn_irq_msk, write_reg_irq_mask, write_reg_irq_src, write_reg_rf_access_code, write_reg_rf_crc, write_reg_rf_irq_mask, write_reg_rf_irq_status, write_reg_rf_mode, write_reg_rf_mode_control, write_reg_rf_sched_tick, write_reg_rf_sn, write_reg_system_tick_irq, write_reg_system_tick_mode};
use crate::sdk::mcu::register::{rega_deepsleep_flag, write_reg_rf_rx_gain_agc};
use crate::sdk::packet_types::{Packet, PacketAttCmd, PacketAttData, PacketAttValue};
use crate::sdk::pm::light_sw_reboot;
use crate::state::{*};
use crate::version::BUILD_VERSION;

const RF_FAST_MODE: bool = true;
const RF_TRX_MODE: u8 = 0x80;
const RF_TRX_OFF: u8 = 0x45;

static TBL_AGC: [u32; 7] = [0x30333231, 0x182C3C38, 0xC0C1C, 0x0, 0x1B150F0A, 0x322E2721, 0x3E38];

const TBL_RF_INI: [TBLCMDSET; 61] = [
    TBLCMDSET {
        adr: 0x5b4,
        dat: 0x02,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x474,
        dat: 0x08,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x85,
        dat: 0x00,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x80,
        dat: 0x61,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x06,
        dat: 0x00,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x8f,
        dat: 0x30,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x81,
        dat: 0xd8,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x8f,
        dat: 0x38,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x8b,
        dat: 0xe3,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x8e,
        dat: 0x6b,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x8d,
        dat: 0x67,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x402,
        dat: 0x26,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x9e,
        dat: 0xad,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0xa0,
        dat: 0x28,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0xa2,
        dat: 0x2c,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0xa3,
        dat: 0x10,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0xac,
        dat: 0xa7,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0xaa,
        dat: 0x2e,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x439,
        dat: 0x72,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x400,
        dat: 0x0b,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x42b,
        dat: 0xf3,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x43b,
        dat: 0xfc,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x74f,
        dat: 0x01,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0xf04,
        dat: 0x50,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0xf06,
        dat: 0x00,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0xf0c,
        dat: 0x50,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0xf10,
        dat: 0x00,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x400,
        dat: 0x0f,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x42b,
        dat: 0xf1,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x420,
        dat: 0x20,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x421,
        dat: 0x04,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x422,
        dat: 0x00,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x424,
        dat: 0x12,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x464,
        dat: 0x07,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x4cd,
        dat: 0x04,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x9e,
        dat: 0x56,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0xa3,
        dat: 0xf0,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0xaa,
        dat: 0x26,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x404,
        dat: 0xf5,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x408,
        dat: 0x8e,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x409,
        dat: 0x89,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x40a,
        dat: 0xbe,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x40b,
        dat: 0xd6,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x401,
        dat: 0x08,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x430,
        dat: 0x12,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x43d,
        dat: 0x71,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x402,
        dat: 0x24,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0xf04,
        dat: 0x68,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x42c,
        dat: 0x30,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x4ca,
        dat: 0x88,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x4cb,
        dat: 0x04,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x42d,
        dat: 0x33,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x433,
        dat: 0x00,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x434,
        dat: 0x01,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x43a,
        dat: 0x77,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x43e,
        dat: 0xc9,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x4cd,
        dat: 0x06,
        cmd: 0xc3,
    },

    // Enable these for BLE to work with 16mhz xtal
    TBLCMDSET {
        adr: 0x4eb,
        dat: 0x60,
        cmd: 0xc3,
    },
    TBLCMDSET {     // gauss filter sel: 16M
        adr: 0x99,
        dat: 0x31,
        cmd: 0xc8,
    },
    TBLCMDSET {     //enable rxadc clock
        adr: 0x82,
        dat: 0x34,
        cmd: 0xc8,
    },
    TBLCMDSET {     //reg_dc_mod (500K)
        adr: 0x9e,
        dat: 0x41,
        cmd: 0xc8,
    }
];

pub unsafe fn rf_drv_init(enable: bool) -> u8
{
    let result = analog_read(rega_deepsleep_flag) & 0x40;

    // This function is not complete
    critical_section::with(|_| {
        if enable {
            #[cfg(feature = "xtal-16mhz")]
            load_tbl_cmd_set(TBL_RF_INI.as_ptr(), TBL_RF_INI.len() as u32);

            #[cfg(not(feature = "xtal-16mhz"))]
            load_tbl_cmd_set(TBL_RF_INI.as_ptr(), TBL_RF_INI.len() as u32 - 4);

            // todo: Should this be 0..7? There's an extra couple of bytes of data in the 7th int
            for i in 0..6 {
                write_reg_rf_rx_gain_agc(TBL_AGC[i], (i << 2) as u32)
            }

            if *(FLASH_ADR_MAC as *const u8).offset(0x11) != 0xff {
                let u_var5 = *(FLASH_ADR_MAC as *const u8).offset(0x11) as u32;
                RF_TP_BASE.set(u_var5);
                RF_TP_GAIN.set(((u_var5 - 0x19) << 8) / 80);
            }
            if *(FLASH_ADR_MAC as *const u8).offset(0x12) != 0xff {
                let u_var5 = RF_TP_BASE.get() - *(FLASH_ADR_MAC as *const u8).offset(0x12) as u32;
                RF_TP_GAIN.set((u_var5 << 8) / 80);
            }
        } else {
            analog_write(6, 0);  // power off sar
        }
    });

    return result;
}

pub fn rf_stop_trx() {
    write_reg_rf_mode_control(0x80);            // stop
}


pub unsafe fn blc_ll_init_basic_mcu()
{
    write_reg16(0xf0a, 700);
    write_reg_dma2_addr(addr_of!(LIGHT_RX_BUFF.lock()[LIGHT_RX_WPTR.get()]) as u16);

    write_reg_dma2_ctrl(0x104);
    write_reg_dma_chn_irq_msk(0);

    write_reg_irq_mask(read_reg_irq_mask() | 0x2000);
    write_reg_system_tick_mode(read_reg_system_tick_mode() | 2);

    write_reg16(0xf1c, 0);
    write_reg_rf_irq_status(0xfffe);

    write_reg_rf_irq_mask(FLD_RF_IRQ_MASK::IRQ_RX as u16 | FLD_RF_IRQ_MASK::IRQ_TX as u16);

    write_reg_system_tick_irq(read_reg_system_tick() | (0x80 << 0x18));
    write_reg_irq_src(0x80 << 0xd);
    write_reg_irq_mask(read_reg_irq_mask() | (0x80 << 0xd));

    write_reg16(0xf2c, 0xc00);
}

fn rf_link_get_rsp_type(opcode: u8, unk: u8) -> u8
{
    let mut result = 0x1b;
    if opcode != 0x1a {
        if opcode == 0x1d {
            result = 0x14;
            if unk != 1 {
                result = 0x15;
                if unk != 2 {
                    result = 0x14;
                    if unk == 3 {
                        result = 0x16;
                    }
                }
            }
        } else {
            result = 0x14;
            if opcode != 0x17 {
                result = 0x21;
                if opcode != 0x20 {
                    result = 1;
                    if opcode != 0 {
                        result = 0x27;
                        if opcode != 0x26 {
                            result = 0x29;
                            if opcode != 0x28 {
                                result = 0x2b;
                                if opcode != 0x2a {
                                    result = 8;
                                    if opcode != 7 {
                                        result = 0xff;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return result;
}

fn rf_link_slave_read_status_start(pkt_light_data: &mut Packet)
{
    SLAVE_READ_STATUS_BUSY_TIME.set(read_reg_system_tick());
    SLAVE_READ_STATUS_BUSY.set(rf_link_get_rsp_type(pkt_light_data.att_cmd_mut().value.val[0] & 0x3f, pkt_light_data.att_cmd_mut().value.val[4]));
    SLAVE_READ_STATUS_UNICAST_FLAG.set(!pkt_light_data.att_cmd_mut().value.dst[1] >> 7);
    if -1 < (!((pkt_light_data.att_cmd_mut().value.dst[1] as u32) << 0x18)) as i32 {
       pkt_light_data.att_cmd_mut().value.val[8..8 + 4].fill(0);
       pkt_light_data.att_cmd_mut().value.val[12] = SLAVE_READ_STATUS_UNICAST_FLAG.get();
    }
    if pkt_light_data.att_cmd_mut().value.val[0] & 0x3f == 0x1a {
       pkt_light_data.att_cmd_mut().value.val[4..8 + 4].fill(0);
       pkt_light_data.att_cmd_mut().value.val[3] = SLAVE_STATUS_TICK.get();
    }

    rf_link_slave_read_status_par_init();

    BUFF_RESPONSE.lock().fill(Packet { att_data: PacketAttData::default() });

    SLAVE_STATUS_RECORD_IDX.set(0);
    SLAVE_STATUS_RECORD.lock().fill(
        StatusRecord { adr: [0; 1], alarm_id: 0 }
    );

    NOTIFY_REQ_MASK_IDX.set(0);
}

fn rf_link_slave_data_write_no_dec(data: &Packet) -> bool {
    if data.head().rf_len < 0x11 {
        return false;
    }

    let sno = &data.att_write().value[0..3];
    if sno == *SLAVE_SNO.lock() {
        return true;
    }

    let mut op_cmd: [u8; 3] = [0; 3];
    let mut op_cmd_len: u8 = 0;
    let mut params: [u8; 16] = [0; 16];
    let mut params_len: u8 = 0;
    rf_link_get_op_para(
        data,
        &mut op_cmd,
        &mut op_cmd_len,
        &mut params,
        &mut params_len,
        false,
    );

    let (group_match, device_match) = rf_link_match_group_mac(data);
    let op;
    let mut tmp = data.att_write().value[6] as u32;
    if op_cmd_len == 3 {
        op = op_cmd[0] & 0x3f;
    } else {
        op = 0;
    }

    if op == LGT_CMD_CONFIG_DEV_ADDR && ((tmp * 0x1000000) as i32) < 0 {
        if params[0] != 0xff {
            return false;
        }
        if params[1] != 0xff {
            return false;
        }
        tmp = tmp << 8 | data.att_write().value[5] as u32;
    } else {
        tmp = tmp << 8 | data.att_write().value[5] as u32;
        if ((!tmp << 0x10) as i32) < 0 && op == LGT_CMD_LIGHT_ONOFF {
            if tmp == 0 {
                tmp = DEVICE_ADDRESS.get() as u32;
            }
            uprintln!("stub: mesh_node_check_force_notify")
            // mesh_node_check_force_notify(uVar4, params[0]);
        }
    }

    let mut pkt_light_status = PKT_LIGHT_STATUS.lock();
    let mut pkt_light_data = PKT_LIGHT_DATA.lock();

    pkt_light_data.att_cmd_mut().value = PacketAttValue::default();
    pkt_light_status.att_cmd_mut().value = PacketAttValue::default();

    unsafe {
        slice::from_raw_parts_mut(
            addr_of_mut!(*pkt_light_data) as *mut u8,
            params_len as usize + 0x11,
        ).copy_from_slice(
            slice::from_raw_parts(
                addr_of!(*data) as *const u8,
                params_len as usize + 0x11,
            )
        )
    }

   pkt_light_data.head_mut().chan_id = 0xff03;
   pkt_light_data.head_mut().dma_len = 0x27;
   pkt_light_data.head_mut().rf_len = 0x25;
   pkt_light_data.head_mut().l2cap_len = 0x21;
    // todo: What type is PKT_LIGHT_DATA here?
    unsafe { *(addr_of_mut!(pkt_light_data.att_cmd_mut().opcode) as *mut u16) = DEVICE_ADDRESS.get() };
    unsafe { *(addr_of_mut!(pkt_light_data.att_cmd_mut().value.src) as *mut u16) = DEVICE_ADDRESS.get() };
    APP_CMD_TIME.set(read_reg_system_tick());
   pkt_light_data.att_cmd_mut().value.val[18] = MAX_RELAY_NUM;

    if device_match || group_match {
        rf_link_data_callback(&*pkt_light_data);
    }
    SLAVE_SNO.lock().copy_from_slice(sno);

    SLAVE_LINK_CMD.set(op);

    if !rf_link_is_notify_req(op) {
        if SLAVE_READ_STATUS_BUSY.get() != 0 {
            rf_link_slave_read_status_stop();
        }
        if device_match == false && unsafe { *(addr_of!(data.att_write().value[5]) as *const u16) } != 0 {
            SLAVE_DATA_VALID.set(BRIDGE_MAX_CNT + 1);
        } else {
            SLAVE_DATA_VALID.set(0);
        }
        return true;
    }

   pkt_light_status.att_cmd_mut().value.val[13] = 0;
   pkt_light_status.att_cmd_mut().value.val[14] = 0;
   pkt_light_status.att_cmd_mut().value.val[18] = MAX_RELAY_NUM;
   pkt_light_data.att_cmd_mut().value.val[16] = (SLAVE_LINK_INTERVAL.get() / (CLOCK_SYS_CLOCK_1US * 1000)) as u8;
    if device_match == false || (tmp != 0 && op == LGT_CMD_CONFIG_DEV_ADDR && dev_addr_with_mac_flag(params.as_ptr())) {
        if op == LGT_CMD_LIGHT_GRP_REQ || op == LGT_CMD_LIGHT_READ_STATUS || op == LGT_CMD_USER_NOTIFY_REQ {
            SLAVE_DATA_VALID.set(params[0] as u32 * 2 + 1);
            if op == LGT_CMD_LIGHT_GRP_REQ {
               pkt_light_data.att_cmd_mut().value.val[15] = params[1];
            } else if op == LGT_CMD_LIGHT_CONFIG_GRP {
               pkt_light_data.att_cmd_mut().value.val[15] = 1;
            } else {
                if op == LGT_CMD_CONFIG_DEV_ADDR {
                   pkt_light_data.att_cmd_mut().value.val[15] = 4;
                }
                if op == LGT_CMD_USER_NOTIFY_REQ {
                   pkt_light_data.att_cmd_mut().value.val[15] = 7;
                } else {
                   pkt_light_data.att_cmd_mut().value.val[15] = 0;
                }
            }
        } else if op != LGT_CMD_CONFIG_DEV_ADDR {
            if op != LGT_CMD_LIGHT_CONFIG_GRP {
                if op == LGT_CMD_LIGHT_GRP_REQ {
                   pkt_light_data.att_cmd_mut().value.val[15] = params[1];
                } else if op == LGT_CMD_LIGHT_CONFIG_GRP {
                   pkt_light_data.att_cmd_mut().value.val[15] = 1;
                } else {
                    if op == LGT_CMD_CONFIG_DEV_ADDR {
                       pkt_light_data.att_cmd_mut().value.val[15] = 4;
                    }
                    if op == LGT_CMD_USER_NOTIFY_REQ {
                       pkt_light_data.att_cmd_mut().value.val[15] = 7;
                    } else {
                       pkt_light_data.att_cmd_mut().value.val[15] = 0;
                    }
                }
            }
            SLAVE_DATA_VALID.set(BRIDGE_MAX_CNT * 2 + 1);
           pkt_light_data.att_cmd_mut().value.val[15] = 1;
        } else if dev_addr_with_mac_flag(params.as_ptr()) == false {
            SLAVE_DATA_VALID.set(BRIDGE_MAX_CNT * 2 + 1);
           pkt_light_data.att_cmd_mut().value.val[15] = 4;
        } else {
            SLAVE_DATA_VALID.set(params[3] as u32 * 2 + 1);
           pkt_light_data.att_cmd_mut().value.val[15] = 4;
        }
    } else {
        SLAVE_DATA_VALID.set(0);
        if op == LGT_CMD_LIGHT_GRP_REQ {
           pkt_light_data.att_cmd_mut().value.val[15] = params[1];
        } else if op == LGT_CMD_LIGHT_CONFIG_GRP {
           pkt_light_data.att_cmd_mut().value.val[15] = 1;
        } else {
            if op == LGT_CMD_CONFIG_DEV_ADDR {
               pkt_light_data.att_cmd_mut().value.val[15] = 4;
            }
            if op == LGT_CMD_USER_NOTIFY_REQ {
               pkt_light_data.att_cmd_mut().value.val[15] = 7;
            } else {
               pkt_light_data.att_cmd_mut().value.val[15] = 0;
            }
        }
    }
    pkt_light_status.att_cmd_mut().value.val[15] = pkt_light_data.att_cmd_mut().value.val[15];
    rf_link_slave_read_status_start(pkt_light_data.deref_mut());

    SLAVE_STAT_SNO.lock().copy_from_slice(sno);

    if device_match || group_match {
        let ptr = &pkt_light_data.att_cmd_mut().value.val[3..];
       pkt_light_status.att_cmd_mut().value.val[3..3 + ptr.len()].copy_from_slice(&ptr);

       pkt_light_status.att_cmd_mut().value.sno = *SLAVE_SNO.lock();

        unsafe { *(addr_of!(pkt_light_status.att_cmd_mut().value.src) as *mut u16) = DEVICE_ADDRESS.get() };

        let tmp_pkt: PacketAttValue = PacketAttValue::default();

        unsafe {
            slice::from_raw_parts_mut(
                addr_of!(tmp_pkt) as *mut u8,
                0x1e,
            ).copy_from_slice(&data.att_write().value);
        }

        unsafe {
            *(addr_of!(tmp_pkt.src) as *mut u16) = DEVICE_ADDRESS.get();
        }

        if rf_link_response_callback(&mut pkt_light_status.att_cmd_mut().value, &tmp_pkt) {
            rf_link_slave_add_status(&*pkt_light_status);
        }
    }

    return true;
}

pub fn rf_link_slave_data_write(data: &Packet) -> bool {
    let mut data: Packet = data.clone();
    if PAIR_LOGIN_OK.get() && pair_dec_packet(&mut data) {
        return rf_link_slave_data_write_no_dec(&mut data);
    }

    return false;
}

fn rf_link_slave_set_adv(adv_data_ptr: &[u8])
{
    let mut pkt_adv = PKT_ADV.lock();

    pkt_adv.head_mut().dma_len = adv_data_ptr.len() as u32 + 8;
    pkt_adv.head_mut().rf_len = adv_data_ptr.len() as u8 + 6;

    pkt_adv.adv_ind_module_mut().data[0..adv_data_ptr.len()].copy_from_slice(&adv_data_ptr[0..adv_data_ptr.len()]);
}

pub fn rf_link_slave_init(interval: u32)
{
    unsafe {
        blc_ll_init_basic_mcu();
        *P_ST_HANDLER.lock() = IrqHandlerStatus::Adv;
        SLAVE_LINK_STATE.set(0);
        SLAVE_LISTEN_INTERVAL.set(interval * CLOCK_SYS_CLOCK_1US);

        SLAVE_CONNECTED_TICK.set(CLOCK_SYS_CLOCK_1US * 100000 + read_reg_system_tick());
        write_reg_system_tick_irq(SLAVE_CONNECTED_TICK.get());

        write_reg8(0xf04, 0x68);

        if *(FLASH_ADR_MAC as *const u32) == u32::MAX {
            let mac: [u16; 2] = [rand(), rand()];
            flash_write_page(FLASH_ADR_MAC, 4, mac.as_ptr() as *const u8);
        }

        let pair_addr = *(FLASH_ADR_PAIRING as *const u32) + 1;
        if pair_addr == 0 {
            let pairing_addr = FLASH_ADR_PAIRING;
            let mut buff: [u8; 16] = [0; 16];
            buff[0..16].copy_from_slice(&PAIR_CONFIG_MESH_LTK.lock()[0..16]);
            flash_write_page(pairing_addr + 48, 16, buff.as_mut_ptr());

            let mut buff: [u8; 16] = [0; 16];
            let len = min(MESH_PWD.len(), buff.len());
            buff[0..len].copy_from_slice(&MESH_PWD.as_bytes()[0..len]);
            encode_password(&mut buff);
            flash_write_page(pairing_addr + 32, 16, buff.as_mut_ptr());

            let mut buff: [u8; 16] = [0; 16];
            let len = min(OUT_OF_MESH.len(), buff.len());
            buff[0..len].copy_from_slice(&OUT_OF_MESH.as_bytes()[0..len]);
            flash_write_page(pairing_addr + 16, 16, buff.as_mut_ptr());

            let mut buff: [u8; 16] = [0; 16];
            buff[0] = PAIR_VALID_FLAG;
            buff[15] = PAIR_VALID_FLAG;

            if MESH_PAIR_ENABLE.get() {
                GET_MAC_EN.set(true);
                buff[1] = 1;
            }
            flash_write_page(pairing_addr, 16, buff.as_mut_ptr());
            irq_disable();
            light_sw_reboot();
            loop {}
        }

        flash_read_page(FLASH_ADR_MAC, 6, MAC_ID.lock().as_mut_ptr());

        PKT_ADV.lock().adv_ind_module_mut().adv_a = *MAC_ID.lock();

        rf_link_slave_set_adv(&ADV_DATA.lock().clone());
        pair_load_key();

       PKT_LIGHT_DATA.lock().att_cmd_mut().value.src[0..2].copy_from_slice(&MAC_ID.lock()[0..2]);

       PKT_LIGHT_STATUS.lock().att_cmd_mut().value.src[0..2].copy_from_slice(&MAC_ID.lock()[0..2]);

        SLAVE_ADV_ENABLE.set(true);

        retrieve_dev_grp_address();

        mesh_node_init();

        write_reg32(0x808004, array4_to_int(&*MAC_ID.lock()));
        write_reg32(0x808008, BUILD_VERSION);
        write_reg16(0x80800c, crc16(&slice::from_raw_parts(0x808004 as *const u8, 8)));
    }
}

#[repr(C, packed)]
struct TblRfPowerT {
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
}

static TBL_RF_POWER: [TblRfPowerT; 12] = [
    TblRfPowerT {
        a: 0x25,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x0a,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x06,
        b: 0x74,
        c: 0x43,
        d: 0x61,
    },
    TblRfPowerT {
        a: 0x06,
        b: 0x64,
        c: 0xc2,
        d: 0x61,
    },
    TblRfPowerT {
        a: 0x06,
        b: 0x64,
        c: 0xc1,
        d: 0x61,
    },
    TblRfPowerT {
        a: 0x05,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x03,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x02,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x01,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x00,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x00,
        b: 0x64,
        c: 0x43,
        d: 0x61,
    },
    TblRfPowerT {
        a: 0x00,
        b: 0x64,
        c: 0xcb,
        d: 0x61,
    }
];

pub fn rf_set_power_level_index(index: u32)
{
    let mut tbl_index = 0x2c;
    if index + 1 < 0xd {
        tbl_index = index << 2;
    }

    let tbl_index = tbl_index as usize;

    unsafe {
        analog_write(0xa2, TBL_RF_POWER[tbl_index].a);
        analog_write(4, TBL_RF_POWER[tbl_index].b);
        analog_write(0xa7, TBL_RF_POWER[tbl_index].c);
        analog_write(0x8d, TBL_RF_POWER[tbl_index].d);
    }
}

pub fn rf_set_rxmode()
{
    write_reg8(0x800428, RF_TRX_MODE | BIT!(0));        // rx enable
    write_reg8(0x800f02, RF_TRX_OFF | BIT!(5));        // RX enable
}

pub fn rf_set_tx_rx_off()
{
    write_reg_rf_mode(0x29);
    write_reg8(0x800428, RF_TRX_MODE);    // rx disable
    write_reg8(0x800f02, RF_TRX_OFF);        // reset tx/rx state machine
}

pub fn rf_set_ble_channel(chn: u8) {
    write_reg8(0x40d, chn);

    let mut gain = 0;
    let mut intgn = 0u16;
    if chn < 11 {
        gain = (chn + 2) * 2;
        intgn = gain as u16 + 2400;
    } else if chn < 37 {
        gain = (chn + 3) * 2;
        intgn = gain as u16 + 2400;
    } else if chn == 37 {
        gain = 2;
        intgn = 2402;
    } else if chn == 38 {
        gain = 0x1a;
        intgn = 2426;
    } else if chn == 39 {
        gain = 0x50;
        intgn = 2480;
    } else if chn < 51 {
        gain = chn * 2;
        intgn = gain as u16 + 2400;
    } else {
        if chn > 61 {
            gain = 0x50;
            intgn = 2480;
        } else {
            gain = (chn - 61) * 2;
            intgn = gain as u16 + 2400;
        }
    }

    analog_write(6, 0);

    /////////////////// turn on LDO and baseband PLL ////////////////
    write_reg_rf_mode(0x29);

    write_reg8(0x800428, RF_TRX_MODE);        // rx disable
    write_reg8(0x800f02, RF_TRX_OFF);    // reset tx/rx state machine

    //auto tx
    write_reg16(0x8004d6, intgn);    // {intg_N}
    // write_reg32 (0x8004d0, (fre - 2) * 58254 + 1125);	// {intg_N, frac}

    rf_set_tp_gain(gain);
}

fn rf_set_tp_gain(gain: u8)
{
    unsafe { analog_write(0x93, RF_TP_BASE.get() as u8 - ((gain as u32 * RF_TP_GAIN.get() + 0x80) >> 8) as u8); }
}

pub fn rf_start_stx2rx(addr: u32, tick: u32)
{
    write_reg_rf_sched_tick(tick);                                // Setting schedule trigger time
    write_reg_rf_mode(read_reg_rf_mode() | 0x04);    // Enable cmd_schedule mode
    write_reg_rf_mode_control(0x87);                                // Set mode
    write_reg_dma3_addr(addr as u16);
}

pub fn rf_start_srx2tx(addr: u32, tick: u32)
{
    write_reg_rf_sched_tick(tick);                                  // Setting schedule trigger time
    write_reg_rf_mode(read_reg_rf_mode() | 0x04);                   // Enable cmd_schedule mode
    write_reg_rf_mode_control(0x85);                                // Set mode
    write_reg_dma3_addr(addr as u16);
}

pub fn rf_start_brx(addr: u32, tick: u32)
{
    write_reg32(0xf28, 0xffffffff);                            // ?
    write_reg_rf_sched_tick(tick);                                // Setting schedule trigger time
    write_reg_rf_mode(read_reg_rf_mode() | 0x04);    // Enable cmd_schedule mode
    write_reg_rf_mode_control(0x82);                                // Set mode
    write_reg_dma3_addr(addr as u16);
}

pub fn rf_reset_sn()
{
    write_reg_rf_sn(0x3f);
    write_reg_rf_sn(0x00);
}

pub fn rf_set_ble_crc(crc: &[u8])
{
    write_reg_rf_crc(crc[0] as u32 | ((crc[1] as u32) << 8) | ((crc[2] as u32) << 16));
}

pub fn rf_set_ble_crc_adv()
{
    write_reg_rf_crc(0x555555);
}

pub fn rf_set_ble_access_code(ac: u32)
{
    write_reg_rf_access_code(ac.swap_bytes());
}

pub fn rf_set_ble_access_code_adv()
{
    write_reg_rf_access_code(0xd6be898e);
}

#[cfg(test)]
mod tests {
    use mry::Any;

    use crate::sdk::mcu::register::*;

    use super::*;

    #[test]
    #[mry::lock(write_reg32)]
    fn test_rf_set_ble_access_code_adv() {
        mock_write_reg32(0x408, 0xd6be898e).returns(());

        rf_set_ble_access_code_adv();

        mock_write_reg32(0x408, 0xd6be898e).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg32)]
    fn test_rf_set_ble_access_code() {
        mock_write_reg32(0x408, Any).returns(());

        // Should swap the order of the bytes
        rf_set_ble_access_code(0x12345678);
        mock_write_reg32(0x408, 0x78563412).assert_called(1);

        rf_set_ble_access_code(0x78563412);
        mock_write_reg32(0x408, 0x12345678).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg32)]
    fn test_rf_set_ble_crc_adv() {
        mock_write_reg32(0x44c, Any).returns(());

        rf_set_ble_crc_adv();

        mock_write_reg32(0x44c, 0x555555).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg32)]
    fn test_rf_set_ble_crc() {
        mock_write_reg32(0x44c, Any).returns(());

        rf_set_ble_crc(&[0x12, 0x34, 0x56]);

        mock_write_reg32(0x44c, 0x563412).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg8)]
    fn test_rf_reset_sn() {
        mock_write_reg8(0xf01, Any).returns(());

        rf_reset_sn();

        mock_write_reg8(0xf01, 0x3f).assert_called(1);
        mock_write_reg8(0xf01, 0).assert_called(1);
    }

    #[test]
    #[mry::lock(write_reg8)]
    #[mry::lock(write_reg16)]
    #[mry::lock(write_reg32)]
    #[mry::lock(read_reg8)]
    fn test_rf_start_brx() {
        mock_read_reg8(0xf16).returns(3);
        mock_write_reg32(0xf28, Any).returns(());
        mock_write_reg32(0xf18, Any).returns(());
        mock_write_reg8(0xf16, Any).returns(());
        mock_write_reg8(0xf00, Any).returns(());
        mock_write_reg16(0x50c, Any).returns(());

        rf_start_brx(0x12345678, 0x87654321);

        mock_write_reg32(0xf28, 0xffffffff).assert_called(1);
        mock_write_reg32(0xf18, 0x87654321).assert_called(1);
        mock_write_reg8(0xf16, 4 | 3).assert_called(1);
        mock_write_reg8(0xf00, 0x82).assert_called(1);
        mock_write_reg16(0x50c, 0x5678).assert_called(1);
    }

    // #[test]
    // fn test_rf_start_srx2tx() {
    //     rf_start_srx2tx(0x12345678, 0x87654321);
    //
    //     assert_eq!(read_reg_rf_sched_tick(), 0x87654321);
    //     assert_eq!(read_reg_rf_mode(), 4);
    //     assert_eq!(read_reg_rf_mode_control(), 0x85);
    //     assert_eq!(read_reg_dma3_addr(), 0x5678);
    // }
    //
    // #[test]
    // fn test_rf_start_stx2rx() {
    //     rf_start_stx2rx(0x12345678, 0x87654321);
    //
    //     assert_eq!(read_reg_rf_sched_tick(), 0x87654321);
    //     assert_eq!(read_reg_rf_mode(), 4);
    //     assert_eq!(read_reg_rf_mode_control(), 0x87);
    //     assert_eq!(read_reg_dma3_addr(), 0x5678);
    // }
}