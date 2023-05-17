use core::arch::asm;
use core::cmp::min;
use core::mem::transmute;
use core::ptr::{addr_of, addr_of_mut, null, null_mut, slice_from_raw_parts};
use core::slice;
use crate::config::{get_flash_adr_mac, get_flash_adr_pairing, MESH_PWD, OUT_OF_MESH, PAIR_VALID_FLAG};
use crate::{BIT, pub_mut, pub_static, regrw, uprintln};
use crate::common::{dev_addr_with_mac_flag, mesh_node_init, pair_load_key, retrieve_dev_grp_address, rf_update_conn_para};
use crate::main_light::{rf_link_data_callback, rf_link_response_callback};
use crate::mesh::{get_mesh_pair_enable, set_get_mac_en};
use crate::ota::rf_link_slave_data_ota;
use crate::sdk::app_att_light::{attribute_t, get_gAttributes_def};
use crate::sdk::ble_app::ble_ll_pair::pair_dec_packet;
use crate::sdk::ble_app::light_ll::{copy_par_user_all, get_bridge_max_cnt, get_p_slave_status_buffer, get_slave_link_interval, get_slave_status_buffer_num, rf_link_get_op_para, rf_link_is_notify_req, rf_link_match_group_mac, rf_link_slave_add_status, rf_link_slave_read_status_par_init, rf_link_slave_read_status_stop, set_p_st_handler};
use crate::sdk::ble_app::ll_irq::irq_st_adv;
use crate::sdk::ble_app::shared_mem::get_light_rx_buff;
use crate::sdk::mcu::register::{FLD_RF_IRQ_MASK, read_reg8, read_reg_irq_mask, read_reg_rnd_number, read_reg_system_tick, read_reg_system_tick_mode, REG_BASE_ADDR, write_reg16, write_reg32, write_reg8, write_reg_rf_access_code, write_reg_dma2_addr, write_reg_dma2_ctrl, write_reg_dma_chn_irq_msk, write_reg_irq_mask, write_reg_irq_src, write_reg_rf_irq_mask, write_reg_rf_irq_status, write_reg_system_tick, write_reg_system_tick_irq, write_reg_system_tick_mode, write_reg_rf_crc, write_reg_rf_sn, write_reg_rf_sched_tick, write_reg_rf_mode_control, write_reg_rf_mode, read_reg_rf_mode, write_reg_dma3_addr};
use crate::sdk::common::compat::{load_tbl_cmd_set, TBLCMDSET};
use crate::sdk::common::crc::crc16;
use crate::sdk::drivers::flash::{flash_read_page, flash_write_page};
use crate::sdk::light::{*};
use crate::sdk::mcu::analog::{analog_read, analog_write};
use crate::sdk::mcu::clock::sleep_us;
use crate::sdk::mcu::crypto::encode_password;
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use crate::sdk::mcu::random::rand;
use crate::sdk::mcu::register::{rega_deepsleep_flag, write_reg_rf_rx_gain_agc};
use crate::sdk::pm::light_sw_reboot;
use crate::version::BUILD_VERSION;

const ENABLE_16MHZ_XTAL: bool = false;

const RF_FAST_MODE: bool = true;
const RF_TRX_MODE: u8 = 0x80;
const RF_TRX_OFF: u8 = 0x45;

pub_mut!(rf_tp_base, u32, 0x1D);
pub_mut!(rf_tp_gain, u32, 0xC);
pub_mut!(rf_tx_mode, u8, 0);
pub_mut!(rfhw_tx_power, u8, 0x40);
pub_mut!(FtoRX, bool, false);

static tbl_agc: [u32; 7] = [0x30333231, 0x182C3C38, 0xC0C1C, 0x0, 0x1B150F0A, 0x322E2721, 0x3E38];

const tbl_rf_ini: [TBLCMDSET; 61] = [
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
    // This function is not complete
    let state = irq_disable();
    let result = analog_read(rega_deepsleep_flag) & 0x40;

    set_rf_tx_mode(0);
    if enable {
        set_rfhw_tx_power(0x40); // FR_TX_PA_MAX_POWER
        if ENABLE_16MHZ_XTAL {
            load_tbl_cmd_set(tbl_rf_ini.as_ptr(), tbl_rf_ini.len() as u32);
        } else {
            load_tbl_cmd_set(tbl_rf_ini.as_ptr(), tbl_rf_ini.len() as u32 - 4);
        }

        // todo: Should this be 0..7? There's an extra couple of bytes of data in the 7th int
        for i in 0..6 {
            write_reg_rf_rx_gain_agc(tbl_agc[i], (i << 2) as u32)
        }

        if *(*get_flash_adr_mac() as *const u8).offset(0x11) != 0xff {
            let u_var5 = *(*get_flash_adr_mac() as *const u8).offset(0x11) as u32;
            set_rf_tp_base(u_var5);
            set_rf_tp_gain(((u_var5 - 0x19) << 8) / 80);
        }
        if *(*get_flash_adr_mac() as *const u8).offset(0x12) != 0xff {
            let u_var5 = *get_rf_tp_base() - *(*get_flash_adr_mac() as *const u8).offset(0x12) as u32;
            set_rf_tp_gain((u_var5 << 8) / 80);
        }
    } else {
        analog_write(6, 0);  // power off sar
    }

    irq_restore(state);

    return result;
}

pub fn rf_stop_trx() {
    write_reg_rf_mode_control(0x80);            // stop
}

pub_mut!(light_rx_wptr, u32, 0);
pub unsafe fn blc_ll_init_basic_mcu()
{
    write_reg16(0xf0a, 700);

    write_reg_dma2_addr(addr_of!((*get_light_rx_buff())[*get_light_rx_wptr() as usize]) as u16);
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

pub_mut!(gAttributes, *mut attribute_t, null_mut());
pub unsafe fn set_spp_write_cb(func: fn(data: *const rf_packet_att_write_t) -> bool)
{
    (*gAttributes.0.offset(21)).w = Some(func);
}

pub unsafe fn set_spp_ota_write_cb(func: fn(data: *const rf_packet_att_write_t) -> bool)
{
    (*gAttributes.0.offset(24)).w = Some(func);
}

pub_mut!(irq_mask_save, u32, 0);
pub_mut!(slave_link_state, u32, 0);
pub_mut!(slave_listen_interval, u32, 0);
pub_mut!(slave_connected_tick, u32, 0);
pub_mut!(slave_adv_enable, bool, false);
pub_mut!(slave_connection_enable, bool, false);
pub_mut!(mac_id, [u8; 6], [0; 6]);
pub_mut!(pkt_ibeacon, rf_packet_adv_ind_module_t, rf_packet_adv_ind_module_t {
    dma_len: 0x27,
    _type: 0,
    rf_len: 0x25,
    advA: [0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5],
    data: [0; 31]
});
pub_mut!(pkt_adv, rf_packet_adv_ind_module_t, rf_packet_adv_ind_module_t {
    dma_len: 0x27,
    _type: 0,
    rf_len: 0x25,
    advA: [0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5],
    data: [0; 31]
});
pub_mut!(pkt_scan_rsp, rf_packet_scan_rsp_t, rf_packet_scan_rsp_t {
    dma_len: 0x27,
    _type: 0x4,
    rf_len: 0x25,
    advA: [0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5],
    data: [0; 31]
});
pub_mut!(pkt_light_data, rf_packet_att_cmd_t, rf_packet_att_cmd_t {
    dma_len: 0x27,
    _type: 2,
    rf_len: 0x25,
    l2capLen: 0xCCDD,
    chanId: 0,
    opcode: 0,
    handle: 0,
    handle1: 0,
    value: [0; 30]
});
pub_mut!(pkt_light_status, rf_packet_att_cmd_t, rf_packet_att_cmd_t {
    dma_len: 0x27,
    _type: 2,
    rf_len: 0x25,
    l2capLen: 0x21,
    chanId: 0,
    opcode: 0,
    handle: 0,
    handle1: 0,
    value: [0; 30]
});
pub_mut!(adv_data, [u8; 3], [2, 1, 5]);
pub_mut!(user_data_len, u8, 0);
pub_mut!(user_data, [u8; 16], [0; 16]);

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
    }
    else {
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

fn rf_link_slave_read_status_start()
{
  set_slave_read_status_busy_time(read_reg_system_tick());
  set_slave_read_status_busy(rf_link_get_rsp_type((*get_pkt_light_data()).value[7] & 0x3f,(*get_pkt_light_data()).value[11]));
  set_slave_read_status_unicast_flag(!(*get_pkt_light_data()).value[6] >> 7);
  if -1 < (!(((*get_pkt_light_data()).value[6] as u32) << 0x18)) as i32 {
    (*get_pkt_light_data()).value[15..15+4].fill(0);
    (*get_pkt_light_data()).value[19] = *get_slave_read_status_unicast_flag();
  }
  if (*get_pkt_light_data()).value[7] & 0x3f == 0x1a {
    (*get_pkt_light_data()).value[11] = 0;
    (*get_pkt_light_data()).value[12] = 0;
    (*get_pkt_light_data()).value[13] = 0;
    (*get_pkt_light_data()).value[14] = 0;
      (*get_pkt_light_data()).value[15..15+4].fill(0);
    (*get_pkt_light_data()).value[10] = *get_slave_status_tick();
  }
  rf_link_slave_read_status_par_init();
    unsafe {
        slice::from_raw_parts_mut(
            *get_p_slave_status_buffer() as *mut u8,
            *get_slave_status_buffer_num() as usize * 0x24
        ).fill(0)
    }

  set_slave_status_record_idx(0);
    (*get_slave_status_record()).fill(
        status_record_t { adr: [0; 1], alarm_id: 0 }
    );

  set_notify_req_mask_idx(0);
}

fn rf_link_slave_data_write_no_dec(data: &mut rf_packet_att_write_t) -> bool {
    if data.rf_len < 0x11 {
        return false;
    }

    let sno = &data.value[0..3];
    if sno == *get_slave_sno() {
        return true;
    }

    let mut op_cmd: [u8; 3] = [0; 3];
    let mut op_cmd_len: u8 = 0;
    let mut params: [u8; 16] = [0; 16];
    let mut params_len: u8 = 0;
    rf_link_get_op_para(
        unsafe { addr_of!(data.l2capLen) as *const ll_packet_l2cap_data_t },
        &mut op_cmd,
        &mut op_cmd_len,
        &mut params,
        &mut params_len,
        false,
    );

    let (result1, result2) = rf_link_match_group_mac(sno.as_ptr() as *const app_cmd_value_t);
    let op;
    let mut op_valid;
    let mut tmp;
    if op_cmd_len == 3 {
        op = op_cmd[0] & 0x3f;
        uprintln!("check here 2");
        // todo: Might need to be ==
        op_valid = op - 26 != 0;
        if op_valid != false {
            op_valid = true;
            if params[1] != 0 {
                tmp = data.value[6] as u32;
            } else {
                tmp = data.value[6] as u32;
                if ((tmp * 0x1000000) as i32) < 0 {
                    return false;
                }
            }
        };
        tmp = data.value[6] as u32;
    } else {
        op = 0;
        op_valid = true;
        if params[1] != 0 {
            tmp = data.value[6] as u32;
        } else {
            tmp = data.value[6] as u32;
            if ((tmp * 0x1000000) as i32) < 0 {
                return false;
            }
        }
    }
    if op == 0x20 && ((tmp * 0x1000000) as i32) < 0 {
        if params[0] != 0xff {
            return false;
        }
        if params[1] != 0xff {
            return false;
        }
        tmp = tmp << 8 | data.value[5] as u32;
    } else {
        tmp = tmp << 8 | data.value[5] as u32;
        if ((!tmp << 0x10) as i32) < 0 && op == 0x10 {
            if tmp == 0 {
                tmp = *get_device_address() as u32;
            }
            uprintln!("stub: mesh_node_check_force_notify")
            // mesh_node_check_force_notify(uVar4, params[0]);
        }
    }
    (*get_pkt_light_data()).value.fill(0);
    (*get_pkt_light_status()).value.fill(0);

    unsafe {
        slice::from_raw_parts_mut(
            addr_of!((*get_pkt_light_data()).l2capLen) as *mut u8,
            params_len as usize + 0x11,
        ).copy_from_slice(
            slice::from_raw_parts(
                addr_of!(data.l2capLen) as *const u8,
                params_len as usize + 0x11,
            )
        )
    }

    (*get_pkt_light_data()).chanId = 0xff03;
    (*get_pkt_light_data()).dma_len = 0x27;
    (*get_pkt_light_data()).rf_len = 0x25;
    (*get_pkt_light_data()).l2capLen = 0x21;
    // todo: What type is pkt_light_data here?
    unsafe { *(addr_of!((*get_pkt_light_data()).opcode) as *mut u16) = *get_device_address() };
    unsafe { *(addr_of!((*get_pkt_light_data()).value[3]) as *mut u16) = *get_device_address() };
    set_app_cmd_time(read_reg_system_tick());
    (*get_pkt_light_data()).value[25] = *get_max_relay_num();

    if result2 != false || result1 != false {
        rf_link_data_callback(addr_of!((*get_pkt_light_data()).l2capLen) as *const ll_packet_l2cap_data_t);
    }
    get_slave_sno().copy_from_slice(sno);

    set_slave_link_cmd(op);
    get_slave_sno_sending().copy_from_slice(sno);

    if rf_link_is_notify_req(op) == false {
        if *get_slave_read_status_busy() != 0 {
            rf_link_slave_read_status_stop();
        }
        if result2 == false && unsafe { *(addr_of!(data.value[5]) as *const u16) } != 0 {
            if op == 6 {
                set_slave_data_valid(0x21);
                if (data.value[0xb] as u16 * 0x100 + data.value[10] as u16) < 0xff00 {
                    uprintln!("check here 4");
                    // todo: 9 may need to be 5 instead
                    set_slave_data_valid(9);
                }
            } else {
                set_slave_data_valid(*get_bridge_max_cnt() + 1);
            }
        } else {
            set_slave_data_valid(0);
        }
        return true;
    }

    (*get_pkt_light_status()).value[20] = 0;
    (*get_pkt_light_status()).value[21] = 0;
    (*get_pkt_light_status()).value[25] = *get_max_relay_num();
    (*get_pkt_light_data()).value[23] = (*get_slave_link_interval() / (*get_tick_per_us() * 1000)) as u8;
    if result2 == false || (tmp != 0 && op == 0x20 && dev_addr_with_mac_flag(params.as_ptr()) != false) {
        if op == 0x1d || op == 0x1a || op_valid != false || op == 0x2a || op == 0x28 || op == 7 {
            set_slave_data_valid(params[0] as u32 * 2 + 1);
            if op == 0x1d {
                (*get_pkt_light_data()).value[22] = params[1];
            } else if op == 0x17 {
                (*get_pkt_light_data()).value[22] = 1;
            } else {
                if op == 0x20 {
                    (*get_pkt_light_data()).value[22] = 4;
                }
                if op == 0x26 {
                    (*get_pkt_light_data()).value[22] = 5;
                } else if op == 0 {
                    (*get_pkt_light_data()).value[22] = 8;
                } else if op == 0x28 {
                    (*get_pkt_light_data()).value[22] = 6;
                } else if op == 0x2a {
                    (*get_pkt_light_data()).value[22] = 7;
                } else if op == 7 {
                    (*get_pkt_light_data()).value[22] = 9;
                } else {
                    (*get_pkt_light_data()).value[22] = 0;
                }
            }
        } else if op != 0x20 {
            if op != 0x17 {
                if op == 0x1d {
                    (*get_pkt_light_data()).value[22] = params[1];
                } else if op == 0x17 {
                    (*get_pkt_light_data()).value[22] = 1;
                } else {
                    if op == 0x20 {
                        (*get_pkt_light_data()).value[22] = 4;
                    }
                    if op == 0x26 {
                        (*get_pkt_light_data()).value[22] = 5;
                    } else if op == 0 {
                        (*get_pkt_light_data()).value[22] = 8;
                    } else if op == 0x28 {
                        (*get_pkt_light_data()).value[22] = 6;
                    } else if op == 0x2a {
                        (*get_pkt_light_data()).value[22] = 7;
                    } else if op == 7 {
                        (*get_pkt_light_data()).value[22] = 9;
                    } else {
                        (*get_pkt_light_data()).value[22] = 0;
                    }
                }
            }
            set_slave_data_valid(*get_bridge_max_cnt() as u32 * 2 + 1);
            (*get_pkt_light_data()).value[22] = 1;
        } else if dev_addr_with_mac_flag(params.as_ptr()) == false {
            set_slave_data_valid(*get_bridge_max_cnt() as u32 * 2 + 1);
            (*get_pkt_light_data()).value[22] = 4;
        } else {
            set_slave_data_valid(params[3] as u32 * 2 + 1);
            (*get_pkt_light_data()).value[22] = 4;
        }
    } else {
        set_slave_data_valid(0);
        if op == 0x1d {
            (*get_pkt_light_data()).value[22] = params[1];
        } else if op == 0x17 {
            (*get_pkt_light_data()).value[22] = 1;
        } else {
            if op == 0x20 {
                (*get_pkt_light_data()).value[22] = 4;
            }
            if op == 0x26 {
                (*get_pkt_light_data()).value[22] = 5;
            } else if op == 0 {
                (*get_pkt_light_data()).value[22] = 8;
            } else if op == 0x28 {
                (*get_pkt_light_data()).value[22] = 6;
            } else if op == 0x2a {
                (*get_pkt_light_data()).value[22] = 7;
            } else if op == 7 {
                (*get_pkt_light_data()).value[22] = 9;
            } else {
                (*get_pkt_light_data()).value[22] = 0;
            }
        }
    }
    (*get_pkt_light_status()).value[22] = (*get_pkt_light_data()).value[22];
    rf_link_slave_read_status_start();

    get_slave_stat_sno().copy_from_slice(sno);

    if result2 != false || result1 != false {
        copy_par_user_all(params_len as u32, (*get_pkt_light_data()).value[10..].as_mut_ptr());
        (*get_pkt_light_status()).value[0..3].copy_from_slice(&*get_slave_sno());

        unsafe { *(addr_of!((*get_pkt_light_status()).value[3]) as *mut u16) = *get_device_address() };

        let tmp_pkt = rf_packet_att_value_t {
            sno: [0; 3],
            src: [0; 2],
            dst: [0; 2],
            val: [0; 23],
        };

        unsafe {
            slice::from_raw_parts_mut(
                addr_of!(tmp_pkt) as *mut u8,
                0x1e,
            ).copy_from_slice(&data.value);
        }

        unsafe {
            *(addr_of!(tmp_pkt.src) as *mut u16) = *get_device_address();
        }

        if rf_link_response_callback(addr_of!((*get_pkt_light_status()).value) as *mut rf_packet_att_value_t, &tmp_pkt) != false {
            rf_link_slave_add_status(unsafe { &*(get_pkt_light_status_addr() as *const mesh_pkt_t) });
        }
    }

    return true;
}

fn rf_link_slave_data_write(data: *const rf_packet_att_write_t) -> bool {
    if *get_pair_login_ok() && unsafe { pair_dec_packet(data as *mut rf_packet_ll_app_t) } {
        return rf_link_slave_data_write_no_dec(unsafe { &mut *(data as *mut rf_packet_att_write_t) });
    }

    return false;
}

fn rf_link_slave_set_adv(adv_data_ptr: &[u8])
{
    get_pkt_adv().data[0..adv_data_ptr.len()].copy_from_slice(&adv_data_ptr[0..adv_data_ptr.len()]);
    get_pkt_adv().dma_len = adv_data_ptr.len() as u32 + 8;
    get_pkt_adv().rf_len = adv_data_ptr.len() as u8 + 6;
}

pub fn rf_link_slave_init(interval: u32)
{
    unsafe {
        blc_ll_init_basic_mcu();
        set_p_st_handler(Some(irq_st_adv));
        set_slave_link_state(0);
        set_slave_listen_interval(interval * *get_tick_per_us());

        set_slave_connected_tick(*get_tick_per_us() * 100000 + read_reg_system_tick());
        write_reg_system_tick_irq(*get_slave_connected_tick());

        if *get_tick_per_us() == 0x10 {
            write_reg8(0xf04, 0x5e);
        } else {
            write_reg8(0xf04, 0x68);
        }

        if *(*get_flash_adr_mac() as *const u32) == u32::MAX {
            let mac: [u16; 2] = [rand(), rand()];
            flash_write_page(*get_flash_adr_mac(), 4, mac.as_ptr() as *const u8);
        }

        let pair_addr = *(*get_flash_adr_pairing() as *const u32) + 1;
        if pair_addr == 0 {
            let pairing_addr = *get_flash_adr_pairing();
            let mut buff: [u8; 16] = [0; 16];
            buff[0..16].copy_from_slice(&get_pair_config_mesh_ltk()[0..16]);
            flash_write_page(pairing_addr + 48, 16, buff.as_mut_ptr());

            let mut buff: [u8; 16] = [0; 16];
            let len = min(MESH_PWD.len(), buff.len());
            buff[0..len].copy_from_slice(&MESH_PWD.as_bytes()[0..len]);
            encode_password(buff.as_mut_slice());
            flash_write_page(pairing_addr + 32, 16, buff.as_mut_ptr());

            let mut buff: [u8; 16] = [0; 16];
            let len = min(OUT_OF_MESH.len(), buff.len());
            buff[0..len].copy_from_slice(&OUT_OF_MESH.as_bytes()[0..len]);
            flash_write_page(pairing_addr + 16, 16, buff.as_mut_ptr());

            let mut buff: [u8; 16] = [0; 16];
            buff[0] = PAIR_VALID_FLAG;
            if *get_pair_config_pwd_encode_enable() != false {
                buff[15] = PAIR_VALID_FLAG;
            }

            if *get_mesh_pair_enable() {
                set_get_mac_en(true);
                buff[1] = 1;
            }
            flash_write_page(pairing_addr, 16, buff.as_mut_ptr());
            irq_disable();
            light_sw_reboot();
            loop {}
        }

        flash_read_page(*get_flash_adr_mac(), 6, (*get_mac_id()).as_mut_ptr());
        set_slave_p_mac((*get_mac_id()).as_ptr());

        (*get_pkt_adv()).advA = *get_mac_id();
        (*get_pkt_scan_rsp()).advA = *get_mac_id();

        rf_link_slave_set_adv(get_adv_data());
        pair_load_key();

        (*get_pkt_light_data()).value[3] = *get_slave_p_mac().offset(0);
        (*get_pkt_light_data()).value[4] = *get_slave_p_mac().offset(1);

        (*get_pkt_light_status()).value[3] = *get_slave_p_mac().offset(0);
        (*get_pkt_light_status()).value[4] = *get_slave_p_mac().offset(1);

        set_slave_adv_enable(true);

        set_gAttributes(get_gAttributes_def().as_mut_ptr());
        set_spp_write_cb(rf_link_slave_data_write);
        set_spp_ota_write_cb(rf_link_slave_data_ota);

        retrieve_dev_grp_address();

        mesh_node_init();

        set_irq_mask_save(read_reg_irq_mask());

        write_reg32(0x808004, *(*get_slave_p_mac() as *const u32));
        write_reg32(0x808008, BUILD_VERSION);
        write_reg16(0x80800c, crc16(&slice::from_raw_parts(0x808004 as *const u8, 8)));
    }
}

struct tbl_rf_power_t {
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
}

static TBL_RF_POWER: [tbl_rf_power_t; 12] = [
    tbl_rf_power_t {
        a: 0x25,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    tbl_rf_power_t {
        a: 0x0a,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    tbl_rf_power_t {
        a: 0x06,
        b: 0x74,
        c: 0x43,
        d: 0x61,
    },
    tbl_rf_power_t {
        a: 0x06,
        b: 0x64,
        c: 0xc2,
        d: 0x61,
    },
    tbl_rf_power_t {
        a: 0x06,
        b: 0x64,
        c: 0xc1,
        d: 0x61,
    },
    tbl_rf_power_t {
        a: 0x05,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    tbl_rf_power_t {
        a: 0x03,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    tbl_rf_power_t {
        a: 0x02,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    tbl_rf_power_t {
        a: 0x01,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    tbl_rf_power_t {
        a: 0x00,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    tbl_rf_power_t {
        a: 0x00,
        b: 0x64,
        c: 0x43,
        d: 0x61,
    },
    tbl_rf_power_t {
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

#[link_section = ".ram_code"]
pub fn rf_set_ble_channel(mut chn: u8) {
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
    unsafe { analog_write(0x93, *get_rf_tp_base() as u8 - ((gain as u32 * *get_rf_tp_gain() + 0x80) >> 8) as u8); }
}

pub fn rf_start_stx2rx(addr: u32, tick: u32)
{
    write_reg_rf_sched_tick(tick);                                // Setting schedule trigger time
    write_reg_rf_mode(read_reg_rf_mode() | 0x04);    // Enable cmd_schedule mode
    write_reg_rf_mode_control(0x87);                                // Set mode
    write_reg_dma3_addr(addr as u16);

    set_FtoRX(true);
}

pub fn rf_start_srx2tx(addr: u32, tick: u32)
{
    write_reg_rf_sched_tick(tick);                                // Setting schedule trigger time
    write_reg_rf_mode(read_reg_rf_mode() | 0x04);    // Enable cmd_schedule mode
    write_reg_rf_mode_control(0x85);                                // Set mode
    write_reg_dma3_addr(addr as u16);
}

pub fn rf_start_brx(addr: u32, tick: u32)
{
    write_reg32(0x800f28, 0xffffffff);                            // ?
    write_reg_rf_sched_tick(tick);                                // Setting schedule trigger time
    write_reg_rf_mode(read_reg_rf_mode() | 0x04);    // Enable cmd_schedule mode
    write_reg_rf_mode_control(0x82);                                // Set mode
    write_reg_dma3_addr(addr as u16);
}

pub fn rf_start_beacon(addr: u32, tick: u32)
{
    write_reg_rf_sched_tick(tick);                                // Setting schedule trigger time
    write_reg_rf_mode(read_reg_rf_mode() | 0x04);    // Enable cmd_schedule mode
    write_reg_rf_mode_control(0x84);                                // Set mode
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
    use crate::sdk::ble_app::rf_drv_8266::*;
    use crate::sdk::mcu::register::{read_reg32, read_reg_dma3_addr, read_reg_rf_access_code, read_reg_rf_crc, read_reg_rf_mode_control, read_reg_rf_sched_tick, read_reg_rf_sn};

    #[test]
    fn test_rf_set_ble_access_code_adv() {
        rf_set_ble_access_code_adv();
        assert_eq!(read_reg_rf_access_code(), 0xd6be898e);
    }

    #[test]
    fn test_rf_set_ble_access_code() {
        // Should swap the order of the bytes
        rf_set_ble_access_code(0x12345678);
        assert_eq!(read_reg_rf_access_code(), 0x78563412);

        rf_set_ble_access_code(read_reg_rf_access_code());
        assert_eq!(read_reg_rf_access_code(), 0x12345678);
    }

    #[test]
    fn test_rf_set_ble_crc_adv() {
        rf_set_ble_crc_adv();

        assert_eq!(read_reg_rf_crc(), 0x555555);
    }

    #[test]
    fn test_rf_set_ble_crc() {
        // todo: Is this really true?
        rf_set_ble_crc(&[0x12, 0x34, 0x56]);

        assert_eq!(read_reg_rf_crc(), 0x563412);
    }

    #[test]
    fn test_rf_reset_sn() {
        rf_reset_sn();

        assert_eq!(read_reg_rf_sn(), 0);
    }

    #[test]
    fn test_rf_start_beacon() {
        rf_start_beacon(0x12345678, 0x87654321);

        assert_eq!(read_reg_rf_sched_tick(), 0x87654321);
        assert_eq!(read_reg_rf_mode(), 4);
        assert_eq!(read_reg_rf_mode_control(), 0x84);
        assert_eq!(read_reg_dma3_addr(), 0x5678);
    }

    #[test]
    fn test_rf_start_brx() {
        rf_start_brx(0x12345678, 0x87654321);

        assert_eq!(read_reg32(0xf28), 0xffffffff);
        assert_eq!(read_reg_rf_sched_tick(), 0x87654321);
        assert_eq!(read_reg_rf_mode(), 4);
        assert_eq!(read_reg_rf_mode_control(), 0x82);
        assert_eq!(read_reg_dma3_addr(), 0x5678);
    }

    #[test]
    fn test_rf_start_srx2tx() {
        rf_start_srx2tx(0x12345678, 0x87654321);

        assert_eq!(read_reg_rf_sched_tick(), 0x87654321);
        assert_eq!(read_reg_rf_mode(), 4);
        assert_eq!(read_reg_rf_mode_control(), 0x85);
        assert_eq!(read_reg_dma3_addr(), 0x5678);
    }

    #[test]
    fn test_rf_start_stx2rx() {
        rf_start_stx2rx(0x12345678, 0x87654321);

        assert_eq!(read_reg_rf_sched_tick(), 0x87654321);
        assert_eq!(read_reg_rf_mode(), 4);
        assert_eq!(read_reg_rf_mode_control(), 0x87);
        assert_eq!(read_reg_dma3_addr(), 0x5678);
    }
}