use core::arch::asm;
use core::cmp::min;
use core::ptr::{addr_of, addr_of_mut, null, null_mut, slice_from_raw_parts};
use core::slice;
use crate::config::{get_flash_adr_mac, get_flash_adr_pairing, MESH_PWD, OUT_OF_MESH, PAIR_VALID_FLAG};
use crate::{no_mangle_fn, pub_mut, regrw};
use crate::common::{mesh_node_init, pair_load_key, retrieve_dev_grp_address, rf_update_conn_para};
use crate::mesh::wrappers::{get_mesh_pair_enable, set_get_mac_en};
use crate::ota::wrappers::my_rf_link_slave_data_ota;
use crate::sdk::app_att_light::{attribute_t, get_gAttributes_def};
use crate::sdk::ble_app::shared_mem::get_light_rx_buff;
use crate::sdk::mcu::register::{FLD_RF_IRQ_MASK, read_reg_irq_mask, read_reg_rnd_number, read_reg_system_tick, read_reg_system_tick_mode, REG_BASE_ADDR, write_reg16, write_reg32, write_reg8, write_reg_dma2_addr, write_reg_dma2_ctrl, write_reg_dma_chn_irq_msk, write_reg_irq_mask, write_reg_irq_src, write_reg_rf_irq_mask, write_reg_rf_irq_status, write_reg_system_tick, write_reg_system_tick_irq, write_reg_system_tick_mode};
use crate::sdk::common::compat::{LoadTblCmdSet, TBLCMDSET};
use crate::sdk::common::crc::crc16;
use crate::sdk::drivers::flash::{flash_read_page, flash_write_page};
use crate::sdk::light::{get_pair_config_mesh_ltk, get_pair_config_pwd_encode_enable, get_tick_per_us, rf_packet_ll_data_t, set_slave_p_mac, rf_packet_adv_ind_module_t, rf_packet_scan_rsp_t, rf_packet_att_cmd_t, get_slave_p_mac, rf_packet_att_write_t};
use crate::sdk::mcu::analog::{analog_read, analog_write};
use crate::sdk::mcu::clock::sleep_us;
use crate::sdk::mcu::crypto::encode_password;
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use crate::sdk::mcu::random::rand;
use crate::sdk::mcu::register::{rega_deepsleep_flag, write_reg_rf_rx_gain_agc};
use crate::sdk::pm::light_sw_reboot;
use crate::version::BUILD_VERSION;

const ENABLE_16MHZ_XTAL: bool = false;

pub_mut!(rf_tp_base, u32);
pub_mut!(rf_tp_gain, u32);
pub_mut!(rf_tx_mode, u8);
pub_mut!(rfhw_tx_power, u8);


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

pub_mut!(tbl_agc, [u32; 6]);

pub unsafe fn rf_drv_init(enable: bool) -> u8
{
    // This function is not complete
    let state = irq_disable();
    let result = analog_read(rega_deepsleep_flag) & 0x40;

    rf_tx_mode = 0;
    if enable {
        rfhw_tx_power = 0x40; // FR_TX_PA_MAX_POWER
        if ENABLE_16MHZ_XTAL {
            LoadTblCmdSet(tbl_rf_ini.as_ptr(), tbl_rf_ini.len() as u32);
        } else {
            LoadTblCmdSet(tbl_rf_ini.as_ptr(), tbl_rf_ini.len() as u32 - 4);
        }
        for i in 0..6 {
            write_reg_rf_rx_gain_agc((*get_tbl_agc())[i], (i << 2) as u32)
        }

        if *(*get_flash_adr_mac() as *const u8).offset(0x11) != 0xff {
            let u_var5 = *(*get_flash_adr_mac() as *const u8).offset(0x11) as u32;
            rf_tp_base = u_var5;
            rf_tp_gain = ((u_var5 - 0x19) << 8) / 80;
        }
        if *(*get_flash_adr_mac() as *const u8).offset(0x12) != 0xff {
            let u_var5 = rf_tp_base - *(*get_flash_adr_mac() as *const u8).offset(0x12) as u32;
            rf_tp_gain = (u_var5 << 8) / 80;
        }
    } else {
        analog_write(6, 0);  // power off sar
    }

    irq_restore(state);

    return result;
}

pub fn rf_stop_trx() {
    write_reg8(0x800f00, 0x80);            // stop
}

pub_mut!(light_rx_wptr, u32);
pub unsafe fn blc_ll_initBasicMCU()
{
    write_reg16(0xf0a, 700);

    write_reg_dma2_addr(((((*get_light_rx_buff()).as_ptr() as u32 + light_rx_wptr * 0x40) * 0x10000) >> 0x10) as u16);
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
pub unsafe fn setSppWriteCB(func: fn(data: *const rf_packet_att_write_t) -> bool)
{
    (*gAttributes.offset(21)).w = Some(func);
}
pub unsafe fn setSppOtaWriteCB(func: fn(data: *const rf_packet_att_write_t) -> bool)
{
    (*gAttributes.offset(24)).w = Some(func);
}

pub_mut!(p_st_handler, u32);
pub_mut!(irq_mask_save, u32);
pub_mut!(slave_link_state, u32);
pub_mut!(slave_listen_interval, u32);
pub_mut!(slave_connected_tick, u32);
pub_mut!(slave_adv_enable, bool);
pub_mut!(mac_id, [u8; 6]);
pub_mut!(pkt_adv, rf_packet_adv_ind_module_t);
pub_mut!(pkt_scan_rsp, rf_packet_scan_rsp_t);
pub_mut!(pkt_light_data, rf_packet_att_cmd_t);
pub_mut!(pkt_light_status, rf_packet_att_cmd_t);
pub_mut!(advData, [u8; 3]);
pub_mut!(user_data_len, u8);
pub_mut!(user_data, [u8; 16]);

#[no_mangle]
extern "C" {
    fn irq_st_adv();
}

#[no_mangle]
extern "C" {
    fn rf_link_slave_data_write(data: *const rf_packet_att_write_t) -> bool;
}

fn my_rf_link_slave_data_write(data: *const rf_packet_att_write_t) -> bool {
    unsafe { return rf_link_slave_data_write(data); }
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
        blc_ll_initBasicMCU();
        p_st_handler = irq_st_adv as u32;
        slave_link_state = 0;
        slave_listen_interval = interval * *get_tick_per_us();

        slave_connected_tick = *get_tick_per_us() * 100000 + read_reg_system_tick();
        write_reg_system_tick_irq(slave_connected_tick);

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

        flash_read_page(*get_flash_adr_mac(), 6, mac_id.as_mut_ptr());
        set_slave_p_mac(mac_id.as_ptr());

        pkt_adv.advA = mac_id;
        pkt_scan_rsp.advA = mac_id;

        rf_link_slave_set_adv(&advData);
        pair_load_key();

        pkt_light_data.value[3] = *get_slave_p_mac().offset(0);
        pkt_light_data.value[4] = *get_slave_p_mac().offset(1);

        pkt_light_status.value[3] = *get_slave_p_mac().offset(0);
        pkt_light_status.value[4] = *get_slave_p_mac().offset(1);

        set_slave_adv_enable(true);

        gAttributes = get_gAttributes_def().as_mut_ptr();
        setSppWriteCB(my_rf_link_slave_data_write);
        setSppOtaWriteCB(my_rf_link_slave_data_ota);

        retrieve_dev_grp_address();

        mesh_node_init();

        irq_mask_save = read_reg_irq_mask();

        write_reg32(0x808004, *(*get_slave_p_mac() as *const u32));
        write_reg32(0x808008, BUILD_VERSION);
        write_reg16(0x80800c, crc16(&slice::from_raw_parts(0x808004 as *const u8, 8)));
    }
}