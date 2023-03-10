use std::arch::asm;
use std::ptr::{addr_of, addr_of_mut};
use crate::config::get_flash_adr_mac;
use crate::{no_mangle_fn, pub_mut, regrw};
use crate::common::rf_update_conn_para;
use crate::sdk::mcu::register::{REG_BASE_ADDR, write_reg8};
use crate::sdk::common::compat::{LoadTblCmdSet, TBLCMDSET};
use crate::sdk::light::{_rf_link_add_tx_packet, get_tick_per_us, rf_packet_ll_data_t};
use crate::sdk::mcu::analog::{analog_read, analog_write};
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use crate::sdk::mcu::register::{rega_deepsleep_flag, write_reg_rf_rx_gain_agc};

pub_mut!(rf_tp_base, u32);
pub_mut!(rf_tp_gain, u32);
pub_mut!(rf_tx_mode, u8);
pub_mut!(rfhw_tx_power, u8);


const tbl_rf_ini: [TBLCMDSET; 57] = [
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


    // Maybe stuff for 16mhz xtal
    // TBLCMDSET {     // gauss filter sel: 16M
    //     adr: 0x99,
    //     dat: 0x31,
    //     cmd: 0xc8,
    // },
    // TBLCMDSET {     //enable rxadc clock
    //     adr: 0x82,
    //     dat: 0x14,
    //     cmd: 0xc8,
    // },
    // TBLCMDSET {     //reg_dc_mod (500K)
    //     adr: 0x9e,
    //     dat: 0x41,
    //     cmd: 0xc8,
    // },
    // TBLCMDSET {
    //     adr: 0x4eb,
    //     dat: 0x60,
    //     cmd: 0xc3,
    // },
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
        LoadTblCmdSet(tbl_rf_ini.as_ptr(), 0x39);
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

pub fn rf_stop_trx () {
	write_reg8(0x800f00, 0x80);			// stop
}