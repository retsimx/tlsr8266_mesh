use core::cell::RefCell;
use core::cmp::min;
use core::ptr::addr_of;
use core::slice;

use crate::{app};
use crate::config::*;
use crate::sdk::ble_app::light_ll::setup_ble_parameter_start;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_write_page};
use crate::sdk::light::*;
use crate::sdk::mcu::crypto::{aes_att_encryption, decode_password};
use crate::sdk::rf_drv::{rf_link_slave_set_adv_mesh_name, rf_link_slave_set_adv_private_data};
use crate::sdk::ble_app::rf_drv_8266::get_mac_id;
use crate::state::{State};

const UPDATE_CONN_PARA_CNT: usize = 4;
const CONN_PARA_DATA: [[u16; 3]; UPDATE_CONN_PARA_CNT] = [
    [16, 16 + 16, 420],
    [18, 18 + 16, 420],
    [32, 32 + 16, 420],
    [48, 48 + 16, 420],
];

pub const SYS_CHN_LISTEN: [u8; 4] = [2, 12, 23, 34]; //8, 30, 52, 74

pub const SYS_CHN_ADV: [u8; 3] = [0x25, 0x26, 0x27];

pub const REGA_LIGHT_OFF: u8 = 0x3a;

pub fn dev_addr_with_mac_flag(params: *const u8) -> bool {
    return DEV_ADDR_PAR_WITH_MAC == unsafe { *params.offset(2) };
}

pub fn dev_addr_with_mac_rsp(par_rsp: &mut [u8]) -> bool {
    par_rsp[4..10].copy_from_slice(&*get_mac_id());
    return true;
}

pub fn dev_addr_with_mac_match(state: &RefCell<State>, params: &[u8]) -> bool {
    return if params[0] == 0xff && params[1] == 0xff {
        // get
        state.borrow().get_mac_en
    } else {
        for i in 0..6 {
            if params[i] != (*get_mac_id())[i] {
                return false;
            }
        }
        return true;
    };
}

// adr:0=flag 16=name 32=pwd 48=ltk
// p:data
// n:length
pub fn save_pair_info(adr: u32, p: *const u8, n: u32) {
    flash_write_page(
        (FLASH_ADR_PAIRING as i32 + *get_adr_flash_cfg_idx() + adr as i32) as u32,
        n,
        p,
    );
}

/************
*
* int setup_ble_parameter_start(u16 delay, u16 interval_min, u16 interval_max, u16 timeout);
*
* delay   :  unit: one ble interval
* interval_min,interval_max:  if all 0,will keep the system parameter for android but not ios.   unit: 1.25ms; must longer than 20ms.
* timeout:  if 0,will keep the system parameter.   unit: 10ms; must longer than 3second for steady connect.
*
* return 0 means setup parameters is valid.
* return -1 means parameter of interval is invalid.
* return -2 means parameter of timeout is invalid.
*
*
* void rf_link_slave_connect_callback()
* system will call this function when receive command of BLE connect request.
    IOS Note:
    20 ms <= interval_min
    interval_min + 20 ms <= interval_max <= 2second
    timeout <= 6second
*/
pub fn update_ble_parameter_cb(state: &RefCell<State>) {
    if !state.borrow().conn_update_successed {
        setup_ble_parameter_start(
            1,
            CONN_PARA_DATA[0][0],
            CONN_PARA_DATA[0][1],
            CONN_PARA_DATA[0][2],
        ); // interval 32: means 40ms;   timeout 200: means 2000ms
        state.borrow_mut().conn_update_cnt += 1;
    }
}

pub fn rf_update_conn_para(state: &RefCell<State>, p: &PacketLlData) -> u8 {
    let pp = unsafe { &*(addr_of!(*p) as *const PktL2capSigConnParaUpRsp) };
    let sig_conn_param_update_rsp: [u8; 9] = [0x0A, 0x06, 0x00, 0x05, 0x00, 0x13, 0x01, 0x02, 0x00];
    let mut equal = true;
    for i in 0..9 {
        unsafe {
            if *((&pp.rf_len) as *const u8).offset(i) != sig_conn_param_update_rsp[i as usize] {
                equal = false;
            }
        }
    }

    if equal && (((*pp)._type & 0b11) == 2) {
        //l2cap data pkt, start pkt
        match (*pp).result {
            0x0000 => {
                state.borrow_mut().conn_update_cnt = 0;
                state.borrow_mut().conn_update_successed = true;
            }
            0x0001 => {
                if state.borrow().conn_update_cnt >= UPDATE_CONN_PARA_CNT {
                    state.borrow_mut().conn_update_cnt = 0;
                } else {
                    setup_ble_parameter_start(
                        1,
                        CONN_PARA_DATA[state.borrow().conn_update_cnt][0],
                        CONN_PARA_DATA[state.borrow().conn_update_cnt][1],
                        CONN_PARA_DATA[state.borrow().conn_update_cnt][2],
                    );
                    state.borrow_mut().conn_update_cnt += 1;
                }
            }
            _ => ()
        }
    }

    return 0;
}

pub fn retrieve_dev_grp_address()
{
    let dev_mask = DEVICE_ADDR_MASK_DEFAULT;
    let mut addr_next_pos = *get_dev_address_next_pos();
    let mut grp_next_pos = 0;
    let mut dest_addr_index = 0;
    let mut grp_addr_ptr = FLASH_ADR_DEV_GRP_ADR;
    loop {
        set_dev_address_next_pos(addr_next_pos);
        set_dev_grp_next_pos(grp_next_pos);
        let grp_addr;
        unsafe { grp_addr = *(grp_addr_ptr as *const u16); }

        grp_next_pos = grp_next_pos + 2;
        if grp_addr == 0xffff { break; }
        if grp_addr == 0 {
            set_dev_address_next_pos(addr_next_pos);
        } else {
            if grp_addr & !dev_mask == 0 {
                set_device_address(grp_addr & dev_mask);
                addr_next_pos = grp_next_pos;
                set_dev_grp_next_pos(addr_next_pos);
                set_dev_address_next_pos(addr_next_pos);
            } else {
                unsafe { *(((*get_group_address()).as_ptr() as u32 + (dest_addr_index & 7) * 2) as *mut u16) = grp_addr; }
                dest_addr_index = dest_addr_index + 1;
                set_dev_grp_next_pos(grp_next_pos);
            }
        }
        grp_addr_ptr = grp_addr_ptr + 2;
        if grp_next_pos == 0x1000 {
            break;
        }
    }
    if *get_device_address() == 0 {
        set_device_address(((*get_mac_id())[0] as u16) | (((*get_mac_id())[1] as u16) << 8));
        if (*get_mac_id())[0] == 0 {
            set_device_address(1);
        }
    }
}

pub fn mesh_node_init(state: &RefCell<State>)
{
    app().mesh_manager.mesh_node_buf_init(state);
    state.borrow_mut().mesh_node_st[0].val.dev_adr = *get_device_address() as u8;
    state.borrow_mut().mesh_node_st[0].val.sn = *get_device_node_sn();
    set_mesh_node_max(1);
}

pub fn pair_flash_clean()
{
    let mut flash_dat_swap: [u8; 64] = [0; 64];
    let flash_dat_swap_len = flash_dat_swap.len();

    if *get_adr_flash_cfg_idx() >= 0xeff {
        let src_addr = (FLASH_ADR_PAIRING as i32 + *get_adr_flash_cfg_idx()) as *const u8;
        unsafe { flash_dat_swap.copy_from_slice(slice::from_raw_parts(src_addr, flash_dat_swap_len)); }

        flash_erase_sector(FLASH_ADR_PAIRING);
        flash_write_page(FLASH_ADR_PAIRING, flash_dat_swap_len as u32, flash_dat_swap.as_ptr());
        set_adr_flash_cfg_idx(0);
    }
}

pub fn pair_flash_config_init() -> bool
{
    let result;

    unsafe {
        if PAIR_CONFIG_VALID_FLAG == *(FLASH_ADR_PAIRING as *const u8) {
            let mut index = 0x40;
            loop {
                if *(FLASH_ADR_PAIRING as *const u8).offset(index) != *(FLASH_ADR_PAIRING as *const u8) {
                    break;
                }
                index = index + 0x40;
                if index == 0x1000 {
                    break;
                }
            };
            set_adr_flash_cfg_idx(index as i32 + -0x40);
            pair_flash_clean();
            result = true;
        } else {
            set_adr_flash_cfg_idx(-0x41);
            result = false;
        }
    }
    return result;
}

pub fn access_code(name: &[u8], pass: &[u8]) -> u32
{
    let mut destbuf = [0u32; 4];

    // todo: Why is this called twice? It's called twice in the original code...
    aes_att_encryption(name.as_ptr(), pass.as_ptr(), destbuf.as_mut_ptr() as *mut u8);
    aes_att_encryption(name.as_ptr(), pass.as_ptr(), destbuf.as_mut_ptr() as *mut u8);

    let mut bit = destbuf[0] >> 1 ^ destbuf[0];
    let mut inner_count = 0;
    for bit_count in 1..0x20 {
        if bit & 1 == 0 {
            break;
        }
        inner_count = inner_count + 1;
        if 5 < inner_count {
            destbuf[0] = destbuf[0] ^ 1 << bit_count;
            inner_count = 0;
        }
    }

    bit = 0xaaaaaaaa;
    for _ in 0..2 {
        inner_count = 0;

        for bit_count in 0..0x20 {
            inner_count = inner_count + if (1 << bit_count & bit) as u32 != 0 { 1 } else { 0 };
        }

        if inner_count < 3 {
            destbuf[0] = destbuf[0] ^ 0xff;
        }

        bit = destbuf[0] ^ 0x55555555;
    }

    return destbuf[0];
}

pub fn pair_update_key(state: &RefCell<State>)
{
    set_pair_ac(access_code(&(*get_pair_nn())[0..16], &(*get_pair_pass())[0..16]));
    let name_len = match (*get_pair_nn()).iter().position(|r| *r == 0) {
        Some(v) => v,
        None => get_pair_nn().len()
    };

    let name_len = min(state.borrow().max_mesh_name_len, name_len);

    rf_link_slave_set_adv_mesh_name(&(*get_pair_nn())[0..name_len as usize]);
    rf_link_slave_set_adv_private_data(unsafe { slice::from_raw_parts(*get_p_adv_pri_data() as *const u8, *get_adv_private_data_len() as usize) });
}

pub fn pair_load_key(state: &RefCell<State>)
{
    pair_flash_config_init();

    let pairing_addr = FLASH_ADR_PAIRING as i32 + *get_adr_flash_cfg_idx();

    if -1 < *get_adr_flash_cfg_idx() && pairing_addr != 0x0 {
        get_pair_nn().iter_mut().for_each(|v| { *v = 0 });
        get_pair_pass().iter_mut().for_each(|v| { *v = 0 });
        get_pair_ltk().iter_mut().for_each(|v| { *v = 0 });

        (*get_pair_nn()).copy_from_slice(unsafe { slice::from_raw_parts((pairing_addr + 0x10) as *const u8, state.borrow().max_mesh_name_len) });
        (*get_pair_pass()).copy_from_slice(unsafe { slice::from_raw_parts((pairing_addr + 0x20) as *const u8, 0x10) });

        if state.borrow().mesh_pair_enable {
            state.borrow_mut().get_mac_en = unsafe { *(pairing_addr as *const bool).offset(0x1) };
        }

        let pair_config_flag = unsafe { *(pairing_addr as *const u8).offset(0xf) };
        if pair_config_flag == PAIR_CONFIG_VALID_FLAG {
            decode_password((*get_pair_pass()).as_mut_slice());
        }

        (*get_pair_ltk()).copy_from_slice(unsafe { slice::from_raw_parts((pairing_addr + 0x30) as *const u8, 0x10) });

        pair_update_key(state);
    }
}