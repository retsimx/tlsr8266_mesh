use core::cmp::min;
use core::mem::size_of;
use core::ptr::addr_of;
use core::slice;

use crate::app;
use crate::config::*;
use crate::sdk::ble_app::light_ll::connection_management::setup_ble_parameter_start;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};
use crate::sdk::light::*;
use crate::sdk::mcu::crypto::{aes_att_encryption, decode_password};
use crate::sdk::packet_types::Packet;
use crate::sdk::rf_drv::{set_advertisement_manufacturer_data, set_advertisement_mesh_name};
use crate::state::*;

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

#[cfg_attr(test, mry::mry)]
pub fn dev_addr_with_mac_flag(params: &[u8]) -> bool {
    DEV_ADDR_PAR_WITH_MAC == params[2]
}

#[cfg_attr(test, mry::mry)]
pub fn dev_addr_with_mac_rsp(par_rsp: &mut [u8]) -> bool {
    par_rsp[4..10].copy_from_slice(&*MAC_ID.lock());
    true
}

#[cfg_attr(test, mry::mry)]
pub fn dev_addr_with_mac_match(params: &[u8]) -> bool {
    if params[0] == 0xff && params[1] == 0xff {
        // get - return current validation pending state
        MESH_DEVICE_ADDRESS_VALIDATION_PENDING.get()
    } else {
        return params[0..6] == *MAC_ID.lock();
    }
}

// adr:0=flag 16=name 32=pwd 48=ltk
// p:data
// n:length
#[cfg_attr(test, mry::mry)]
pub fn save_pair_info(adr: u32, p: &[u8]) {
    flash_write_page(
        (FLASH_ADR_PAIRING as i32 + FLASH_CONFIGURATION_INDEX.get() + adr as i32) as u32,
        p.len() as u32,
        p.as_ptr(),
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
#[cfg_attr(test, mry::mry)]
pub fn update_ble_parameter_cb() {
    if !CONN_UPDATE_SUCCESSED.get() {
        setup_ble_parameter_start(
            CONN_PARA_DATA[0][0],
            CONN_PARA_DATA[0][1],
            CONN_PARA_DATA[0][2] as u32,
        ); // interval 32: means 40ms;   timeout 200: means 2000ms
        CONN_UPDATE_CNT.inc();
    }
}

pub fn rf_update_conn_para(p: &Packet) -> u8 {
    let sig_conn_param_update_rsp: [u8; 9] = [0x0A, 0x06, 0x00, 0x05, 0x00, 0x13, 0x01, 0x02, 0x00];
    let mut equal = true;
    for i in 0..9 {
        unsafe {
            if *((&p.head().rf_len) as *const u8).offset(i) != sig_conn_param_update_rsp[i as usize]
            {
                equal = false;
            }
        }
    }

    if equal && (p.head()._type & 0b11) == 2 {
        //l2cap data pkt, start pkt
        match p.sig_conn_para_up_rsp().result {
            0x0000 => {
                CONN_UPDATE_CNT.set(0);
                CONN_UPDATE_SUCCESSED.set(true);
            }
            0x0001 => {
                if CONN_UPDATE_CNT.get() >= UPDATE_CONN_PARA_CNT {
                    CONN_UPDATE_CNT.set(0);
                } else {
                    setup_ble_parameter_start(
                        CONN_PARA_DATA[CONN_UPDATE_CNT.get()][0],
                        CONN_PARA_DATA[CONN_UPDATE_CNT.get()][1],
                        CONN_PARA_DATA[CONN_UPDATE_CNT.get()][2] as u32,
                    );
                    CONN_UPDATE_CNT.inc();
                }
            }
            _ => (),
        }
    }

    0
}

#[cfg_attr(test, mry::mry)]
pub fn retrieve_dev_grp_address() {
    let dev_mask = DEVICE_ADDR_MASK_DEFAULT;
    let mut addr_next_pos = MESH_DEVICE_ADDRESS_NEXT_POSITION.get();
    let mut grp_next_pos = 0;
    let mut dest_addr_index = 0;
    let mut grp_addr_ptr = FLASH_ADR_DEV_GRP_ADR;
    loop {
        MESH_DEVICE_ADDRESS_NEXT_POSITION.set(addr_next_pos);
        MESH_GROUP_ADDRESS_NEXT_POSITION.set(grp_next_pos);
        let grp_addr;
        unsafe {
            grp_addr = *(grp_addr_ptr as *const u16);
        }

        grp_next_pos += 2;
        if grp_addr == 0xffff {
            break;
        }
        if grp_addr == 0 {
            MESH_DEVICE_ADDRESS_NEXT_POSITION.set(addr_next_pos);
        } else if grp_addr & !dev_mask == 0 {
            DEVICE_ADDRESS.set(grp_addr & dev_mask);
            addr_next_pos = grp_next_pos;
            MESH_GROUP_ADDRESS_NEXT_POSITION.set(addr_next_pos);
            MESH_DEVICE_ADDRESS_NEXT_POSITION.set(addr_next_pos);
        } else {
            unsafe {
                *((GROUP_ADDRESS.lock().as_ptr() as u32 + (dest_addr_index & 7) * 2) as *mut u16) =
                    grp_addr;
            }
            dest_addr_index += 1;
            MESH_GROUP_ADDRESS_NEXT_POSITION.set(grp_next_pos);
        }
        grp_addr_ptr += 2;
        if grp_next_pos == 0x1000 {
            break;
        }
    }
    if DEVICE_ADDRESS.get() == 0 {
        DEVICE_ADDRESS.set((MAC_ID.lock()[0] as u16) | ((MAC_ID.lock()[1] as u16) << 8));
        if MAC_ID.lock()[0] == 0 {
            DEVICE_ADDRESS.set(1);
        }
    }
}

#[cfg_attr(test, mry::mry)]
pub fn mesh_node_init() {
    app().mesh_manager.mesh_node_buf_init();

    let mut mesh_node_st = MESH_NODE_ST.lock();

    mesh_node_st[0].val.dev_adr = DEVICE_ADDRESS.get() as u8;
    mesh_node_st[0].val.sn = DEVICE_NODE_SN.get();
    MESH_NODE_MAX.set(1);
}

#[cfg_attr(test, mry::mry)]
pub fn pair_flash_clean() {
    let mut flash_dat_swap: [u8; 64] = [0; 64];
    let flash_dat_swap_len = flash_dat_swap.len();

    // If configuration has reached the last entry (0xfc0), wrap to the beginning
    if FLASH_CONFIGURATION_INDEX.get() >= 0xfc0 {
        let src_addr = (FLASH_ADR_PAIRING as i32 + FLASH_CONFIGURATION_INDEX.get()) as *const u8;
        unsafe {
            flash_dat_swap.copy_from_slice(slice::from_raw_parts(src_addr, flash_dat_swap_len));
        }

        flash_erase_sector(FLASH_ADR_PAIRING);
        flash_write_page(
            FLASH_ADR_PAIRING,
            flash_dat_swap_len as u32,
            flash_dat_swap.as_ptr(),
        );
        FLASH_CONFIGURATION_INDEX.set(0);
    }
}

pub fn pair_flash_config_init() -> bool {
    // Read first 0x40 byte chunk to get the valid flag marker
    let mut first_buffer = [0u8; 0x40];
    flash_read_page(FLASH_ADR_PAIRING, 0x40, first_buffer.as_mut_ptr());

    // If first byte is not valid, no configuration exists
    if first_buffer[0] != PAIR_CONFIG_VALID_FLAG {
        FLASH_CONFIGURATION_INDEX.set(-1);
        return false;
    }

    // Scan through all possible 0x40 byte chunks to find the last valid configuration
    // The algorithm walks through offsets (0x40, 0x80, ..., 0xfc0) until it finds
    // an empty slot (first byte != PAIR_CONFIG_VALID_FLAG). The last valid position
    // is then used as the configuration index.
    let mut last_valid_index = 0i32;
    let mut buffer = [0u8; 0x40];

    for offset in (0x40..=0xfc0).step_by(0x40) {
        flash_read_page(FLASH_ADR_PAIRING + offset as u32, 0x40, buffer.as_mut_ptr());

        // If this slot is empty (first byte != PAIR_CONFIG_VALID_FLAG), we've found
        // the boundary between used and unused space
        if buffer[0] != PAIR_CONFIG_VALID_FLAG {
            // The last valid configuration is at the previous offset
            break;
        }

        // This slot is valid, so it becomes the new last_valid_index
        last_valid_index = offset;
    }

    FLASH_CONFIGURATION_INDEX.set(last_valid_index);
    pair_flash_clean();
    true
}

#[cfg_attr(test, mry::mry)]
pub fn access_code(name: &[u8], pass: &[u8]) -> u32 {
    // Ensure the input slices are copied into fixed-size arrays
    let mut name_arr = [0u8; 16];
    let mut pass_arr = [0u8; 16];

    // Copy the minimum of slice length or 16 bytes to avoid panic
    let name_len = name.len().min(16);
    let pass_len = pass.len().min(16);
    name_arr[..name_len].copy_from_slice(&name[..name_len]);
    pass_arr[..pass_len].copy_from_slice(&pass[..pass_len]);

    let destbuf = aes_att_encryption(&name_arr, &pass_arr);

    let mut destbuf: [u32; 4] = bytemuck::cast(destbuf);

    let mut bit = destbuf[0] >> 1 ^ destbuf[0];
    let mut inner_count = 0;
    for bit_count in 1..0x20 {
        if bit & 1 == 0 {
            break;
        }
        inner_count += 1;
        if 5 < inner_count {
            destbuf[0] ^= 1 << bit_count;
            inner_count = 0;
        }
    }

    bit = 0xaaaaaaaa;
    for _ in 0..2 {
        inner_count = 0;

        for bit_count in 0..0x20 {
            inner_count += if (1 << bit_count & bit) as u32 != 0 {
                1
            } else {
                0
            };
        }

        if inner_count < 3 {
            destbuf[0] ^= 0xff;
        }

        bit = destbuf[0] ^ 0x55555555;
    }

    destbuf[0]
}

#[cfg_attr(test, mry::mry)]
pub fn pair_update_key() {
    let pair_state = PAIR_STATE.lock();

    PAIR_AC.set(access_code(&pair_state.pair_nn, &pair_state.pair_pass));
    let name_len = match pair_state.pair_nn.iter().position(|r| *r == 0) {
        Some(v) => v,
        None => pair_state.pair_nn.len(),
    };

    let name_len = min(MAX_MESH_NAME_LEN, name_len);

    set_advertisement_mesh_name(&pair_state.pair_nn[0..name_len]);
    let tmp = *ADV_PRI_DATA.lock();
    set_advertisement_manufacturer_data(unsafe {
        slice::from_raw_parts(addr_of!(tmp) as *const u8, size_of::<AdvPrivate>())
    });
}

#[cfg_attr(test, mry::mry)]
pub fn pair_load_key() {
    pair_flash_config_init();

    let pairing_addr = FLASH_ADR_PAIRING as i32 + FLASH_CONFIGURATION_INDEX.get();

    if FLASH_CONFIGURATION_INDEX.get() >= 0 && pairing_addr != 0x0 {
        {
            let mut pair_state = PAIR_STATE.lock();

            pair_state.pair_nn.iter_mut().for_each(|v| *v = 0);
            pair_state.pair_pass.iter_mut().for_each(|v| *v = 0);
            pair_state.pair_ltk.iter_mut().for_each(|v| *v = 0);

            pair_state.pair_nn.copy_from_slice(unsafe {
                slice::from_raw_parts((pairing_addr + 0x10) as *const u8, MAX_MESH_NAME_LEN)
            });
            pair_state.pair_pass.copy_from_slice(unsafe {
                slice::from_raw_parts((pairing_addr + 0x20) as *const u8, 0x10)
            });

            if MESH_PAIR_ENABLE.get() {
                MESH_DEVICE_ADDRESS_VALIDATION_PENDING
                    .set(unsafe { *(pairing_addr as *const bool).offset(0x1) });
            }

            let pair_config_flag = unsafe { *(pairing_addr as *const u8).offset(0xf) };
            if pair_config_flag == PAIR_CONFIG_VALID_FLAG {
                pair_state.pair_pass = decode_password(&pair_state.pair_pass);
            }

            pair_state.pair_ltk.copy_from_slice(unsafe {
                slice::from_raw_parts((pairing_addr + 0x30) as *const u8, 0x10)
            });
        }

        pair_update_key();
    }
}
