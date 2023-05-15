use core::ptr::{addr_of, addr_of_mut, null, null_mut, slice_from_raw_parts};
use core::slice;

use crate::{blinken, uprintln};
use crate::common::{pair_flash_clean, pair_load_key, pair_update_key, save_pair_info};
use crate::config::get_flash_adr_pairing;
use crate::main_light::{get_max_mesh_name_len, rf_link_light_event_callback};
use crate::mesh::{get_get_mac_en, get_mesh_pair_enable, set_get_mac_en};
use crate::sdk::ble_app::light_ll::rf_link_delete_pair;
use crate::sdk::light::{get_adr_flash_cfg_idx, get_ble_pair_st, get_enc_disable, get_pair_config_pwd_encode_enable, get_pair_config_valid_flag, get_pair_ivm, get_pair_ivs, get_pair_login_ok, get_pair_ltk, get_pair_ltk_mesh, get_pair_ltk_org, get_pair_nn, get_pair_pass, get_pair_randm, get_pair_rands, get_pair_setting_flag, get_pair_sk, get_pair_sk_copy, get_pair_work, get_pairRead_pending, get_pkt_read_rsp, get_rands_fix_flag, get_security_enable, get_set_mesh_info_expired_flag, get_set_mesh_info_time, get_slave_p_mac, get_sw_no_pair, LGT_CMD_DEL_PAIR, mesh_pkt_t, PairState, rf_packet_att_readRsp_t, rf_packet_att_write_t, rf_packet_ll_app_t, set_adr_flash_cfg_idx, set_ble_pair_st, set_pair_enc_enable, set_pair_login_ok, set_pair_setting_flag, set_pairRead_pending};
use crate::sdk::mcu::crypto::{aes_att_decryption, aes_att_decryption_packet, aes_att_encryption, aes_att_encryption_packet, encode_password};
use crate::sdk::mcu::register::read_reg_system_tick;

pub unsafe fn pair_dec_packet(ps: *mut rf_packet_ll_app_t) -> bool {
    let mut result = true;
    if *get_security_enable() != false {
        (*get_pair_ivm())[5..5 + 3].copy_from_slice(&(*ps).app_cmd_v.sno);

        result = aes_att_decryption_packet(
            get_pair_sk(),
            get_pair_ivm(),
            slice::from_raw_parts(addr_of!((*ps).app_cmd_v.src) as *const u8, 2),
            addr_of!((*ps).app_cmd_v.dst) as *mut u8,
            ((*ps).l2capLen as u8) - 8,
        );
    }
    return result;
}

pub fn pair_enc_packet(ps: *mut rf_packet_ll_app_t) -> bool
{
    unsafe {
        if *get_security_enable() && (*ps).chanId == 4 && (*ps).opcode == 0x1b && (*ps).handle == 0x12 {
            let tick = read_reg_system_tick();
            (*ps).app_cmd_v.sno.copy_from_slice(slice::from_raw_parts(addr_of!(tick) as *const u8, 3));

            (*get_pair_ivs())[3..3 + 5].copy_from_slice(slice::from_raw_parts(addr_of!((*ps).app_cmd_v.sno) as *const u8, 5));

            aes_att_encryption_packet(
                &(*get_pair_sk())[0..16],
                &(*get_pair_ivs())[0..8],
                slice::from_raw_parts_mut(addr_of!((*ps).app_cmd_v.dst) as *mut u8, 2),
                addr_of!((*ps).app_cmd_v.op) as *mut u8,
                (((*ps).l2capLen & 0xff) - 10) as u8,
            );
        }
    }
    return true;
}

pub unsafe fn pair_dec_packet_mesh(ps: *const mesh_pkt_t) -> bool {
    let mut ltk = [0u8; 16];

    if *get_security_enable() == false {
        return true;
    }

    if *get_enc_disable() != false {
        return true;
    }

    if -1 < (*ps)._type as i8 {
        return false;
    }

    let rf_len = (*ps).rf_len;
    if 0x13 < rf_len - 0x12 {
        return *get_enc_disable();
    }

    if *get_sw_no_pair() == false {
        ltk.copy_from_slice(&(*get_pair_ltk())[0..16]);
    } else {
        let mut breakit = false;
        if *get_mesh_pair_enable() == true {
            if *get_pair_setting_flag() != PairState::PairSetted || (*ps).op & 0x3f == 9 {
                ltk.copy_from_slice(&(*get_pair_ltk_org())[0..16]);
                breakit = true;
            }
        }
        if !breakit {
            let mut index = 0;
            loop {
                ltk[index] = (*get_pair_nn())[index] ^ (*get_pair_pass())[index] ^ (*get_pair_ltk())[index];
                index = index + 1;

                if index == 0x10 {
                    break;
                }
            }
        }
    }
    let mut result;
    if (*ps).chanId as u16 == 0xffff {
        result = aes_att_decryption_packet(
            ltk.as_slice(),
            slice::from_raw_parts(addr_of!((*ps).rf_len) as *const u8, 8),
            slice::from_raw_parts(addr_of!((*ps).internal_par2[1]) as *const u8, 2),
            addr_of!((*ps).sno) as *mut u8,
            0x1c,
        );
    } else {
        result = aes_att_decryption_packet(
            ltk.as_slice(),
            slice::from_raw_parts(addr_of!((*ps).handle1) as *const u8, 8),
            slice::from_raw_parts((addr_of!((*ps).sno) as u32 + (rf_len as u32 - 0xb)) as *const u8, 4),
            addr_of!((*ps).op) as *mut u8,
            rf_len - 0x12,
        );
    }
    return result;
}

pub fn pair_enc_packet_mesh(ps: *mut mesh_pkt_t) -> bool
{
    let mut result = true;

    unsafe {
        let mut ltk = [0u8; 16];

        ltk[0] = if *get_enc_disable() {1} else {0};
        if ltk[0] == 0 {
            result = false;
            if *get_security_enable() {
                if *get_sw_no_pair() {
                    if *get_mesh_pair_enable() && (*get_pair_setting_flag() != PairState::PairSetted || (*ps).op & 0x3f == 9) {
                        ltk.copy_from_slice(&(*get_pair_ltk_org())[0..16]);
                    } else {
                        let mut index = 0;
                        loop {
                            ltk[index] = (*get_pair_nn())[index] ^ (*get_pair_pass())[index] ^ (*get_pair_ltk())[index];
                            index = index + 1;

                            if index == 0x10 {
                                break;
                            }
                        }
                    }
                } else {
                    ltk.copy_from_slice(&(*get_pair_ltk())[0..16]);
                }

                if (*ps).chanId == 0xffff {
                    aes_att_encryption_packet(
                        ltk.as_slice(),
                        slice::from_raw_parts(addr_of!((*ps).rf_len) as *const u8, 8),
                        slice::from_raw_parts_mut(addr_of!((*ps).internal_par2[1]) as *mut u8, 2),
                        addr_of!((*ps).sno) as *mut u8,
                        0x1c,
                    );

                    result = true;
                } else {
                    aes_att_encryption_packet(
                        ltk.as_slice(),
                        slice::from_raw_parts(addr_of!((*ps).handle1) as *const u8, 8),
                        slice::from_raw_parts_mut((addr_of!((*ps).sno) as u32 + ((*ps).rf_len as u32 - 0xb)) as *mut u8, 4),
                        addr_of!((*ps).op) as *mut u8,
                        (*ps).rf_len - 0x12,
                    );

                    result = true;
                }
            }
        }

        return result;
    }
}

pub fn pair_flash_save_config(addr: u32, data: *const u8, length: u32)
{
    if addr == 0 {
        pair_flash_clean();
        set_adr_flash_cfg_idx(*get_adr_flash_cfg_idx() + 0x40);
    }
    save_pair_info(addr, data, length);
}

pub fn pair_save_key()
{
    let mut pass = [0u8; 16];

    pass[0] = *get_pair_config_valid_flag();

    if *get_pair_config_pwd_encode_enable() != false {
        pass[15] = *get_pair_config_valid_flag();
    }

    if *get_mesh_pair_enable() != false {
        pass[1] = if *get_get_mac_en() { 1 } else { 0 };
    }

    pair_flash_save_config(0, pass.as_ptr(), pass.len() as u32);
    pair_flash_save_config(0x10, (*get_pair_nn()).as_ptr(), get_pair_nn().len() as u32);

    pass.copy_from_slice(&get_pair_pass()[0..16]);
    encode_password(pass.as_mut_slice());

    pair_flash_save_config(0x20, pass.as_ptr(), pass.len() as u32);
    pair_flash_save_config(0x30, (*get_pair_ltk()).as_ptr(), get_pair_ltk().len() as u32);

    pair_update_key();
}

pub fn pair_init()
{
    set_ble_pair_st(0);
    set_pair_enc_enable(false);
    unsafe { (*get_pair_ivm())[0..4].copy_from_slice(slice::from_raw_parts(*get_slave_p_mac() as *const u8, 4)); }
    unsafe { (*get_pair_ivs())[0..4].copy_from_slice(slice::from_raw_parts(*get_slave_p_mac() as *const u8, 4)); }
}

pub fn pair_par_init()
{
    set_ble_pair_st(0xe);
    pair_load_key();
}

pub fn pair_proc() -> *const rf_packet_att_readRsp_t
{
    let pair_st = *get_ble_pair_st();
    if *get_pairRead_pending() == false {
        return null_mut();
    }
    set_pairRead_pending(false);

    if *get_ble_pair_st() == 2 {
        if *get_security_enable() == false && *get_pair_login_ok() == false {
            pair_par_init();
            set_pair_enc_enable(false);
            return null_mut();
        }
        get_pkt_read_rsp().l2capLen = 10;
        get_pkt_read_rsp().value[1..1 + 8].copy_from_slice(get_pair_rands());
        set_ble_pair_st(0xc);
    } else if *get_ble_pair_st() == 0x7 {
        if *get_security_enable() == false && *get_pair_login_ok() == false {
            pair_par_init();
            set_pair_enc_enable(false);
            return null_mut();
        }
        get_pkt_read_rsp().l2capLen = 0x12;
        let mut index = 0;
        loop {
            (*get_pair_work())[index] = (*get_pair_nn())[index] ^ (*get_pair_pass())[index] ^ (*get_pair_ltk())[index];
            index = index + 1;
            if index == 0x10 {
                break;
            }
        }

        if *get_security_enable() == false {
            get_pkt_read_rsp().value[1..1 + 0x10].copy_from_slice(&get_pair_work()[0..10]);
        } else {
            aes_att_encryption(get_pair_sk().as_ptr(), get_pair_work().as_ptr(), addr_of_mut!(get_pkt_read_rsp().value[1]));
        }
        set_ble_pair_st(0xf);
    } else if *get_ble_pair_st() == 0xd {
        if *get_security_enable() == false {
            get_pkt_read_rsp().l2capLen = 0x12;
            let mut index = 0;
            loop {
                (*get_pair_work())[index] = (*get_pair_pass())[index] ^ (*get_pair_nn())[index];
                index = index + 1;
                if index == 0x10 {
                    break;
                }
            }
            get_pkt_read_rsp().value[1..1 + 0x10].copy_from_slice(&get_pair_work()[0..0x10]);

            set_ble_pair_st(0xf);
            set_pair_enc_enable(false);
        } else {
            get_pkt_read_rsp().l2capLen = 0x12;
            if *get_rands_fix_flag() == false {
                unsafe { *(get_pair_rands().as_mut_ptr() as *mut u32) = read_reg_system_tick(); }
                aes_att_encryption(get_pair_randm().as_ptr(), get_pair_rands().as_ptr(), get_pair_sk().as_mut_ptr());
            } else {
                (*get_pair_sk())[0..8].copy_from_slice(&(*get_pair_rands())[0..8]);
            }
            (*get_pair_rands())[0..8].copy_from_slice(&(*get_pair_sk())[0..8]);

            let mut index = 0;
            (*get_pair_sk())[8..16].fill(0);

            loop {
                (*get_pair_work())[index] = (*get_pair_pass())[index] ^ (*get_pair_nn())[index];
                index = index + 1;
                if index == 0x10 {
                    break;
                }
            }
            aes_att_encryption(get_pair_sk().as_ptr(), get_pair_work().as_ptr(), get_pair_work().as_mut_ptr());
            get_pkt_read_rsp().value[1..1 + 8].copy_from_slice(&get_pair_rands()[0..8]);
            get_pkt_read_rsp().value[9..9 + 8].copy_from_slice(&get_pair_work()[0..8]);

            let mut index = 0;
            loop {
                (*get_pair_work())[index] = (*get_pair_pass())[index] ^ (*get_pair_nn())[index];
                index = index + 1;
                if index == 0x10 {
                    break;
                }
            }
            (*get_pair_sk())[0..8].copy_from_slice(&get_pair_randm()[0..8]);
            (*get_pair_sk())[8..16].copy_from_slice(&get_pair_rands()[0..8]);

            aes_att_encryption(get_pair_work().as_ptr(), get_pair_sk().as_ptr(), get_pair_sk().as_mut_ptr());
            set_ble_pair_st(0xf);
            set_pair_enc_enable(true);
        }
    } else if *get_ble_pair_st() == 0x9 {
        get_pkt_read_rsp().l2capLen = 0x12;
        if *get_security_enable() == false {
            get_pkt_read_rsp().value[1..1 + 0x10].copy_from_slice(get_pair_ltk());
        } else {
            (*get_pair_work())[0..8].copy_from_slice(get_pair_randm());

            (*get_pair_work())[8..16].fill(0);

            let mut index = 0;
            loop {
                (*get_pair_sk())[index] = (*get_pair_nn())[index] ^ (*get_pair_pass())[index] ^ (*get_pair_work())[index];
                index = index + 1;
                if index == 0x10 {
                    break;
                }
            }
            aes_att_encryption(get_pair_sk().as_ptr(), get_pair_ltk().as_ptr(), get_pkt_read_rsp().value[1..1 + 0x10].as_mut_ptr());
            set_ble_pair_st(0xf);
            get_pair_sk().copy_from_slice(get_pair_sk_copy());
        }
    } else if *get_ble_pair_st() == 0xb {
        get_pkt_read_rsp().l2capLen = 2;
        set_ble_pair_st(0);
        if *get_mesh_pair_enable() != false {
            set_get_mac_en(true);
        }
        rf_link_delete_pair();
        rf_link_light_event_callback(LGT_CMD_DEL_PAIR);
    } else if *get_ble_pair_st() == 0xe {
        get_pkt_read_rsp().l2capLen = 2;
        set_ble_pair_st(0);
        get_pkt_read_rsp().dma_len = 8;
        get_pkt_read_rsp().rf_len = 0x6;

        get_pkt_read_rsp().value[0] = pair_st;
        return get_pkt_read_rsp();
    }

    get_pkt_read_rsp().rf_len = get_pkt_read_rsp().l2capLen as u8 + 0x4;
    get_pkt_read_rsp().dma_len = get_pkt_read_rsp().l2capLen as u32 + 6;

    get_pkt_read_rsp().value[0] = pair_st;
    return get_pkt_read_rsp();
}

pub fn pair_set_key(key: *const u8)
{
    (*get_pair_nn())[0..*get_max_mesh_name_len() as usize].copy_from_slice(unsafe { slice::from_raw_parts(key, *get_max_mesh_name_len() as usize) });
    (*get_pair_pass()).copy_from_slice(unsafe { slice::from_raw_parts(key.offset(0x10), 0x10) });
    (*get_pair_ltk()).copy_from_slice(unsafe { slice::from_raw_parts(key.offset(0x20), 0x10) });

    pair_update_key();
}

pub fn pair_read(_data: *const rf_packet_att_write_t) -> bool
{
    set_pairRead_pending(true);

    true
}

pub fn pair_write(data: *const rf_packet_att_write_t) -> bool
{
    let opcode = unsafe { (*data).value[0] as i8 };
    let src = unsafe { addr_of!((*data).value[1]) as *const u8 };

    if opcode == 1 {
        (*get_pair_randm()).copy_from_slice(unsafe { slice::from_raw_parts(src, 8) });

        set_ble_pair_st(2);
        return true;
    }

    if opcode == 4 {
        if *get_sw_no_pair() && *get_mesh_pair_enable() {
            let mut index = 0;
            loop {
                (*get_pair_ltk_org())[index] = (*get_pair_nn())[index] ^ (*get_pair_pass())[index] ^ (*get_pair_ltk())[index];
                index = index + 1;
                if index == 0x10 {
                    break;
                }
            }
        }
        if *get_security_enable() == false {
            if *get_pair_login_ok() != false && *get_ble_pair_st() == 0xf {
                (*get_pair_nn()).copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
                set_pair_setting_flag(PairState::PairSetting);
                set_ble_pair_st(5);
                return true;
            }
        } else if *get_ble_pair_st() == 0xf && (*get_set_mesh_info_time() == 0 || *get_set_mesh_info_expired_flag() == false) {
            (*get_pair_work()).copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });

            aes_att_decryption(get_pair_sk().as_ptr(), get_pair_work().as_ptr(), get_pair_nn().as_mut_ptr());
            let mut name_len = match get_pair_nn().iter().position(|r| *r == 0) {
                Some(v) => v,
                None => get_pair_nn().len()
            } as u8;

            if name_len <= *get_max_mesh_name_len() {
                set_pair_setting_flag(PairState::PairSetting);
                set_ble_pair_st(5);
                return true;
            }
        }

        pair_par_init();
        set_pair_enc_enable(false);
        return true;
    }
    if opcode != 5 {
        if opcode == 6 {
            if *get_security_enable() {
                if *get_ble_pair_st() == 6 {
                    (*get_pair_work()).copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
                    set_ble_pair_st(7);
                    unsafe {
                        if *get_mesh_pair_enable() != false && 0x14 < (*data).l2capLen && (*data).value[0x11] != 0 {
                            aes_att_decryption(get_pair_sk().as_ptr(), get_pair_work().as_ptr(), get_pair_ltk_mesh().as_mut_ptr());
                            set_pair_setting_flag(PairState::PairSetMeshTxStart);
                            return true;
                        }
                    }
                    aes_att_decryption(get_pair_sk().as_ptr(), get_pair_work().as_ptr(), get_pair_ltk().as_mut_ptr());
                    pair_save_key();
                    set_pair_setting_flag(PairState::PairSetted);
                    rf_link_light_event_callback(0xc5);
                    return true;
                }
            } else if *get_pair_login_ok() && *get_ble_pair_st() == 6 {
                (*get_pair_ltk()).fill(0);
                set_ble_pair_st(7);
                unsafe {
                    if *get_mesh_pair_enable() && 0x14 < (*data).l2capLen && ((((*data).value[0x11] as u32) << 0x1f) as i32) < 0 {
                        (*get_pair_ltk_mesh()).copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
                        set_pair_setting_flag(PairState::PairSetMeshTxStart);
                        return true;
                    }
                }
                (*get_pair_ltk()).copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
                pair_save_key();
                set_pair_setting_flag(PairState::PairSetted);
                rf_link_light_event_callback(0xc5);
                return true;
            }
            pair_par_init();
            set_pair_setting_flag(PairState::PairSetted);
            set_pair_enc_enable(false);
            return true;
        }

        let mut index = if opcode - 8 != 0 { 0 } else { u8::MAX };
        let iVar2 = if opcode - 0xc != 0 { 0 } else { u8::MAX };
        if iVar2 == 0 {
            if index == 0 {
                if opcode != 10 {
                    if opcode == 0xe {
                        set_pair_enc_enable(false);
                        set_ble_pair_st(0);
                        return true;
                    }
                    return true;
                }

                index = 0;
            }
        } else if index != 0 {
            (*get_pair_sk_copy()).copy_from_slice(&(*get_pair_sk())[0..16]);
        }

        (*get_pair_randm()).copy_from_slice(unsafe { slice::from_raw_parts(src, 8) });
        (*get_pair_sk())[0..8].copy_from_slice(&(*get_pair_randm())[0..8]);
        (*get_pair_sk())[8..16].fill(0);

        set_pair_enc_enable(false);

        let mut index = 0;
        loop {
            (*get_pair_work())[index] = (*get_pair_pass())[index] ^ (*get_pair_nn())[index];
            index = index + 1;
            if index == 0x10 {
                break;
            }
        }
        if *get_security_enable() == false {
            index = if &(*get_pair_work())[0..16] == unsafe { slice::from_raw_parts(src, 0x10) } { usize::MAX } else { 0 };
        } else {
            aes_att_encryption(get_pair_sk().as_ptr(), get_pair_work().as_ptr(), get_pair_work().as_mut_ptr());
            index = if &(*get_pair_work())[0..8] == unsafe { slice::from_raw_parts(addr_of!((*data).value[9]), 8) } { usize::MAX } else { 0 };
        }
        if iVar2 != 0 || *get_pair_login_ok() != false {
            if index != 0 {
                if opcode == 8 {
                    set_ble_pair_st(9);
                    return true;
                }
                if iVar2 == 0 {
                    set_ble_pair_st(0xb);
                    return true;
                }
                set_pair_login_ok(true);
                set_ble_pair_st(0xd);
                return true;
            }
            if iVar2 != 0 {
                set_pair_login_ok(false);
            }
        }
        pair_par_init();
        return true;
    }
    if *get_security_enable() {
        if *get_ble_pair_st() != 5 {
            pair_par_init();
            set_pair_enc_enable(false);
            return true;
        }
        (*get_pair_work()).copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
        aes_att_decryption(get_pair_sk().as_ptr(), get_pair_work().as_ptr(), get_pair_pass().as_mut_ptr());

    } else {
        if !*get_pair_login_ok() || *get_ble_pair_st() != 5 {
            pair_par_init();
            set_pair_enc_enable(false);
            return true;
        }

        (*get_pair_pass()).copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
    }

    set_ble_pair_st(6);
    if (if &(*get_pair_nn())[0..16] == &(*get_pair_pass())[0..16] { usize::MAX } else { 0 }) == 0 {
        set_ble_pair_st(6);
        return true;
    }

    pair_par_init();
    set_pair_enc_enable(false);
    return true;
}