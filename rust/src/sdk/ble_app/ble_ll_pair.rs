use core::cell::RefCell;
use core::ptr::{addr_of, addr_of_mut, null_mut};
use core::slice;

use crate::common::{pair_flash_clean, pair_load_key, pair_update_key, save_pair_info};
use crate::main_light::{rf_link_light_event_callback};
use crate::sdk::ble_app::light_ll::rf_link_delete_pair;
use crate::sdk::ble_app::rf_drv_8266::get_mac_id;
use crate::sdk::light::{*};
use crate::sdk::mcu::crypto::{aes_att_decryption, aes_att_decryption_packet, aes_att_encryption, aes_att_encryption_packet, encode_password};
use crate::sdk::mcu::register::read_reg_system_tick;
use crate::state::State;

pub unsafe fn pair_dec_packet(ps: *mut PacketLlApp) -> bool {
    let mut result = true;
    if *get_security_enable() {
        (*get_pair_ivm())[5..5 + 3].copy_from_slice(&(*ps).app_cmd_v.sno);

        result = aes_att_decryption_packet(
            get_pair_sk(),
            get_pair_ivm(),
            slice::from_raw_parts(addr_of!((*ps).app_cmd_v.src) as *const u8, 2),
            addr_of!((*ps).app_cmd_v.dst) as *mut u8,
            ((*ps).l2cap_len as u8) - 8,
        );
    }
    return result;
}

pub fn pair_enc_packet(ps: &mut PacketLlApp)
{
    if *get_security_enable() && ps.chan_id == 4 && ps.opcode == 0x1b && ps.handle == 0x12 {
        let tick = read_reg_system_tick();
        unsafe {
            ps.app_cmd_v.sno.copy_from_slice(slice::from_raw_parts(addr_of!(tick) as *const u8, 3));

            (*get_pair_ivs())[3..3 + 5].copy_from_slice(slice::from_raw_parts(addr_of!(ps.app_cmd_v.sno) as *const u8, 5));

            aes_att_encryption_packet(
                &(*get_pair_sk())[0..16],
                &(*get_pair_ivs())[0..8],
                slice::from_raw_parts_mut(addr_of!(ps.app_cmd_v.dst) as *mut u8, 2),
                addr_of!(ps.app_cmd_v.op) as *mut u8,
                ((ps.l2cap_len & 0xff) - 10) as u8,
            );
        }
    }
}

pub unsafe fn pair_dec_packet_mesh(ps: *const MeshPkt) -> bool {
    let mut ltk = [0u8; 16];

    if !*get_security_enable() {
        return true;
    }

    if (*ps)._type & 0x80 == 0 {
        return false;
    }

    let rf_len = (*ps).rf_len;
    if 0x13 < rf_len - 0x12 {
        return false;
    }

    ltk.copy_from_slice(&(*get_pair_ltk())[0..16]);

    let mut result;
    if (*ps).chan_id as u16 == 0xffff {
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

pub fn pair_enc_packet_mesh(ps: *mut MeshPkt) -> bool
{
    let mut result = true;

    unsafe {
        let mut ltk = [0u8; 16];

        result = false;
        if *get_security_enable() {
            ltk.copy_from_slice(&(*get_pair_ltk())[0..16]);

            if (*ps).chan_id == 0xffff {
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

pub fn pair_save_key(state: &RefCell<State>)
{
    let mut pass = [0u8; 16];

    pass[0] = PAIR_CONFIG_VALID_FLAG;
    pass[15] = PAIR_CONFIG_VALID_FLAG;

    if state.borrow().mesh_pair_enable {
        pass[1] = if state.borrow().get_mac_en { 1 } else { 0 };
    }

    pair_flash_save_config(0, pass.as_ptr(), pass.len() as u32);
    pair_flash_save_config(0x10, (*get_pair_nn()).as_ptr(), get_pair_nn().len() as u32);

    pass.copy_from_slice(&get_pair_pass()[0..16]);
    encode_password(pass.as_mut_slice());

    pair_flash_save_config(0x20, pass.as_ptr(), pass.len() as u32);
    pair_flash_save_config(0x30, (*get_pair_ltk()).as_ptr(), get_pair_ltk().len() as u32);

    pair_update_key(state);
}

pub fn pair_init()
{
    set_ble_pair_st(0);
    set_pair_enc_enable(false);
    (*get_pair_ivm())[0..4].copy_from_slice(&(*get_mac_id())[0..4]);
    (*get_pair_ivs())[0..4].copy_from_slice(&(*get_mac_id())[0..4]);
}

pub fn pair_par_init(state: &RefCell<State>)
{
    set_ble_pair_st(0xe);
    pair_load_key(state);
}

pub fn pair_proc(state: &RefCell<State>) -> *const PacketAttReadRsp
{
    let pair_st = *get_ble_pair_st();
    if *get_pairRead_pending() == false {
        return null_mut();
    }
    set_pairRead_pending(false);

    if *get_ble_pair_st() == 2 {
        if *get_security_enable() == false && *get_pair_login_ok() == false {
            pair_par_init(state);
            set_pair_enc_enable(false);
            return null_mut();
        }
        get_pkt_read_rsp().l2cap_len = 10;
        get_pkt_read_rsp().value[1..1 + 8].copy_from_slice(&state.borrow().pair_rands);
        set_ble_pair_st(0xc);
    } else if *get_ble_pair_st() == 0x7 {
        if *get_security_enable() == false && *get_pair_login_ok() == false {
            pair_par_init(state);
            set_pair_enc_enable(false);
            return null_mut();
        }
        get_pkt_read_rsp().l2cap_len = 0x12;
        for index in 0..0x10 {
            (*get_pair_work())[index] = (*get_pair_nn())[index] ^ (*get_pair_pass())[index] ^ (*get_pair_ltk())[index];
        }

        if *get_security_enable() == false {
            get_pkt_read_rsp().value[1..1 + 0x10].copy_from_slice(&get_pair_work()[0..10]);
        } else {
            aes_att_encryption(get_pair_sk().as_ptr(), get_pair_work().as_ptr(), addr_of_mut!(get_pkt_read_rsp().value[1]));
        }
        set_ble_pair_st(0xf);
    } else if *get_ble_pair_st() == 0xd {
        if *get_security_enable() == false {
            get_pkt_read_rsp().l2cap_len = 0x12;
            for index in 0..0x10 {
                (*get_pair_work())[index] = (*get_pair_pass())[index] ^ (*get_pair_nn())[index];
            }
            get_pkt_read_rsp().value[1..1 + 0x10].copy_from_slice(&get_pair_work()[0..0x10]);

            set_ble_pair_st(0xf);
            set_pair_enc_enable(false);
        } else {
            get_pkt_read_rsp().l2cap_len = 0x12;
            unsafe { *(state.borrow_mut().pair_rands.as_mut_ptr() as *mut u32) = read_reg_system_tick(); }
            aes_att_encryption(state.borrow().pair_randm.as_ptr(), state.borrow().pair_rands.as_ptr(), get_pair_sk().as_mut_ptr());
            state.borrow_mut().pair_rands.copy_from_slice(&(*get_pair_sk())[0..8]);

            (*get_pair_sk())[8..16].fill(0);

            for index in 0..0x10 {
                (*get_pair_work())[index] = (*get_pair_pass())[index] ^ (*get_pair_nn())[index];
            }
            aes_att_encryption(get_pair_sk().as_ptr(), get_pair_work().as_ptr(), get_pair_work().as_mut_ptr());
            get_pkt_read_rsp().value[1..1 + 8].copy_from_slice(&state.borrow().pair_rands);
            get_pkt_read_rsp().value[9..9 + 8].copy_from_slice(&get_pair_work()[0..8]);

            for index in 0..0x10 {
                (*get_pair_work())[index] = (*get_pair_pass())[index] ^ (*get_pair_nn())[index];
            }
            (*get_pair_sk())[0..8].copy_from_slice(&state.borrow().pair_randm);
            (*get_pair_sk())[8..16].copy_from_slice(&state.borrow().pair_rands);

            aes_att_encryption(get_pair_work().as_ptr(), get_pair_sk().as_ptr(), get_pair_sk().as_mut_ptr());
            set_ble_pair_st(0xf);
            set_pair_enc_enable(true);
        }
    } else if *get_ble_pair_st() == 0x9 {
        get_pkt_read_rsp().l2cap_len = 0x12;
        if *get_security_enable() == false {
            get_pkt_read_rsp().value[1..1 + 0x10].copy_from_slice(get_pair_ltk());
        } else {
            (*get_pair_work())[0..8].copy_from_slice(&state.borrow().pair_randm);

            (*get_pair_work())[8..16].fill(0);

            for index in 0..0x10 {
                (*get_pair_sk())[index] = (*get_pair_nn())[index] ^ (*get_pair_pass())[index] ^ (*get_pair_work())[index];
            }
            aes_att_encryption(get_pair_sk().as_ptr(), get_pair_ltk().as_ptr(), get_pkt_read_rsp().value[1..1 + 0x10].as_mut_ptr());
            set_ble_pair_st(0xf);
            get_pair_sk().copy_from_slice(get_pair_sk_copy());
        }
    } else if *get_ble_pair_st() == 0xb {
        get_pkt_read_rsp().l2cap_len = 2;
        set_ble_pair_st(0);
        if state.borrow().mesh_pair_enable {
            state.borrow_mut().get_mac_en = true;
        }
        rf_link_delete_pair(state);
        rf_link_light_event_callback(state, LGT_CMD_DEL_PAIR);
    } else if *get_ble_pair_st() == 0xe {
        get_pkt_read_rsp().l2cap_len = 2;
        set_ble_pair_st(0);
        get_pkt_read_rsp().dma_len = 8;
        get_pkt_read_rsp().rf_len = 0x6;

        get_pkt_read_rsp().value[0] = pair_st;
        return get_pkt_read_rsp();
    }

    get_pkt_read_rsp().rf_len = get_pkt_read_rsp().l2cap_len as u8 + 0x4;
    get_pkt_read_rsp().dma_len = get_pkt_read_rsp().l2cap_len as u32 + 6;

    get_pkt_read_rsp().value[0] = pair_st;
    return get_pkt_read_rsp();
}

pub fn pair_set_key(state: &RefCell<State>, key: *const u8)
{
    (*get_pair_nn())[0..state.borrow().max_mesh_name_len].copy_from_slice(unsafe { slice::from_raw_parts(key, state.borrow().max_mesh_name_len) });
    (*get_pair_pass()).copy_from_slice(unsafe { slice::from_raw_parts(key.offset(0x10), 0x10) });
    (*get_pair_ltk()).copy_from_slice(unsafe { slice::from_raw_parts(key.offset(0x20), 0x10) });

    pair_update_key(state);
}

pub fn pair_read(_: &RefCell<State>, _: *const PacketAttWrite) -> bool
{
    set_pairRead_pending(true);

    true
}

pub fn pair_write(state: &RefCell<State>, data: *const PacketAttWrite) -> bool
{
    let opcode = unsafe { (*data).value[0] as i8 };
    let src = unsafe { addr_of!((*data).value[1]) as *const u8 };

    if opcode == 1 {
        state.borrow_mut().pair_randm.copy_from_slice(unsafe { slice::from_raw_parts(src, 8) });

        set_ble_pair_st(2);
        return true;
    }

    if opcode == 4 {
        if *get_security_enable() == false {
            if *get_pair_login_ok() && *get_ble_pair_st() == 0xf {
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
            };

            if name_len <= state.borrow().max_mesh_name_len {
                set_pair_setting_flag(PairState::PairSetting);
                set_ble_pair_st(5);
                return true;
            }
        }

        pair_par_init(state);
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
                        if state.borrow().mesh_pair_enable && 0x14 < (*data).l2cap_len && (*data).value[0x11] != 0 {
                            aes_att_decryption(get_pair_sk().as_ptr(), get_pair_work().as_ptr(), get_pair_ltk_mesh().as_mut_ptr());
                            set_pair_setting_flag(PairState::PairSetMeshTxStart);
                            return true;
                        }
                    }
                    aes_att_decryption(get_pair_sk().as_ptr(), get_pair_work().as_ptr(), get_pair_ltk().as_mut_ptr());
                    pair_save_key(state);
                    set_pair_setting_flag(PairState::PairSetted);
                    rf_link_light_event_callback(state, 0xc5);
                    return true;
                }
            } else if *get_pair_login_ok() && *get_ble_pair_st() == 6 {
                (*get_pair_ltk()).fill(0);
                set_ble_pair_st(7);
                unsafe {
                    if state.borrow().mesh_pair_enable && 0x14 < (*data).l2cap_len && ((((*data).value[0x11] as u32) << 0x1f) as i32) < 0 {
                        (*get_pair_ltk_mesh()).copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
                        set_pair_setting_flag(PairState::PairSetMeshTxStart);
                        return true;
                    }
                }
                (*get_pair_ltk()).copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
                pair_save_key(state);
                set_pair_setting_flag(PairState::PairSetted);
                rf_link_light_event_callback(state, 0xc5);
                return true;
            }
            pair_par_init(state);
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

        state.borrow_mut().pair_randm.copy_from_slice(unsafe { slice::from_raw_parts(src, 8) });
        (*get_pair_sk())[0..8].copy_from_slice(&state.borrow().pair_randm);
        (*get_pair_sk())[8..16].fill(0);

        set_pair_enc_enable(false);

        for index in 0..0x10 {
            (*get_pair_work())[index] = (*get_pair_pass())[index] ^ (*get_pair_nn())[index];
        }
        if *get_security_enable() == false {
            index = if &(*get_pair_work())[0..16] == unsafe { slice::from_raw_parts(src, 0x10) } { u8::MAX } else { 0 };
        } else {
            aes_att_encryption(get_pair_sk().as_ptr(), get_pair_work().as_ptr(), get_pair_work().as_mut_ptr());
            index = if &(*get_pair_work())[0..8] == unsafe { slice::from_raw_parts(addr_of!((*data).value[9]), 8) } { u8::MAX } else { 0 };
        }
        if iVar2 != 0 || *get_pair_login_ok() {
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
        pair_par_init(state);
        return true;
    }
    if *get_security_enable() {
        if *get_ble_pair_st() != 5 {
            pair_par_init(state);
            set_pair_enc_enable(false);
            return true;
        }
        (*get_pair_work()).copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
        aes_att_decryption(get_pair_sk().as_ptr(), get_pair_work().as_ptr(), get_pair_pass().as_mut_ptr());

    } else {
        if !*get_pair_login_ok() || *get_ble_pair_st() != 5 {
            pair_par_init(state);
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

    pair_par_init(state);
    set_pair_enc_enable(false);
    return true;
}