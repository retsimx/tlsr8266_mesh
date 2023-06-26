use core::cell::RefCell;
use core::ops::Deref;
use core::ptr::{addr_of, addr_of_mut, null_mut};
use core::slice;

use crate::common::{pair_flash_clean, pair_load_key, pair_update_key, save_pair_info};
use crate::main_light::{rf_link_light_event_callback};
use crate::sdk::ble_app::light_ll::rf_link_delete_pair;
use crate::sdk::light::{*};
use crate::sdk::mcu::crypto::{aes_att_decryption, aes_att_decryption_packet, aes_att_encryption, aes_att_encryption_packet, encode_password};
use crate::sdk::mcu::register::read_reg_system_tick;
use crate::state::{PAIR_LTK, SECURITY_ENABLE, SimplifyLS, State};

pub unsafe fn pair_dec_packet(state: &mut State, ps: *mut PacketLlApp) -> bool {
    let mut result = true;
    if SECURITY_ENABLE.get() {
        state.pair_ivm[5..5 + 3].copy_from_slice(&(*ps).app_cmd_v.sno);

        result = aes_att_decryption_packet(
            &state.pair_sk,
            &state.pair_ivm,
            slice::from_raw_parts(addr_of!((*ps).app_cmd_v.src) as *const u8, 2),
            addr_of!((*ps).app_cmd_v.dst) as *mut u8,
            ((*ps).l2cap_len as u8) - 8,
        );
    }
    return result;
}

pub fn pair_enc_packet(state: &mut State, ps: &mut PacketLlApp)
{
    if SECURITY_ENABLE.get() && ps.chan_id == 4 && ps.opcode == 0x1b && ps.handle == 0x12 {
        let tick = read_reg_system_tick();
        unsafe {
            ps.app_cmd_v.sno.copy_from_slice(slice::from_raw_parts(addr_of!(tick) as *const u8, 3));

            state.pair_ivs[3..3 + 5].copy_from_slice(slice::from_raw_parts(addr_of!(ps.app_cmd_v.sno) as *const u8, 5));

            aes_att_encryption_packet(
                &state.pair_sk,
                &state.pair_ivs,
                slice::from_raw_parts_mut(addr_of!(ps.app_cmd_v.dst) as *mut u8, 2),
                addr_of!(ps.app_cmd_v.op) as *mut u8,
                ((ps.l2cap_len & 0xff) - 10) as u8,
            );
        }
    }
}

pub unsafe fn pair_dec_packet_mesh(state: &mut State, ps: *const MeshPkt) -> bool {
    let mut ltk = [0u8; 16];

    if !SECURITY_ENABLE.get() {
        return true;
    }

    if (*ps)._type & 0x80 == 0 {
        return false;
    }

    let rf_len = (*ps).rf_len;
    if 0x13 < rf_len - 0x12 {
        return false;
    }

    PAIR_LTK.lock(|pair_ltk| {
        ltk.copy_from_slice(pair_ltk.borrow().deref());
    });

    let mut result;
    if (*ps).chan_id == 0xffff {
        result = aes_att_decryption_packet(
            &ltk,
            slice::from_raw_parts(addr_of!((*ps).rf_len), 8),
            slice::from_raw_parts(addr_of!((*ps).internal_par2[1]), 2),
            addr_of!((*ps).sno) as *mut u8,
            0x1c,
        );
    } else {
        result = aes_att_decryption_packet(
            &ltk,
            slice::from_raw_parts(addr_of!((*ps).handle1), 8),
            slice::from_raw_parts((addr_of!((*ps).sno) as u32 + (rf_len as u32 - 0xb)) as *const u8, 4),
            addr_of!((*ps).op) as *mut u8,
            rf_len - 0x12,
        );
    }
    return result;
}

pub fn pair_enc_packet_mesh(ps: &mut MeshPkt) -> bool
{
    if SECURITY_ENABLE.get() {
        let ltk = PAIR_LTK.lock(|pair_ltk| {
            pair_ltk.borrow().deref().clone()
        });

        if ps.chan_id == 0xffff {
            unsafe {
                aes_att_encryption_packet(
                    &ltk,
                    slice::from_raw_parts(addr_of!(ps.rf_len), 8),
                    slice::from_raw_parts_mut(addr_of!(ps.internal_par2[1]) as *mut u8, 2),
                    addr_of!(ps.sno) as *mut u8,
                    0x1c,
                );
            }

            return true
        } else {
            unsafe {
                aes_att_encryption_packet(
                    &ltk,
                    slice::from_raw_parts(addr_of!(ps.handle1), 8),
                    slice::from_raw_parts_mut((addr_of!(ps.sno) as u32 + (ps.rf_len as u32 - 0xb)) as *mut u8, 4),
                    addr_of!(ps.op) as *mut u8,
                    ps.rf_len - 0x12,
                );
            }

            return true
        }
    }

    false
}

pub fn pair_flash_save_config(state: &mut State, addr: u32, data: *const u8, length: u32)
{
    if addr == 0 {
        pair_flash_clean(state);
        state.adr_flash_cfg_idx = state.adr_flash_cfg_idx + 0x40;
    }

    save_pair_info(state, addr, data, length);
}

pub fn pair_save_key(state: &mut State)
{
    let mut pass = [0u8; 16];

    pass[0] = PAIR_CONFIG_VALID_FLAG;
    pass[15] = PAIR_CONFIG_VALID_FLAG;

    if state.mesh_pair_enable {
        pass[1] = if state.get_mac_en { 1 } else { 0 };
    }

    pair_flash_save_config(state, 0, pass.as_ptr(), pass.len() as u32);
    pair_flash_save_config(state, 0x10, state.pair_nn.as_ptr(), state.pair_nn.len() as u32);

    pass.copy_from_slice(&state.pair_pass[0..16]);
    encode_password(state, pass.as_mut_slice());

    pair_flash_save_config(state, 0x20, pass.as_ptr(), pass.len() as u32);
    PAIR_LTK.lock(|pair_ltk| {
        pair_flash_save_config(state, 0x30, pair_ltk.borrow().deref().as_ptr(), pair_ltk.borrow().deref().len() as u32);
    });

    pair_update_key(state);
}

pub fn pair_init(state: &mut State)
{
    state.ble_pair_st = 0;
    state.pair_enc_enable = false;
    state.pair_ivm[0..4].copy_from_slice(&state.mac_id[0..4]);
    state.pair_ivs[0..4].copy_from_slice(&state.mac_id[0..4]);
}

pub fn pair_par_init(state: &mut State)
{
    state.ble_pair_st = 0xe;
    pair_load_key(state);
}

pub fn pair_proc(state: &mut State) -> *const PacketAttReadRsp
{
    let pair_st = state.ble_pair_st;
    if state.pair_read_pending == false {
        return null_mut();
    }
    state.pair_read_pending = false;

    if state.ble_pair_st == 2 {
        if SECURITY_ENABLE.get() == false && state.pair_login_ok == false {
            pair_par_init(state);
            state.pair_enc_enable = false;
            return null_mut();
        }
        state.pkt_read_rsp.l2cap_len = 10;
        state.pkt_read_rsp.value[1..1 + 8].copy_from_slice(&state.pair_rands);
        state.ble_pair_st = 0xc;
    } else if state.ble_pair_st == 0x7 {
        if SECURITY_ENABLE.get() == false && state.pair_login_ok == false {
            pair_par_init(state);
            state.pair_enc_enable = false;
            return null_mut();
        }
        state.pkt_read_rsp.l2cap_len = 0x12;
        PAIR_LTK.lock(|pair_ltk| {
            for index in 0..0x10 {
                state.pair_work[index] = state.pair_nn[index] ^ state.pair_pass[index] ^ pair_ltk.borrow()[index];
            }
        });

        if SECURITY_ENABLE.get() == false {
            state.pkt_read_rsp.value[1..1 + 0x10].copy_from_slice(&state.pair_work[0..10]);
        } else {
            aes_att_encryption(state.pair_sk.as_ptr(), state.pair_work.as_ptr(), addr_of_mut!(state.pkt_read_rsp.value[1]));
        }
        state.ble_pair_st = 0xf;
    } else if state.ble_pair_st == 0xd {
        if SECURITY_ENABLE.get() == false {
            state.pkt_read_rsp.l2cap_len = 0x12;
            for index in 0..0x10 {
                state.pair_work[index] = state.pair_pass[index] ^ state.pair_nn[index];
            }
            state.pkt_read_rsp.value[1..1 + 0x10].copy_from_slice(&state.pair_work[0..0x10]);

            state.ble_pair_st = 0xf;
            state.pair_enc_enable = false;
        } else {
            state.pkt_read_rsp.l2cap_len = 0x12;
            unsafe { *(state.pair_rands.as_mut_ptr() as *mut u32) = read_reg_system_tick(); }
            aes_att_encryption(state.pair_randm.as_ptr(), state.pair_rands.as_ptr(), state.pair_sk.as_mut_ptr());
            state.pair_rands.copy_from_slice(&state.pair_sk[0..8]);

            state.pair_sk[8..16].fill(0);

            for index in 0..0x10 {
                state.pair_work[index] = state.pair_pass[index] ^ state.pair_nn[index];
            }
            aes_att_encryption(state.pair_sk.as_ptr(), state.pair_work.as_ptr(), state.pair_work.as_mut_ptr());
            state.pkt_read_rsp.value[1..1 + 8].copy_from_slice(&state.pair_rands);
            state.pkt_read_rsp.value[9..9 + 8].copy_from_slice(&state.pair_work[0..8]);

            for index in 0..0x10 {
                state.pair_work[index] = state.pair_pass[index] ^ state.pair_nn[index];
            }
            state.pair_sk[0..8].copy_from_slice(&state.pair_randm);
            state.pair_sk[8..16].copy_from_slice(&state.pair_rands);

            aes_att_encryption(state.pair_work.as_ptr(), state.pair_sk.as_ptr(), state.pair_sk.as_mut_ptr());
            state.ble_pair_st = 0xf;
            state.pair_enc_enable = true;
        }
    } else if state.ble_pair_st == 0x9 {
        state.pkt_read_rsp.l2cap_len = 0x12;
        if SECURITY_ENABLE.get() == false {
            PAIR_LTK.lock(|pair_ltk| {
                state.pkt_read_rsp.value[1..1 + 0x10].copy_from_slice(pair_ltk.borrow().deref());
            });
        } else {
            state.pair_work[0..8].copy_from_slice(&state.pair_randm);

            state.pair_work[8..16].fill(0);

            for index in 0..0x10 {
                state.pair_sk[index] = state.pair_nn[index] ^ state.pair_pass[index] ^ state.pair_work[index];
            }
            PAIR_LTK.lock(|pair_ltk| {
                aes_att_encryption(state.pair_sk.as_ptr(), pair_ltk.borrow().deref().as_ptr(), state.pkt_read_rsp.value[1..1 + 0x10].as_mut_ptr());
            });
            state.ble_pair_st = 0xf;
            &state.pair_sk.copy_from_slice(&state.pair_sk_copy);
        }
    } else if state.ble_pair_st == 0xb {
        state.pkt_read_rsp.l2cap_len = 2;
        state.ble_pair_st = 0;
        if state.mesh_pair_enable {
            state.get_mac_en = true;
        }
        rf_link_delete_pair(state);
        rf_link_light_event_callback(state, LGT_CMD_DEL_PAIR);
    } else if state.ble_pair_st == 0xe {
        state.pkt_read_rsp.l2cap_len = 2;
        state.ble_pair_st = 0;
        state.pkt_read_rsp.dma_len = 8;
        state.pkt_read_rsp.rf_len = 0x6;

        state.pkt_read_rsp.value[0] = pair_st;
        return &state.pkt_read_rsp;
    }

    state.pkt_read_rsp.rf_len = state.pkt_read_rsp.l2cap_len as u8 + 0x4;
    state.pkt_read_rsp.dma_len = state.pkt_read_rsp.l2cap_len as u32 + 6;

    state.pkt_read_rsp.value[0] = pair_st;

    return &state.pkt_read_rsp;
}

pub fn pair_set_key(state: &mut State, key: *const u8)
{
    state.pair_nn[0..state.max_mesh_name_len].copy_from_slice(unsafe { slice::from_raw_parts(key, state.max_mesh_name_len) });
    state.pair_pass.copy_from_slice(unsafe { slice::from_raw_parts(key.offset(0x10), 0x10) });
    PAIR_LTK.lock(|pair_ltk| {
        pair_ltk.borrow_mut().copy_from_slice(unsafe { slice::from_raw_parts(key.offset(0x20), 0x10) });
    });

    pair_update_key(state);
}

pub fn pair_read(state: &mut State, _: *const PacketAttWrite) -> bool
{
    state.pair_read_pending = true;

    true
}

pub fn pair_write(state: &mut State, data: *const PacketAttWrite) -> bool
{
    let opcode = unsafe { (*data).value[0] as i8 };
    let src = unsafe { addr_of!((*data).value[1]) as *const u8 };

    if opcode == 1 {
        state.pair_randm.copy_from_slice(unsafe { slice::from_raw_parts(src, 8) });

        state.ble_pair_st = 2;
        return true;
    }

    if opcode == 4 {
        if SECURITY_ENABLE.get() == false {
            if state.pair_login_ok && state.ble_pair_st == 0xf {
                state.pair_nn.copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
                state.pair_setting_flag = PairState::PairSetting;
                state.ble_pair_st = 5;
                return true;
            }
        } else if state.ble_pair_st == 0xf && (state.set_mesh_info_time == 0 || state.set_mesh_info_expired_flag == false) {
            state.pair_work.copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });

            aes_att_decryption(state.pair_sk.as_ptr(), state.pair_work.as_ptr(), state.pair_nn.as_mut_ptr());
            let mut name_len = match state.pair_nn.iter().position(|r| *r == 0) {
                Some(v) => v,
                None => state.pair_nn.len()
            };

            if name_len <= state.max_mesh_name_len {
                state.pair_setting_flag = PairState::PairSetting;
                state.ble_pair_st = 5;
                return true;
            }
        }

        pair_par_init(state);
        state.pair_enc_enable = false;
        return true;
    }
    if opcode != 5 {
        if opcode == 6 {
            if SECURITY_ENABLE.get() {
                if state.ble_pair_st == 6 {
                    state.pair_work.copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
                    state.ble_pair_st = 7;
                    unsafe {
                        if state.mesh_pair_enable && 0x14 < (*data).l2cap_len && (*data).value[0x11] != 0 {
                            aes_att_decryption(state.pair_sk.as_ptr(), state.pair_work.as_ptr(), state.pair_ltk_mesh.as_mut_ptr());
                            state.pair_setting_flag = PairState::PairSetMeshTxStart;
                            return true;
                        }
                    }
                    PAIR_LTK.lock(|pair_ltk| {
                        aes_att_decryption(state.pair_sk.as_ptr(), state.pair_work.as_ptr(), pair_ltk.borrow_mut().as_mut_ptr());
                    });
                    pair_save_key(state);
                    state.pair_setting_flag = PairState::PairSetted;
                    rf_link_light_event_callback(state, 0xc5);
                    return true;
                }
            } else if state.pair_login_ok && state.ble_pair_st == 6 {
                PAIR_LTK.lock(|pair_ltk| {
                    pair_ltk.borrow_mut().fill(0);
                });
                state.ble_pair_st = 7;
                unsafe {
                    if state.mesh_pair_enable && 0x14 < (*data).l2cap_len && ((((*data).value[0x11] as u32) << 0x1f) as i32) < 0 {
                        state.pair_ltk_mesh.copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
                        state.pair_setting_flag = PairState::PairSetMeshTxStart;
                        return true;
                    }
                }
                PAIR_LTK.lock(|pair_ltk| {
                    pair_ltk.borrow_mut().copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
                });
                pair_save_key(state);
                state.pair_setting_flag = PairState::PairSetted;
                rf_link_light_event_callback(state, 0xc5);
                return true;
            }
            pair_par_init(state);
            state.pair_setting_flag = PairState::PairSetted;
            state.pair_enc_enable = false;
            return true;
        }

        let mut index = if opcode - 8 != 0 { 0 } else { u8::MAX };
        let iVar2 = if opcode - 0xc != 0 { 0 } else { u8::MAX };
        if iVar2 == 0 {
            if index == 0 {
                if opcode != 10 {
                    if opcode == 0xe {
                        state.pair_enc_enable = false;
                        state.ble_pair_st = 0;
                        return true;
                    }
                    return true;
                }

                index = 0;
            }
        } else if index != 0 {
            state.pair_sk_copy.copy_from_slice(&state.pair_sk[0..16]);
        }

        state.pair_randm.copy_from_slice(unsafe { slice::from_raw_parts(src, 8) });
        state.pair_sk[0..8].copy_from_slice(&state.pair_randm);
        state.pair_sk[8..16].fill(0);

        state.pair_enc_enable = false;

        for index in 0..0x10 {
            state.pair_work[index] = state.pair_pass[index] ^ state.pair_nn[index];
        }
        if SECURITY_ENABLE.get() == false {
            index = if &state.pair_work[0..16] == unsafe { slice::from_raw_parts(src, 0x10) } { u8::MAX } else { 0 };
        } else {
            aes_att_encryption(state.pair_sk.as_ptr(), state.pair_work.as_ptr(), state.pair_work.as_mut_ptr());
            index = if &state.pair_work[0..8] == unsafe { slice::from_raw_parts(addr_of!((*data).value[9]), 8) } { u8::MAX } else { 0 };
        }
        if iVar2 != 0 || state.pair_login_ok {
            if index != 0 {
                if opcode == 8 {
                    state.ble_pair_st = 9;
                    return true;
                }
                if iVar2 == 0 {
                    state.ble_pair_st = 0xb;
                    return true;
                }
                state.pair_login_ok = true;
                state.ble_pair_st = 0xd;
                return true;
            }
            if iVar2 != 0 {
                state.pair_login_ok = false;
            }
        }
        pair_par_init(state);
        return true;
    }
    if SECURITY_ENABLE.get() {
        if state.ble_pair_st != 5 {
            pair_par_init(state);
            state.pair_enc_enable = false;
            return true;
        }
        state.pair_work.copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
        aes_att_decryption(state.pair_sk.as_ptr(), state.pair_work.as_ptr(), state.pair_pass.as_mut_ptr());

    } else {
        if !state.pair_login_ok || state.ble_pair_st != 5 {
            pair_par_init(state);
            state.pair_enc_enable = false;
            return true;
        }

        state.pair_pass.copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
    }

    state.ble_pair_st = 6;
    if (if &state.pair_nn[0..16] == &state.pair_pass[0..16] { usize::MAX } else { 0 }) == 0 {
        state.ble_pair_st = 6;
        return true;
    }

    pair_par_init(state);
    state.pair_enc_enable = false;
    return true;
}