use core::ptr::{addr_of, addr_of_mut};
use core::slice;

use crate::common::{pair_flash_clean, pair_load_key, pair_update_key, save_pair_info};
use crate::main_light::rf_link_light_event_callback;
use crate::sdk::ble_app::light_ll::rf_link_delete_pair;
use crate::sdk::light::{*};
use crate::sdk::mcu::crypto::{aes_att_decryption, aes_att_decryption_packet, aes_att_encryption, aes_att_encryption_packet, encode_password};
use crate::sdk::mcu::register::read_reg_system_tick;
use crate::sdk::packet_types::Packet;
use crate::state::{*};

pub fn pair_dec_packet(state: &mut State, ps: &mut Packet) -> bool {
    if SECURITY_ENABLE.get() {
        state.pair_ivm[5..5 + 3].copy_from_slice(&ps.att_write().value[0..3]);

        let mut src: [u8; 2] = [0; 2];
        src.copy_from_slice(&ps.att_write().value[3..3 + 2]);

        let l2len = ps.head().l2cap_len as usize;

        aes_att_decryption_packet(
            &PAIR_STATE.lock().borrow().pair_sk,
            &state.pair_ivm,
            &src,
            &mut ps.att_write_mut().value[5..5 + l2len - 8],
        )
    } else {
        false
    }
}

pub fn pair_enc_packet(state: &mut State, ps: &mut Packet)
{
    if SECURITY_ENABLE.get() && ps.head().chan_id == 4 && ps.ll_app().opcode == 0x1b && ps.ll_app().handle == 0x12 {
        let tick = read_reg_system_tick();
        ps.ll_app_mut().value.sno[0] = tick as u8;
        ps.ll_app_mut().value.sno[1] = (tick >> 8) as u8;
        ps.ll_app_mut().value.sno[2] = (tick >> 16) as u8;

        state.pair_ivs[3..3 + 3].copy_from_slice(&ps.ll_app().value.sno);
        state.pair_ivs[6] = ps.ll_app().value.src as u8;
        state.pair_ivs[7] = (ps.ll_app().value.src >> 8) as u8;

        aes_att_encryption_packet(
            &PAIR_STATE.lock().borrow().pair_sk,
            &state.pair_ivs,
            unsafe { slice::from_raw_parts_mut(addr_of!(ps.ll_app().value.dst) as *mut u8, 2) },
            unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.ll_app_mut().value.op), ((ps.head().l2cap_len & 0xff) - 10) as usize) },
        );
    }
}

pub fn pair_dec_packet_mesh(ps: &mut Packet) -> bool {
    let mut ltk = [0u8; 16];

    if !SECURITY_ENABLE.get() {
        return true;
    }

    if ps.head()._type & 0x80 == 0 {
        return false;
    }

    let rf_len = ps.head().rf_len;
    if 0x13 < rf_len - 0x12 {
        return false;
    }

    ltk.copy_from_slice(&PAIR_STATE.lock().borrow().pair_ltk);

    if ps.head().chan_id == 0xffff {
        aes_att_decryption_packet(
            &ltk,
            unsafe { slice::from_raw_parts(addr_of!(ps.head().rf_len), 8) },
            unsafe { slice::from_raw_parts(addr_of!(ps.mesh().internal_par2[1]), 2) },
            unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().sno) as *mut u8, 0x1c) },
        )
    } else {
        aes_att_decryption_packet(
            &ltk,
            unsafe { slice::from_raw_parts(addr_of!(ps.mesh().handle1), 8) },
            unsafe { slice::from_raw_parts((addr_of!(ps.mesh().sno) as u32 + (rf_len as u32 - 0xb)) as *const u8, 4) },
            unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().op), rf_len as usize - 0x12) },
        )
    }
}

pub fn pair_enc_packet_mesh(ps: &mut Packet) -> bool
{
    if SECURITY_ENABLE.get() {
        let pair_state_binding = PAIR_STATE.lock();
        let mut pair_state = pair_state_binding.borrow_mut();

        if ps.head().chan_id == 0xffff {
            aes_att_encryption_packet(
                &pair_state.pair_ltk,
                unsafe { slice::from_raw_parts(addr_of!(ps.head().rf_len), 8) },
                unsafe { slice::from_raw_parts_mut(addr_of!(ps.mesh().internal_par2[1]) as *mut u8, 2) },
                unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().sno) as *mut u8, 0x1c) },
            );

            return true;
        } else {
            aes_att_encryption_packet(
                &pair_state.pair_ltk,
                unsafe { slice::from_raw_parts(addr_of!(ps.mesh().handle1), 8) },
                unsafe { slice::from_raw_parts_mut((addr_of!(ps.mesh().sno) as u32 + (ps.head().rf_len as u32 - 0xb)) as *mut u8, 4) },
                unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.mesh_mut().op), ps.head().rf_len as usize - 0x12) },
            );

            return true;
        }
    }

    false
}

pub fn pair_flash_save_config(state: &mut State, addr: u32, data: &[u8])
{
    if addr == 0 {
        pair_flash_clean(state);
        state.adr_flash_cfg_idx = state.adr_flash_cfg_idx + 0x40;
    }

    save_pair_info(state, addr, data);
}

pub fn pair_save_key(state: &mut State)
{
    let mut pass = [0u8; 16];

    pass[0] = PAIR_CONFIG_VALID_FLAG;
    pass[15] = PAIR_CONFIG_VALID_FLAG;

    if MESH_PAIR_ENABLE.get() {
        pass[1] = if state.get_mac_en { 1 } else { 0 };
    }

    let pair_state_binding = PAIR_STATE.lock();
    let pair_state = pair_state_binding.borrow();

    pair_flash_save_config(state, 0, &pass);
    pair_flash_save_config(state, 0x10, &pair_state.pair_nn);

    pass = pair_state.pair_pass;

    encode_password(state, &mut pass);

    pair_flash_save_config(state, 0x20, &pass);
    pair_flash_save_config(state, 0x30, &pair_state.pair_ltk);

    pair_update_key(state);
}

pub fn pair_init(state: &mut State)
{
    BLE_PAIR_ST.set(0);
    PAIR_ENC_ENABLE.set(false);
    state.pair_ivm[0..4].copy_from_slice(&state.mac_id[0..4]);
    state.pair_ivs[0..4].copy_from_slice(&state.mac_id[0..4]);
}

pub fn pair_par_init(state: &mut State)
{
    BLE_PAIR_ST.set(0xe);
    pair_load_key(state);
}

pub fn pair_proc(state: &mut State) -> Option<&Packet>
{
    let pair_st = BLE_PAIR_ST.get();
    if !PAIR_READ_PENDING.get() {
        return None;
    }
    PAIR_READ_PENDING.set(false);

    let pair_state_binding = PAIR_STATE.lock();
    let mut pair_state = pair_state_binding.borrow_mut();

    if BLE_PAIR_ST.get() == 2 {
        if SECURITY_ENABLE.get() == false && !PAIR_LOGIN_OK.get() {
            pair_par_init(state);
            PAIR_ENC_ENABLE.set(false);
            return None;
        }
        state.pkt_read_rsp.head_mut().l2cap_len = 10;
        state.pkt_read_rsp.att_read_rsp_mut().value[1..1 + 8].copy_from_slice(&pair_state.pair_rands);
        BLE_PAIR_ST.set(0xc);
    } else if BLE_PAIR_ST.get() == 0x7 {
        if SECURITY_ENABLE.get() == false && !PAIR_LOGIN_OK.get() {
            pair_par_init(state);
            PAIR_ENC_ENABLE.set(false);
            return None;
        }
        state.pkt_read_rsp.head_mut().l2cap_len = 0x12;
        for index in 0..0x10 {
            pair_state.pair_work[index] = pair_state.pair_nn[index] ^ pair_state.pair_pass[index] ^ pair_state.pair_ltk[index];
        }

        if SECURITY_ENABLE.get() == false {
            state.pkt_read_rsp.att_read_rsp_mut().value[1..1 + 0x10].copy_from_slice(&pair_state.pair_work[0..10])
        } else {
            aes_att_encryption(&pair_state.pair_sk, &pair_state.pair_work, &mut state.pkt_read_rsp.att_read_rsp_mut().value[1..]);
        }
        BLE_PAIR_ST.set(0xf);
    } else if BLE_PAIR_ST.get() == 0xd {
        if SECURITY_ENABLE.get() == false {
            state.pkt_read_rsp.head_mut().l2cap_len = 0x12;
            for index in 0..0x10 {
                pair_state.pair_work[index] = pair_state.pair_nn[index] ^ pair_state.pair_pass[index];
            }

            state.pkt_read_rsp.att_read_rsp_mut().value[1..1 + 0x10].copy_from_slice(&pair_state.pair_work);

            BLE_PAIR_ST.set(0xf);
            PAIR_ENC_ENABLE.set(false);
        } else {
            state.pkt_read_rsp.head_mut().l2cap_len = 0x12;
            pair_state.pair_rands[0..4].copy_from_slice(bytemuck::bytes_of(&read_reg_system_tick()));

            aes_att_encryption(&pair_state.pair_randm.clone(), &pair_state.pair_rands.clone(), &mut pair_state.pair_sk);
            let mut tmp = [0; 8];
            tmp.copy_from_slice(&pair_state.pair_sk[0..8]);
            pair_state.pair_rands.copy_from_slice(&tmp);

            pair_state.pair_sk[8..16].fill(0);

            for index in 0..0x10 {
                pair_state.pair_work[index] = pair_state.pair_nn[index] ^ pair_state.pair_pass[index];
            }

            aes_att_encryption(&pair_state.pair_sk.clone(), &pair_state.pair_work.clone(), &mut pair_state.pair_work);

            state.pkt_read_rsp.att_read_rsp_mut().value[1..1 + 8].copy_from_slice(&pair_state.pair_rands);
            state.pkt_read_rsp.att_read_rsp_mut().value[9..9 + 8].copy_from_slice(&pair_state.pair_work[0..8]);

            for index in 0..0x10 {
                pair_state.pair_work[index] = pair_state.pair_nn[index] ^ pair_state.pair_pass[index];
            }

            let pair_randm = pair_state.pair_randm;
            let pair_rands = pair_state.pair_rands;
            pair_state.pair_sk[0..8].copy_from_slice(&pair_randm);
            pair_state.pair_sk[8..16].copy_from_slice(&pair_rands);

            aes_att_encryption(&pair_state.pair_work.clone(), &pair_state.pair_sk.clone(), &mut pair_state.pair_sk);

            BLE_PAIR_ST.set(0xf);
            PAIR_ENC_ENABLE.set(true);
        }
    } else if BLE_PAIR_ST.get() == 0x9 {
        state.pkt_read_rsp.head_mut().l2cap_len = 0x12;
        if SECURITY_ENABLE.get() == false {
            state.pkt_read_rsp.att_read_rsp_mut().value[1..1 + 0x10].copy_from_slice(&pair_state.pair_ltk);
        } else {
            let pair_randm = pair_state.pair_randm;
            pair_state.pair_work[0..8].copy_from_slice(&pair_randm);
            pair_state.pair_work[8..16].fill(0);

            for index in 0..0x10 {
                pair_state.pair_sk[index] = pair_state.pair_nn[index] ^ pair_state.pair_pass[index] ^ pair_state.pair_work[index];
            }

            aes_att_encryption(&pair_state.pair_sk, &pair_state.pair_ltk, &mut state.pkt_read_rsp.att_read_rsp_mut().value[1..1 + 0x10]);
            BLE_PAIR_ST.set(0xf);

            pair_state.pair_sk = pair_state.pair_sk_copy;
        }
    } else if BLE_PAIR_ST.get() == 0xb {
        state.pkt_read_rsp.head_mut().l2cap_len = 2;
        BLE_PAIR_ST.set(0);
        if MESH_PAIR_ENABLE.get() {
            state.get_mac_en = true;
        }
        rf_link_delete_pair(state);
        rf_link_light_event_callback(state, LGT_CMD_DEL_PAIR);
    } else if BLE_PAIR_ST.get() == 0xe {
        state.pkt_read_rsp.head_mut().l2cap_len = 2;
        state.pkt_read_rsp.head_mut().dma_len = 8;
        state.pkt_read_rsp.head_mut().rf_len = 0x6;

        state.pkt_read_rsp.att_read_rsp_mut().value[0] = pair_st;

        BLE_PAIR_ST.set(0);

        return Some(&state.pkt_read_rsp);
    }

    state.pkt_read_rsp.head_mut().rf_len = state.pkt_read_rsp.head().l2cap_len as u8 + 0x4;
    state.pkt_read_rsp.head_mut().dma_len = state.pkt_read_rsp.head().l2cap_len as u32 + 6;

    state.pkt_read_rsp.att_read_rsp_mut().value[0] = pair_st;

    return Some(&state.pkt_read_rsp);
}

pub fn pair_set_key(state: &mut State, key: &[u8])
{
    {
        let pair_state_binding = PAIR_STATE.lock();
        let mut pair_state = pair_state_binding.borrow_mut();
        pair_state.pair_nn[0..state.max_mesh_name_len].copy_from_slice(&key[0..state.max_mesh_name_len]);
        pair_state.pair_pass.copy_from_slice(&key[16..16 + 16]);
        pair_state.pair_ltk.copy_from_slice(&key[32..32 + 16]);
    }

    pair_update_key(state);
}

pub fn pair_read(_: &mut State, _: &Packet) -> bool
{
    PAIR_READ_PENDING.set(true);

    true
}

pub fn pair_write(state: &mut State, data: &Packet) -> bool
{
    let opcode = data.att_write().value[0];
    let src = &data.att_write().value[1..];

    let pair_state_binding = PAIR_STATE.lock();
    let mut pair_state = pair_state_binding.borrow_mut();

    if opcode == 1 {
        pair_state.pair_randm.copy_from_slice(&src[0..8]);

        BLE_PAIR_ST.set(2);
        return true;
    }

    if opcode == 4 {
        if SECURITY_ENABLE.get() == false {
            if PAIR_LOGIN_OK.get() && BLE_PAIR_ST.get() == 0xf {
                pair_state.pair_nn.copy_from_slice(&src[0..16]);
                state.pair_setting_flag = ePairState::PairSetting;
                BLE_PAIR_ST.set(5);
                return true;
            }
        } else if BLE_PAIR_ST.get() == 0xf && (state.set_mesh_info_time == 0 || state.set_mesh_info_expired_flag == false) {
            pair_state.pair_work.copy_from_slice(&src[0..16]);

            aes_att_decryption(&pair_state.pair_sk.clone(), &pair_state.pair_work.clone(), &mut pair_state.pair_nn);

            let mut name_len = match pair_state.pair_nn.iter().position(|r| *r == 0) {
                Some(v) => v,
                None => pair_state.pair_nn.len()
            };

            if name_len <= state.max_mesh_name_len {
                state.pair_setting_flag = ePairState::PairSetting;
                BLE_PAIR_ST.set(5);
                return true;
            }
        }

        pair_par_init(state);
        PAIR_ENC_ENABLE.set(false);
        return true;
    }
    if opcode != 5 {
        if opcode == 6 {
            if SECURITY_ENABLE.get() {
                if BLE_PAIR_ST.get() == 6 {
                    pair_state.pair_work.copy_from_slice(&src[0..16]);
                    BLE_PAIR_ST.set(7);

                    if MESH_PAIR_ENABLE.get() && 0x14 < data.head().l2cap_len && data.att_write().value[0x11] != 0 {
                        aes_att_decryption(&pair_state.pair_sk.clone(), &pair_state.pair_work.clone(), &mut pair_state.pair_ltk_mesh);
                        state.pair_setting_flag = ePairState::PairSetMeshTxStart;
                        return true;
                    }

                    aes_att_decryption(&pair_state.pair_sk.clone(), &pair_state.pair_work.clone(), &mut pair_state.pair_ltk);
                    pair_save_key(state);
                    state.pair_setting_flag = ePairState::PairSetted;
                    rf_link_light_event_callback(state, 0xc5);
                    return true;
                }
            } else if PAIR_LOGIN_OK.get() && BLE_PAIR_ST.get() == 6 {
                pair_state.pair_ltk.fill(0);

                BLE_PAIR_ST.set(7);
                if MESH_PAIR_ENABLE.get() && 0x14 < data.head().l2cap_len && (((data.att_write().value[0x11] as u32) << 0x1f) as i32) < 0 {
                    pair_state.pair_ltk_mesh.copy_from_slice(&src[0..16]);
                    state.pair_setting_flag = ePairState::PairSetMeshTxStart;
                    return true;
                }

                pair_state.pair_ltk.copy_from_slice(&src[0..16]);

                pair_save_key(state);
                state.pair_setting_flag = ePairState::PairSetted;
                rf_link_light_event_callback(state, 0xc5);
                return true;
            }
            pair_par_init(state);
            state.pair_setting_flag = ePairState::PairSetted;
            PAIR_ENC_ENABLE.set(false);
            return true;
        }

        let mut index = if opcode - 8 != 0 { 0 } else { u8::MAX };
        let iVar2 = if opcode - 0xc != 0 { 0 } else { u8::MAX };
        if iVar2 == 0 {
            if index == 0 {
                if opcode != 10 {
                    if opcode == 0xe {
                        PAIR_ENC_ENABLE.set(false);
                        BLE_PAIR_ST.set(0);
                        return true;
                    }
                    return true;
                }

                index = 0;
            }
        } else if index != 0 {
            pair_state.pair_sk_copy = pair_state.pair_sk;
        }

        pair_state.pair_randm.copy_from_slice(&src[0..8]);
        let pair_randm = pair_state.pair_randm;
        pair_state.pair_sk[0..8].copy_from_slice(&pair_randm);
        pair_state.pair_sk[8..16].fill(0);

        PAIR_ENC_ENABLE.set(false);

        for index in 0..0x10 {
            pair_state.pair_work[index] = pair_state.pair_pass[index] ^ pair_state.pair_nn[index];
        }

        if SECURITY_ENABLE.get() == false {
            index = if pair_state.pair_work == src[0..16] { u8::MAX } else { 0 };
        } else {
            aes_att_encryption(&pair_state.pair_sk.clone(), &pair_state.pair_work.clone(), &mut pair_state.pair_work);
            index = if &pair_state.pair_work[0..8] == &data.att_write().value[9..9 + 8] { u8::MAX } else { 0 };
        }
        if iVar2 != 0 || PAIR_LOGIN_OK.get() {
            if index != 0 {
                if opcode == 8 {
                    BLE_PAIR_ST.set(9);
                    return true;
                }
                if iVar2 == 0 {
                    BLE_PAIR_ST.set(0xb);
                    return true;
                }
                PAIR_LOGIN_OK.set(true);
                BLE_PAIR_ST.set(0xd);
                return true;
            }
            if iVar2 != 0 {
                PAIR_LOGIN_OK.set(false);
            }
        }
        pair_par_init(state);
        return true;
    }
    if SECURITY_ENABLE.get() {
        if BLE_PAIR_ST.get() != 5 {
            pair_par_init(state);
            PAIR_ENC_ENABLE.set(false);
            return true;
        }
        pair_state.pair_work.copy_from_slice(&src[0..16]);
        aes_att_decryption(&pair_state.pair_sk.clone(), &pair_state.pair_work.clone(), &mut pair_state.pair_pass);
    } else {
        if !PAIR_LOGIN_OK.get() || BLE_PAIR_ST.get() != 5 {
            pair_par_init(state);
            PAIR_ENC_ENABLE.set(false);
            return true;
        }

        pair_state.pair_pass.copy_from_slice(&src[0..16]);
    }

    BLE_PAIR_ST.set(6);
    if (if pair_state.pair_nn == pair_state.pair_pass { usize::MAX } else { 0 }) == 0 {
        return true;
    }

    pair_par_init(state);
    PAIR_ENC_ENABLE.set(false);
    return true;
}