use core::cell::RefCell;
use core::ops::{Deref, DerefMut};
use core::ptr::{addr_of, addr_of_mut, null_mut};
use core::slice;

use crate::common::{pair_flash_clean, pair_load_key, pair_update_key, save_pair_info};
use crate::main_light::{rf_link_light_event_callback};
use crate::sdk::ble_app::light_ll::rf_link_delete_pair;
use crate::sdk::light::{*};
use crate::sdk::mcu::crypto::{aes_att_decryption, aes_att_decryption_packet, aes_att_encryption, aes_att_encryption_packet, encode_password};
use crate::sdk::mcu::register::read_reg_system_tick;
use crate::state::{*};

pub fn pair_dec_packet(state: &mut State, ps: &mut PacketAttWrite) -> bool {
    if SECURITY_ENABLE.get() {
        state.pair_ivm[5..5 + 3].copy_from_slice(&ps.value[0..3]);

        let mut src: [u8; 2] = [0; 2];
        src.copy_from_slice(&ps.value[3..3+2]);

         aes_att_decryption_packet(
             PAIR_SK.lock().unwrap().borrow().deref(),
            &state.pair_ivm,
            &src,
            &mut ps.value[5..5+ps.l2cap_len as usize - 8]
        )
    } else {
        false
    }
}

pub fn pair_enc_packet(state: &mut State, ps: &mut PacketLlApp)
{
    if SECURITY_ENABLE.get() && ps.chan_id == 4 && ps.opcode == 0x1b && ps.handle == 0x12 {
        let tick = read_reg_system_tick();
        ps.app_cmd_v.sno[0] = tick as u8;
        ps.app_cmd_v.sno[1] = (tick >> 8) as u8;
        ps.app_cmd_v.sno[2] = (tick >> 16) as u8;

        state.pair_ivs[3..3 + 3].copy_from_slice(&ps.app_cmd_v.sno);
        state.pair_ivs[6] = ps.app_cmd_v.src as u8;
        state.pair_ivs[7] = (ps.app_cmd_v.src >> 8) as u8;

        aes_att_encryption_packet(
            PAIR_SK.lock().unwrap().borrow().deref(),
        &state.pair_ivs,
        unsafe { slice::from_raw_parts_mut(addr_of!(ps.app_cmd_v.dst) as *mut u8, 2) },
        unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.app_cmd_v.op),((ps.l2cap_len & 0xff) - 10) as usize) }
        );
    }
}

pub fn pair_dec_packet_mesh(ps: &mut MeshPkt) -> bool {
    let mut ltk = [0u8; 16];

    if !SECURITY_ENABLE.get() {
        return true;
    }

    if ps._type & 0x80 == 0 {
        return false;
    }

    let rf_len = ps.rf_len;
    if 0x13 < rf_len - 0x12 {
        return false;
    }

    ltk.copy_from_slice(PAIR_LTK.lock().unwrap().borrow().deref());

    if ps.chan_id == 0xffff {
        aes_att_decryption_packet(
            &ltk,
            unsafe { slice::from_raw_parts(addr_of!(ps.rf_len), 8) },
            unsafe { slice::from_raw_parts(addr_of!(ps.internal_par2[1]), 2) },
            unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.sno) as *mut u8, 0x1c) }
        )
    } else {
        aes_att_decryption_packet(
            &ltk,
            unsafe { slice::from_raw_parts(addr_of!(ps.handle1), 8) },
            unsafe { slice::from_raw_parts((addr_of!(ps.sno) as u32 + (rf_len as u32 - 0xb)) as *const u8, 4) },
            unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.op), rf_len as usize - 0x12) }
        )
    }
}

pub fn pair_enc_packet_mesh(ps: &mut MeshPkt) -> bool
{
    if SECURITY_ENABLE.get() {
        let ltk = PAIR_LTK.lock().unwrap().borrow().deref().clone();

        if ps.chan_id == 0xffff {
                aes_att_encryption_packet(
                    &ltk,
                    unsafe { slice::from_raw_parts(addr_of!(ps.rf_len), 8) } ,
                    unsafe { slice::from_raw_parts_mut(addr_of!(ps.internal_par2[1]) as *mut u8, 2) },
                    unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.sno) as *mut u8, 0x1c) }
                );

            return true
        } else {
            aes_att_encryption_packet(
                &ltk,
                unsafe { slice::from_raw_parts(addr_of!(ps.handle1), 8) },
                unsafe { slice::from_raw_parts_mut((addr_of!(ps.sno) as u32 + (ps.rf_len as u32 - 0xb)) as *mut u8, 4) },
                unsafe { slice::from_raw_parts_mut(addr_of_mut!(ps.op), ps.rf_len as usize - 0x12) },
            );

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
    let binding = PAIR_NN.lock().unwrap();
    let pair_nn = binding.borrow();
    pair_flash_save_config(state, 0x10, pair_nn.as_ptr(), pair_nn.len() as u32);

    pass.copy_from_slice(PAIR_PASS.lock().unwrap().borrow().deref());

    encode_password(state, &mut pass);

    pair_flash_save_config(state, 0x20, pass.as_ptr(), pass.len() as u32);
    let binding = PAIR_LTK.lock().unwrap();
    let pair_ltk = binding.borrow();
    pair_flash_save_config(state, 0x30, pair_ltk.as_ptr(), pair_ltk.len() as u32);

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

pub fn pair_proc(state: &mut State) -> *const PacketAttReadRsp
{
    let pair_st = BLE_PAIR_ST.get();
    if !PAIR_READ_PENDING.get() {
        return null_mut();
    }
    PAIR_READ_PENDING.set(false);

    let mut pair_ltk_binding = PAIR_LTK.lock().unwrap();
    let mut pair_ltk = pair_ltk_binding.borrow_mut();
    let mut pair_work_binding = PAIR_WORK.lock().unwrap();
    let mut pair_work = pair_work_binding.borrow_mut();
    let mut pair_nn_binding = PAIR_NN.lock().unwrap();
    let mut pair_nn = pair_nn_binding.borrow_mut();
    let mut pair_pass_binding = PAIR_PASS.lock().unwrap();
    let mut pair_pass = pair_pass_binding.borrow_mut();
    let mut pair_sk_binding = PAIR_SK.lock().unwrap();
    let mut pair_sk = pair_sk_binding.borrow_mut();
    let mut pair_rands_binding = PAIR_RANDS.lock().unwrap();
    let mut pair_rands = pair_rands_binding.borrow_mut();
    let mut pair_randm_binding = PAIR_RANDM.lock().unwrap();
    let mut pair_randm = pair_randm_binding.borrow_mut();
    let mut pair_sk_copy_binding = PAIR_SK_COPY.lock().unwrap();
    let mut pair_sk_copy = pair_sk_copy_binding.borrow_mut();

    if BLE_PAIR_ST.get() == 2 {
        if SECURITY_ENABLE.get() == false && !PAIR_LOGIN_OK.get() {
            pair_par_init(state);
            PAIR_ENC_ENABLE.set(false);
            return null_mut();
        }
        state.pkt_read_rsp.l2cap_len = 10;
        state.pkt_read_rsp.value[1..1 + 8].copy_from_slice(pair_rands.deref());
        BLE_PAIR_ST.set(0xc);
    } else if BLE_PAIR_ST.get() == 0x7 {
        if SECURITY_ENABLE.get() == false && !PAIR_LOGIN_OK.get() {
            pair_par_init(state);
            PAIR_ENC_ENABLE.set(false);
            return null_mut();
        }
        state.pkt_read_rsp.l2cap_len = 0x12;
        for index in 0..0x10 {
            pair_work[index] = pair_nn[index] ^ pair_pass[index] ^ pair_ltk[index];
        }

        if SECURITY_ENABLE.get() == false {
            state.pkt_read_rsp.value[1..1 + 0x10].copy_from_slice(&pair_work[0..10])
        } else {
             aes_att_encryption(pair_sk.deref(),  pair_work.deref(), &mut state.pkt_read_rsp.value[1..]);
        }
        BLE_PAIR_ST.set(0xf);
    } else if BLE_PAIR_ST.get() == 0xd {
        if SECURITY_ENABLE.get() == false {
            state.pkt_read_rsp.l2cap_len = 0x12;
            for index in 0..0x10 {
                pair_work[index] = pair_nn[index] ^ pair_pass[index];
            }

            state.pkt_read_rsp.value[1..1 + 0x10].copy_from_slice(&pair_work[0..0x10]);

            BLE_PAIR_ST.set(0xf);
            PAIR_ENC_ENABLE.set(false);
        } else {
            state.pkt_read_rsp.l2cap_len = 0x12;

            let tick = read_reg_system_tick();
            pair_rands[0] = tick as u8;
            pair_rands[1] = (tick >> 8) as u8;
            pair_rands[2] = (tick >> 16) as u8;
            pair_rands[3] = (tick >> 24) as u8;

            aes_att_encryption(pair_randm.deref(), pair_rands.deref(), pair_sk.deref_mut());
            pair_rands.copy_from_slice(&pair_sk[0..8]);

            pair_sk[8..16].fill(0);

            for index in 0..0x10 {
                pair_work[index] = pair_nn[index] ^ pair_pass[index];
            }

            let mut pair_work_copy = [0u8; 16];
            pair_work_copy.copy_from_slice(pair_work.deref());
            aes_att_encryption(pair_sk.deref(), &pair_work_copy, pair_work.deref_mut());

            state.pkt_read_rsp.value[1..1 + 8].copy_from_slice( pair_rands.deref());

            state.pkt_read_rsp.value[9..9 + 8].copy_from_slice( &pair_work[0..8]);

            for index in 0..0x10 {
                pair_work[index] = pair_nn[index] ^ pair_pass[index];
            }

            pair_sk[0..8].copy_from_slice( pair_randm.deref());
            pair_sk[8..16].copy_from_slice(pair_rands.deref());

            let mut pair_sk_copy = [0u8; 16];
            pair_sk_copy.copy_from_slice(pair_sk.deref());
            aes_att_encryption(pair_work.deref(), &pair_sk_copy, pair_sk.deref_mut());

            BLE_PAIR_ST.set(0xf);
            PAIR_ENC_ENABLE.set(true);
        }
    } else if BLE_PAIR_ST.get() == 0x9 {
        state.pkt_read_rsp.l2cap_len = 0x12;
        if SECURITY_ENABLE.get() == false {
            state.pkt_read_rsp.value[1..1 + 0x10].copy_from_slice(pair_ltk.deref());
        } else {
            pair_work[0..8].copy_from_slice(pair_randm.deref());
            pair_work[8..16].fill(0);

            for index in 0..0x10 {
                pair_sk[index] = pair_nn[index] ^ pair_pass[index] ^ pair_work[index];
            }
            
            aes_att_encryption(pair_sk.deref(),  pair_ltk.deref(), &mut state.pkt_read_rsp.value[1..1 + 0x10]);
            BLE_PAIR_ST.set(0xf);

            pair_sk.copy_from_slice(pair_sk_copy.deref());
        }
    } else if BLE_PAIR_ST.get() == 0xb {
        state.pkt_read_rsp.l2cap_len = 2;
        BLE_PAIR_ST.set(0);
        if state.mesh_pair_enable {
            state.get_mac_en = true;
        }
        rf_link_delete_pair(state);
        rf_link_light_event_callback(state, LGT_CMD_DEL_PAIR);
    } else if BLE_PAIR_ST.get() == 0xe {
        state.pkt_read_rsp.l2cap_len = 2;
        state.pkt_read_rsp.dma_len = 8;
        state.pkt_read_rsp.rf_len = 0x6;

        state.pkt_read_rsp.value[0] = pair_st;

        BLE_PAIR_ST.set(0);

        return &state.pkt_read_rsp;
    }

    state.pkt_read_rsp.rf_len = state.pkt_read_rsp.l2cap_len as u8 + 0x4;
    state.pkt_read_rsp.dma_len = state.pkt_read_rsp.l2cap_len as u32 + 6;

    state.pkt_read_rsp.value[0] = pair_st;

    return &state.pkt_read_rsp;
}

pub fn pair_set_key(state: &mut State, key: *const u8)
{
    PAIR_NN.lock().unwrap().borrow_mut()[0..state.max_mesh_name_len].copy_from_slice(unsafe { slice::from_raw_parts(key, state.max_mesh_name_len) });
    PAIR_PASS.lock().unwrap().borrow_mut().copy_from_slice(unsafe { slice::from_raw_parts(key.offset(0x10), 0x10) });
    PAIR_LTK.lock().unwrap().borrow_mut().copy_from_slice(unsafe { slice::from_raw_parts(key.offset(0x20), 0x10) });

    pair_update_key(state);
}

pub fn pair_read(_: &mut State, _: &mut PacketAttWrite) -> bool
{
    PAIR_READ_PENDING.set(true);

    true
}

pub fn pair_write(state: &mut State, data: &mut PacketAttWrite) -> bool
{
    let opcode = unsafe { (*data).value[0] as i8 };
    let src = unsafe { addr_of!((*data).value[1]) };

    if opcode == 1 {
        PAIR_RANDM.lock().unwrap().borrow_mut().copy_from_slice(unsafe { slice::from_raw_parts(src, 8) });

        BLE_PAIR_ST.set(2);
        return true;
    }

    let mut pair_ltk_binding = PAIR_LTK.lock().unwrap();
    let mut pair_ltk = pair_ltk_binding.borrow_mut();
    let mut pair_work_binding = PAIR_WORK.lock().unwrap();
    let mut pair_work = pair_work_binding.borrow_mut();
    let mut pair_nn_binding = PAIR_NN.lock().unwrap();
    let mut pair_nn = pair_nn_binding.borrow_mut();
    let mut pair_pass_binding = PAIR_PASS.lock().unwrap();
    let mut pair_pass = pair_pass_binding.borrow_mut();
    let mut pair_sk_binding = PAIR_SK.lock().unwrap();
    let mut pair_sk = pair_sk_binding.borrow_mut();
    let mut pair_randm_binding = PAIR_RANDM.lock().unwrap();
    let mut pair_randm = pair_randm_binding.borrow_mut();
    let mut pair_sk_copy_binding = PAIR_SK_COPY.lock().unwrap();
    let mut pair_sk_copy = pair_sk_copy_binding.borrow_mut();
    let mut pair_ltk_mesh_binding = PAIR_LTK_MESH.lock().unwrap();
    let mut pair_ltk_mesh = pair_ltk_mesh_binding.borrow_mut();

    if opcode == 4 {
        if SECURITY_ENABLE.get() == false {
            if PAIR_LOGIN_OK.get() && BLE_PAIR_ST.get() == 0xf {
                pair_nn.copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
                state.pair_setting_flag = PairState::PairSetting;
                BLE_PAIR_ST.set(5);
                return true;
            }
        } else if BLE_PAIR_ST.get() == 0xf && (state.set_mesh_info_time == 0 || state.set_mesh_info_expired_flag == false) {
            pair_work.copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });

            aes_att_decryption(pair_sk.deref(),  pair_work.deref(), pair_nn.deref_mut());

            let mut name_len = match pair_nn.iter().position(|r| *r == 0) {
                Some(v) => v,
                None => pair_nn.len()
            };

            if name_len <= state.max_mesh_name_len {
                state.pair_setting_flag = PairState::PairSetting;
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
                    pair_work.copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
                    BLE_PAIR_ST.set(7);
                    unsafe {
                        if state.mesh_pair_enable && 0x14 < (*data).l2cap_len && (*data).value[0x11] != 0 {
                            aes_att_decryption(pair_sk.deref(), pair_work.deref(), pair_ltk_mesh.deref_mut());
                            state.pair_setting_flag = PairState::PairSetMeshTxStart;
                            return true;
                        }
                    }

                    aes_att_decryption(pair_sk.deref(), pair_work.deref(), pair_ltk.deref_mut());
                    pair_save_key(state);
                    state.pair_setting_flag = PairState::PairSetted;
                    rf_link_light_event_callback(state, 0xc5);
                    return true;
                }
            } else if PAIR_LOGIN_OK.get() && BLE_PAIR_ST.get() == 6 {
                pair_ltk.fill(0);

                BLE_PAIR_ST.set(7);
                unsafe {
                    if state.mesh_pair_enable && 0x14 < (*data).l2cap_len && ((((*data).value[0x11] as u32) << 0x1f) as i32) < 0 {
                        pair_ltk_mesh.copy_from_slice(slice::from_raw_parts(src, 0x10));
                        state.pair_setting_flag = PairState::PairSetMeshTxStart;
                        return true;
                    }
                }

                pair_ltk.copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });

                pair_save_key(state);
                state.pair_setting_flag = PairState::PairSetted;
                rf_link_light_event_callback(state, 0xc5);
                return true;
            }
            pair_par_init(state);
            state.pair_setting_flag = PairState::PairSetted;
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
            pair_sk_copy.copy_from_slice(pair_sk.deref());
        }

        pair_randm.copy_from_slice(unsafe { slice::from_raw_parts(src, 8) });
        pair_sk[0..8].copy_from_slice(pair_randm.deref());
        pair_sk[8..16].fill(0);

        PAIR_ENC_ENABLE.set(false);

        for index in 0..0x10 {
            pair_work[index] = pair_pass[index] ^ pair_nn[index];
        }

        if SECURITY_ENABLE.get() == false {
            index = if pair_work.deref() == unsafe { slice::from_raw_parts(src, 0x10) } { u8::MAX } else { 0 };
        } else {
            let mut pair_work_copy = [0u8; 16];
            pair_work_copy.copy_from_slice(pair_work.deref());
            aes_att_encryption(pair_sk.deref(), &pair_work_copy, pair_work.deref_mut());
            index = if &pair_work[0..8] == unsafe { slice::from_raw_parts(addr_of!((*data).value[9]), 8) } { u8::MAX } else { 0 };
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
        pair_work.copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
        aes_att_decryption( pair_sk.deref(), pair_work.deref(), pair_pass.deref_mut());
    } else {
        if !PAIR_LOGIN_OK.get() || BLE_PAIR_ST.get() != 5 {
            pair_par_init(state);
            PAIR_ENC_ENABLE.set(false);
            return true;
        }

        pair_pass.copy_from_slice(unsafe { slice::from_raw_parts(src, 0x10) });
    }

    BLE_PAIR_ST.set(6);
    if (if pair_nn.deref() == pair_pass.deref() { usize::MAX } else { 0 }) == 0 {
        return true;
    }

    pair_par_init(state);
    PAIR_ENC_ENABLE.set(false);
    return true;
}