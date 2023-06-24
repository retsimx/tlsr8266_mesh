use core::ptr::addr_of;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use crate::config::FLASH_ADR_DEV_GRP_ADR;
use crate::main_light::rf_link_light_event_callback;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_write_page};
use crate::sdk::light::{DEVICE_ADDR_MASK_DEFAULT};
use crate::state::State;

pub enum RfPower {
    RfPower8dBm = 0,
    RfPower4dBm = 1,
    RfPower0dBm = 2,
    RfPowerM4dBm = 3,
    RfPowerM10dBm = 4,
    RfPowerM14dBm = 5,
    RfPowerM20dBm = 6,
    RfPowerM24dBm = 8,
    RfPowerM28dBm = 9,
    RfPowerM30dBm = 10,
    RfPowerM37dBm = 11,
    RfPowerOff = 16,
}

pub fn rf_link_slave_set_adv_mesh_name(state: &mut State, name: &[u8])
{
    let mut iVar1 = 0;
    let mut iVar2;
    let mut iVar3;

    if state.pkt_adv.rf_len as i8 - 6 < 1 {
        iVar3 = 1;
        iVar1 = 2;
        iVar2 = 0;
    } else {
        iVar2 = 0;
        let mut breakit = false;
        loop {
            iVar3 = iVar2 + 1;
            if state.pkt_adv.data[iVar2 + 1] == 9 {
                iVar1 = iVar2 + 2;
                breakit = true;
                break;
            }
            iVar2 = state.pkt_adv.data[iVar2] as usize + 1 + iVar2;
            if iVar2 as i8 >= state.pkt_adv.rf_len as i8 - 6 {
                break;
            }
        }
        if !breakit {
            iVar1 = iVar2 + 2;
            iVar3 = iVar2 + 1;
        }
    }

    state.pkt_adv.data[iVar1..iVar1 + name.len()].copy_from_slice(name);

    state.pkt_adv.data[iVar2] = name.len() as u8 + 1;
    state.pkt_adv.data[iVar3] = 9;
    state.pkt_adv.dma_len = (iVar2 + name.len() + 2 + 8) as u32;
    state.pkt_adv.rf_len = (iVar2 + name.len() + 2 + 6) as u8;
}

pub fn rf_link_slave_set_adv_private_data(state: &mut State, data: &[u8])
{
    let mut iVar3 = 0;
    let iVar4;
    let mut iVar5;

    let mut rf_len = state.pkt_adv.rf_len;
    if rf_len as i8 - 6 < 1 {
        iVar4 = 1;
        iVar3 = 2;
        iVar5 = 0;
    } else {
        iVar5 = 0;
        iVar3 = 0;
        loop {
            let uVar6 = state.pkt_adv.data[iVar3] + 1;
            let adv_data = state.pkt_adv.data.clone();
            state.pkt_adv.data[iVar5 as usize..iVar5 as usize + uVar6 as usize].copy_from_slice(&adv_data[iVar3..iVar3 + uVar6 as usize]);
            if state.pkt_adv.data[iVar3 + 1] == 0xff {
                if uVar6 == 10 {
                    break;
                }
            } else {
                iVar5 = iVar5 + uVar6;
            }
            iVar3 = iVar3 + uVar6 as usize;
            if iVar3 as i8 >= rf_len as i8 - 6 {
                break;
            }
        }
        iVar3 = iVar5 as usize + 2;
        iVar4 = iVar5 + 1;
    }
    state.pkt_adv.data[iVar3..iVar3 + data.len()].copy_from_slice(data);

    state.pkt_adv.data[iVar5 as usize] = data.len() as u8 + 1;
    state.pkt_adv.data[iVar4 as usize] = 0xff;
    iVar5 = data.len() as u8 + 2 + iVar5;
    state.pkt_adv.dma_len = iVar5 as u32 + 8;
    state.pkt_adv.rf_len = iVar5 + 6;

    if iVar5 + 6 + state.user_data_len < 0x26 && state.user_data_len != 0 {
        state.pkt_adv.data[iVar5 as usize..iVar5 as usize + state.user_data_len as usize].copy_from_slice(state.user_data.as_slice());
        state.pkt_adv.dma_len += state.user_data_len as u32;
        state.pkt_adv.rf_len += state.user_data_len;
    }
}

pub fn rf_link_slave_set_adv_uuid_data(state: &mut State, uuid_data: &[u8])
{
    let mut rf_len = state.pkt_adv.rf_len as usize;
    if uuid_data.len() as i8 <= 0x25 - rf_len as i8 {
        if state.set_uuid_flag == false {
            rf_len = rf_len - 9;

            let mut tmp_data = [0u8; 31];
            tmp_data[0..rf_len].copy_from_slice(&state.pkt_adv.data[3..3 + rf_len]);

            state.pkt_adv.data[3..3 + uuid_data.len()].copy_from_slice(uuid_data);
            state.pkt_adv.data[3 + uuid_data.len()..3 + uuid_data.len() + rf_len].copy_from_slice(&tmp_data[0..rf_len]);

            state.set_uuid_flag = true;
            state.pkt_adv.rf_len += uuid_data.len() as u8;
            state.pkt_adv.dma_len += uuid_data.len() as u32;
        } else {
            let uuid_len = uuid_data.len() + 3;
            rf_len = (rf_len - 6) - uuid_len;

            let mut tmp_data = [0u8; 31];
            tmp_data[0..uuid_len].copy_from_slice(&state.pkt_adv.data[0..0 + uuid_len]);

            state.pkt_adv.data[3..3 + uuid_data.len()].copy_from_slice(uuid_data);
            state.pkt_adv.data[3 + uuid_data.len()..3 + uuid_data.len() + rf_len].copy_from_slice(&tmp_data[0..rf_len]);
        }
    }
}


pub fn dev_grp_flash_clean(state: &mut State)
{
    if 0xfff < state.dev_grp_next_pos || 0xfff < state.dev_address_next_pos {
        flash_erase_sector(FLASH_ADR_DEV_GRP_ADR);
        flash_write_page(FLASH_ADR_DEV_GRP_ADR, 0x10, state.group_address.as_ptr() as *const u8);
        flash_write_page(FLASH_ADR_DEV_GRP_ADR + 0x10, 2, addr_of!(state.device_address) as *const u8);
        state.dev_address_next_pos = 0x12;
        state.dev_grp_next_pos = 0x12;
    }
    return;
}

pub fn rf_link_add_dev_addr(state: &mut State, dev_id: u16) -> bool
{
  let mut tmp_dev_id = dev_id;
  let mut result = false;
  if tmp_dev_id != 0 && tmp_dev_id & !DEVICE_ADDR_MASK_DEFAULT == 0 && state.device_address != tmp_dev_id {
    result = true;
    let write_buffer = dev_id;
    dev_grp_flash_clean(state);
    tmp_dev_id = state.dev_grp_next_pos;
    if tmp_dev_id != 0 && state.dev_address_next_pos != 0 {
      let zero_data = 0u16;
      flash_write_page(FLASH_ADR_DEV_GRP_ADR -2 + state.dev_address_next_pos as u32, 2, addr_of!(zero_data) as *const u8);
      tmp_dev_id = state.dev_grp_next_pos;
    }
    state.device_address = write_buffer;
    flash_write_page(tmp_dev_id as u32 + FLASH_ADR_DEV_GRP_ADR, 2, addr_of!(write_buffer) as *const u8);
    state.dev_grp_next_pos = state.dev_grp_next_pos + 2;
    state.dev_address_next_pos = state.dev_grp_next_pos;
    rf_link_light_event_callback(state, 0xc6);
    result = true;
  }
  return result;
}


pub fn rf_link_del_group(state: &mut State, group_id: u16) -> bool
{
    let mut grp_next_pos = state.dev_grp_next_pos;
    let mut grp_index = 0;
    let mut result = false;
    let mut iVar4 = 0;
    if grp_next_pos != 0 {
        let mut iVar3 = 0;
        let mut iVar1 = 0;
        let mut breakit = false;
        while iVar3 != 8 {
            iVar4 = iVar1;
            while group_id == 0xffff {
                state.group_address[grp_index] = 0;
                iVar4 = 1;
                iVar3 = iVar3 + 1;
                grp_index += 1;
                iVar1 = 1;
                if iVar3 == 8 {
                    breakit = true;
                    break;
                }

                iVar4 = iVar1;
            }
            if breakit {
                break;
            }
            if state.group_address[grp_index] == group_id {
                state.group_address[iVar3] = 0;
                iVar4 = 2;
                break;
            }
            iVar3 = iVar3 + 1;
            grp_index += 1;
            iVar1 = iVar4;
        }

        let zero_short = 0u16;
        let mut bVar2 = 0;
        result = false;
        while grp_next_pos >= 0 {
            grp_next_pos = grp_next_pos - 2;

            let p_group_address = (FLASH_ADR_DEV_GRP_ADR + grp_next_pos as u32) as *const u16;
            unsafe {
                if *p_group_address & !DEVICE_ADDR_MASK_DEFAULT != 0 {
                    if iVar4 == 1 {
                        flash_write_page(p_group_address as u32, 2, addr_of!(zero_short) as *const u8);
                        result = true;
                        bVar2 = bVar2 + 1;
                        if 7 < bVar2 {
                            return true;
                        }
                    } else if *p_group_address == group_id && iVar4 != 0 {
                        flash_write_page(p_group_address as u32, 2, addr_of!(zero_short) as *const u8);
                        return true;
                    }
                }
            }
        }
    }
    return result;
}

pub fn rf_link_add_group(state: &mut State, group_id: u16) -> bool
{
    static OLDEST_POS: AtomicUsize = AtomicUsize::new(0xffffffff);

    if group_id + 0x8000 < 0x7fff {
        dev_grp_flash_clean(state);
        if state.dev_grp_next_pos == 0 {
            state.group_address[0] = group_id;
        } else {
            for index in 0..8 {
                if group_id == state.group_address[index] {
                    return false;
                }
                if state.group_address[index] == 0 {
                    state.group_address[index] = group_id;

                    flash_write_page(state.dev_grp_next_pos as u32 + FLASH_ADR_DEV_GRP_ADR, 2, addr_of!(group_id) as *const u8);
                    state.dev_grp_next_pos = state.dev_grp_next_pos + 2;
                    return true;
                }
            }

            if OLDEST_POS.load(Ordering::Relaxed) == 0xffffffff {
                OLDEST_POS.store(0, Ordering::Relaxed);
            }
            rf_link_del_group(state, state.group_address[OLDEST_POS.load(Ordering::Relaxed)]);
            state.group_address[OLDEST_POS.load(Ordering::Relaxed)] = group_id;
            OLDEST_POS.store((OLDEST_POS.load(Ordering::Relaxed) + 1) % 8, Ordering::Relaxed);
        }

        flash_write_page(state.dev_grp_next_pos as u32 + FLASH_ADR_DEV_GRP_ADR, 2, addr_of!(group_id) as *const u8);
        state.dev_grp_next_pos = state.dev_grp_next_pos + 2;
        return true;
    }
    return false;
}

