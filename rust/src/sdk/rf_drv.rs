use core::ptr::addr_of;
use crate::common::{get_set_uuid_flag, set_set_uuid_flag};
use crate::config::get_flash_adr_dev_grp_adr;
use crate::main_light::rf_link_light_event_callback;
use crate::{uprintln};
use crate::sdk::ble_app::rf_drv_8266::{get_pkt_adv, get_user_data, get_user_data_len};
use crate::sdk::drivers::flash::{flash_erase_sector, flash_write_page};
use crate::sdk::light::{get_dev_address_next_pos, get_dev_grp_next_pos, get_device_address, get_device_address_addr, get_device_address_mask, get_group_address, ll_packet_l2cap_data_t, set_dev_address_next_pos, set_dev_grp_next_pos, set_device_address};
use crate::sdk::mcu::register::write_reg32;

pub enum RF_POWER {
    RF_POWER_8dBm = 0,
    RF_POWER_4dBm = 1,
    RF_POWER_0dBm = 2,
    RF_POWER_m4dBm = 3,
    RF_POWER_m10dBm = 4,
    RF_POWER_m14dBm = 5,
    RF_POWER_m20dBm = 6,
    RF_POWER_m24dBm = 8,
    RF_POWER_m28dBm = 9,
    RF_POWER_m30dBm = 10,
    RF_POWER_m37dBm = 11,
    RF_POWER_OFF = 16,
}

pub fn rf_link_slave_set_adv_mesh_name(name: &[u8])
{
    let mut iVar1 = 0;
    let mut iVar2;
    let mut iVar3;

    if get_pkt_adv().rf_len as i8 - 6 < 1 {
        iVar3 = 1;
        iVar1 = 2;
        iVar2 = 0;
    } else {
        iVar2 = 0;
        let mut breakit = false;
        loop {
            iVar3 = iVar2 + 1;
            if get_pkt_adv().data[iVar2 + 1] == 9 {
                iVar1 = iVar2 + 2;
                breakit = true;
                break;
            }
            iVar2 = get_pkt_adv().data[iVar2] as usize + 1 + iVar2;
            if iVar2 as i8 >= get_pkt_adv().rf_len as i8 - 6 {
                break;
            }
        }
        if !breakit {
            iVar1 = iVar2 + 2;
            iVar3 = iVar2 + 1;
        }
    }

    get_pkt_adv().data[iVar1..iVar1 + name.len()].copy_from_slice(name);

    get_pkt_adv().data[iVar2] = name.len() as u8 + 1;
    get_pkt_adv().data[iVar3] = 9;
    get_pkt_adv().dma_len = (iVar2 + name.len() + 2 + 8) as u32;
    get_pkt_adv().rf_len = (iVar2 + name.len() + 2 + 6) as u8;
}

pub fn rf_link_slave_set_adv_private_data(data: &[u8])
{
    let mut iVar3 = 0;
    let iVar4;
    let mut iVar5;

    let mut rf_len = get_pkt_adv().rf_len;
    if rf_len as i8 - 6 < 1 {
        iVar4 = 1;
        iVar3 = 2;
        iVar5 = 0;
    } else {
        iVar5 = 0;
        iVar3 = 0;
        loop {
            let uVar6 = get_pkt_adv().data[iVar3] + 1;
            get_pkt_adv().data[iVar5 as usize..iVar5 as usize + uVar6 as usize].copy_from_slice(&get_pkt_adv().data[iVar3 as usize..iVar3 as usize + uVar6 as usize]);
            if get_pkt_adv().data[iVar3 as usize + 1] == 0xff {
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
    get_pkt_adv().data[iVar3..iVar3 + data.len()].copy_from_slice(data);

    get_pkt_adv().data[iVar5 as usize] = data.len() as u8 + 1;
    get_pkt_adv().data[iVar4 as usize] = 0xff;
    iVar5 = data.len() as u8 + 2 + iVar5;
    get_pkt_adv().dma_len = iVar5 as u32 + 8;
    get_pkt_adv().rf_len = iVar5 + 6;

    if iVar5 + 6 + *get_user_data_len() < 0x26 && *get_user_data_len() != 0 {
        get_pkt_adv().data[iVar5 as usize..iVar5 as usize + *get_user_data_len() as usize].copy_from_slice(get_user_data().as_slice());
        get_pkt_adv().dma_len += *get_user_data_len() as u32;
        get_pkt_adv().rf_len += *get_user_data_len();
    }
}

pub fn rf_link_slave_set_adv_uuid_data(uuid_data: &[u8])
{
    let mut rf_len = get_pkt_adv().rf_len;
    if uuid_data.len() as i8 <= 0x25 - rf_len as i8 {
        if *get_set_uuid_flag() == false {
            rf_len = rf_len - 9;

            let mut tmp_data = [0u8; 31];
            tmp_data[0..rf_len as usize].copy_from_slice(&get_pkt_adv().data[3..3 + rf_len as usize]);

            get_pkt_adv().data[3..3 + uuid_data.len()].copy_from_slice(uuid_data);
            get_pkt_adv().data[3 + uuid_data.len()..3 + uuid_data.len() + rf_len as usize].copy_from_slice(&tmp_data[0..rf_len as usize]);

            set_set_uuid_flag(true);
            get_pkt_adv().rf_len += uuid_data.len() as u8;
            get_pkt_adv().dma_len += uuid_data.len() as u32;
        } else {
            let uuid_len = uuid_data.len() + 3;
            rf_len = (rf_len - 6) - uuid_len as u8;

            let mut tmp_data = [0u8; 31];
            tmp_data[0..uuid_len as usize].copy_from_slice(&get_pkt_adv().data[0..0 + uuid_len as usize]);

            get_pkt_adv().data[3..3 + uuid_data.len()].copy_from_slice(uuid_data);
            get_pkt_adv().data[3 + uuid_data.len()..3 + uuid_data.len() + rf_len as usize].copy_from_slice(&tmp_data[0..rf_len as usize]);
        }
    }
}


pub fn dev_grp_flash_clean()
{
    if 0xfff < *get_dev_grp_next_pos() || 0xfff < *get_dev_address_next_pos() {
        flash_erase_sector(*get_flash_adr_dev_grp_adr());
        flash_write_page(*get_flash_adr_dev_grp_adr(), 0x10, get_group_address().as_ptr() as *const u8);
        flash_write_page(*get_flash_adr_dev_grp_adr() + 0x10, 2, get_device_address_addr() as *const u8);
        set_dev_address_next_pos(0x12);
        set_dev_grp_next_pos(0x12);
    }
    return;
}

pub fn rf_link_add_dev_addr(dev_id: u16) -> bool
{
  let mut tmp_dev_id = dev_id;
  let mut result = false;
  if tmp_dev_id != 0 && tmp_dev_id & !*get_device_address_mask() == 0 && *get_device_address() != tmp_dev_id {
    result = true;
    let write_buffer = dev_id;
    dev_grp_flash_clean();
    tmp_dev_id = *get_dev_grp_next_pos();
    if tmp_dev_id != 0 && *get_dev_address_next_pos() != 0 {
      let zero_data = 0u16;
      flash_write_page(*get_flash_adr_dev_grp_adr() -2 + *get_dev_address_next_pos() as u32,2,addr_of!(zero_data) as *const u8);
      tmp_dev_id = *get_dev_grp_next_pos();
    }
    set_device_address(write_buffer);
    flash_write_page(tmp_dev_id as u32 + *get_flash_adr_dev_grp_adr(),2,addr_of!(write_buffer) as *const u8);
    set_dev_grp_next_pos(*get_dev_grp_next_pos() + 2);
    set_dev_address_next_pos(*get_dev_grp_next_pos());
    rf_link_light_event_callback(0xc6);
    result = true;
  }
  return result;
}


pub fn rf_link_del_group(group_id: u16) -> bool
{
    let mut grp_next_pos = *get_dev_grp_next_pos();
    let mut grp_index = 0;
    let mut result = false;
    let mut iVar4 = 0;
    if grp_next_pos != 0 {
        let mut iVar3 = 0;
        let mut iVar1 = 0;
        let mut breakit = false;
        loop {
            iVar4 = iVar1;
            while group_id == 0xffff {
                (*get_group_address())[grp_index] = 0;
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
            if (*get_group_address())[grp_index] == group_id {
                (*get_group_address())[iVar3] = 0;
                iVar4 = 2;
                break;
            }
            iVar3 = iVar3 + 1;
            grp_index += 1;
            iVar1 = iVar4;

            if iVar3 == 8 {
                break;
            }
        }

        let zero_short = 0u16;
        let mut bVar2 = 0;
        result = false;
        loop {
            grp_next_pos = grp_next_pos - 2;

            let p_group_address = (*get_flash_adr_dev_grp_adr() + grp_next_pos as u32) as *const u16;
            unsafe {
                if *p_group_address & !*get_device_address_mask() != 0 {
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
            if (grp_next_pos as i16) < 0 {
                break;
            }
        }
    }
    return result;
}

static mut oldest_pos: u32 = 0xffffffff;

pub fn rf_link_add_group(group_id: u16) -> bool
{
    if group_id + 0x8000 < 0x7fff {
        dev_grp_flash_clean();
        if *get_dev_grp_next_pos() == 0 {
            (*get_group_address())[0] = group_id;
        } else {
            let mut index = 0;
            loop {
                if group_id == (*get_group_address())[index] {
                    return false;
                }
                if (*get_group_address())[index] == 0 {
                    (*get_group_address())[index] = group_id;

                    flash_write_page(*get_dev_grp_next_pos() as u32 + *get_flash_adr_dev_grp_adr(), 2, addr_of!(group_id) as *const u8);
                    set_dev_grp_next_pos(*get_dev_grp_next_pos() + 2);
                    return true;
                }
                index = index + 1;

                if index == 8 {
                    break;
                }
            }
            unsafe {
                if oldest_pos == 0xffffffff {
                    oldest_pos = 0;
                }
                rf_link_del_group((*get_group_address())[oldest_pos as usize]);
                (*get_group_address())[oldest_pos as usize] = group_id;
                oldest_pos = (oldest_pos + 1) % 8;
            }
        }

        flash_write_page(*get_dev_grp_next_pos() as u32 + *get_flash_adr_dev_grp_adr(), 2, addr_of!(group_id) as *const u8);
        set_dev_grp_next_pos(*get_dev_grp_next_pos() + 2);
        return true;
    }
    return false;
}

