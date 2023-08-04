use core::ptr::addr_of;
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::config::FLASH_ADR_DEV_GRP_ADR;
use crate::main_light::rf_link_light_event_callback;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_write_page};
use crate::sdk::light::{DEVICE_ADDR_MASK_DEFAULT};
use crate::state::{*};

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

pub fn rf_link_slave_set_adv_mesh_name(name: &[u8])
{
    let mut iVar1 = 0;
    let mut iVar2;
    let mut iVar3;

    let mut pkt_adv = PKT_ADV.lock();

    if pkt_adv.head().rf_len as i8 - 6 < 1 {
        iVar3 = 1;
        iVar1 = 2;
        iVar2 = 0;
    } else {
        iVar2 = 0;
        let mut breakit = false;
        loop {
            iVar3 = iVar2 + 1;
            if pkt_adv.adv_ind_module().data[iVar2 + 1] == 9 {
                iVar1 = iVar2 + 2;
                breakit = true;
                break;
            }
            iVar2 = pkt_adv.adv_ind_module().data[iVar2] as usize + 1 + iVar2;
            if iVar2 as i8 >= pkt_adv.head().rf_len as i8 - 6 {
                break;
            }
        }
        if !breakit {
            iVar1 = iVar2 + 2;
            iVar3 = iVar2 + 1;
        }
    }

    pkt_adv.adv_ind_module_mut().data[iVar1..iVar1 + name.len()].copy_from_slice(name);

    pkt_adv.adv_ind_module_mut().data[iVar2] = name.len() as u8 + 1;
    pkt_adv.adv_ind_module_mut().data[iVar3] = 9;
    pkt_adv.head_mut().dma_len = (iVar2 + name.len() + 2 + 8) as u32;
    pkt_adv.head_mut().rf_len = (iVar2 + name.len() + 2 + 6) as u8;
}

pub fn rf_link_slave_set_adv_private_data(data: &[u8])
{
    let mut iVar3 = 0;
    let iVar4;
    let mut iVar5;

    let mut pkt_adv = PKT_ADV.lock();

    let mut rf_len = pkt_adv.head().rf_len;
    if rf_len as i8 - 6 < 1 {
        iVar4 = 1;
        iVar3 = 2;
        iVar5 = 0;
    } else {
        iVar5 = 0;
        iVar3 = 0;
        loop {
            let uVar6 = pkt_adv.adv_ind_module().data[iVar3] + 1;
            let adv_data = pkt_adv.adv_ind_module().data.clone();
            pkt_adv.adv_ind_module_mut().data[iVar5 as usize..iVar5 as usize + uVar6 as usize].copy_from_slice(&adv_data[iVar3..iVar3 + uVar6 as usize]);
            if pkt_adv.adv_ind_module().data[iVar3 + 1] == 0xff {
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
    pkt_adv.adv_ind_module_mut().data[iVar3..iVar3 + data.len()].copy_from_slice(data);

    pkt_adv.adv_ind_module_mut().data[iVar5 as usize] = data.len() as u8 + 1;
    pkt_adv.adv_ind_module_mut().data[iVar4 as usize] = 0xff;
    iVar5 = data.len() as u8 + 2 + iVar5;
    pkt_adv.head_mut().dma_len = iVar5 as u32 + 8;
    pkt_adv.head_mut().rf_len = iVar5 + 6;
}

pub fn rf_link_slave_set_adv_uuid_data(uuid_data: &[u8])
{
    let mut pkt_adv = PKT_ADV.lock();

    let mut rf_len = pkt_adv.head().rf_len as usize;
    if uuid_data.len() as i8 <= 0x25 - rf_len as i8 {
        if SET_UUID_FLAG.get() == false {
            rf_len = rf_len - 9;

            let mut tmp_data = [0u8; 31];
            tmp_data[0..rf_len].copy_from_slice(&pkt_adv.adv_ind_module().data[3..3 + rf_len]);

            pkt_adv.adv_ind_module_mut().data[3..3 + uuid_data.len()].copy_from_slice(uuid_data);
            pkt_adv.adv_ind_module_mut().data[3 + uuid_data.len()..3 + uuid_data.len() + rf_len].copy_from_slice(&tmp_data[0..rf_len]);

            SET_UUID_FLAG.set(true);
            pkt_adv.head_mut().rf_len += uuid_data.len() as u8;
            pkt_adv.head_mut().dma_len += uuid_data.len() as u32;
        } else {
            let uuid_len = uuid_data.len() + 3;
            rf_len = (rf_len - 6) - uuid_len;

            let mut tmp_data = [0u8; 31];
            tmp_data[0..uuid_len].copy_from_slice(&pkt_adv.adv_ind_module().data[0..0 + uuid_len]);

            pkt_adv.adv_ind_module_mut().data[3..3 + uuid_data.len()].copy_from_slice(uuid_data);
            pkt_adv.adv_ind_module_mut().data[3 + uuid_data.len()..3 + uuid_data.len() + rf_len].copy_from_slice(&tmp_data[0..rf_len]);
        }
    }
}


pub fn dev_grp_flash_clean()
{
    if 0xfff < DEV_GRP_NEXT_POS.get() || 0xfff < DEV_ADDRESS_NEXT_POS.get() {
        flash_erase_sector(FLASH_ADR_DEV_GRP_ADR);
        flash_write_page(FLASH_ADR_DEV_GRP_ADR, 0x10, GROUP_ADDRESS.lock().as_ptr() as *const u8);
        let device_address = DEVICE_ADDRESS.get();
        flash_write_page(FLASH_ADR_DEV_GRP_ADR + 0x10, 2, addr_of!(device_address) as *const u8);
        DEV_ADDRESS_NEXT_POS.set(0x12);
        DEV_GRP_NEXT_POS.set(0x12);
    }
    return;
}

pub fn rf_link_add_dev_addr(dev_id: u16) -> bool
{
  let mut tmp_dev_id = dev_id;
  let mut result = false;
  if tmp_dev_id != 0 && tmp_dev_id & !DEVICE_ADDR_MASK_DEFAULT == 0 && DEVICE_ADDRESS.get() != tmp_dev_id {
    result = true;
    let write_buffer = dev_id;
    dev_grp_flash_clean();
    tmp_dev_id = DEV_GRP_NEXT_POS.get();
    if tmp_dev_id != 0 && DEV_ADDRESS_NEXT_POS.get() != 0 {
      let zero_data = 0u16;
      flash_write_page(FLASH_ADR_DEV_GRP_ADR -2 + DEV_ADDRESS_NEXT_POS.get() as u32, 2, addr_of!(zero_data) as *const u8);
      tmp_dev_id = DEV_GRP_NEXT_POS.get();
    }
    DEVICE_ADDRESS.set(write_buffer);
    flash_write_page(tmp_dev_id as u32 + FLASH_ADR_DEV_GRP_ADR, 2, addr_of!(write_buffer) as *const u8);
    DEV_GRP_NEXT_POS.set(DEV_GRP_NEXT_POS.get() + 2);
    DEV_ADDRESS_NEXT_POS.set(DEV_GRP_NEXT_POS.get());
    rf_link_light_event_callback(0xc6);
    result = true;
  }
  return result;
}


pub fn rf_link_del_group(group_id: u16) -> bool
{
    let mut grp_next_pos: i16 = DEV_GRP_NEXT_POS.get().try_into().unwrap();
    let mut grp_index = 0;
    let mut result = false;
    let mut iVar4 = 0;
    if grp_next_pos != 0 {
        let mut iVar3 = 0;
        let mut iVar1 = 0;
        let mut breakit = false;
        while iVar3 != 8 {
            iVar4 = iVar1;
            if group_id == 0xffff {
                GROUP_ADDRESS.lock()[grp_index] = 0;
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
            if GROUP_ADDRESS.lock()[grp_index] == group_id {
                GROUP_ADDRESS.lock()[iVar3] = 0;
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

pub fn rf_link_add_group(group_id: u16) -> bool
{
    static OLDEST_POS: AtomicUsize = AtomicUsize::new(0xffffffff);

    if group_id + 0x8000 < 0x7fff {
        dev_grp_flash_clean();
        if DEV_GRP_NEXT_POS.get() == 0 {
            GROUP_ADDRESS.lock()[0] = group_id;
        } else {
            for index in 0..8 {
                if group_id == GROUP_ADDRESS.lock()[index] {
                    return false;
                }
                if GROUP_ADDRESS.lock()[index] == 0 {
                    GROUP_ADDRESS.lock()[index] = group_id;

                    flash_write_page(DEV_GRP_NEXT_POS.get() as u32 + FLASH_ADR_DEV_GRP_ADR, 2, addr_of!(group_id) as *const u8);
                    DEV_GRP_NEXT_POS.set(DEV_GRP_NEXT_POS.get() + 2);
                    return true;
                }
            }

            if OLDEST_POS.load(Ordering::Relaxed) == 0xffffffff {
                OLDEST_POS.store(0, Ordering::Relaxed);
            }
            rf_link_del_group(GROUP_ADDRESS.lock()[OLDEST_POS.load(Ordering::Relaxed)]);
            GROUP_ADDRESS.lock()[OLDEST_POS.load(Ordering::Relaxed)] = group_id;
            OLDEST_POS.store((OLDEST_POS.load(Ordering::Relaxed) + 1) % 8, Ordering::Relaxed);
        }

        flash_write_page(DEV_GRP_NEXT_POS.get() as u32 + FLASH_ADR_DEV_GRP_ADR, 2, addr_of!(group_id) as *const u8);
        DEV_GRP_NEXT_POS.set(DEV_GRP_NEXT_POS.get() + 2);
        return true;
    }
    return false;
}

