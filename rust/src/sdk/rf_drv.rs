use crate::common::{get_set_uuid_flag, set_set_uuid_flag};
use crate::no_mangle_fn;
use crate::sdk::ble_app::rf_drv_8266::{get_pkt_adv, get_user_data, get_user_data_len};
use crate::sdk::light::ll_packet_l2cap_data_t;
use crate::sdk::mcu::register::write_reg32;

no_mangle_fn!(rf_set_power_level_index, level: u32);
no_mangle_fn!(rf_link_slave_pairing_enable, en: u32);
no_mangle_fn!(rf_link_slave_set_buffer, p: *mut [u32; 9], n: u8);
no_mangle_fn!(rf_link_set_max_bridge, num: u32);
no_mangle_fn!(
    rf_link_get_op_para,
    u32,
    p: *const ll_packet_l2cap_data_t,
    op: *mut u8,
    op_len: &u8,
    para: *mut u8,
    para_len: &u8,
    mesh_flag: u8
);
no_mangle_fn!(rf_link_del_group, bool, group: u16);
no_mangle_fn!(rf_link_add_group, bool, group: u16);
no_mangle_fn!(rf_link_add_dev_addr, bool, deviceaddress: u16);

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

pub fn rf_set_ble_access_code(p: *const u8) {
    unsafe {
        write_reg32(
            0x800408,
            *p.offset(3) as u32
                | (*p.offset(2) as u32) << 8
                | (*p.offset(1) as u32) << 16
                | (*p.offset(0) as u32) << 24,
        );
    }
}

pub fn rf_link_slave_set_adv_mesh_name(name: &[u8])
{
    let mut iVar1 = 0;
    let mut iVar2;
    let mut iVar3;

    if get_pkt_adv().rf_len as i8 - 6 <= 0 {
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
            if get_pkt_adv().data[iVar3 as usize + 1] as i8 == -1 {
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
    get_pkt_adv().rf_len = ((iVar5 as u32 + 6) * 0x1000000 >> 0x18) as u8;

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