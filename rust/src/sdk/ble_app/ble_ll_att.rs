use core::slice;
use crate::{pub_mut, uprintln};

pub_mut!(ble_ll_channelNum, u32, 0);
pub_mut!(ble_ll_lastUnmappedCh, u32, 0);
pub_mut!(ble_ll_channelTable, [u8; 40], [0; 40]);

pub fn ble_ll_channel_table_calc(mut channel: *const u8, reset: bool)
{
    set_ble_ll_channelNum(0);
    if reset {
        set_ble_ll_lastUnmappedCh(0);
    }
    for chan_id in (0..0x28).step_by(8) {
        for shift in 0..8 {
            if (unsafe { *channel } >> shift) & 1 != 0 {
                (*get_ble_ll_channelTable())[*get_ble_ll_channelNum() as usize] = chan_id + shift;
                set_ble_ll_channelNum(*get_ble_ll_channelNum() + 1);
            }
        };
        channel = unsafe { channel.offset(1) };
    }
}

pub fn ble_ll_conn_get_next_channel(channel_map: *const u8, hop: u8) -> u32
{
    let mut index = (*get_ble_ll_lastUnmappedCh() as u8 + hop) % 0x25;
    set_ble_ll_lastUnmappedCh(index as u32);
    unsafe {
        if (*channel_map.offset((index >> 3) as isize) >> (index & 7)) & 0x80 == 0 {
            index = (*get_ble_ll_channelTable())[(index % *get_ble_ll_channelNum() as u8) as usize];
        }
    }
    return index as u32;
}