use crate::state::{*};

pub fn ble_ll_channel_table_calc(channel: &[u8], reset: bool)
{
    BLE_LL_CHANNEL_NUM.set(0);

    if reset {
        BLE_LL_LAST_UNMAPPED_CH.set(0);
    }

    for chan_id in (0..0x28).step_by(8) {
        for shift in 0..8 {
            if (channel[chan_id / 8] >> shift) & 1 != 0 {
                let chan_num = BLE_LL_CHANNEL_NUM.get();
                BLE_LL_CHANNEL_TABLE.lock()[chan_num] = chan_id as u8 + shift;

                BLE_LL_CHANNEL_NUM.inc();
            }
        };
    }
}

pub fn ble_ll_conn_get_next_channel(channel_map: &[u8], hop: u8) -> u32
{
    let mut index = (BLE_LL_LAST_UNMAPPED_CH.get() + hop as usize) % 0x25;
    BLE_LL_LAST_UNMAPPED_CH.set(index);

    if (channel_map[(index >> 3)] >> (index & 7)) & 0x80 == 0 {
        index = BLE_LL_CHANNEL_TABLE.lock()[(index % BLE_LL_CHANNEL_NUM.get())] as usize;
    }

    return index as u32;
}