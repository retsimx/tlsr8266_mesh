use core::cell::RefCell;

use crate::state::State;

pub fn ble_ll_channel_table_calc(state: &RefCell<State>, channel: &[u8], reset: bool)
{
    let mut state = state.borrow_mut();

    state.ble_ll_channel_num = 0;

    if reset {
        state.ble_ll_last_unmapped_ch = 0;
    }

    for chan_id in (0..0x28).step_by(8) {
        for shift in 0..8 {
            if (channel[chan_id / 8] >> shift) & 1 != 0 {
                let chan_num = state.ble_ll_channel_num;
                state.ble_ll_channel_table[chan_num] = chan_id as u8 + shift;

                state.ble_ll_channel_num += 1;
            }
        };
    }
}

pub fn ble_ll_conn_get_next_channel(state: &RefCell<State>, channel_map: &[u8], hop: u8) -> u32
{
    let mut state = state.borrow_mut();

    let mut index = (state.ble_ll_last_unmapped_ch + hop as usize) % 0x25;
    state.ble_ll_last_unmapped_ch = index;

    if (channel_map[(index >> 3)] >> (index & 7)) & 0x80 == 0 {
        index = state.ble_ll_channel_table[(index % state.ble_ll_channel_num)] as usize;
    }

    return index as u32;
}