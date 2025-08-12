use crate::sdk::ble_app::ble_ll_pair::{pair_set_key, pair_save_key};
use crate::state::{PAIR_CONFIG_MESH_NAME, PAIR_CONFIG_MESH_PWD, PAIR_LOGIN_OK, SimplifyLS};

/// Deletes the current pairing configuration
#[cfg_attr(test, mry::mry)]
pub fn rf_link_delete_pair()
{
    let mut key = [0u8; 16 * 3];

    key[0..0x10].copy_from_slice(&*PAIR_CONFIG_MESH_NAME.lock());
    key[0x10..0x20].copy_from_slice(&*PAIR_CONFIG_MESH_PWD.lock());

    pair_set_key(&key);
    pair_save_key();

    PAIR_LOGIN_OK.set(false);
}
