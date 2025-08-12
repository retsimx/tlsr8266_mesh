use crate::config::FLASH_ADR_LIGHT_NEW_FW;
use crate::sdk::drivers::flash::{flash_read_page, flash_write_page};
use crate::sdk::light::OtaState;
use crate::state::{OTA_UPDATE_CURRENT_FLASH_ADDRESS, SimplifyLS};

/// Saves OTA data to flash memory
pub fn rf_ota_save_data(data: &[u8]) -> OtaState
{
    let addr = OTA_UPDATE_CURRENT_FLASH_ADDRESS.get() + FLASH_ADR_LIGHT_NEW_FW;
    flash_write_page(addr, data.len() as u32, data.as_ptr());

    let mut tmp = [0u8; 0x10];
    flash_read_page(addr, data.len() as u32, tmp.as_mut_ptr());

    if data == &tmp[..data.len()] {
        OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(OTA_UPDATE_CURRENT_FLASH_ADDRESS.get() + data.len() as u32);
        return OtaState::Continue;
    } else {
        return OtaState::Error;
    }
}
