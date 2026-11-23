use core::mem::size_of_val;
use core::ptr::addr_of;
use core::sync::atomic::{AtomicU16, Ordering};

use crate::config::{FLASH_ADR_LIGHT_NEW_FW, FLASH_SECTOR_SIZE, OTA_LED};
use crate::sdk::ble_app::ble_ll_pair::PAIR_LOGIN_OK;
use crate::sdk::ble_app::light_ll::connection_management::BUFF_RESPONSE;
use crate::sdk::ble_app::light_ll::ota_management::{
    rf_ota_save_data, OTA_UPDATE_CURRENT_FLASH_ADDRESS, OTA_UPDATE_IN_PROGRESS,
    OTA_UPDATE_MESH_OPERATIONS_BLOCKED, OTA_UPDATE_TERMINATION_REQUESTED,
    OTA_UPDATE_TIMEOUT_SECONDS, RF_SLAVE_OTA_FINISHED_FLAG,
};
use crate::sdk::ble_app::light_ll::packet_processing::{
    is_add_packet_buf_ready, rf_link_add_tx_packet,
};
use crate::sdk::ble_app::light_ll::status_management::{
    rf_link_slave_read_status_stop, SLAVE_READ_STATUS_BUSY,
};
use crate::sdk::ble_app::rf_drv_8266::PKT_TERMINATE;
use crate::sdk::light::*;
use mesh_protocol::crc::crc16;
use mesh_protocol::packet_types::{Packet, PacketAttData};
use tlsr8266_hal::common::bit::ONES_32;
use tlsr8266_hal::common::sync::CriticalSectionMutex;
use tlsr8266_hal::common::sync::SimplifyLS;
use tlsr8266_hal::drivers::flash::{
    flash_erase_sector, flash_read_page, flash_write_page, PAGE_SIZE,
};
use tlsr8266_hal::mcu::clock::{clock_time, clock_time_exceed, sleep_us};
use tlsr8266_hal::mcu::gpio::{gpio_set_func, gpio_set_output_en, gpio_write, AS_GPIO};
use tlsr8266_hal::mcu::irq_i::irq_disable;
use tlsr8266_hal::mcu::watchdog::wd_clear;
use tlsr8266_hal::pm::light_sw_reboot;

/// The mutable OTA runtime scalars shared by the GATT write handler, the mesh
/// OTA command path and the system-timer interrupt.
///
/// Everything that blocks (flash erase/write/read, the LED delay loop, the RF
/// terminate transmission) is deliberately kept *outside* this type: the owners
/// above take the [`OTA_STATE`] lock just long enough to move these four values
/// in or out, never across blocking I/O. See [`rf_link_slave_data_ota_save`] for
/// the pattern.
pub struct OtaRuntimeState {
    /// Index of the last OTA packet accepted in the current transfer.
    ota_rcv_last_idx: u16,
    /// Number of packets currently staged in [`BUFF_RESPONSE`].
    slave_ota_data_cache_idx: usize,
    /// `clock_time()` stamp taken when the transfer-finish flag was set, used by
    /// the reboot timeout.
    rf_slave_ota_finished_time: u32,
    /// Remaining `PKT_TERMINATE` notifications before the OTA reboot is allowed.
    terminate_cnt: u8,
}

impl OtaRuntimeState {
    /// Boot values, matching `App::default_const()` before the ownership split.
    pub const fn new() -> Self {
        Self {
            ota_rcv_last_idx: 0xffff,
            slave_ota_data_cache_idx: 0,
            rf_slave_ota_finished_time: 0,
            terminate_cnt: 0,
        }
    }
}

impl Default for OtaRuntimeState {
    fn default() -> Self {
        Self::new()
    }
}

/// Sole owner of the OTA runtime scalars.
pub static OTA_STATE: CriticalSectionMutex<OtaRuntimeState> =
    CriticalSectionMutex::new(OtaRuntimeState::new());

/// Restore [`OTA_STATE`] to its boot values.
///
/// Host tests run in one process, so the global would otherwise leak OTA state
/// from one test into the next. There is no production cost.
#[cfg(test)]
pub fn reset_for_test() {
    *OTA_STATE.lock() = OtaRuntimeState::new();
}

/// Flash address of the "firmware staged and ready to copy" sentinel byte.
///
/// Part of the wire/flash contract: `rust/src/wire_contract_tests.rs` asserts
/// these four constants against `tests/fixtures/wire_vectors.json`.
pub const FLASH_ADR_OTA_READY_FLAG: u32 = 0x3F000;
/// Sentinel written to [`FLASH_ADR_OTA_READY_FLAG`] once an image is staged.
pub const FLASH_OTA_READY_FLAG: u8 = 0xa5;
/// Upper bound on the staged image size, in KiB.
pub const FW_SIZE_MAX_K: u32 = 128;
/// Sectors erased before a staged image is copied out.
pub const ERASE_SECTORS_FOR_OTA: u32 = FW_SIZE_MAX_K.div_ceil(4);

/// Apply a staged firmware image from the OTA area and reboot into it.
///
/// Runs at boot before any task or interrupt exists, so it takes no lock at all
/// and must stay fully RAM-resident: flash reads/writes need the RAM code path.
#[inline(never)]
#[link_section = ".ram_code"]
// Boot-only `.ram_code` entrypoint: it needs a staged image in flash and a
// reset to run, so it cannot execute on the host.
#[coverage(off)]
pub fn handle_ota_update() {
    // This function requires that *everything* be in ram
    // SAFETY: (2) boot-OTA flag read. `FLASH_ADR_OTA_READY_FLAG` is a fixed byte
    // address in the flash window, read as one `u8` before any code from the staged
    // image runs; no reference to it is created.
    unsafe {
        if *(FLASH_ADR_OTA_READY_FLAG as *const u8) != FLASH_OTA_READY_FLAG {
            // Nothing to do
            return;
        }
    }

    // First configure the system clock. This lets delays work as expected for reading/writing
    // flash
    tlsr8266_hal::pm::ota_enable_clocks_for_copy();
    irq_disable();

    // Read the first page of the new firmware
    let mut buff: [u8; PAGE_SIZE as usize] = [0; PAGE_SIZE as usize];
    flash_read_page(FLASH_ADR_LIGHT_NEW_FW, PAGE_SIZE, buff.as_mut_ptr());

    // Get the size of the binary being copied
    // SAFETY: (2) boot-OTA image header. `buff` is the 256-byte page just read from
    // the staged image, so the 4-byte word at offset 0x18 (the `_bin_size_` field
    // of the header contract in `startup.rs`) is in bounds. The load needs a
    // 4-byte-aligned page; the 256-byte TC32 frame slot provides that alignment.
    let new_fw_size = unsafe { *(buff.as_ptr().offset(0x18) as *const u32) };

    let mut current_addr = 0;
    while current_addr < new_fw_size {
        // Check if we need to clear the next sector
        if current_addr % FLASH_SECTOR_SIZE as u32 == 0 {
            flash_erase_sector(current_addr);
        }

        // Read the page at this address from the new firmware
        flash_read_page(
            FLASH_ADR_LIGHT_NEW_FW + current_addr,
            PAGE_SIZE,
            buff.as_mut_ptr(),
        );

        // Write the page to the real firmware
        flash_write_page(current_addr, PAGE_SIZE, buff.as_mut_ptr());

        current_addr += PAGE_SIZE;
    }

    // Clear the OTA ready flag
    buff[0] = 0;
    flash_write_page(FLASH_ADR_OTA_READY_FLAG, 1, buff.as_mut_ptr());

    // Force reboot
    tlsr8266_hal::pm::mcu_trigger_reboot();

    #[allow(clippy::empty_loop)]
    loop {}
}

/// Check if OTA area is valid; if not or on error, erase it.
#[cfg_attr(test, mry::mry)]
pub fn check_ota_area_startup() {
    if let Ok(false) | Err(()) = is_ota_area_valid() {
        erase_ota_data();
    }
}

/// Validates the OTA firmware area by checking if it's in a clean state.
///
/// Performs hardware flash I/O to read OTA area sectors. Returns:
/// - `Ok(true)` if OTA area is valid (all sectors contain 0xFFFFFFFF)
/// - `Ok(false)` if OTA area is invalid (contains non-erased data)
/// - `Err(())` if hardware I/O error occurs
pub fn is_ota_area_valid() -> Result<bool, ()> {
    let mut buf: [u8; 4] = [0; 4];
    for i in 0..ERASE_SECTORS_FOR_OTA {
        flash_read_page(FLASH_ADR_LIGHT_NEW_FW + i * 0x1000, 4, buf.as_mut_ptr());
        let tmp =
            buf[0] as u32 | (buf[1] as u32) << 8 | (buf[2] as u32) << 16 | (buf[3] as u32) << 24;
        if tmp != ONES_32 {
            return Ok(false);
        }
    }

    Ok(true)
}

/// Erase every sector of the OTA staging area.
pub fn erase_ota_data() {
    for i in 0..ERASE_SECTORS_FOR_OTA {
        flash_erase_sector(FLASH_ADR_LIGHT_NEW_FW + (ERASE_SECTORS_FOR_OTA - 1 - i) * 0x1000);
    }
}

/// GATT OTA write handler (registered with `set_ota_write_handler`).
///
/// Stages one packet in [`BUFF_RESPONSE`]; the cache cursor moves in and out of
/// [`OTA_STATE`] under a short critical section, so no lock is held while the
/// packet is copied.
pub fn rf_link_slave_data_ota(data: &Packet) {
    if *RF_SLAVE_OTA_FINISHED_FLAG.lock() != OtaState::Continue
        || OTA_UPDATE_MESH_OPERATIONS_BLOCKED.get()
    {
        return;
    }

    if !OTA_UPDATE_IN_PROGRESS.get() {
        if !PAIR_LOGIN_OK.get() {
            return;
        }

        OTA_UPDATE_IN_PROGRESS.set(true);
        tlsr8266_hal::pm::set_reboot_ota_active(true);
        if SLAVE_READ_STATUS_BUSY.get() != 0 {
            rf_link_slave_read_status_stop();
        }
    }

    // Claim the next staging slot under a short critical section, then drop the
    // guard before touching BUFF_RESPONSE (a second, independent lock).
    let cache_idx = {
        let mut st = OTA_STATE.lock();
        let cache_idx = st.slave_ota_data_cache_idx;
        st.slave_ota_data_cache_idx += 1;
        cache_idx
    };

    BUFF_RESPONSE.lock()[cache_idx % 16] = *data;
}

/// Mesh OTA data command handler.
///
/// Returns the last accepted packet index (or `u16::MAX` when the packet was
/// rejected). Every [`OTA_STATE`] access is a short snapshot or store; the
/// persistence work happens in [`rf_link_slave_data_ota_save`].
#[cfg_attr(test, mry::mry)]
pub fn rf_mesh_data_ota(pkt_data: &[u8], last: bool) -> u16 {
    if *RF_SLAVE_OTA_FINISHED_FLAG.lock() != OtaState::Continue
        || OTA_UPDATE_IN_PROGRESS.get()
        || !OTA_UPDATE_MESH_OPERATIONS_BLOCKED.get()
    {
        return u16::MAX;
    }

    let mut data: PacketAttData = PacketAttData::default();
    data.head.l2cap_len = if last { 7 } else { 10 + 7 };
    data.dat[0..pkt_data.len()].copy_from_slice(pkt_data);

    let index = data.dat[0] as u16 | ((data.dat[1] as u16) << 8);

    static LAST_INDEX: AtomicU16 = AtomicU16::new(u16::MAX);
    if LAST_INDEX.load(Ordering::Relaxed) == u16::MAX || LAST_INDEX.load(Ordering::Relaxed) < index
    {
        LAST_INDEX.store(index, Ordering::Relaxed);

        BUFF_RESPONSE.lock()[0] = Packet { att_data: data };

        {
            let mut st = OTA_STATE.lock();
            st.slave_ota_data_cache_idx = 1;
        }

        rf_link_slave_data_ota_save();

        OTA_UPDATE_TIMEOUT_SECONDS.set(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS); // refresh timeout

        if *RF_SLAVE_OTA_FINISHED_FLAG.lock() != OtaState::Continue {
            OTA_UPDATE_TIMEOUT_SECONDS.set(4);
        }
    }

    LAST_INDEX.load(Ordering::Relaxed)
}

/// Persist the packets staged in [`BUFF_RESPONSE`].
///
/// # Critical-section invariant
///
/// [`OTA_STATE`] is locked twice, briefly: once to snapshot the staged-packet
/// count and the accepted-packet cursor, and once to publish the updated cursor.
/// The CRC pass, `flash_read_page` and `rf_ota_save_data` (which erase/write
/// flash) all run with **no** lock held.
fn rf_link_slave_data_ota_save() {
    let packet_len = if OTA_UPDATE_MESH_OPERATIONS_BLOCKED.get() {
        8
    } else {
        16
    };

    // Short critical section: copy the two cursors out, then drop the guard.
    let (cache_len, mut ota_rcv_last_idx) = {
        let st = OTA_STATE.lock();
        (st.slave_ota_data_cache_idx, st.ota_rcv_last_idx)
    };

    let mut reset_flag = OtaState::Continue;
    for i in 0..cache_len {
        // Copy the staged packet out; the BUFF_RESPONSE guard is a temporary
        // that dies at the end of this statement.
        let p = BUFF_RESPONSE.lock()[i];
        let n_data_len = (p.head().l2cap_len - 7) as usize;

        if OTA_UPDATE_MESH_OPERATIONS_BLOCKED.get()
            || crc16(&p.att_data().dat[0..n_data_len + 2])
                == p.att_data().dat[n_data_len + 2] as u16
                    | (p.att_data().dat[n_data_len + 3] as u16) << 8
        {
            OTA_UPDATE_TIMEOUT_SECONDS.set(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS); // refresh timeout

            let cur_idx = p.att_data().dat[0] as u16 | (p.att_data().dat[1] as u16) << 8;
            if n_data_len == 0 {
                let mut ota_pkt_total = [0u8; 4];
                // Blocking flash read: no OTA_STATE guard is held here.
                flash_read_page(FLASH_ADR_LIGHT_NEW_FW + 0x18, 4, ota_pkt_total.as_mut_ptr());
                let ota_pkt_total = u32::from_le_bytes(ota_pkt_total).div_ceil(packet_len) as u16;
                if ota_pkt_total < 3 {
                    // invalid fw
                    OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);
                    ota_rcv_last_idx = 0;
                    reset_flag = OtaState::Error;
                } else if cur_idx == ota_rcv_last_idx + 1 && cur_idx == ota_pkt_total {
                    // ota ok, save, reboot
                    reset_flag = OtaState::Ok;
                } else {
                    // ota err
                    OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);
                    ota_rcv_last_idx = 0;
                    reset_flag = OtaState::Error;
                }
            } else {
                if cur_idx == 0 {
                    // start ota
                    if OTA_UPDATE_CURRENT_FLASH_ADDRESS.get() != 0 {
                        // 0x10000 should be 0x00. The cursor is published as
                        // `cur_idx` below, matching the original field order.
                        OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);
                        reset_flag = OtaState::Error;
                    } else {
                        reset_flag =
                            rf_ota_save_data(&p.att_data().dat[2..2 + packet_len as usize]);
                    }
                } else if cur_idx == ota_rcv_last_idx + 1 {
                    if reset_flag != OtaState::Error {
                        if OTA_UPDATE_CURRENT_FLASH_ADDRESS.get() + packet_len
                            > (FW_SIZE_MAX_K * 1024)
                        {
                            // !is_valid_fw_len()
                            reset_flag = OtaState::Error;
                        } else {
                            reset_flag =
                                rf_ota_save_data(&p.att_data().dat[2..2 + packet_len as usize]);
                        }
                    }
                } else {
                    // error, ota failed. The cursor is published as `cur_idx`
                    // below, matching the original field order.
                    OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);
                    reset_flag = OtaState::Error;
                }

                ota_rcv_last_idx = cur_idx;
            }
        } else {
            // error, ota failed
            OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);
            ota_rcv_last_idx = 0;
            reset_flag = OtaState::Error;
        }

        if reset_flag != OtaState::Continue {
            // Copy the flag out: the guard must not span the flag-set path,
            // which takes the same lock again.
            let finished = *RF_SLAVE_OTA_FINISHED_FLAG.lock();
            if finished == OtaState::Continue {
                rf_slave_ota_finished_flag_set(reset_flag);
            }

            break;
        }
    }

    // Short critical section: publish the cursors, then drop the guard.
    {
        let mut st = OTA_STATE.lock();
        st.ota_rcv_last_idx = ota_rcv_last_idx;
        st.slave_ota_data_cache_idx = 0;
    }
}

/// Fix up the firmware flag of a staged image and arm the OTA ready flag.
pub fn rf_ota_set_flag() {
    let fw_flag_telink = START_UP_FLAG;
    let mut flag_new = 0;
    flash_read_page(
        FLASH_ADR_LIGHT_NEW_FW + 8,
        size_of_val(&flag_new) as u32,
        addr_of!(flag_new) as *mut u8,
    );
    flag_new &= 0xffffff4b;
    if flag_new != fw_flag_telink {
        return; // invalid flag
    }

    let mut flag0 = 0;
    flash_read_page(FLASH_ADR_LIGHT_NEW_FW + 8, 1, addr_of!(flag0) as *mut u8);
    if 0x4b != flag0 {
        flag0 = 0x4b;
        flash_write_page(FLASH_ADR_LIGHT_NEW_FW + 8, 1, addr_of!(flag0) as *mut u8);
        //Set FW flag, make sure valid. because the firmware may be from 8267 by mesh ota
    }

    flash_erase_sector(FLASH_ADR_OTA_READY_FLAG);
    let flag: u32 = FLASH_OTA_READY_FLAG as u32;
    flash_write_page(FLASH_ADR_OTA_READY_FLAG, 4, addr_of!(flag) as *mut u8);
}

/// Signal a successful OTA with the LED pattern.
pub fn rf_led_ota_ok() {
    tlsr8266_hal::pm::blink_ota_led_ok(OTA_LED as u32);
}

/// Signal a failed OTA by blinking the LED: 60 toggles of 100 ms.
///
/// The `sleep_us` delay loop runs with no lock held (there is nothing to lock).
pub fn rf_led_ota_error() {
    gpio_set_func(OTA_LED as u32, AS_GPIO);
    gpio_set_output_en(OTA_LED as u32, 1);
    let mut led_on = true;
    for _ in 0..60 {
        gpio_write(OTA_LED as u32, u32::from(led_on));
        led_on = !led_on;
        wd_clear();
        sleep_us(100 * 1000);
    }
}

/// Finish the OTA transfer: persist the flag / erase the staging area, give the
/// LED feedback, then reboot.
#[cfg_attr(test, mry::mry)]
pub fn rf_link_slave_ota_finish_led_and_reboot(st: OtaState) {
    match st {
        OtaState::Error => {
            erase_ota_data();
            rf_led_ota_error();
        }
        OtaState::Ok => {
            //rf_ota_save_data(0);
            rf_ota_set_flag();
            rf_led_ota_ok();
        }
        _ => (),
    }
    irq_disable();
    light_sw_reboot();
}

/// Publish the transfer-finish flag and stamp the time it was set.
fn rf_slave_ota_finished_flag_set(reset_flag: OtaState) {
    let finished_time = clock_time();
    *RF_SLAVE_OTA_FINISHED_FLAG.lock() = reset_flag;

    let mut st = OTA_STATE.lock();
    st.rf_slave_ota_finished_time = finished_time;
}

/// Poll when an OTA transfer is busy (bridge/connection path).
///
/// Every [`OTA_STATE`] touch is a short snapshot or decrement, so the RF
/// terminate transmission and the LED/erase/reboot path never run under the
/// lock.
#[cfg_attr(test, mry::mry)]
pub fn rf_link_slave_ota_finish_handle() {
    rf_link_slave_data_ota_save();

    // Copy the flag out: the guard must not span the handlers below, which take
    // the same lock again.
    let finished = *RF_SLAVE_OTA_FINISHED_FLAG.lock();
    if finished != OtaState::Continue {
        let mut reboot_flag = false;

        let terminate_idle = {
            let st = OTA_STATE.lock();
            st.terminate_cnt == 0
        };
        if terminate_idle && OTA_UPDATE_TERMINATION_REQUESTED.get() && is_add_packet_buf_ready() {
            {
                let mut st = OTA_STATE.lock();
                st.terminate_cnt = 6;
            }
            // Blocking RF transmit: no OTA_STATE guard held.
            let _ = rf_link_add_tx_packet(&PKT_TERMINATE);
        }

        {
            let mut st = OTA_STATE.lock();
            if st.terminate_cnt != 0 {
                st.terminate_cnt -= 1;
                if st.terminate_cnt == 0 {
                    reboot_flag = true;
                }
            }
        }

        let rf_slave_ota_finished_time = {
            let st = OTA_STATE.lock();
            st.rf_slave_ota_finished_time
        };

        if !OTA_UPDATE_TERMINATION_REQUESTED.get()
            && clock_time_exceed(rf_slave_ota_finished_time, 2000 * 1000)
        {
            OTA_UPDATE_TERMINATION_REQUESTED.set(true); // for ios: no last read command
        }

        if clock_time_exceed(rf_slave_ota_finished_time, 4000 * 1000) {
            reboot_flag = true;
        }

        if reboot_flag {
            // Copy the state out first: the guard must die before the erase /
            // LED / reboot path runs.
            let finished_state = *RF_SLAVE_OTA_FINISHED_FLAG.lock();
            rf_link_slave_ota_finish_led_and_reboot(finished_state);
            // have been rebooted
        }
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use mry::Any;
    use std::sync::{Arc, Mutex};
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::FakeMmio;
    use tlsr8266_pac::{
        mock_read_reg_gpio_oen, mock_read_reg_gpio_out, mock_write_reg_gpio_oen,
        mock_write_reg_gpio_out, read_reg_gpio_oen, read_reg_gpio_out, write_reg_gpio_oen,
        write_reg_gpio_out,
    };

    /// `reg_gpio_gpio_func`: per-port GPIO-function select register.
    const REG_GPIO_GPIO_FUNC: u32 = 0x586;
    /// `reg_tmr_ctrl`: the watchdog clear bit lives here.
    const REG_TMR_CTRL: u32 = 0x620;

    /// The HAL host seams are process-wide singletons, so every test that
    /// observes them starts from a known-empty bank and tick count.
    fn reset_fakes() {
        FakeMmio::reset();
        fake_clock::reset();
        FakeFlash::reset();
    }

    /// The GATT write handler stages packets without holding `OTA_STATE` while
    /// it copies into `BUFF_RESPONSE`, and advances the cursor once per packet.
    #[test]
    fn test_rf_link_slave_data_ota_stages_packets_and_advances_cursor() {
        reset_for_test();
        *RF_SLAVE_OTA_FINISHED_FLAG.lock() = OtaState::Continue;
        OTA_UPDATE_MESH_OPERATIONS_BLOCKED.set(false);
        OTA_UPDATE_IN_PROGRESS.set(true);
        PAIR_LOGIN_OK.set(true);

        let mut staged = PacketAttData::default();
        staged.dat[0] = 0xAB;
        let packet = Packet { att_data: staged };

        rf_link_slave_data_ota(&packet);
        rf_link_slave_data_ota(&packet);

        assert_eq!(OTA_STATE.lock().slave_ota_data_cache_idx, 2);
        assert_eq!(BUFF_RESPONSE.lock()[0].att_data().dat[0], 0xAB);
        assert_eq!(BUFF_RESPONSE.lock()[1].att_data().dat[0], 0xAB);

        PAIR_LOGIN_OK.set(false);
        OTA_UPDATE_IN_PROGRESS.set(false);
        reset_for_test();
    }

    /// `terminate_cnt` is the last owner-move: the countdown and the reboot must
    /// happen outside the critical section.
    #[test]
    #[mry::lock(
        rf_link_slave_ota_finish_handle,
        rf_link_slave_ota_finish_led_and_reboot
    )]
    fn test_finish_handle_counts_down_terminate_then_reboots() {
        reset_fakes();
        reset_for_test();
        *RF_SLAVE_OTA_FINISHED_FLAG.lock() = OtaState::Ok;
        OTA_UPDATE_TERMINATION_REQUESTED.set(true);
        OTA_STATE.lock().terminate_cnt = 1;

        fake_clock::set_ticks(0);
        mock_rf_link_slave_ota_finish_handle().calls_real_impl();
        mock_rf_link_slave_ota_finish_led_and_reboot(Any).returns(());

        rf_link_slave_ota_finish_handle();

        assert_eq!(OTA_STATE.lock().terminate_cnt, 0);
        mock_rf_link_slave_ota_finish_led_and_reboot(OtaState::Ok).assert_called(1);

        OTA_UPDATE_TERMINATION_REQUESTED.set(false);
        *RF_SLAVE_OTA_FINISHED_FLAG.lock() = OtaState::Continue;
        reset_for_test();
    }

    /// The OTA scalars boot at the values the old `OtaManager::default_const()`
    /// used, and `reset_for_test()` restores them after a transfer.
    #[test]
    fn test_ota_runtime_state_defaults_and_reset_for_test() {
        let defaults = OtaRuntimeState::default();
        assert_eq!(defaults.ota_rcv_last_idx, 0xffff);
        assert_eq!(defaults.slave_ota_data_cache_idx, 0);
        assert_eq!(defaults.rf_slave_ota_finished_time, 0);
        assert_eq!(defaults.terminate_cnt, 0);

        {
            let mut st = OTA_STATE.lock();
            st.ota_rcv_last_idx = 7;
            st.slave_ota_data_cache_idx = 3;
            st.rf_slave_ota_finished_time = 42;
            st.terminate_cnt = 2;
        }

        reset_for_test();

        let st = OTA_STATE.lock();
        assert_eq!(st.ota_rcv_last_idx, 0xffff);
        assert_eq!(st.slave_ota_data_cache_idx, 0);
        assert_eq!(st.rf_slave_ota_finished_time, 0);
        assert_eq!(st.terminate_cnt, 0);
    }

    /// Staging one mesh packet drives the real flash path: the payload is
    /// programmed into the OTA staging area and the write cursor only advances
    /// after the driver's read-back verification matches.
    ///
    /// This test cannot observe lock re-entrancy: the driver owns its fake
    /// device, so nothing here can take `OTA_STATE` on the flash side. That
    /// invariant is documented at the production call site instead — see the
    /// "Critical-section invariant" note on `rf_link_slave_data_ota_save`.
    #[test]
    fn test_ota_save_programs_and_verifies_staged_packet() {
        reset_fakes();
        reset_for_test();
        *RF_SLAVE_OTA_FINISHED_FLAG.lock() = OtaState::Continue;
        // Mesh OTA mode: skips the CRC check and uses 8-byte packets.
        OTA_UPDATE_MESH_OPERATIONS_BLOCKED.set(true);
        OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);

        let mut staged = PacketAttData::default();
        staged.head.l2cap_len = 7 + 8;
        staged.dat[0] = 0; // cur_idx low byte: first data packet
        staged.dat[1] = 0; // cur_idx high byte
        staged.dat[2..10].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        BUFF_RESPONSE.lock()[0] = Packet { att_data: staged };
        OTA_STATE.lock().slave_ota_data_cache_idx = 1;

        fake_clock::set_ticks(1234);

        rf_link_slave_data_ota_save();

        // The 8-byte payload is programmed at the start of the staging area.
        let mut staged_bytes = [0u8; 8];
        FakeFlash::read_slice(FLASH_ADR_LIGHT_NEW_FW, &mut staged_bytes);
        assert_eq!(staged_bytes, [1, 2, 3, 4, 5, 6, 7, 8]);

        // `rf_ota_save_data` advances the cursor only when its read-back
        // verification matches, so address 8 proves write and verify both ran.
        assert_eq!(OTA_UPDATE_CURRENT_FLASH_ADDRESS.get(), 8);

        let st = OTA_STATE.lock();
        assert_eq!(
            st.slave_ota_data_cache_idx, 0,
            "the staging window is drained"
        );
        assert_eq!(
            st.ota_rcv_last_idx, 0,
            "the first data packet leaves the cursor at index 0"
        );
        // The write read back byte-exact, so verification succeeded and the
        // transfer stays in progress (no finish time, flag still `Continue`).
        assert_eq!(
            st.rf_slave_ota_finished_time, 0,
            "a verified mid-transfer packet must not finish the transfer"
        );
        assert_eq!(*RF_SLAVE_OTA_FINISHED_FLAG.lock(), OtaState::Continue);

        OTA_UPDATE_MESH_OPERATIONS_BLOCKED.set(false);
        OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);
    }

    /// The failure indicator must genuinely alternate: `gpio_write` drives the pin
    /// high for any non-zero value, so the old `!`-on-`u32` toggle (`0xFFFF_FFFE`)
    /// held the pin high for the whole 6 s and looked identical to success.
    #[test]
    #[mry::lock(
        read_reg_gpio_oen,
        write_reg_gpio_oen,
        read_reg_gpio_out,
        write_reg_gpio_out
    )]
    fn test_rf_led_ota_error_alternates_pin_sixty_times() {
        // `OTA_LED` is port C, pin 0: the pin id's low byte is the bit mask and
        // the port number is the register offset, so (pin >> 8) << 3 == 0x10.
        const PIN_MASK: u8 = 0x01;
        const REG_OFFSET: u32 = 0x10;

        reset_fakes();
        reset_for_test();

        mock_read_reg_gpio_oen(Any).returns(0);
        mock_write_reg_gpio_oen(Any, Any).returns(());
        mock_read_reg_gpio_out(Any).returns(0);

        // Record every OUT register write in call order.
        let writes: Arc<Mutex<Vec<u8>>> = Arc::new(Mutex::new(Vec::new()));
        let recorded_writes = Arc::clone(&writes);
        mock_write_reg_gpio_out(Any, Any).returns_with(move |value: u8, _offset: u32| {
            recorded_writes.lock().unwrap().push(value);
        });

        rf_led_ota_error();

        // `gpio_set_func` runs for real now: it selects the GPIO function for
        // the LED pin by writing the pin mask into `reg_gpio_gpio_func`.
        let func_writes: Vec<_> = FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == (REG_GPIO_GPIO_FUNC + REG_OFFSET) | tlsr8266_pac::REG_BASE_ADDR)
            .collect();
        assert_eq!(
            func_writes.len(),
            1,
            "gpio_set_func must select the GPIO function exactly once"
        );
        assert_eq!(func_writes[0].value, u32::from(PIN_MASK));

        // Output enabled once, before the first write.
        mock_write_reg_gpio_oen(Any, Any).assert_called(1);
        // Sixty writes that alternate high, low, high, low, ...
        let expected: Vec<u8> = (0..60)
            .map(|i| if i % 2 == 0 { PIN_MASK } else { 0 })
            .collect();
        assert_eq!(
            *writes.lock().unwrap(),
            expected,
            "the OTA error LED must be driven high and low alternately, sixty times"
        );
        mock_write_reg_gpio_out(Any, REG_OFFSET).assert_called(60);
        // The real `wd_clear` pokes the CLR_WD bit of `reg_tmr_ctrl` once per
        // iteration; the write log is the observable that replaces the mock.
        let wd_writes = FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == REG_TMR_CTRL | tlsr8266_pac::REG_BASE_ADDR)
            .count();
        assert_eq!(wd_writes, 60, "the watchdog must be cleared once per blink");
        // Cadence unchanged: 60 iterations of 100 ms, recorded by `sleep_us`.
        let expected_sleeps = vec![100 * 1000u32; 60];
        assert_eq!(fake_clock::sleeps(), expected_sleeps);
    }
}
