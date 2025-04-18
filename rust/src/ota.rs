use core::mem::size_of_val;
use core::ptr::addr_of;
use core::sync::atomic::{AtomicU16, Ordering};

use crate::app;
use crate::config::{FLASH_ADR_LIGHT_NEW_FW, FLASH_SECTOR_SIZE, OTA_LED};
use crate::sdk::ble_app::light_ll::{is_add_packet_buf_ready, rf_link_add_tx_packet, rf_link_slave_read_status_stop, rf_ota_save_data};
use crate::sdk::common::bit::ONES_32;
use crate::sdk::common::compat::array4_to_int;
use crate::sdk::common::crc::crc16;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page, PAGE_SIZE};
use crate::sdk::light::*;
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed, sleep_us};
use crate::sdk::mcu::gpio::{AS_GPIO, gpio_set_func, gpio_set_output_en, gpio_write};
use crate::sdk::mcu::register::{FldPwdnCtrl, write_reg_clk_en1, write_reg_pwdn_ctrl, write_reg_rst1, write_reg_system_tick_ctrl};
use crate::sdk::mcu::watchdog::wd_clear;
use crate::sdk::packet_types::{Packet, PacketAttData};
use crate::sdk::pm::light_sw_reboot;
use crate::state::{*};
use crate::sdk::mcu::irq_i::irq_disable;

#[cfg_attr(test, mry::mry)]
pub struct OtaManager {
    ota_rcv_last_idx: u16,
    slave_ota_data_cache_idx: usize,
    pub rf_slave_ota_finished_time: u32,
    terminate_cnt: u8,
}

#[cfg_attr(test, mry::mry(skip_fns(default_const)))]
impl OtaManager {
    pub const FLASH_ADR_OTA_READY_FLAG: u32 = 0x3F000;
    pub const FLASH_OTA_READY_FLAG: u8 = 0xa5;
    pub const FW_SIZE_MAX_K: u32 = 128;
    pub const ERASE_SECTORS_FOR_OTA: u32 = (OtaManager::FW_SIZE_MAX_K + 3) / 4;

    #[cfg(not(test))]
    pub const fn default_const() -> OtaManager {
        OtaManager {
            ota_rcv_last_idx: 0xffff,
            slave_ota_data_cache_idx: 0,
            rf_slave_ota_finished_time: 0,
            terminate_cnt: 0,
        }
    }

    #[cfg(test)]
    pub fn default() -> OtaManager {
        OtaManager {
            ota_rcv_last_idx: 0xffff,
            slave_ota_data_cache_idx: 0,
            rf_slave_ota_finished_time: 0,
            terminate_cnt: 0,
            mry: Default::default(),
        }
    }

    #[inline(never)]
    #[link_section = ".ram_code"]
    pub fn handle_ota_update() {
        // This function requires that *everything* be in ram
        unsafe {
            if *(OtaManager::FLASH_ADR_OTA_READY_FLAG as *const u8)
                != OtaManager::FLASH_OTA_READY_FLAG
            {
                // Nothing to do
                return;
            }
        }

        // First configure the system clock. This lets delays work as expected for reading/writing
        // flash
        write_reg_rst1(0);
        write_reg_clk_en1(0xff);
        write_reg_system_tick_ctrl(0x01);
        irq_disable();

        // Read the first page of the new firmware
        let mut buff: [u8; PAGE_SIZE as usize] = [0; PAGE_SIZE as usize];
        flash_read_page(FLASH_ADR_LIGHT_NEW_FW, PAGE_SIZE, buff.as_mut_ptr());

        // Get the size of the binary being copied
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
        flash_write_page(OtaManager::FLASH_ADR_OTA_READY_FLAG, 1, buff.as_mut_ptr());

        // Force reboot
        write_reg_pwdn_ctrl(FldPwdnCtrl::Reboot.bits());

        #[allow(clippy::empty_loop)]
        loop {}
    }

    pub fn check_ota_area_startup(&self) {
        if !self.is_ota_area_valid() {
            self.erase_ota_data();
        }
    }

    pub fn is_ota_area_valid(&self) -> bool {
        let mut buf: [u8; 4] = [0; 4];
        for i in 0..OtaManager::ERASE_SECTORS_FOR_OTA {
            flash_read_page(
                FLASH_ADR_LIGHT_NEW_FW + i * 0x1000,
                4,
                buf.as_mut_ptr(),
            );
            let tmp = buf[0] as u32
                | (buf[1] as u32) << 8
                | (buf[2] as u32) << 16
                | (buf[3] as u32) << 24;
            if tmp != ONES_32 {
                return false;
            }
        }

        true
    }

    pub fn erase_ota_data(&self) {
        for i in 0..OtaManager::ERASE_SECTORS_FOR_OTA {
            flash_erase_sector(
                FLASH_ADR_LIGHT_NEW_FW
                    + (OtaManager::ERASE_SECTORS_FOR_OTA - 1 - i) * 0x1000,
            );
        }
    }

    pub fn rf_link_slave_data_ota(&mut self, data: &Packet) {
        if *RF_SLAVE_OTA_FINISHED_FLAG.lock() != OtaState::Continue || RF_SLAVE_OTA_BUSY_MESH.get() {
            return;
        }

        if !RF_SLAVE_OTA_BUSY.get() {
            if !PAIR_LOGIN_OK.get()
            {
                return;
            }

            RF_SLAVE_OTA_BUSY.set(true);
            if SLAVE_READ_STATUS_BUSY.get() != 0 {
                rf_link_slave_read_status_stop();
            }
        }

        BUFF_RESPONSE.lock()[self.slave_ota_data_cache_idx % 16] = *data;

        self.slave_ota_data_cache_idx += 1;
    }

    pub fn rf_mesh_data_ota(&mut self, pkt_data: &[u8], last: bool) -> u16 {
        if *RF_SLAVE_OTA_FINISHED_FLAG.lock() != OtaState::Continue || RF_SLAVE_OTA_BUSY.get() || !RF_SLAVE_OTA_BUSY_MESH.get() {
            return u16::MAX;
        }

        let mut data: PacketAttData = PacketAttData::default();
        data.head.l2cap_len = if last { 7 } else { 10 + 7 };
        data.dat[0..pkt_data.len()].copy_from_slice(pkt_data);

        let index = data.dat[0] as u16 | ((data.dat[1] as u16) << 8);

        static LAST_INDEX: AtomicU16 = AtomicU16::new(u16::MAX);
        if LAST_INDEX.load(Ordering::Relaxed) == u16::MAX || LAST_INDEX.load(Ordering::Relaxed) < index {
            LAST_INDEX.store(index, Ordering::Relaxed);

            BUFF_RESPONSE.lock()[0] = Packet { att_data: data };
            self.slave_ota_data_cache_idx = 1;

            self.rf_link_slave_data_ota_save();

            RF_SLAVE_OTA_TIMEOUT_S.set(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS); // refresh timeout

            if *RF_SLAVE_OTA_FINISHED_FLAG.lock() != OtaState::Continue {
                RF_SLAVE_OTA_TIMEOUT_S.set(4);
            }
        }

        LAST_INDEX.load(Ordering::Relaxed)
    }

    pub fn rf_link_slave_data_ota_save(&mut self) -> bool {
        let packet_len = if RF_SLAVE_OTA_BUSY_MESH.get() { 8 } else { 16 };

        let mut reset_flag = OtaState::Continue;
        for i in 0..self.slave_ota_data_cache_idx {
            let p = BUFF_RESPONSE.lock()[i];
            let n_data_len = (p.head().l2cap_len - 7) as usize;

            if RF_SLAVE_OTA_BUSY_MESH.get() || crc16(&p.att_data().dat[0..n_data_len + 2]) == p.att_data().dat[n_data_len + 2] as u16 | (p.att_data().dat[n_data_len + 3] as u16) << 8
            {
                RF_SLAVE_OTA_TIMEOUT_S.set(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS); // refresh timeout

                let cur_idx = p.att_data().dat[0] as u16 | (p.att_data().dat[1] as u16) << 8;
                if n_data_len == 0 {
                    let mut ota_pkt_total = [0u8; 4];
                    flash_read_page(FLASH_ADR_LIGHT_NEW_FW + 0x18, 4, ota_pkt_total.as_mut_ptr());
                    let ota_pkt_total = ((array4_to_int(&ota_pkt_total) + (packet_len - 1)) / packet_len) as u16;
                    if ota_pkt_total < 3 {
                        // invalid fw
                        CUR_OTA_FLASH_ADDR.set(0);
                        self.ota_rcv_last_idx = 0;
                        reset_flag = OtaState::Error;
                    } else if cur_idx == self.ota_rcv_last_idx + 1 && cur_idx == ota_pkt_total {
                        // ota ok, save, reboot
                        reset_flag = OtaState::Ok;
                    } else {
                        // ota err
                        CUR_OTA_FLASH_ADDR.set(0);
                        self.ota_rcv_last_idx = 0;
                        reset_flag = OtaState::Error;
                    }
                } else {
                    if cur_idx == 0 {
                        // start ota
                        if CUR_OTA_FLASH_ADDR.get() != 0 {
                            // 0x10000 should be 0x00
                            CUR_OTA_FLASH_ADDR.set(0);
                            self.ota_rcv_last_idx = 0;
                            reset_flag = OtaState::Error;
                        } else {
                            reset_flag = rf_ota_save_data(&p.att_data().dat[2..2 + packet_len as usize]);
                        }
                    } else if cur_idx == self.ota_rcv_last_idx + 1 {
                        if reset_flag != OtaState::Error {
                            if CUR_OTA_FLASH_ADDR.get() + packet_len > (OtaManager::FW_SIZE_MAX_K * 1024) {
                                // !is_valid_fw_len()
                                reset_flag = OtaState::Error;
                            } else {
                                reset_flag = rf_ota_save_data(&p.att_data().dat[2..2 + packet_len as usize]);
                            }
                        }
                    } else {
                        // error, ota failed
                        CUR_OTA_FLASH_ADDR.set(0);
                        self.ota_rcv_last_idx = 0;
                        reset_flag = OtaState::Error;
                    }

                    self.ota_rcv_last_idx = cur_idx;
                }
            } else {
                // error, ota failed
                CUR_OTA_FLASH_ADDR.set(0);
                self.ota_rcv_last_idx = 0;
                reset_flag = OtaState::Error;
            }

            if reset_flag != OtaState::Continue {
                if *RF_SLAVE_OTA_FINISHED_FLAG.lock() == OtaState::Continue {
                    self.rf_slave_ota_finished_flag_set(reset_flag);
                }

                break;
            }
        }
        self.slave_ota_data_cache_idx = 0;
        true
    }

    pub fn rf_ota_set_flag(&self) {
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
        flash_read_page(
            FLASH_ADR_LIGHT_NEW_FW + 8,
            1,
            addr_of!(flag0) as *mut u8,
        );
        if 0x4b != flag0 {
            flag0 = 0x4b;
            flash_write_page(
                FLASH_ADR_LIGHT_NEW_FW + 8,
                1,
                addr_of!(flag0) as *mut u8,
            ); //Set FW flag, make sure valid. because the firmware may be from 8267 by mesh ota
        }

        flash_erase_sector(OtaManager::FLASH_ADR_OTA_READY_FLAG);
        let flag: u32 = OtaManager::FLASH_OTA_READY_FLAG as u32;
        flash_write_page(
            OtaManager::FLASH_ADR_OTA_READY_FLAG,
            4,
            addr_of!(flag) as *mut u8,
        );
    }

    pub fn rf_led_ota_ok(&self) {
        gpio_set_func(OTA_LED as u32, AS_GPIO);
        gpio_set_output_en(OTA_LED as u32, 1);
        let mut led_onoff = 1;
        for _ in 0..6 {
            gpio_write(OTA_LED as u32, led_onoff);
            led_onoff = !led_onoff;
            wd_clear();
            sleep_us(1000 * 1000);
        }
    }

    pub fn rf_led_ota_error(&self) {
        gpio_set_func(OTA_LED as u32, AS_GPIO);
        gpio_set_output_en(OTA_LED as u32, 1);
        let mut led_onoff = 1;
        for _ in 0..60 {
            gpio_write(OTA_LED as u32, led_onoff);
            led_onoff = !led_onoff;
            wd_clear();
            sleep_us(100 * 1000);
        }
    }

    pub fn rf_link_slave_ota_finish_led_and_reboot(&self, st: OtaState) {
        match st {
            OtaState::Error => {
                self.erase_ota_data();
                self.rf_led_ota_error();
            }
            OtaState::Ok => {
                //rf_ota_save_data(0);
                self.rf_ota_set_flag();
                self.rf_led_ota_ok();
            }
            _ => ()
        }
        irq_disable();
        light_sw_reboot();
    }

    fn rf_slave_ota_finished_flag_set(&mut self, reset_flag: OtaState) {
        *RF_SLAVE_OTA_FINISHED_FLAG.lock() = reset_flag;
        self.rf_slave_ota_finished_time = clock_time();
    }

    pub fn rf_link_slave_ota_finish_handle(&mut self) // poll when ota busy in bridge
    {
        self.rf_link_slave_data_ota_save();

        if *RF_SLAVE_OTA_FINISHED_FLAG.lock() != OtaState::Continue {
            let mut reboot_flag = false;
            if 0 == self.terminate_cnt && RF_SLAVE_OTA_TERMINATE_FLAG.get() && is_add_packet_buf_ready() {
                self.terminate_cnt = 6;
                rf_link_add_tx_packet(&PKT_TERMINATE);
            }

            if self.terminate_cnt != 0 {
                self.terminate_cnt -= 1;
                if self.terminate_cnt == 0 {
                    reboot_flag = true;
                }
            }

            if !RF_SLAVE_OTA_TERMINATE_FLAG.get()
                && clock_time_exceed(self.rf_slave_ota_finished_time, 2000 * 1000)
            {
                RF_SLAVE_OTA_TERMINATE_FLAG.set(true); // for ios: no last read command
            }

            if clock_time_exceed(self.rf_slave_ota_finished_time, 4000 * 1000) {
                reboot_flag = true;
            }

            if reboot_flag {
                self.rf_link_slave_ota_finish_led_and_reboot(*RF_SLAVE_OTA_FINISHED_FLAG.lock());
                // have been rebooted
            }
        }
    }

    pub fn get_ota_erase_sectors(&self) -> u32 {
        OtaManager::ERASE_SECTORS_FOR_OTA
    }

    pub fn is_valid_fw_len(&self, fw_len: u32) -> bool {
        fw_len <= (OtaManager::FW_SIZE_MAX_K * 1024)
    }
}

pub fn rf_link_slave_data_ota(data: &Packet) -> bool {
    app().ota_manager.rf_link_slave_data_ota(data);

    true
}
