use crate::config::{FLASH_ADR_LIGHT_NEW_FW, OTA_LED};
use crate::sdk::common::bit::ONES_32;
use crate::sdk::common::crc::crc16;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page, PAGE_SIZE};
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed, sleep_us};
use crate::sdk::mcu::gpio::{gpio_set_func, gpio_set_output_en, gpio_write, AS_GPIO};
use crate::sdk::mcu::irq_i::irq_disable;
use crate::sdk::mcu::register::{
    write_reg_clk_en1, write_reg_pwdn_ctrl, write_reg_rst1, write_reg_system_tick_ctrl,
};
use crate::sdk::mcu::watchdog::wd_clear;
use core::mem::{size_of_val, MaybeUninit};
use core::ptr::addr_of;
use crate::app;
use crate::main_light::get_buff_response;
use crate::sdk::ble_app::light_ll::{is_add_packet_buf_ready, rf_link_add_tx_packet, rf_link_slave_read_status_stop, rf_ota_save_data};
use crate::sdk::light::*;
use crate::sdk::pm::light_sw_reboot;

pub struct OtaManager {
    ota_pkt_cnt: u16,
    ota_rcv_last_idx: u16,
    fw_check_val: u32,
    need_check_type: u8,
    //=1:crc val sum
    ota_pkt_total: u16,
    slave_ota_data_cache_idx: u8,
    pub rf_slave_ota_finished_time: u32,
    terminate_cnt: u8,
}

impl OtaManager {
    pub const FLASH_ADR_OTA_READY_FLAG: u32 = 0x3F000;
    pub const FLASH_OTA_READY_FLAG: u8 = 0xa5;
    pub const FW_SIZE_MAX_K: u32 = 128;
    pub const ERASE_SECTORS_FOR_OTA: u32 = (OtaManager::FW_SIZE_MAX_K + 3) / 4;

    pub const fn default() -> OtaManager {
        OtaManager {
            ota_pkt_cnt: 0,
            ota_rcv_last_idx: 0xffff,
            fw_check_val: 0,
            need_check_type: 0, //=1:crc val sum
            ota_pkt_total: 0,
            slave_ota_data_cache_idx: 0,
            rf_slave_ota_finished_time: 0,
            terminate_cnt: 0,
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

        // First configure the system clock
        write_reg_rst1(0);
        write_reg_clk_en1(0xff);
        write_reg_system_tick_ctrl(0x01);
        irq_disable();

        #[allow(invalid_value)]
            let mut buff: [u8; 256] = unsafe { MaybeUninit::uninit().assume_init() };

        flash_read_page(FLASH_ADR_LIGHT_NEW_FW, PAGE_SIZE, buff.as_mut_ptr());
        let n = unsafe { *(buff.as_ptr().offset(0x18) as *const u32) };
        let mut i = 0;
        while i < n {
            if (i & 0xfff) == 0 {
                flash_erase_sector(i);
            }

            flash_read_page(
                FLASH_ADR_LIGHT_NEW_FW + i,
                PAGE_SIZE,
                buff.as_mut_ptr(),
            );
            flash_write_page(i, PAGE_SIZE, buff.as_mut_ptr());

            i += PAGE_SIZE;
        }

        buff[0] = 0;

        flash_write_page(OtaManager::FLASH_ADR_OTA_READY_FLAG, 1, buff.as_mut_ptr()); //clear OTA flag

        // Force reboot
        write_reg_pwdn_ctrl(0x20);

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
        return true;
    }

    pub fn erase_ota_data(&self) {
        for i in 0..OtaManager::ERASE_SECTORS_FOR_OTA {
            flash_erase_sector(
                FLASH_ADR_LIGHT_NEW_FW
                    + (OtaManager::ERASE_SECTORS_FOR_OTA - 1 - i) * 0x1000,
            );
        }
    }

    pub fn rf_link_slave_data_ota(&mut self, data: &rf_packet_att_data_t) {
        if *get_rf_slave_ota_finished_flag() != OtaState::CONTINUE {
            return;
        }

        if !*get_rf_slave_ota_busy() {
            if !*get_pair_login_ok()
            {
                return;
            }

            set_rf_slave_ota_busy(true);
            if *get_slave_read_status_busy() != 0 {
                rf_link_slave_read_status_stop();
            }
        }

        get_buff_response()[(self.slave_ota_data_cache_idx % 16) as usize] = *data;

        self.slave_ota_data_cache_idx += 1;
    }

    pub fn rf_link_slave_data_ota_save(&mut self) -> bool {
        let mut reset_flag = OtaState::CONTINUE;
        for i in 0..self.slave_ota_data_cache_idx {
            let p = &get_buff_response()[i as usize];
            let n_data_len = p.l2cap - 7;

            if crc16(&p.dat[0..(n_data_len + 2) as usize])
                == p.dat[(n_data_len + 2) as usize] as u16
                | (p.dat[(n_data_len + 3) as usize] as u16) << 8
            {
                set_rf_slave_ota_timeout_s(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS); // refresh timeout

                let cur_idx = p.dat[0] as u16 | (p.dat[1] as u16) << 8;
                if n_data_len == 0 {
                    if (cur_idx == self.ota_rcv_last_idx + 1) && (cur_idx == self.ota_pkt_total) {
                        // ota ok, save, reboot
                        reset_flag = OtaState::OK;
                    } else {
                        // ota err
                        set_cur_ota_flash_addr(0);
                        self.ota_pkt_cnt = 0;
                        self.ota_rcv_last_idx = 0;
                        reset_flag = OtaState::ERROR;
                    }
                } else {
                    if cur_idx == 0 {
                        // start ota
                        if *get_cur_ota_flash_addr() != 0 {
                            // 0x10000 should be 0x00
                            set_cur_ota_flash_addr(0);
                            self.ota_pkt_cnt = 0;
                            self.ota_rcv_last_idx = 0;
                            reset_flag = OtaState::ERROR;
                        } else {
                            self.need_check_type = get_ota_check_type(&p.dat[8]);
                            if self.need_check_type == 1 {
                                self.fw_check_val = (p.dat[(n_data_len + 2) as usize] as u16
                                    | (p.dat[(n_data_len + 3) as usize] as u16) << 8)
                                    as u32;
                            }
                            reset_flag = rf_ota_save_data(p.dat[2..].as_ptr());
                        }
                    } else if cur_idx == self.ota_rcv_last_idx + 1 {
                        // ota fw check
                        if cur_idx == 1 {
                            self.ota_pkt_total = ((((p.dat[10] as u32)
                                | (((p.dat[11] as u32) << 8) & 0xFF00)
                                | (((p.dat[12] as u32) << 16) & 0xFF0000)
                                | (((p.dat[13] as u32) << 24) & 0xFF000000))
                                + 15)
                                / 16) as u16;
                            if self.ota_pkt_total < 3 {
                                // invalid fw
                                set_cur_ota_flash_addr(0);
                                self.ota_pkt_cnt = 0;
                                self.ota_rcv_last_idx = 0;
                                reset_flag = OtaState::ERROR;
                            } else if self.need_check_type == 1 {
                                self.fw_check_val += (p.dat[(n_data_len + 2) as usize] as u16
                                    | (p.dat[(n_data_len + 3) as usize] as u16) << 8)
                                    as u32;
                            }
                        } else if cur_idx < self.ota_pkt_total - 1 && self.need_check_type == 1 {
                            self.fw_check_val += (p.dat[(n_data_len + 2) as usize] as u16
                                | (p.dat[(n_data_len + 3) as usize] as u16) << 8)
                                as u32;
                        } else if cur_idx == self.ota_pkt_total - 1 && self.need_check_type == 1 {
                            if self.fw_check_val
                                != ((p.dat[2] as u32)
                                | (((p.dat[3] as u32) << 8) & 0xFF00)
                                | (((p.dat[4] as u32) << 16) & 0xFF0000)
                                | (((p.dat[5] as u32) << 24) & 0xFF000000))
                            {
                                set_cur_ota_flash_addr(0);
                                self.ota_pkt_cnt = 0;
                                self.ota_rcv_last_idx = 0;
                                reset_flag = OtaState::ERROR;
                            }
                        }

                        if reset_flag != OtaState::ERROR {
                            if *get_cur_ota_flash_addr() + 16 > (OtaManager::FW_SIZE_MAX_K * 1024) {
                                // !is_valid_fw_len()
                                reset_flag = OtaState::ERROR;
                            } else {
                                reset_flag = rf_ota_save_data(&p.dat[2]);
                            }
                        }
                    } else {
                        // error, ota failed
                        set_cur_ota_flash_addr(0);
                        self.ota_pkt_cnt = 0;
                        self.ota_rcv_last_idx = 0;
                        reset_flag = OtaState::ERROR;
                    }

                    self.ota_rcv_last_idx = cur_idx;
                }
            } else {
                // error, ota failed
                set_cur_ota_flash_addr(0);
                self.ota_pkt_cnt = 0;
                self.ota_rcv_last_idx = 0;
                reset_flag = OtaState::ERROR;
            }

            if reset_flag != OtaState::CONTINUE {
                if *get_rf_slave_ota_finished_flag() == OtaState::CONTINUE {
                    self.rf_slave_ota_finished_flag_set(reset_flag);
                }

                break;
            }
        }
        self.slave_ota_data_cache_idx = 0;
        return true;
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
            OtaState::ERROR => {
                self.erase_ota_data();
                self.rf_led_ota_error();
            }
            OtaState::OK => {
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
        set_rf_slave_ota_finished_flag(reset_flag);
        self.rf_slave_ota_finished_time = clock_time();
    }

    pub(crate) fn rf_link_slave_ota_finish_handle(&mut self) // poll when ota busy in bridge
    {
        self.rf_link_slave_data_ota_save();

        let pkt_terminate: [u8; 8] = [0x04, 0x00, 0x00, 0x00, 0x03, 0x02, 0x02, 0x13];

        if *get_rf_slave_ota_finished_flag() != OtaState::CONTINUE {
            let mut reboot_flag = false;
            if 0 == self.terminate_cnt && *get_rf_slave_ota_terminate_flag() {
                if is_add_packet_buf_ready() {
                    self.terminate_cnt = 6;
                    rf_link_add_tx_packet(pkt_terminate.as_ptr() as *const rf_packet_att_cmd_t, pkt_terminate.len());
                }
            }

            if self.terminate_cnt != 0 {
                self.terminate_cnt -= 1;
                if self.terminate_cnt == 0 {
                    reboot_flag = true;
                }
            }

            if !*get_rf_slave_ota_terminate_flag()
                && clock_time_exceed(self.rf_slave_ota_finished_time, 2000 * 1000)
            {
                set_rf_slave_ota_terminate_flag(true); // for ios: no last read command
            }

            if clock_time_exceed(self.rf_slave_ota_finished_time, 4000 * 1000) {
                reboot_flag = true;
            }

            if reboot_flag {
                self.rf_link_slave_ota_finish_led_and_reboot(*get_rf_slave_ota_finished_flag());
                // have been rebooted
            }
        }
    }

    pub fn mesh_ota_led_cb(&self, _type: MeshOtaLed) {
        match _type {
            MeshOtaLed::OK => self.rf_led_ota_ok(),
            MeshOtaLed::ERROR => self.rf_led_ota_error(),
            MeshOtaLed::STOP => self.rf_led_ota_error()
        }
    }

    pub fn get_ota_erase_sectors(&self) -> u32 {
        return OtaManager::ERASE_SECTORS_FOR_OTA;
    }

    pub fn is_valid_fw_len(&self, fw_len: u32) -> bool {
        return fw_len <= (OtaManager::FW_SIZE_MAX_K * 1024);
    }
}

pub fn get_ota_check_type(par: *const u8) -> u8 {
    unsafe {
        if *par.offset(0) == 0x5D {
            return *par.offset(1);
        }
    }
    return 0;
}

pub fn rf_link_slave_data_ota(data: *const rf_packet_att_write_t) -> bool {
    app().ota_manager.rf_link_slave_data_ota(unsafe { &*(data as *const rf_packet_att_data_t) });

    return true;
}
