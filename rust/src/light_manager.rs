use core::cmp::{min};
use core::mem::size_of;
use core::ptr::addr_of;
use embassy_executor::Spawner;
use crate::sdk::light::{_ll_device_status_update, LGT_CMD_LIGHT_ONOFF, LGT_CMD_SET_LIGHT, LIGHT_OFF_PARAM, LIGHT_ON_PARAM, PMW_MAX_TICK, RecoverStatus};
use embassy_time::{Duration, Instant};
use heapless::Deque;
use crate::{app};
use crate::common::REGA_LIGHT_OFF;
use crate::config::{FLASH_SECTOR_SIZE, get_flash_adr_lum, MAX_LUM_BRIGHTNESS_VALUE, PWMID_B, PWMID_G};
use crate::embassy::yield_now::yield_now;
use crate::mesh::MESH_NODE_ST_PAR_LEN;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_write_page};
use crate::sdk::drivers::pwm::pwm_set_cmp;
use crate::sdk::mcu::analog::{analog_read, analog_write};
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed};
use crate::sdk::mcu::register::{FLD_TMR, read_reg_tmr_ctrl, write_reg_tmr1_tick, write_reg_tmr_ctrl};
use fixed::types::{I16F16, U16F16};
use const_format::formatcp;

const TRANSITION_TIME_MS: u64 = 1500;
const LIGHT_SAVE_VALID_FLAG: u8 = 0xA5;

#[derive(Copy, Clone, Debug)]
struct Message {
    cmd: u8,
    params: [u8; 16]
}

#[derive(Copy, Clone, Debug)]
pub struct LightState {
    pub cw: I16F16,
    pub ww: I16F16,
    pub brightness: I16F16,
    timestamp: Instant
}

#[repr(C, align(4))]
struct LumSaveT {
    save_flag: u8,
    brightness: u16,
    cw: u16,
    ww: u16
}

impl LightState {
    pub const fn default() -> Self {
        Self {
            cw: I16F16::lit(formatcp!("{}", MAX_LUM_BRIGHTNESS_VALUE)),
            ww: I16F16::lit(formatcp!("{}", 0u16)),
            brightness: I16F16::lit(formatcp!("{}", 0u16)),
            timestamp: Instant::from_ticks(0)
        }
    }
}

pub struct LightManager {
    channel: Deque<Message, 5>,

    old_light_state: LightState,
    new_light_state: LightState,
    current_light_state: LightState,

    light_lum_addr: u32,
    last_transition_time: u32,

    // This brightness is separate to the current light state brightness since it stores the brightness if the light is off
    brightness: u16
}

impl LightManager {
    pub const fn default() -> Self {
        Self {
            channel: Deque::new(),
            old_light_state: LightState::default(),
            new_light_state: LightState::default(),
            current_light_state: LightState::default(),
            light_lum_addr: 0,
            last_transition_time: 0,
            brightness: MAX_LUM_BRIGHTNESS_VALUE
        }
    }

    fn handle_on_off(&mut self, on: u8) {
        self.light_onoff(match on {
            LIGHT_ON_PARAM => true,
            LIGHT_OFF_PARAM => false,
            _ => true
        });
    }

    fn handle_transition(&mut self, params: &[u8; 16]) {
        let mut brightness = self.brightness;
        let mut cw = self.current_light_state.cw.to_num();
        let mut ww = self.current_light_state.ww.to_num();

        if params[8] & 0x1 != 0 {
            // Brightness
            brightness = (params[1] as u16) << 8 | params[0] as u16;

            // If we're really changing the brightness, then we should save it soon
            if self.brightness != brightness {
                self.last_transition_time = clock_time();
            }

            self.brightness = brightness
        }

        if params[8] & 0x2 != 0 {
            // Temperature
            let value = (params[3] as u16) << 8 | params[2] as u16;

            cw = MAX_LUM_BRIGHTNESS_VALUE - value;
            ww = value;
        }

        if params[8] & 0x4 != 0 {
            // Temperature (independent CW/WW)
            cw = (params[3] as u16) << 8 | params[2] as u16;
            ww = (params[5] as u16) << 8 | params[4] as u16;
        }

        if self.is_light_off() {
            self.begin_transition(cw, ww, 0);
        } else {
            self.begin_transition(cw, ww, brightness);
        }
    }

    pub fn send_message(&mut self, cmd: u8, params: [u8; 16]) {
        if !self.channel.is_full() {
            critical_section::with(|_| {
                self.channel.push_back(Message { cmd, params }).unwrap();
            });
        }
    }

    pub async fn run(&mut self, _spawner: Spawner) {
        loop {
            while self.channel.is_empty() {
                yield_now().await;
            }

            let msg = critical_section::with(|_| {
                self.channel.pop_front().unwrap()
            });

            match msg.cmd {
                LGT_CMD_LIGHT_ONOFF => self.handle_on_off(msg.params[0]),
                LGT_CMD_SET_LIGHT => self.handle_transition(&msg.params),
                _ => {}
            }
        }
    }

    pub fn begin_transition(&mut self, cw: u16, ww: u16, brightness: u16) {
        critical_section::with(|_| {
            // Check if the cw or ww are changing and update the transition time so we save in a bit
            if self.current_light_state.cw != cw || self.current_light_state.ww != ww {
                self.last_transition_time = clock_time();
            }

            self.old_light_state = self.current_light_state;
            self.old_light_state.timestamp = Instant::now();

            self.new_light_state = LightState {
                cw: I16F16::from_num(cw),
                ww: I16F16::from_num(ww),
                brightness: I16F16::from_num(brightness),
                timestamp: Instant::now() + Duration::from_millis(TRANSITION_TIME_MS)
            };

            // Enable timer1
            write_reg_tmr1_tick(0);
            write_reg_tmr_ctrl(read_reg_tmr_ctrl() | FLD_TMR::TMR1_EN as u32);

            // Run a single transition now to avoid anything bugging out
            self.transition_step();
        });
    }

    pub fn ease_in_out(&self, t: I16F16, b: I16F16, c: I16F16) -> I16F16 {
        static TWO: I16F16 = I16F16::lit(formatcp!("{}", 2u16));
        static D: I16F16 = I16F16::lit(formatcp!("{}", MAX_LUM_BRIGHTNESS_VALUE));

        let t = t / (D / TWO);
        if t < 1 {
            c / TWO * (t * t * t) + b
        }
        else {
            let t = t - TWO;
            c / TWO * (t * t * t + TWO) + b
        }
    }

    pub fn transition_step(&mut self) {
        self.current_light_state.timestamp = min(Instant::now(), self.new_light_state.timestamp);

        if self.current_light_state.timestamp == self.new_light_state.timestamp {
            // Disable timer1
            write_reg_tmr_ctrl(read_reg_tmr_ctrl() & !(FLD_TMR::TMR1_EN as u32));

            // Save a computation
            self.current_light_state.cw = self.new_light_state.cw;
            self.current_light_state.ww = self.new_light_state.ww;
            self.current_light_state.brightness = self.new_light_state.brightness;

            // Make sure we do a final light update
            self.light_adjust_rgb_hw(
                self.current_light_state.cw,
                self.current_light_state.ww,
                self.current_light_state.brightness
            );

            // Nothing more to do
            return;
        }

        // We're still transitioning. Run the calculations
        let time = I16F16::from_num((self.current_light_state.timestamp - self.old_light_state.timestamp).as_ticks() * MAX_LUM_BRIGHTNESS_VALUE as u64 / (self.new_light_state.timestamp - self.old_light_state.timestamp).as_ticks());
        self.current_light_state.cw =
            self.ease_in_out(
                time,
                self.old_light_state.cw,
                self.new_light_state.cw - self.old_light_state.cw
            );

        self.current_light_state.ww =
            self.ease_in_out(
                time,
                self.old_light_state.ww,
                self.new_light_state.ww - self.old_light_state.ww
            );

        self.current_light_state.brightness =
            self.ease_in_out(
                time,
                self.old_light_state.brightness,
                self.new_light_state.brightness - self.old_light_state.brightness
            );

        self.light_adjust_rgb_hw(
            self.current_light_state.cw,
            self.current_light_state.ww,
            self.current_light_state.brightness
        );
    }

    pub fn is_light_off(&self) -> bool {
        if self.new_light_state.timestamp > self.current_light_state.timestamp {
            return self.new_light_state.brightness == 0;
        }
        self.current_light_state.brightness == 0
    }

    //erase flash
    fn light_lum_erase(&mut self) {
        self.light_lum_addr = *get_flash_adr_lum();
        flash_erase_sector(*get_flash_adr_lum());
        self.light_state_save();
    }

    //save cur lum value, if disconnected for a while
    pub fn light_state_save(&mut self) {
        if self.light_lum_addr >= (*get_flash_adr_lum() + FLASH_SECTOR_SIZE as u32 - size_of::<LumSaveT>() as u32)
        {
            self.light_lum_erase();
            return;
        }

        let lum_save = LumSaveT {
            save_flag: LIGHT_SAVE_VALID_FLAG,
            brightness: self.brightness,
            cw: self.current_light_state.cw.to_num(),
            ww: self.current_light_state.ww.to_num()
        };

        flash_write_page(
            self.light_lum_addr,
            size_of::<LumSaveT>() as u32,
            addr_of!(lum_save) as *const u8,
        );

        self.light_lum_addr += size_of::<LumSaveT>() as u32;
    }

    //retrieve LUM : brightness or RGB/CT value
    pub fn light_lum_retrieve(&mut self) {
        let mut i = 0;
        while i < FLASH_SECTOR_SIZE {
            self.light_lum_addr = *get_flash_adr_lum() + i as u32;

            let lum_save = unsafe { &*(self.light_lum_addr as *const LumSaveT) };
            match lum_save.save_flag {
                LIGHT_SAVE_VALID_FLAG => {
                    self.brightness = min(lum_save.brightness, MAX_LUM_BRIGHTNESS_VALUE);
                    self.current_light_state.cw = I16F16::from_num(min(lum_save.cw, MAX_LUM_BRIGHTNESS_VALUE));
                    self.current_light_state.ww = I16F16::from_num(min(lum_save.ww, MAX_LUM_BRIGHTNESS_VALUE));
                },
                0xFF => {
                    break;
                },
                _ => ()
            }

            i += size_of::<LumSaveT>() as u16
        }

        app().ota_manager.mesh_ota_master_100_flag_check();

        let val = analog_read(REGA_LIGHT_OFF);
        if val & RecoverStatus::LightOff as u8 != 0 {
            analog_write(REGA_LIGHT_OFF, val & !(RecoverStatus::LightOff as u8));
            self.light_onoff(false);
        } else {
            self.light_onoff(true);
        }
    }

    pub fn check_light_state_save(&mut self) {
        // Save the light state if it's been more than 5 seconds since the last transition
        if self.last_transition_time != 0 && clock_time_exceed(self.last_transition_time, 5000 * 1000) {
            self.last_transition_time = 0;
            self.light_state_save();
        }
    }

    pub fn get_current_light_state(&mut self) -> &mut LightState {
        if self.new_light_state.timestamp > self.current_light_state.timestamp {
            return &mut self.new_light_state;
        }

        &mut self.current_light_state
    }

    pub fn calculate_lumen_map(&self, val: I16F16) -> u32 {
        static COEFF1: U16F16 = U16F16::lit("5.2221");
        static COEFF2: U16F16 = U16F16::lit("130.5908");

        let percentage = U16F16::from_num(val) / MAX_LUM_BRIGHTNESS_VALUE as u32 * 100;
        ((COEFF1 * (percentage * percentage)) + COEFF2 * percentage).to_num()
    }

    fn pwm_set_lum(&self, id: u32, y: u16, pol: bool) {
        let lum = (y as u32 * PMW_MAX_TICK as u32) / (255*256) as u32;

        pwm_set_cmp(id, if pol { PMW_MAX_TICK as u32 - lum } else { lum } as u16);
    }

    fn get_pwm_cmp(&self, val: I16F16, lum: I16F16) -> u16 {
        let val_lumen_map = self.calculate_lumen_map(lum);

        return ((val.to_num::<u32>() * val_lumen_map) / MAX_LUM_BRIGHTNESS_VALUE as u32) as u16;
    }

    pub fn light_adjust_cw(&self, val: I16F16, lum: I16F16) {
        self.pwm_set_lum(PWMID_G, self.get_pwm_cmp(val, lum), false);
    }

    pub fn light_adjust_ww(&self, val: I16F16, lum: I16F16) {
        self.pwm_set_lum(PWMID_B, self.get_pwm_cmp(val, lum), true);
    }

    pub fn light_adjust_rgb_hw(&self, val_cw: I16F16, val_ww: I16F16, lum: I16F16) {
        self.light_adjust_cw(val_cw, lum);
        self.light_adjust_ww(val_ww, lum);
    }

    pub fn light_onoff_hw(&mut self, on: bool) {
        let state = self.current_light_state;
        self.begin_transition(state.cw.to_num(), state.ww.to_num(), match on { true => self.brightness, false => 0 });
    }

    pub fn light_onoff(&mut self, on: bool) {
        self.light_onoff_hw(on);
        self.device_status_update();
    }

    pub fn device_status_update(&self) {
        // packet
        let mut st_val_par: [u8; MESH_NODE_ST_PAR_LEN as usize] = [0xff; MESH_NODE_ST_PAR_LEN as usize];

        let on = if self.is_light_off() {
            0
        } else {
            1
        };

        st_val_par[0] = on;
        st_val_par[1] = 0xff;

        _ll_device_status_update(st_val_par.as_ptr(), st_val_par.len() as u8);
    }
}