use std::cmp::{max, min};
use std::mem::size_of;
use std::ptr::addr_of;
use easer::functions::{Cubic, Easing};
use embassy_executor::Spawner;
use crate::sdk::light::{_ll_device_status_update, LGT_CMD_LIGHT_ONOFF, LIGHT_OFF_PARAM, LIGHT_ON_PARAM, PMW_MAX_TICK, RecoverStatus};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant};
use crate::app;
use crate::common::REGA_LIGHT_OFF;
use crate::config::{FLASH_SECTOR_SIZE, get_flash_adr_lum, MAX_LUM_BRIGHTNESS_VALUE, PWMID_B, PWMID_G};
use crate::embassy::yield_now::yield_now;
use crate::mesh::MESH_NODE_ST_PAR_LEN;
use crate::sdk::drivers::flash::{flash_erase_sector, flash_write_page};
use crate::sdk::drivers::pwm::pwm_set_cmp;
use crate::sdk::mcu::analog::{analog_read, analog_write};
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed};

const TRANSITION_TIME_MS: u64 = 1000;
const LIGHT_SAVE_VALID_FLAG: u8 = 0xA5;

#[derive(Copy, Clone, Debug)]
struct Message {
    cmd: u8,
    params: [u8; 16]
}

#[derive(Copy, Clone, Debug)]
pub struct LightState {
    pub g: u16,
    pub b: u16,
    pub brightness: u16,
    timestamp: Instant
}

#[repr(C, packed)]
struct LumSaveT {
    save_flag: u8,
    brightness: u16,
    g: u16,
    b: u16
}

impl LightState {
    pub const fn default() -> Self {
        Self {
            g: u16::MAX,
            b: 0,
            brightness: 0,
            timestamp: Instant::from_ticks(0)
        }
    }
}

pub struct LightManager {
    channel: Channel::<NoopRawMutex, Message, 5>,
    transition_signal: Channel::<NoopRawMutex, bool, 1>,

    old_light_state: LightState,
    new_light_state: LightState,
    current_light_state: LightState,

    light_lum_addr: u32,
    last_transition_time: u32,

    // This brightness is separate to the current light state brightness since it stores the brightness if the light is off
    brightness: u16
}

#[embassy_executor::task]
async fn start_transitioner() {
    app().light_manager.run_transitioner().await;
}

impl LightManager {
    pub const fn default() -> Self {
        Self {
            channel: Channel::<NoopRawMutex, Message, 5>::new(),
            transition_signal: Channel::<NoopRawMutex, bool, 1>::new(),
            old_light_state: LightState::default(),
            new_light_state: LightState::default(),
            current_light_state: LightState::default(),
            light_lum_addr: 0,
            last_transition_time: 0,
            brightness: u16::MAX
        }
    }

    fn handle_on_off(&mut self, on: u8) {
        self.light_onoff(match on {
            LIGHT_ON_PARAM => true,
            LIGHT_OFF_PARAM => false,
            _ => true
        });
    }

    pub fn send_message(&self, cmd: u8, params: [u8; 16]) {
        self.channel.try_send(Message{ cmd, params }).unwrap();
    }

    pub async fn run(&mut self, spawner: Spawner) {
        // Start the transitioner
        spawner.spawn(start_transitioner()).unwrap();

        loop {
            let msg = self.channel.recv().await;

            match msg.cmd {
                LGT_CMD_LIGHT_ONOFF => {
                    self.handle_on_off(msg.params[0])
                }
                _ => {}
            }
        }
    }

    pub fn begin_transition(&mut self, g: u16, b: u16, brightness: u16) {
        self.old_light_state = self.current_light_state;
        self.old_light_state.timestamp = Instant::now();

        self.new_light_state = LightState {
            g, b, brightness,
            timestamp: Instant::now() + Duration::from_millis(TRANSITION_TIME_MS)
        };

        self.last_transition_time = clock_time();

        // Ignore any errors on this send
        let _ = self.transition_signal.try_send(true);
    }

    pub async fn run_transitioner(&mut self) {
        loop {
            self.transition_signal.recv().await;

            let mut now = Instant::now();
            let mut last = false;

            while now < self.new_light_state.timestamp || last {
                let time = (now - self.old_light_state.timestamp).as_ticks() as f32 / (self.new_light_state.timestamp - self.old_light_state.timestamp).as_ticks() as f32;
                self.current_light_state = LightState {
                    g: Cubic::ease_in_out(time, self.old_light_state.g as f32, (self.new_light_state.g as i32 - self.old_light_state.g as i32) as f32, 1.0) as u16,
                    b: Cubic::ease_in_out(time, self.old_light_state.b as f32, (self.new_light_state.b as i32 - self.old_light_state.b as i32) as f32, 1.0) as u16,
                    brightness: Cubic::ease_in_out(time, self.old_light_state.brightness as f32, (self.new_light_state.brightness as i32 - self.old_light_state.brightness as i32) as f32, 1.0) as u16,
                    timestamp: now
                };

                self.light_adjust_rgb_hw(self.current_light_state.g, self.current_light_state.b, self.current_light_state.brightness);

                if last {
                    break
                }

                yield_now().await;

                now = min(Instant::now(), self.new_light_state.timestamp);

                if now == self.new_light_state.timestamp {
                    last = true
                }
            }
        }
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
            g: self.current_light_state.g,
            b: self.current_light_state.b
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
            if LIGHT_SAVE_VALID_FLAG == lum_save.save_flag {
                self.brightness = lum_save.brightness;
                self.current_light_state.g = lum_save.g;
                self.current_light_state.b = lum_save.b;
            } else if lum_save.save_flag == 0xFF {
                break;
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

    pub fn set_brightness(&mut self, brightness: u16) {
        self.brightness = brightness
    }

    pub fn get_brightness(&self) -> u16 {
        self.brightness
    }

    fn calculate_lumen_map(&self, val: u16) -> f32 {
        let percentage = (val as f32 / MAX_LUM_BRIGHTNESS_VALUE as f32) * 100.;
        // return keyframe::ease(keyframe::functions::EaseInOutCubic, 0., 0xffff as f32, percentage);
        return (-0.00539160 * libm::powf(percentage, 3.0))
            + (4.47709595 * libm::powf(percentage, 2.0))
            + (153.72442036 * percentage);
    }

    fn pwm_set_lum(&self, id: u32, y: u32, pol: bool) {
        let lum = (y * PMW_MAX_TICK as u32) / MAX_LUM_BRIGHTNESS_VALUE as u32;

        pwm_set_cmp(id, if pol { PMW_MAX_TICK as u32 - lum } else { lum } as u16);
    }

    fn get_pwm_cmp(&self, val: u16, lum: u16) -> u32 {
        let val_lumen_map = self.calculate_lumen_map(lum);

        return ((val as f32 * val_lumen_map) / MAX_LUM_BRIGHTNESS_VALUE as f32) as u32;
    }

    pub fn light_adjust_g(&self, val: u16, lum: u16) {
        self.pwm_set_lum(PWMID_G, self.get_pwm_cmp(val, lum), false);
    }

    pub fn light_adjust_b(&self, val: u16, lum: u16) {
        self.pwm_set_lum(PWMID_B, self.get_pwm_cmp(val, lum), true);
    }

    pub fn light_adjust_rgb_hw(&self, val_g: u16, val_b: u16, lum: u16) {
        self.light_adjust_g(val_g, lum);
        self.light_adjust_b(val_b, lum);
    }

    pub fn light_onoff_hw(&mut self, on: bool) {
        let state = app().light_manager.get_current_light_state();
        self.begin_transition(state.g, state.b, match on { true => self.brightness, false => 0 });
    }

    pub fn light_onoff(&mut self, on: bool) {
        self.light_onoff_hw(on);
        self.device_status_update();
    }

    pub fn device_status_update(&self) {
        // packet
        let mut st_val_par: [u8; MESH_NODE_ST_PAR_LEN as usize] = [0xff; MESH_NODE_ST_PAR_LEN as usize];

        let brightness = if app().light_manager.is_light_off() {
            0
        } else {
            max(app().light_manager.get_brightness(), 1)
        };

        // let state = app().light_manager.get_current_light_state();

        st_val_par[0] = (brightness & 0xff) as u8;
        st_val_par[1] = ((brightness >> 8) & 0xff) as u8;
        // st_val_par[2] = (state.g & 0xff) as u8;
        // st_val_par[3] = ((state.g >> 8) & 0xff) as u8;
        // st_val_par[4] = (state.b & 0xff) as u8;
        // st_val_par[5] = ((state.b >> 8) & 0xff) as u8;

        _ll_device_status_update(st_val_par.as_ptr(), st_val_par.len() as u8);
    }
}