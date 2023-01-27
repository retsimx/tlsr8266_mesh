use std::cmp::min;
use easer::functions::{Cubic, Easing};
use embassy_executor::Spawner;
use crate::main_light::{light_adjust_rgb_hw, light_onoff, set_light_off};
use crate::sdk::light::{LGT_CMD_LIGHT_ONOFF, LIGHT_OFF_PARAM, LIGHT_ON_PARAM};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant};
use crate::app;
use crate::embassy::yield_now::yield_now;

const TRANSITION_MILLIS: u64 = 1000;

#[derive(Copy, Clone, Debug)]
struct Message {
    cmd: u8,
    params: [u8; 16]
}

#[derive(Copy, Clone, Debug)]
struct LightState {
    r: u16,
    g: u16,
    b: u16,
    brightness: u16,
    timestamp: Instant
}

impl LightState {
    pub const fn default() -> Self {
        Self {
            r: 0,
            g: 0,
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
    current_light_state: LightState
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
            current_light_state: LightState::default()
        }
    }

    fn handle_on_off(&self, on: u8) {
        light_onoff(match on {
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

    pub fn begin_transition(&mut self, r: u16, g: u16, b: u16, brightness: u16) {
        set_light_off(self.current_light_state.brightness == 0);

        self.old_light_state = self.current_light_state;
        self.old_light_state.timestamp = Instant::now();

        self.new_light_state = LightState {
            r, g, b, brightness,
            timestamp: Instant::now() + Duration::from_millis(TRANSITION_MILLIS)
        };

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
                    r: Cubic::ease_in_out(time, self.old_light_state.r as f32, (self.new_light_state.r as i32 - self.old_light_state.r as i32) as f32, 1.0) as u16,
                    g: Cubic::ease_in_out(time, self.old_light_state.g as f32, (self.new_light_state.g as i32 - self.old_light_state.g as i32) as f32, 1.0) as u16,
                    b: Cubic::ease_in_out(time, self.old_light_state.b as f32, (self.new_light_state.b as i32 - self.old_light_state.b as i32) as f32, 1.0) as u16,
                    brightness: Cubic::ease_in_out(time, self.old_light_state.brightness as f32, (self.new_light_state.brightness as i32 - self.old_light_state.brightness as i32) as f32, 1.0) as u16,
                    timestamp: now
                };

                light_adjust_rgb_hw(self.current_light_state.r, self.current_light_state.g, self.current_light_state.b, self.current_light_state.brightness);

                if last {
                    break
                }

                yield_now().await;

                now = min(Instant::now(), self.new_light_state.timestamp);

                if now == self.new_light_state.timestamp {
                    last = true
                }
            }

            set_light_off(self.current_light_state.brightness == 0);
        }
    }
}