use std::ptr::addr_of;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use crate::{BIT, blinken, pub_mut};
use crate::main_light::light_slave_tx_command;
use crate::sdk::common::compat::ultoa;
use crate::sdk::common::crc::crc16;
use crate::sdk::drivers::uart::{UART_DATA_LEN, uart_data_t, UartDriver, UARTIRQMASK};
use crate::sdk::light::{_mesh_send_command, _mesh_send_user_command, _pair_enc_packet_mesh, get_security_enable, LGT_CMD_LIGHT_ONOFF, rf_packet_att_cmd_t};

pub static mut GATEWAY_CMD_SNO: u32 = 0x123456;

pub_mut!(pkt_user_cmd, rf_packet_att_cmd_t);

pub struct UartManager {
    pub driver: UartDriver,
    channel: Channel::<CriticalSectionRawMutex, uart_data_t, 5>
}

impl UartManager {
    pub const fn default() -> Self {
        Self {
            driver: UartDriver::default(),
            channel: Channel::<CriticalSectionRawMutex, uart_data_t, 5>::new()
        }
    }

    pub fn init(&mut self) {
        // Set up the driver
        self.driver.init();
    }

    #[inline(always)]
    pub fn check_irq(&mut self) {
        let irq_s = UartDriver::uart_irqsource_get();
        if irq_s & UARTIRQMASK::RX as u8 != 0 {
            let _ = self.channel.try_send(self.driver.rxdata_buf);
        }

        if irq_s & UARTIRQMASK::TX as u8 != 0 {
            self.driver.uart_clr_tx_busy_flag();
        }
    }

    #[inline(never)]
    async fn ack_msg(&mut self, msg: &uart_data_t) {
        let mut result = uart_data_t {
            len: UART_DATA_LEN as u32, // - 4 here is to shorten the message by 4 bytes which is what the rx side does
            data: [0; UART_DATA_LEN]
        };

        // Set the counter
        result.data[0] = msg.data[0];

        // Ack the counter
        result.data[1] = 0xff;

        // Set the crc16
        let crc = crc16(&result.data[0..42]);
        result.data[42] = (crc & 0xff) as u8;
        result.data[43] = ((crc >> 8) & 0xff) as u8;

        self.driver.uart_send_async(&result).await;
    }

    #[inline(never)]
    pub async fn run(&mut self, _spawner: Spawner) {
        loop {
            let msg = self.channel.recv().await;
            self.ack_msg(&msg).await;

            // Light ctrl
            if msg.data[1] == 0x01 {
                // p_cmd : cmd[3]+para[10]

                // packet
                let mut packet: [u8; 13] = [0; 13];

                packet[0] = LGT_CMD_LIGHT_ONOFF;
                packet[3] = msg.data[4];

                let destination = msg.data[2] as u16 | (msg.data[3] as u16) << 8;

                light_slave_tx_command(&packet, destination);

                if *get_security_enable()
                {
                    get_pkt_user_cmd()._type |= BIT!(7);
                    unsafe {
                        _pair_enc_packet_mesh(addr_of!(pkt_user_cmd) as *const u8);
                        _mesh_send_command(addr_of!(pkt_user_cmd) as *const u8, 0xff, 0);
                    }
                }


                let s: heapless::String<43> = heapless::String::from("msuc: sent");
                // let s = ultoa(result as u32, s, 16);

                let mut data = uart_data_t {
                    len: UART_DATA_LEN as u32,
                    data: [0; UART_DATA_LEN]
                };

                data.data[0..s.len()].copy_from_slice(&s.as_bytes()[0..s.len()]);

                self.driver.uart_send_async(&data).await;
            }

            // self.driver.txdata_user = msg;
            // self.driver.uart_send_async().await;

            // let s: heapless::String<43> = heapless::String::from("Data! Time: ");
            // let s = ultoa(clock_time(), s, 10);
            //
            // let mut data = uart_data_t {
            //     len: s.len() as u32,
            //     data: [0; UART_DATA_LEN]
            // };
            //
            // data.data[0..s.len()].copy_from_slice(&s.as_bytes()[0..s.len()]);
            //
            // self.driver.uart_send_async(&data).await;
        }
    }
}