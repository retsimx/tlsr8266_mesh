use std::mem::size_of;
use std::ptr::{addr_of};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use crate::{app, BIT, pub_mut};
use crate::embassy::yield_now::yield_now;
use crate::main_light::light_slave_tx_command;
use crate::mesh::mesh_node_st_val_t;
use crate::mesh::wrappers::get_mesh_node_st;
use crate::sdk::common::crc::crc16;
use crate::sdk::common::string::memcpy;
use crate::sdk::drivers::uart::{UART_DATA_LEN, uart_data_t, UartDriver, UARTIRQMASK};
use crate::sdk::light::{_mesh_send_command, _pair_enc_packet_mesh, app_cmd_value_t, get_mesh_node_max, get_security_enable, MESH_NODE_MAX_NUM, rf_packet_att_cmd_t, rf_packet_att_value_t, set_p_cb_rx_from_mesh, set_p_mesh_node_status_callback, set_p_vendor_mesh_node_rcv_rsp, set_p_vendor_mesh_node_status_report};
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed};
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};
use crate::sdk::mcu::watchdog::wd_clear;

pub_mut!(pkt_user_cmd, rf_packet_att_cmd_t);

enum UartMsg {
    EnableUart = 0x01,      // Sent by the client to enable uart comms
    LightCtrl = 0x02,       // Sent by the client to control the mesh
    LightOnline = 0x03,     // Sent by us to notify when a light goes on/offline or turns on/off
    MeshMessage = 0x04,     // Sent by us to notify when a mesh message is sent
    Ack = 0xff
}

// app_cmd_value_t
fn light_mesh_rx_cb(data: *const u8) {
    let mut msg: uart_data_t = uart_data_t {
        len: UART_DATA_LEN as u32,
        data: [0; UART_DATA_LEN]
    };

    msg.data[2] = UartMsg::MeshMessage as u8;
    unsafe { memcpy(msg.data.as_mut_ptr().offset(3), data as *const u8, size_of::<app_cmd_value_t>() as u32); }

    app().uart_manager.send_message(&msg);
}

#[embassy_executor::task]
async fn uart_sender() {
    app().uart_manager.sender().await;
}


#[embassy_executor::task]
async fn notification_parser() {
    let mut msg = uart_data_t {
        len: UART_DATA_LEN as u32,
        data: [0; UART_DATA_LEN]
    };

    msg.data[2] = UartMsg::LightOnline as u8;

    let mut last_state: [u8; MESH_NODE_MAX_NUM as usize / 8] = [0; MESH_NODE_MAX_NUM as usize / 8];
    let mut last_val: [[u8; 2]; MESH_NODE_MAX_NUM as usize] = [[0; 2]; MESH_NODE_MAX_NUM as usize];
    loop {
        for i in 0..MESH_NODE_MAX_NUM as usize {
            if get_mesh_node_st()[i].tick != 0 && get_mesh_node_st()[i].val.sn != 0 {
                if last_state[i / 8] & BIT!(i % 8) == 0 || last_val[i] != get_mesh_node_st()[i].val.par {
                    last_state[i / 8] |= BIT!(i % 8);
                    last_val[i] = get_mesh_node_st()[i].val.par;

                    msg.data[3] = 1;
                    unsafe { memcpy(msg.data.as_mut_ptr().offset(4), addr_of!(get_mesh_node_st()[i].val) as *const u8, size_of::<mesh_node_st_val_t>() as u32); }

                    while !app().uart_manager.send_message(&msg) {
                        yield_now().await;
                    }
                }
            } else {
                if last_state[i / 8] & BIT!(i % 8) != 0 {
                    last_state[i / 8] &= !BIT!(i % 8);
                    last_val[i] = [0; 2];

                    msg.data[3] = 0;
                    unsafe { memcpy(msg.data.as_mut_ptr().offset(4), addr_of!(get_mesh_node_st()[i].val) as *const u8, size_of::<mesh_node_st_val_t>() as u32); }

                    while !app().uart_manager.send_message(&msg) {
                        yield_now().await;
                    }
                }
            }

            yield_now().await;
        }
    }
}


pub struct UartManager {
    pub driver: UartDriver,
    recv_channel: Channel::<CriticalSectionRawMutex, uart_data_t, 1>,
    send_channel: Channel::<CriticalSectionRawMutex, uart_data_t, 1>,
    ack_counter: u8,
    last_ack: u8,
    sender_started: bool
}

impl UartManager {
    pub const fn default() -> Self {
        Self {
            driver: UartDriver::default(),
            recv_channel: Channel::<CriticalSectionRawMutex, uart_data_t, 1>::new(),
            send_channel: Channel::<CriticalSectionRawMutex, uart_data_t, 1>::new(),
            ack_counter: 0,
            last_ack: 0,
            sender_started: false
        }
    }

    pub fn init(&mut self) {
        // Set up the driver
        self.driver.init();
    }

    pub fn send_message(&mut self, msg: &uart_data_t) -> bool {
        match self.send_channel.try_send(*msg) {
            Ok(..) => true,
            Err(..) => false
        }
    }

    #[inline(always)]
    pub fn check_irq(&mut self) {
        let irq_s = UartDriver::uart_irqsource_get();
        if irq_s & UARTIRQMASK::RX as u8 != 0 {
            let _ = self.recv_channel.try_send(self.driver.rxdata_buf);
        }

        if irq_s & UARTIRQMASK::TX as u8 != 0 {
            self.driver.uart_clr_tx_busy_flag();
        }
    }

    fn compute_crc(msg: &mut uart_data_t) {
        let crc = crc16(&msg.data[0..42]);
        msg.data[42] = (crc & 0xff) as u8;
        msg.data[43] = ((crc >> 8) & 0xff) as u8;
    }

    async fn ack_msg(&mut self, msg: &uart_data_t) -> bool {
        // If data[1] is 0xff, it means this message is an ack from the client
        if msg.data[1] == UartMsg::Ack as u8 {
            self.last_ack = msg.data[0];
            return false;
        }

        let mut result = uart_data_t {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN]
        };

        // Set the counter
        result.data[0] = msg.data[0];

        // Ack the counter
        result.data[1] = 0xff;

        // Set the crc16
        Self::compute_crc(&mut result);

        // Send the ack
        self.driver.uart_send_async(&result).await;

        return true;
    }

    pub async fn sender(&mut self) {
        loop {
            // Wait for a message to send
            let mut msg = self.send_channel.recv().await;

            // Set the counter
            self.ack_counter += 1;
            msg.data[0] = self.ack_counter;

            // Set the CRC
            Self::compute_crc(&mut msg);

            // Keep sending the message until we get an ack from the other side
            loop {
                self.driver.uart_send_async(&msg).await;

                // If 100ms passes without an ack from the other side, assume the send failed and try again
                let t_timeout = clock_time();
                while self.last_ack != self.ack_counter && !clock_time_exceed(t_timeout, 100 * 1000) {
                    wd_clear();

                    yield_now().await;
                }

                // Did we get an ack?
                if self.last_ack == self.ack_counter {
                    // Yes, sending the message was successful
                    break;
                }
            }
        }
    }

    pub async fn run(&mut self, spawner: Spawner) {
        spawner.spawn(uart_sender()).unwrap();

        loop {
            let msg = self.recv_channel.recv().await;

            // Check the crc of the packet
            let crc = crc16(&msg.data[0..42]);
            if (crc & 0xff) as u8 != msg.data[42] || ((crc >> 8) & 0xff) as u8 != msg.data[43] {
                // CRC is invalid, drop the packet
                continue;
            }

            if !self.ack_msg(&msg).await {
                // Message was an ack message, nothing more to do
                continue;
            }

            // If this message is to enable uart, then just enable the status update callbacks
            if msg.data[2] == UartMsg::EnableUart as u8 {
                // Start the sender if it's not already running
                if !self.sender_started {
                    self.sender_started = true;

                    spawner.spawn(notification_parser()).unwrap();

                    set_p_cb_rx_from_mesh(Some(light_mesh_rx_cb));
                }
            }

            // Light ctrl
            if msg.data[2] == UartMsg::LightCtrl as u8 {
                // p_cmd : cmd[3]+para[10]
                let destination = msg.data[3] as u16 | (msg.data[4] as u16) << 8;

                light_slave_tx_command(&msg.data[5..5+13], destination);

                if *get_security_enable()
                {
                    get_pkt_user_cmd()._type |= BIT!(7);
                    unsafe {
                        _pair_enc_packet_mesh(addr_of!(pkt_user_cmd) as *const u8);
                        _mesh_send_command(addr_of!(pkt_user_cmd) as *const u8, 0xff, 0);
                    }
                }

                // let s: heapless::String<43> = heapless::String::from("msuc: sent");
                // // let s = ultoa(result as u32, s, 16);
                //
                // let mut data = uart_data_t {
                //     len: UART_DATA_LEN as u32,
                //     data: [0; UART_DATA_LEN]
                // };
                //
                // data.data[0..s.len()].copy_from_slice(&s.as_bytes()[0..s.len()]);
                //
                // self.driver.uart_send_async(&data).await;
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

//     let mut msg = uart_data_t {
//         len: UART_DATA_LEN as u32,
//         data: [0; UART_DATA_LEN]
//     };
//
//     msg.data[0] = 0x02; // notify
//     unsafe { msg.data[1..1+size_of::<rf_packet_att_value_t>()].copy_from_slice(&*slice_from_raw_parts(data as *const u8, size_of::<rf_packet_att_value_t>())); }
//
//     app().uart_manager.driver.uart_send(&msg);