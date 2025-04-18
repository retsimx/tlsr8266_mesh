use core::mem::size_of;
use core::ptr::addr_of;
use core::slice;

use embassy_time::{Duration, Timer};
use heapless::Deque;

use crate::{app, SPAWNER, uprintln};
use crate::embassy::yield_now::yield_now;
use crate::mesh::MESH_NODE_ST_VAL_LEN;
use crate::sdk::ble_app::light_ll::mesh_report_status_enable;
use crate::sdk::ble_app::ll_irq::mesh_node_report_status;
use crate::sdk::common::crc::crc16;
use crate::sdk::drivers::uart::{UART_DATA_LEN, UartData, UartDriver, UartIrqMask};
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed};
use crate::sdk::mcu::watchdog::wd_clear;
use crate::sdk::packet_types::{AppCmdValue, Packet};
use crate::state::{DEVICE_ADDRESS, SimplifyLS};

pub enum UartMsg {
    //EnableUart = 0x01,      // Sent by the client to enable uart comms - not handled, just a dummy message
    LightCtrl = 0x02,       // Sent by the client to control the mesh
    LightStatus = 0x03,     // Sent by us to notify when a bulk status message is sent. Sent by the client to force a full refresh of light statuses
    MeshMessage = 0x04,     // Sent by us to notify when a mesh message is sent
    PanicMessage = 0x05,    // Sent by us to provide details of a panic
    PrintMessage = 0x06,    // Sent by us to provide print output
    Ack = 0xff
}

// AppCmdValueT
pub fn light_mesh_rx_cb(data: &Packet) {
    // Don't report messages that we sent
    if !app().uart_manager.started() || (data.ll_app().value.src == DEVICE_ADDRESS.get() && data.ll_app().value.dst != DEVICE_ADDRESS.get()){
        return;
    }

    let data = addr_of!(data.ll_app().value) as *const u8;
    // Compute the CRC of important bits. This is from start of AppCmdValueT.dst (5) to end of AppCmdValueT.par (20)
    let msg = unsafe {slice::from_raw_parts((data as u32 + 5) as *const u8, 20-5)};

    // Check if this message is one we sent
    if critical_section::with(|_| { app().uart_manager.sent.iter().any(|c| {c == msg}) }) {
        // We did, nothing more to do here
        return;
    }

    let mut msg: UartData = UartData {
        len: UART_DATA_LEN as u32,
        data: [0; UART_DATA_LEN]
    };

    msg.data[2] = UartMsg::MeshMessage as u8;
    for i in 0..size_of::<AppCmdValue>() {
        unsafe { *msg.data.as_mut_ptr().offset(3 + i as isize) = *data.add(i) };
    }

    app().uart_manager.send_message(&msg);
}

#[embassy_executor::task]
async fn uart_sender() {
    app().uart_manager.sender().await;
}

#[embassy_executor::task]
async fn uart_receiver() {
    app().uart_manager.receiver().await;
}

#[embassy_executor::task]
async fn node_report_task() {
    let mut msg = UartData {
        len: UART_DATA_LEN as u32,
        data: [0; UART_DATA_LEN]
    };

    msg.data[2] = UartMsg::LightStatus as u8;

    loop {
        let mut data = [0; UART_DATA_LEN-5];

        while mesh_node_report_status(&mut data, (UART_DATA_LEN-5) / MESH_NODE_ST_VAL_LEN) != 0 {
            msg.data[3..UART_DATA_LEN-2].copy_from_slice(&data);
            while !app().uart_manager.send_message(&msg) {
                yield_now().await;
            }
        }

        Timer::after(Duration::from_millis(200)).await;
    }
}

#[cfg_attr(test, mry::mry)]
pub struct UartManager {
    pub driver: UartDriver,
    send_channel: Deque<UartData, 6>,
    recv_channel: Deque<UartData, 6>,
    ack_counter: u8,
    last_ack: u8,
    sender_started: bool,
    sent: Deque<[u8; 15], 6>
}

#[cfg_attr(test, mry::mry(skip_fns(default_const)))]
impl UartManager {
    #[cfg(not(test))]
    pub const fn default_const() -> Self {
        Self {
            driver: UartDriver::default_const(),
            send_channel: Deque::new(),
            recv_channel: Deque::new(),
            ack_counter: 0,
            last_ack: 0,
            sender_started: false,
            sent: Deque::new()
        }
    }

    #[cfg(test)]
    pub fn default() -> Self {
        Self {
            driver: UartDriver::default(),
            send_channel: Deque::new(),
            recv_channel: Deque::new(),
            ack_counter: 0,
            last_ack: 0,
            sender_started: false,
            sent: Deque::new(),
            mry: Default::default()
        }
    }

    pub fn started(&self) -> bool {
        self.sender_started
    }

    pub fn init(&mut self) {
        // Set up the driver
        self.driver.init();
    }

    pub fn send_message(&mut self, msg: &UartData) -> bool {
        critical_section::with(|_| {
            if !self.send_channel.is_full() {
                let _ = self.send_channel.push_back(*msg);
                return true;
            }
            
            return false;
        })
    }

    #[inline(always)]
    pub fn check_irq(&mut self) {
        let irq_s = UartDriver::get_and_clear_irq_source();
        if irq_s & UartIrqMask::Rx as u8 != 0 {
            self.handle_rx(self.driver.rx_data_buf);
        }
        
        if irq_s & UartIrqMask::Tx as u8 != 0 {
            self.driver.clear_tx_busy_flag();
        }
    }

    fn compute_crc(msg: &mut UartData) {
        let crc = crc16(&msg.data[0..42]);
        msg.data[42] = (crc & 0xff) as u8;
        msg.data[43] = ((crc >> 8) & 0xff) as u8;
    }

    async fn ack_msg(&mut self, msg: &UartData, sno: &[u8]) {
        // If data[1] is 0xff, it means this message is an ack from the client
        if msg.data[1] == UartMsg::Ack as u8 {
            self.last_ack = msg.data[0];
            return;
        }

        let mut result = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN]
        };

        // Set the counter
        result.data[0] = msg.data[0];

        // Ack the counter
        result.data[1] = 0xff;

        // Set the sno
        result.data[2..2+3].copy_from_slice(sno);

        // Set the crc16
        Self::compute_crc(&mut result);

        // Send the ack
        self.driver.send_async(&result).await;
    }

    pub async fn receiver(&mut self) {
        loop {
            while self.recv_channel.is_empty() {
                yield_now().await;
            }

            let msg = critical_section::with(|_| {
                self.recv_channel.pop_front().unwrap()
            });

            let mut sno = [0u8; 3];

            // Light ctrl
            if msg.data[2] == UartMsg::LightCtrl as u8 {
                // p_cmd : cmd[3]+para[10]
                let destination = msg.data[3] as u16 | (msg.data[4] as u16) << 8;

                let mut data = [0; 13];
                data.copy_from_slice(&msg.data[5..18]);

                // Record the message
                if self.sent.is_full() {
                    self.sent.pop_front();
                }

                self.sent.push_back(<[u8; 15]>::try_from(&msg.data[3..18]).unwrap()).unwrap();

                // Send the message in to the mesh
                sno = app().mesh_manager.send_mesh_message(&data, destination, msg.data[18], if msg.data[19] != 0 { true } else { false });
            }

            if msg.data[2] == UartMsg::LightStatus as u8 {
                mesh_report_status_enable(true);
            }

            // Finally ack the message once we've handled it
            self.ack_msg(&msg, &sno).await;
        }
    }
    pub async fn sender(&mut self) {
        loop {
            // Wait for a message to send
            while self.send_channel.is_empty() {
                yield_now().await;
            }

            let mut msg = critical_section::with(|_| {
                self.send_channel.pop_front().unwrap()
            });

            let mymsg = unsafe {slice::from_raw_parts((addr_of!(msg.data) as u32 + 7) as *const u8, 20-5)};
            critical_section::with(|_| {
                // Record the message
                if self.sent.is_full() {
                    self.sent.pop_front();
                }

                self.sent.push_back(<[u8; 15]>::try_from(mymsg).unwrap()).unwrap();
            });

            // Set the counter
            self.ack_counter += 1;
            msg.data[0] = self.ack_counter;

            // Set the CRC
            Self::compute_crc(&mut msg);

            // Keep sending the message until we get an ack from the other side
            loop {
                self.driver.send_async(&msg).await;

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

    pub fn handle_rx(&mut self, msg: UartData) {
        // Check the crc of the packet
        let crc = crc16(&msg.data[0..42]);
        if (crc & 0xff) as u8 != msg.data[42] || ((crc >> 8) & 0xff) as u8 != msg.data[43] {
            // CRC is invalid, drop the packet
            return;
        }

        // If we receive a message, then it means we need to start the uart and notification
        // tasks
        if !self.sender_started {
            self.sender_started = true;

            unsafe {
                // Start the receiver
                (*SPAWNER).spawn(uart_receiver()).unwrap();
                
                // Start the sender
                (*SPAWNER).spawn(uart_sender()).unwrap();
                
                // Start the node reporter
                (*SPAWNER).spawn(node_report_task()).unwrap();
            }

            mesh_report_status_enable(true);
        }

        if self.recv_channel.push_back(msg).is_err() {
            uprintln!("uart rx handler recv_channel full");
        }
    }
}
