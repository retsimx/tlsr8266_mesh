use core::mem::size_of;
use core::ptr::{addr_of, addr_of_mut};
use core::slice;
use embassy_executor::Spawner;
use heapless::Deque;
use crate::{app, BIT, pub_mut};
use crate::embassy::yield_now::yield_now;
use crate::mesh::{get_mesh_node_st, mesh_node_st_val_t};
use crate::sdk::common::crc::crc16;
use crate::sdk::drivers::uart::{UART_DATA_LEN, uart_data_t, UartDriver, UARTIRQMASK};
use crate::sdk::light::{AppCmdValue, MESH_NODE_MAX_NUM, MeshPkt, set_p_cb_rx_from_mesh};
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed};
use crate::sdk::mcu::watchdog::wd_clear;

pub_mut!(pkt_user_cmd, MeshPkt, MeshPkt {
    dma_len: 0x27,
    _type: 2,
    rf_len: 0x25,
    l2cap_len: 0xCCDD,
    chan_id: 0,
    src_tx: 0,
    handle1: 0,
    sno: [0; 3],
    src_adr: 0,
    dst_adr: 0,
    op: 0,
    vendor_id: 0,
    par: [0; 10],
    internal_par1: [0; 5],
    ttl: 0,
    internal_par2: [0; 4],
    no_use: [0; 4]
});

pub enum UartMsg {
    //EnableUart = 0x01,      // Sent by the client to enable uart comms - not handled, just a dummy message
    LightCtrl = 0x02,       // Sent by the client to control the mesh
    LightOnline = 0x03,     // Sent by us to notify when a light goes on/offline or turns on/off
    MeshMessage = 0x04,     // Sent by us to notify when a mesh message is sent
    PanicMessage = 0x05,    // Sent by us to provide details of a panic
    PrintMessage = 0x06,    // Sent by us to provide print output
    Ack = 0xff
}

// AppCmdValueT
fn light_mesh_rx_cb(data: &AppCmdValue) {
    let data = addr_of!(*data) as *const u8;
    // Compute the CRC of important bits. This is from start of AppCmdValueT.dst (5) to end of AppCmdValueT.par (20)
    let msg = unsafe {slice::from_raw_parts((data as u32 + 5) as *const u8, 20-5)};

    // Check if this message is one we sent
    if app().uart_manager.sent.iter().any(|c| {c == msg}) {
        // We did, nothing more to do here
        return;
    }

    let mut msg: uart_data_t = uart_data_t {
        len: UART_DATA_LEN as u32,
        data: [0; UART_DATA_LEN]
    };

    msg.data[2] = UartMsg::MeshMessage as u8;
    for i in 0..size_of::<AppCmdValue>() {
        unsafe { *msg.data.as_mut_ptr().offset(3 + i as isize) = *data.offset(i as isize) };
    }

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
            if get_mesh_node_st()[i].tick != 0 {
                if last_state[i / 8] & BIT!(i % 8) == 0 || last_val[i] != get_mesh_node_st()[i].val.par {
                    last_state[i / 8] |= BIT!(i % 8);
                    last_val[i] = get_mesh_node_st()[i].val.par;

                    msg.data[3] = 1;
                    for j in 0..size_of::<mesh_node_st_val_t>() {
                        unsafe { *msg.data.as_mut_ptr().offset(4 + j as isize) = *(addr_of!(get_mesh_node_st()[i].val) as *const u8).offset(j as isize) };
                    }

                    while !app().uart_manager.send_message(&msg) {
                        yield_now().await;
                    }
                }
            } else {
                if last_state[i / 8] & BIT!(i % 8) != 0 {
                    last_state[i / 8] &= !BIT!(i % 8);
                    last_val[i] = [0; 2];

                    msg.data[3] = 0;
                    for j in 0..size_of::<mesh_node_st_val_t>() {
                        unsafe { *msg.data.as_mut_ptr().offset(4 + j as isize) = *(addr_of!(get_mesh_node_st()[i].val) as *const u8).offset(j as isize) };
                    }

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
    recv_channel: Deque<uart_data_t, 5>,
    send_channel: Deque<uart_data_t, 5>,
    ack_counter: u8,
    last_ack: u8,
    sender_started: bool,
    sent: Deque<[u8; 15], 10>
}

impl UartManager {
    pub const fn default() -> Self {
        Self {
            driver: UartDriver::default(),
            recv_channel: Deque::new(),
            send_channel: Deque::new(),
            ack_counter: 0,
            last_ack: 0,
            sender_started: false,
            sent: Deque::new()
        }
    }

    pub fn init(&mut self) {
        // Set up the driver
        self.driver.init();
    }

    pub fn send_message(&mut self, msg: &uart_data_t) -> bool {
        if self.send_channel.is_full() {
            return false;
        }

        critical_section::with(|_| {
            let _ = self.send_channel.push_back(*msg);
        });

        return true;
    }

    #[inline(always)]
    pub fn check_irq(&mut self) {
        let irq_s = UartDriver::uart_irqsource_get();
        if irq_s & UARTIRQMASK::RX as u8 != 0 {
            if !self.recv_channel.is_full() {
                let _ = self.recv_channel.push_back(self.driver.rxdata_buf);
            }
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
            while self.send_channel.is_empty() {
                yield_now().await;
            }

            let mut msg = critical_section::with(|_| {
                self.send_channel.pop_front().unwrap()
            });

            // Record the message
            if self.sent.is_full() {
                self.sent.pop_front();
            }

            let mymsg = unsafe {slice::from_raw_parts((addr_of!(msg.data) as u32 + 7) as *const u8, 20-5)};
            self.sent.push_back(<[u8; 15]>::try_from(mymsg).unwrap()).unwrap();

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
        loop {
            // Wait for a message
            while self.recv_channel.is_empty() {
                yield_now().await;
            }

            let msg = critical_section::with(|_| {
                self.recv_channel.pop_front().unwrap()
            });

            // Check the crc of the packet
            let crc = crc16(&msg.data[0..42]);
            if (crc & 0xff) as u8 != msg.data[42] || ((crc >> 8) & 0xff) as u8 != msg.data[43] {
                // CRC is invalid, drop the packet
                continue;
            }

            // If we receive a message, then it means we need to start the uart and notification
            // tasks
            if !self.sender_started {
                self.sender_started = true;

                spawner.spawn(uart_sender()).unwrap();
                spawner.spawn(notification_parser()).unwrap();

                set_p_cb_rx_from_mesh(Some(light_mesh_rx_cb));
            }

            // Light ctrl
            if msg.data[2] == UartMsg::LightCtrl as u8 {
                // p_cmd : cmd[3]+para[10]
                let destination = msg.data[3] as u16 | (msg.data[4] as u16) << 8;

                let mut data = [0; 13];
                data.copy_from_slice(&msg.data[5..5+13]);

                // Record the message
                if self.sent.is_full() {
                    self.sent.pop_front();
                }

                let mymsg = unsafe {slice::from_raw_parts(addr_of!(get_pkt_user_cmd().dst_adr) as *const u8, 15)};
                self.sent.push_back(<[u8; 15]>::try_from(mymsg).unwrap()).unwrap();

                // Send the message in to the mesh
                app().mesh_manager.send_mesh_message(&data, destination);
            }

            // Finally ack the message once we've handled it
            if !self.ack_msg(&msg).await {
                // Message was an ack message, nothing more to do
                continue;
            }
        }
    }
}
