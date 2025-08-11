use core::mem::size_of;
use core::ptr::addr_of;
use core::slice;

use heapless::Deque;

use crate::{app, uprintln};
#[cfg(not(test))]
use crate::SPAWNER;
use crate::embassy::yield_now::yield_now;
use crate::sdk::ble_app::light_ll::mesh_report_status_enable;
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

#[cfg_attr(test, mry::mry)]
pub struct UartManager {
    pub driver: UartDriver,
    send_channel: Deque<UartData, 6>,
    recv_channel: Deque<UartData, 6>,
    ack_counter: u8,
    last_ack: u8,
    sender_started: bool,
    uart_status_reporting: bool,
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
            uart_status_reporting: false,
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
            uart_status_reporting: false,
            sent: Deque::new(),
            mry: Default::default()
        }
    }

    pub fn started(&self) -> bool {
        self.sender_started
    }

    pub fn enable_uart_status_reporting(&mut self) {
        self.uart_status_reporting = true;
    }

    pub fn disable_uart_status_reporting(&mut self) {
        self.uart_status_reporting = false;
    }

    pub fn uart_status_reporting_enabled(&self) -> bool {
        self.uart_status_reporting
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
            
            false
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
                sno = app().mesh_manager.send_mesh_message(&data, destination, msg.data[18], msg.data[19] != 0);
            }

            if msg.data[2] == UartMsg::LightStatus as u8 {
                mesh_report_status_enable(true);
                app().uart_manager.enable_uart_status_reporting();
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

            #[cfg(not(test))]
            unsafe {
                // Start the receiver
                (*SPAWNER).spawn(uart_receiver()).unwrap();
                
                // Start the sender
                (*SPAWNER).spawn(uart_sender()).unwrap();
            }

            mesh_report_status_enable(true);
        }

        if self.recv_channel.push_back(msg).is_err() {
            uprintln!("uart rx handler recv_channel full");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdk::packet_types::{PacketLlApp, PacketL2capHead};

    
    /// Clear the UART manager state to ensure clean test environment
    fn clear_uart_manager_state() {
        critical_section::with(|_| {
            app().uart_manager = UartManager::default();
        });
        // Also reset the global DEVICE_ADDRESS to ensure clean state
        DEVICE_ADDRESS.set(0);
    }
    
    /// Create a test packet with the given parameters
    fn create_test_packet(sno: [u8; 3], src: u16, dst: u16, op: u8, vendor_id: u16, par: [u8; 10]) -> Packet {
        Packet {
            ll_app: PacketLlApp {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 0,
                    rf_len: 0,
                    l2cap_len: 0,
                    chan_id: 0,
                },
                opcode: 0,
                handle: 0,
                handle1: 0,
                value: AppCmdValue {
                    sno,
                    src,
                    dst,
                    op,
                    vendor_id,
                    par,
                },
                rsv: [0; 10],
            }
        }
    }
    
    /// Tests the send_message method when the channel has space.
    ///
    /// This test verifies that send_message correctly adds a message to the send queue
    /// when there's space available.
    ///
    /// # Algorithm
    ///
    /// 1. Create a UartManager with an empty send channel
    /// 2. Create a test message
    /// 3. Call send_message with the test message
    /// 4. Verify the function returns true (message accepted)
    #[test]
    fn test_send_message_with_space() {
        // Create a UartManager with empty queue
        let mut manager = UartManager::default();
        
        // Create a test message
        let msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN]
        };
        
        // Send message
        let result = manager.send_message(&msg);
        
        // Verify result
        assert!(result);
        
        // Verify channel has one message
        critical_section::with(|_| {
            assert_eq!(manager.send_channel.len(), 1);
        });
    }
    
    /// Tests the send_message method when the channel is full.
    ///
    /// This test verifies that send_message correctly rejects messages
    /// when the send channel is full.
    ///
    /// # Algorithm
    ///
    /// 1. Create a UartManager
    /// 2. Fill the send channel to capacity
    /// 3. Try to send one more message
    /// 4. Verify the function returns false (message rejected)
    #[test]
    fn test_send_message_when_full() {
        // Create a UartManager
        let mut manager = UartManager::default();
        
        // Create a test message
        let msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN]
        };
        
        // Fill the channel to capacity
        let capacity = 6; // This should match the capacity in UartManager
        for _ in 0..capacity {
            let result = manager.send_message(&msg);
            assert!(result);
        }
        
        // Try to send one more message
        let result = manager.send_message(&msg);
        
        // Verify the message was rejected
        assert!(!result);
        
        // Verify channel length is still at capacity
        critical_section::with(|_| {
            assert_eq!(manager.send_channel.len(), capacity);
        });
    }
    
    /// Tests the compute_crc method.
    ///
    /// This test verifies that the compute_crc method correctly calculates
    /// and sets the CRC in the specified message.
    ///
    /// # Algorithm
    ///
    /// 1. Create a test message with known data
    /// 2. Call compute_crc on the message
    /// 3. Manually compute the expected CRC
    /// 4. Verify the CRC in the message matches the expected value
    #[test]
    fn test_compute_crc() {
        // Create a test message with known data
        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN]
        };
        
        // Fill with test pattern
        for i in 0..42 {
            msg.data[i] = i as u8;
        }
        
        // Call compute_crc
        UartManager::compute_crc(&mut msg);
        
        // Compute expected CRC
        let expected_crc = crc16(&msg.data[0..42]);
        let expected_crc_low = (expected_crc & 0xff) as u8;
        let expected_crc_high = ((expected_crc >> 8) & 0xff) as u8;
        
        // Verify CRC in message
        assert_eq!(msg.data[42], expected_crc_low);
        assert_eq!(msg.data[43], expected_crc_high);
    }
    
    /// Tests the handle_rx method with a valid CRC.
    ///
    /// This test verifies that handle_rx correctly processes messages
    /// with valid CRC and adds them to the receive queue.
    ///
    /// # Algorithm
    ///
    /// 1. Create a UartManager
    /// 2. Create a test message with valid CRC
    /// 3. Call handle_rx with the test message
    /// 4. Verify message was added to receive queue
    #[test]
    fn test_handle_rx_with_valid_crc() {
        // Create a UartManager
        let mut manager = UartManager::default();
        manager.sender_started = false;
        
        // Create a test message with known data
        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN]
        };
        
        // Fill with test pattern
        for i in 0..42 {
            msg.data[i] = i as u8;
        }
        
        // Set the CRC
        UartManager::compute_crc(&mut msg);
        
        // Call handle_rx
        manager.handle_rx(msg);
        
        // Verify message was added to receive queue
        critical_section::with(|_| {
            assert_eq!(manager.recv_channel.len(), 1);
        });
        
        // Verify sender_started was set to true
        assert!(manager.sender_started);
    }
    
    /// Tests the handle_rx method with an invalid CRC.
    ///
    /// This test verifies that handle_rx correctly rejects messages
    /// with invalid CRC.
    ///
    /// # Algorithm
    ///
    /// 1. Create a UartManager
    /// 2. Create a test message with invalid CRC
    /// 3. Call handle_rx with the test message
    /// 4. Verify message was not added to receive queue
    #[test]
    fn test_handle_rx_with_invalid_crc() {
        // Create a UartManager
        let mut manager = UartManager::default();
        
        // Create a test message with invalid CRC
        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN]
        };
        
        // Fill with test pattern
        for i in 0..42 {
            msg.data[i] = i as u8;
        }
        
        // Set invalid CRC
        msg.data[42] = 0;
        msg.data[43] = 0;
        
        // Call handle_rx
        manager.handle_rx(msg);
        
        // Verify message was not added to receive queue
        critical_section::with(|_| {
            assert_eq!(manager.recv_channel.len(), 0);
        });
    }
    
    /// Tests the started method.
    ///
    /// This test verifies that the started method correctly returns
    /// the sender_started flag.
    ///
    /// # Algorithm
    ///
    /// 1. Create a UartManager with sender_started = false
    /// 2. Verify started() returns false
    /// 3. Set sender_started = true
    /// 4. Verify started() returns true
    #[test]
    fn test_started() {
        // Create a UartManager with sender_started = false
        let mut manager = UartManager::default();
        manager.sender_started = false;
        
        // Verify started() returns false
        assert!(!manager.started());
        
        // Set sender_started = true
        manager.sender_started = true;
        
        // Verify started() returns true
        assert!(manager.started());
    }
    
    /// Tests the UartManager default constructor.
    ///
    /// This test verifies that the UartManager::default constructor
    /// correctly initializes all fields with expected default values.
    ///
    /// # Algorithm
    ///
    /// 1. Create a UartManager using default()
    /// 2. Verify all fields have expected default values
    #[test]
    fn test_uart_manager_default() {
        let manager = UartManager::default();
        
        // Verify default values
        assert_eq!(manager.ack_counter, 0);
        assert_eq!(manager.last_ack, 0);
        assert!(!manager.sender_started);
        
        // Verify channels are empty
        critical_section::with(|_| {
            assert_eq!(manager.send_channel.len(), 0);
            assert_eq!(manager.recv_channel.len(), 0);
            assert_eq!(manager.sent.len(), 0);
        });
    }
    
    /// Tests the light_mesh_rx_cb function when UART manager is not started.
    ///
    /// This test verifies that light_mesh_rx_cb correctly ignores messages
    /// when the UART manager has not been started.
    ///
    /// # Algorithm
    ///
    /// 1. Create a test packet
    /// 2. Ensure UART manager is not started
    /// 3. Call light_mesh_rx_cb with the test packet
    /// 4. Verify no message was sent to UART
    #[test]
    fn test_light_mesh_rx_cb_not_started() {
        // Clear the UART manager state
        clear_uart_manager_state();
        
        // Create a test packet
        let packet = create_test_packet([1, 2, 3], 0x1234, 0x5678, 0x01, 0xABCD, [0; 10]);
        
        // Ensure UART manager is not started
        app().uart_manager.sender_started = false;
        
        // Call the function
        light_mesh_rx_cb(&packet);
        
        // Verify no message was sent to UART
        critical_section::with(|_| {
            assert_eq!(app().uart_manager.send_channel.len(), 0);
        });
    }
    
    /// Tests the light_mesh_rx_cb function when message is from our device.
    ///
    /// This test verifies that light_mesh_rx_cb correctly ignores messages
    /// that were sent by our own device.
    ///
    /// # Algorithm
    ///
    /// 1. Create a test packet with our device address as source
    /// 2. Start the UART manager
    /// 3. Call light_mesh_rx_cb with the test packet
    /// 4. Verify no message was sent to UART
    #[test]
    fn test_light_mesh_rx_cb_from_our_device() {
        // Clear the UART manager state
        clear_uart_manager_state();
        
        // Create a test packet with our device address as source
        let packet = create_test_packet([1, 2, 3], DEVICE_ADDRESS.get(), 0x5678, 0x01, 0xABCD, [0; 10]);
        
        // Start the UART manager
        app().uart_manager.sender_started = true;
        
        // Call the function
        light_mesh_rx_cb(&packet);
        
        // Verify no message was sent to UART
        critical_section::with(|_| {
            assert_eq!(app().uart_manager.send_channel.len(), 0);
        });
    }
    
    /// Tests the light_mesh_rx_cb function when message is a duplicate.
    ///
    /// This test verifies that light_mesh_rx_cb correctly ignores messages
    /// that have already been sent by our device (duplicate detection).
    ///
    /// # Algorithm
    ///
    /// 1. Create a test packet
    /// 2. Start the UART manager
    /// 3. Add the message to the sent queue
    /// 4. Call light_mesh_rx_cb with the test packet
    /// 5. Verify no message was sent to UART
    #[test]
    fn test_light_mesh_rx_cb_duplicate_message() {
        // Clear the UART manager state
        clear_uart_manager_state();
        
        // Create a test packet
        let packet = create_test_packet([1, 2, 3], 0x1234, 0x5678, 0x01, 0xABCD, [0; 10]);
        
        // Start the UART manager
        app().uart_manager.sender_started = true;
        
        // Add the message to the sent queue (simulating we sent this message)
        // The light_mesh_rx_cb function extracts data from offset 5 (dst) for 15 bytes
        // This corresponds to: dst(2) + op(1) + vendor_id(2) + par(10) = 15 bytes
        let msg_data = [0x78, 0x56, 0x01, 0xCD, 0xAB, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        critical_section::with(|_| {
            app().uart_manager.sent.push_back(msg_data).unwrap();
        });
        
        // Call the function
        light_mesh_rx_cb(&packet);
        
        // Verify no message was sent to UART
        critical_section::with(|_| {
            assert_eq!(app().uart_manager.send_channel.len(), 0);
        });
    }
    
    /// Tests the light_mesh_rx_cb function with a valid new message.
    ///
    /// This test verifies that light_mesh_rx_cb correctly processes
    /// valid new messages and forwards them to the UART.
    ///
    /// # Algorithm
    ///
    /// 1. Create a test packet with valid data
    /// 2. Start the UART manager
    /// 3. Call light_mesh_rx_cb with the test packet
    /// 4. Verify a message was sent to UART with correct format
    #[test]
    fn test_light_mesh_rx_cb_valid_message() {
        // Clear the UART manager state
        clear_uart_manager_state();
        
        // Create a test packet with valid data
        let packet = create_test_packet([1, 2, 3], 0x1234, 0x5678, 0x01, 0xABCD, [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0, 0, 0, 0, 0]);
        
        // Start the UART manager
        app().uart_manager.sender_started = true;
        
        // Call the function
        light_mesh_rx_cb(&packet);
        
        // Verify a message was sent to UART
        critical_section::with(|_| {
            assert_eq!(app().uart_manager.send_channel.len(), 1);
            
            // Get the sent message
            let sent_msg = app().uart_manager.send_channel.front().unwrap();
            
            // Verify message format
            assert_eq!(sent_msg.data[2], UartMsg::MeshMessage as u8);
            
            // Verify the AppCmdValue data was copied correctly
            // sno (3 bytes)
            assert_eq!(sent_msg.data[3], 1);
            assert_eq!(sent_msg.data[4], 2);
            assert_eq!(sent_msg.data[5], 3);
            
            // src (2 bytes) - little endian
            assert_eq!(sent_msg.data[6], 0x34);
            assert_eq!(sent_msg.data[7], 0x12);
            
            // dst (2 bytes) - little endian
            assert_eq!(sent_msg.data[8], 0x78);
            assert_eq!(sent_msg.data[9], 0x56);
            
            // op (1 byte)
            assert_eq!(sent_msg.data[10], 0x01);
            
            // vendor_id (2 bytes) - little endian
            assert_eq!(sent_msg.data[11], 0xCD);
            assert_eq!(sent_msg.data[12], 0xAB);
            
            // par (10 bytes)
            assert_eq!(sent_msg.data[13], 0xAA);
            assert_eq!(sent_msg.data[14], 0xBB);
            assert_eq!(sent_msg.data[15], 0xCC);
            assert_eq!(sent_msg.data[16], 0xDD);
            assert_eq!(sent_msg.data[17], 0xEE);
            assert_eq!(sent_msg.data[18], 0);
            assert_eq!(sent_msg.data[19], 0);
            assert_eq!(sent_msg.data[20], 0);
            assert_eq!(sent_msg.data[21], 0);
            assert_eq!(sent_msg.data[22], 0);
        });
    }
    
    /// Tests the light_mesh_rx_cb function with different message types.
    ///
    /// This test verifies that light_mesh_rx_cb correctly processes
    /// messages with different operation codes and parameters.
    ///
    /// # Algorithm
    ///
    /// 1. Create test packets with different op codes and parameters
    /// 2. Start the UART manager
    /// 3. Call light_mesh_rx_cb with each test packet
    /// 4. Verify messages were sent to UART with correct data
    #[test]
    fn test_light_mesh_rx_cb_different_message_types() {
        // Clear the UART manager state
        clear_uart_manager_state();
        
        // Start the UART manager
        app().uart_manager.sender_started = true;
        
        // Test case 1: Light control message
        let packet1 = create_test_packet([10, 20, 30], 0x1111, 0x2222, 0x02, 0x1111, [0x01, 0x02, 0x03, 0x04, 0x05, 0, 0, 0, 0, 0]);
        
        light_mesh_rx_cb(&packet1);
        
        // Test case 2: Status request message
        let packet2 = create_test_packet([40, 50, 60], 0x3333, 0x4444, 0x03, 0x2222, [0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0, 0, 0, 0, 0]);
        
        light_mesh_rx_cb(&packet2);
        
        // Verify both messages were sent to UART
        critical_section::with(|_| {
            assert_eq!(app().uart_manager.send_channel.len(), 2);
            
            // Check first message by popping and checking
            let msg1 = app().uart_manager.send_channel.pop_front().unwrap();
            assert_eq!(msg1.data[2], UartMsg::MeshMessage as u8);
            assert_eq!(msg1.data[10], 0x02); // op code
            
            // Check second message
            let msg2 = app().uart_manager.send_channel.pop_front().unwrap();
            assert_eq!(msg2.data[2], UartMsg::MeshMessage as u8);
            assert_eq!(msg2.data[10], 0x03); // op code
        });
    }
    

}
