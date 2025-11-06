use core::convert::{TryFrom, TryInto};
use core::mem::size_of;
use core::ops::Range;
use core::ptr::addr_of;
use core::slice;

use heapless::Deque;

use crate::embassy::yield_now::yield_now;
use crate::sdk::ble_app::light_ll::mesh_management::mesh_report_status_enable;
use crate::sdk::common::crc::crc16;
use crate::sdk::drivers::uart::{UartData, UartDriver, UartIrqMask, UART_DATA_LEN};
use crate::sdk::mcu::clock::{clock_time, clock_time_exceed};
use crate::sdk::mcu::watchdog::wd_clear;
use crate::sdk::packet_types::{AppCmdValue, Packet};
#[cfg(not(test))]
use crate::SPAWNER;
use crate::{app, uprintln};
use heapless::Vec;

const MSG_TYPE_INDEX: usize = 2;
const CTRL_DESTINATION_RANGE: Range<usize> = 3..5;
const CTRL_PAYLOAD_RANGE: Range<usize> = 5..18;
const CTRL_FINGERPRINT_RANGE: Range<usize> = 3..18;
const CTRL_RETRANSMIT_INDEX: usize = 18;
const CTRL_ACK_FLAG_INDEX: usize = 19;
const OUTGOING_FINGERPRINT_RANGE: Range<usize> = 7..22;
const CRC_RANGE: Range<usize> = 0..42;
const SNO_RANGE: Range<usize> = 2..5;
const CTRL_PAYLOAD_LEN: usize = CTRL_PAYLOAD_RANGE.end - CTRL_PAYLOAD_RANGE.start;
const CTRL_FINGERPRINT_LEN: usize = CTRL_FINGERPRINT_RANGE.end - CTRL_FINGERPRINT_RANGE.start;

type Fingerprint = [u8; CTRL_FINGERPRINT_LEN];
use crate::state::{SimplifyLS, DEVICE_ADDRESS};

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum UartMsg {
    //EnableUart = 0x01,      // Sent by the client to enable uart comms - not handled, just a dummy message
    LightCtrl = 0x02,    // Sent by the client to control the mesh
    LightStatus = 0x03, // Sent by us to notify when a bulk status message is sent. Sent by the client to force a full refresh of light statuses
    MeshMessage = 0x04, // Sent by us to notify when a mesh message is sent
    PanicMessage = 0x05, // Sent by us to provide details of a panic
    PrintMessage = 0x06, // Sent by us to provide print output
    Ack = 0xff,
}

impl From<UartMsg> for u8 {
    fn from(value: UartMsg) -> Self {
        match value {
            UartMsg::LightCtrl => 0x02,
            UartMsg::LightStatus => 0x03,
            UartMsg::MeshMessage => 0x04,
            UartMsg::PanicMessage => 0x05,
            UartMsg::PrintMessage => 0x06,
            UartMsg::Ack => 0xff,
        }
    }
}

impl TryFrom<u8> for UartMsg {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x02 => Ok(UartMsg::LightCtrl),
            0x03 => Ok(UartMsg::LightStatus),
            0x04 => Ok(UartMsg::MeshMessage),
            0x05 => Ok(UartMsg::PanicMessage),
            0x06 => Ok(UartMsg::PrintMessage),
            0xff => Ok(UartMsg::Ack),
            _ => Err(()),
        }
    }
}

// AppCmdValueT
#[cfg_attr(test, mry::mry)]
pub fn light_mesh_rx_cb(data: &Packet) {
    // Don't report messages that we sent
    if !app().uart_manager.started()
        || (data.ll_app().value.src == DEVICE_ADDRESS.get()
            && data.ll_app().value.dst != DEVICE_ADDRESS.get())
    {
        return;
    }

    let data = addr_of!(data.ll_app().value) as *const u8;
    // Compute the CRC of important bits. This is from start of AppCmdValueT.dst (5) to end of AppCmdValueT.par (20)
    let msg = unsafe { slice::from_raw_parts((data as u32 + 5) as *const u8, 20 - 5) };

    // Check if this message is one we sent
    if critical_section::with(|_| app().uart_manager.sent.iter().any(|c| c == msg)) {
        // We did, nothing more to do here
        return;
    }

    let mut msg: UartData = UartData {
        len: UART_DATA_LEN as u32,
        data: [0; UART_DATA_LEN],
    };

    msg.data[2] = u8::from(UartMsg::MeshMessage);
    for i in 0..size_of::<AppCmdValue>() {
        unsafe { *msg.data.as_mut_ptr().offset(3 + i as isize) = *data.add(i) };
    }

    app().uart_manager.send_message(&msg);
}

#[coverage(off)]
#[embassy_executor::task]
async fn uart_sender() {
    app().uart_manager.sender().await;
}

#[coverage(off)]
#[embassy_executor::task]
async fn uart_receiver() {
    app().uart_manager.receiver().await;
}

#[cfg_attr(test, mry::mry)]
pub struct UartManager {
    pub driver: UartDriver,
    send_channel: Deque<UartData, 12>,
    recv_channel: Deque<UartData, 12>,
    ack_counter: u8,
    last_ack: u8,
    sender_started: bool,
    uart_status_reporting: bool,
    sent: Deque<Fingerprint, 12>,
}

#[cfg_attr(test, mry::mry(skip_fns(default_const, wait_for_message)))]
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
            sent: Deque::new(),
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
            mry: Default::default(),
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
        let crc = crc16(&msg.data[CRC_RANGE]);
        msg.data[42] = (crc & 0xff) as u8;
        msg.data[43] = ((crc >> 8) & 0xff) as u8;
    }

    async fn ack_msg(&mut self, msg: &UartData, sno: &Vec<u8, 3>) {
        // If data[1] is 0xff, it means this message is an ack from the client
        if msg.data[1] == u8::from(UartMsg::Ack) {
            self.last_ack = msg.data[0];
            return;
        }

        let mut result = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };

        // Set the counter
        result.data[0] = msg.data[0];

        // Ack the counter
        result.data[1] = 0xff;

        // Set the sno
        result.data[SNO_RANGE].copy_from_slice(sno.as_slice());

        // Set the crc16
        Self::compute_crc(&mut result);

        // Send the ack
        self.driver.send_async(&result).await;
    }

    pub async fn receiver(&mut self) {
        loop {
            let msg = self.wait_for_received_message().await;
            let sno = self.process_received_message(&msg);
            self.ack_msg(&msg, &Vec::from_slice(&sno).unwrap()).await;
        }
    }
    pub async fn sender(&mut self) {
        loop {
            let mut msg = self.wait_for_outgoing_message().await;
            self.prepare_outgoing_message(&mut msg);
            self.send_with_retry(&msg).await;
        }
    }

    async fn wait_for_message<F, P>(&mut self, is_empty: F, mut pop: P) -> UartData
    where
        F: Fn(&Self) -> bool,
        P: FnMut(&mut Self) -> Option<UartData>,
    {
        while is_empty(self) {
            yield_now().await;
        }

        critical_section::with(|_| pop(self)).expect("message should be available")
    }

    async fn wait_for_received_message(&mut self) -> UartData {
        self.wait_for_message(
            |this| this.recv_channel.is_empty(),
            |this| this.recv_channel.pop_front(),
        )
        .await
    }

    async fn wait_for_outgoing_message(&mut self) -> UartData {
        self.wait_for_message(
            |this| this.send_channel.is_empty(),
            |this| this.send_channel.pop_front(),
        )
        .await
    }

    fn record_sent_fingerprint(&mut self, fingerprint: Fingerprint) {
        critical_section::with(|_| {
            if self.sent.is_full() {
                self.sent.pop_front();
            }

            let push_result = self.sent.push_back(fingerprint);
            debug_assert!(push_result.is_ok());
            let _ = push_result;
        });
    }

    fn mesh_send_message(
        &mut self,
        data: &Vec<u8, CTRL_PAYLOAD_LEN>,
        destination: u16,
        retransmit_count: u8,
        send_ack: bool,
    ) -> [u8; 3] {
        app()
            .mesh_manager
            .send_mesh_message(data, destination, retransmit_count, send_ack)
    }

    fn process_received_message(&mut self, msg: &UartData) -> [u8; 3] {
        match UartMsg::try_from(msg.data[MSG_TYPE_INDEX]) {
            Ok(UartMsg::LightCtrl) => self.handle_light_ctrl(msg),
            Ok(UartMsg::LightStatus) => {
                self.handle_light_status();
                [0; 3]
            }
            _ => [0; 3],
        }
    }

    fn handle_light_ctrl(&mut self, msg: &UartData) -> [u8; 3] {
        let destination = u16::from_le_bytes(
            msg.data[CTRL_DESTINATION_RANGE]
                .try_into()
                .expect("destination bytes"),
        );

        let mut data = [0u8; CTRL_PAYLOAD_LEN];
        data.copy_from_slice(&msg.data[CTRL_PAYLOAD_RANGE]);

        let fingerprint: Fingerprint = msg.data[CTRL_FINGERPRINT_RANGE]
            .try_into()
            .expect("fingerprint slice");
        self.record_sent_fingerprint(fingerprint);

        self.mesh_send_message(
            &Vec::from_slice(&data).unwrap(),
            destination,
            msg.data[CTRL_RETRANSMIT_INDEX],
            msg.data[CTRL_ACK_FLAG_INDEX] != 0,
        )
    }

    fn handle_light_status(&mut self) {
        mesh_report_status_enable(true);
        self.enable_uart_status_reporting();
    }

    fn prepare_outgoing_message(&mut self, msg: &mut UartData) {
        let fingerprint: Fingerprint = msg.data[OUTGOING_FINGERPRINT_RANGE]
            .try_into()
            .expect("outgoing fingerprint slice");
        self.record_sent_fingerprint(fingerprint);

        self.ack_counter = self.ack_counter.wrapping_add(1);
        msg.data[0] = self.ack_counter;
        Self::compute_crc(msg);
    }

    async fn driver_send_async(&mut self, msg: &UartData) -> bool {
        self.driver.send_async(msg).await
    }

    async fn send_with_retry(&mut self, msg: &UartData) {
        loop {
            self.driver_send_async(msg).await;

            let t_timeout = clock_time();
            while self.last_ack != self.ack_counter && !clock_time_exceed(t_timeout, 100 * 1000) {
                wd_clear();
                yield_now().await;
            }

            if self.last_ack == self.ack_counter {
                break;
            }
        }
    }

    pub fn handle_rx(&mut self, msg: UartData) {
        // Check the crc of the packet
        let crc = crc16(&msg.data[CRC_RANGE]);
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
#[coverage(off)]
mod tests {
    use super::*;
    use crate::sdk::ble_app::light_ll::mesh_management::mock_mesh_report_status_enable;
    use crate::sdk::mcu::clock::{mock_clock_time, mock_clock_time_exceed};
    use crate::sdk::mcu::watchdog::mock_wd_clear;
    use crate::sdk::packet_types::{PacketL2capHead, PacketLlApp};
    use core::future::Future;
    use core::sync::atomic::{AtomicU8, Ordering};
    use futures::executor::block_on;
    use futures::task::noop_waker;
    use mry::send_wrapper::SendWrapper;
    use mry::Any;
    use std::task::{Context, Poll};

    /// Clear the UART manager state to ensure clean test environment
    fn clear_uart_manager_state() {
        critical_section::with(|_| {
            app().uart_manager = UartManager::default();
        });
        // Also reset the global DEVICE_ADDRESS to ensure clean state
        DEVICE_ADDRESS.set(0);
    }

    /// Create a test packet with the given parameters
    fn create_test_packet(
        sno: [u8; 3],
        src: u16,
        dst: u16,
        op: u8,
        vendor_id: u16,
        par: [u8; 10],
    ) -> Packet {
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
            },
        }
    }

    #[test]
    fn test_uart_msg_try_from() {
        let cases = [
            (UartMsg::LightCtrl, 0x02),
            (UartMsg::LightStatus, 0x03),
            (UartMsg::MeshMessage, 0x04),
            (UartMsg::PanicMessage, 0x05),
            (UartMsg::PrintMessage, 0x06),
            (UartMsg::Ack, 0xff),
        ];

        for (variant, byte) in cases.iter() {
            assert_eq!(u8::from(*variant), *byte);
            assert_eq!(UartMsg::try_from(*byte).unwrap(), *variant);
        }

        assert!(UartMsg::try_from(0x11).is_err());
    }

    #[test]
    fn test_wait_for_received_message_returns_message() {
        let mut manager = UartManager::default();
        let mut msg = UartData {
            len: 1,
            data: [0; UART_DATA_LEN],
        };
        msg.data[0] = 0xAA;
        let expected_len = msg.len;
        let expected_data = msg.data;
        critical_section::with(|_| manager.recv_channel.push_back(msg).unwrap());

        let result = block_on(manager.wait_for_received_message());

        assert_eq!(result.len, expected_len);
        assert_eq!(result.data, expected_data);
    }

    #[test]
    fn test_wait_for_outgoing_message_returns_message() {
        let mut manager = UartManager::default();
        let mut msg = UartData {
            len: 2,
            data: [0; UART_DATA_LEN],
        };
        msg.data[0] = 0xAB;
        msg.data[1] = 0xCD;
        let expected_len = msg.len;
        let expected_data = msg.data;
        critical_section::with(|_| manager.send_channel.push_back(msg).unwrap());

        let result = block_on(manager.wait_for_outgoing_message());

        assert_eq!(result.len, expected_len);
        assert_eq!(result.data, expected_data);
    }

    #[test]
    fn test_wait_for_outgoing_message_waits_until_message_available() {
        let mut manager = UartManager::default();
        let mut manager_ptr = SendWrapper::new(&mut manager as *mut UartManager);
        let future = manager.wait_for_outgoing_message();
        futures::pin_mut!(future);

        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);

        assert!(matches!(future.as_mut().poll(&mut cx), Poll::Pending));

        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };
        msg.data[0] = 0xEF;
        unsafe {
            (*(*manager_ptr)).send_channel.push_back(msg).unwrap();
        }

        let poll_result = future.as_mut().poll(&mut cx);
        assert!(
            matches!(poll_result, Poll::Ready(_)),
            "future should be ready after message enqueued"
        );
        let Poll::Ready(result) = poll_result else {
            unreachable!()
        };
        assert_eq!(result.data[0], 0xEF);
    }

    #[test]
    fn test_process_received_message_light_ctrl_records_message_and_calls_mesh() {
        let mut manager = UartManager::default();

        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };

        msg.data[MSG_TYPE_INDEX] = u8::from(UartMsg::LightCtrl);
        let destination = u16::to_le_bytes(0x1234);
        msg.data[CTRL_DESTINATION_RANGE].copy_from_slice(&destination);
        msg.data[CTRL_RETRANSMIT_INDEX] = 2; // retransmit count
        msg.data[CTRL_ACK_FLAG_INDEX] = 1; // send ack
        for (idx, byte) in msg.data[CTRL_PAYLOAD_RANGE].iter_mut().enumerate() {
            *byte = (idx as u8) + 1;
        }
        manager
            .mock_mesh_send_message(Any, Any, Any, Any)
            .returns([1, 2, 3]);
        manager.mock_process_received_message(Any).calls_real_impl();
        manager.mock_handle_light_ctrl(Any).calls_real_impl();
        manager.mock_record_sent_fingerprint(Any).calls_real_impl();

        let sno = manager.process_received_message(&msg);

        assert_eq!(sno, [1, 2, 3]);

        let expected_fingerprint: Fingerprint = msg.data[CTRL_FINGERPRINT_RANGE]
            .try_into()
            .expect("fingerprint slice");
        critical_section::with(|_| {
            assert_eq!(manager.sent.len(), 1);
            assert_eq!(manager.sent.front().unwrap(), &expected_fingerprint);
        });
    }

    #[test]
    fn test_process_received_message_unknown_type_returns_zero() {
        let mut manager = UartManager::default();
        let msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0x99; UART_DATA_LEN],
        };

        let sno = manager.process_received_message(&msg);

        assert_eq!(sno, [0; 3]);
        critical_section::with(|_| assert!(manager.sent.is_empty()));
    }

    #[test]
    #[mry::lock(mesh_report_status_enable)]
    fn test_process_received_message_light_status_enables_reporting() {
        let mut manager = UartManager::default();
        manager.disable_uart_status_reporting();

        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };
        msg.data[MSG_TYPE_INDEX] = u8::from(UartMsg::LightStatus);

        mock_mesh_report_status_enable(true).returns(());

        let sno = manager.process_received_message(&msg);

        assert_eq!(sno, [0, 0, 0]);
        assert!(manager.uart_status_reporting_enabled());
        mock_mesh_report_status_enable(true).assert_called(1);
    }

    #[test]
    fn test_prepare_outgoing_message_updates_counter_crc_and_fingerprint() {
        let mut manager = UartManager::default();
        manager.ack_counter = 0;

        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };

        for (idx, byte) in msg.data[OUTGOING_FINGERPRINT_RANGE].iter_mut().enumerate() {
            *byte = (idx as u8) ^ 0xAA;
        }
        manager.prepare_outgoing_message(&mut msg);

        assert_eq!(manager.ack_counter, 1);
        assert_eq!(msg.data[0], 1);

        let expected_crc = crc16(&msg.data[CRC_RANGE]);
        assert_eq!(msg.data[42], (expected_crc & 0xff) as u8);
        assert_eq!(msg.data[43], ((expected_crc >> 8) & 0xff) as u8);

        let expected_fingerprint: Fingerprint = msg.data[OUTGOING_FINGERPRINT_RANGE]
            .try_into()
            .expect("fingerprint slice");
        critical_section::with(|_| {
            assert_eq!(manager.sent.len(), 1);
            assert_eq!(manager.sent.front().unwrap(), &expected_fingerprint);
        });
    }

    #[test]
    #[mry::lock(clock_time, clock_time_exceed, wd_clear)]
    fn test_send_with_retry_exits_when_ack_already_received() {
        let mut manager = UartManager::default();
        manager.ack_counter = 5;
        manager.last_ack = 5;

        let msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };

        manager.mock_driver_send_async(Any).returns(true);
        manager.mock_send_with_retry(Any).calls_real_impl();
        mock_wd_clear().returns(());
        mock_clock_time().returns(10);
        mock_clock_time_exceed(10, 100 * 1000).returns(false);

        block_on(manager.send_with_retry(&msg));

        manager.mock_driver_send_async(Any).assert_called(1);
    }

    #[test]
    #[mry::lock(clock_time, clock_time_exceed, wd_clear)]
    fn test_send_with_retry_retries_until_ack_is_received() {
        let mut manager = UartManager::default();
        manager.ack_counter = 1;
        manager.last_ack = 0;

        let msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };

        let mut manager_ptr = SendWrapper::new(&mut manager as *mut UartManager);
        let send_calls = AtomicU8::new(0);
        manager.mock_driver_send_async(Any).returns_with(move |_| {
            let count = send_calls.fetch_add(1, Ordering::SeqCst) + 1;
            if count == 2 {
                unsafe {
                    let manager_ref: &mut UartManager = &mut *(*manager_ptr);
                    manager_ref.last_ack = manager_ref.ack_counter;
                }
            }
            true
        });
        manager.mock_send_with_retry(Any).calls_real_impl();
        mock_wd_clear().returns(());

        // First check should continue waiting, second indicates timeout and then we mark the ack.
        let exceed_calls = AtomicU8::new(0);
        mock_clock_time().returns(10);
        mock_clock_time_exceed(Any, Any).returns_with(move |_, _| {
            let count = exceed_calls.fetch_add(1, Ordering::SeqCst);
            count >= 1
        });

        block_on(manager.send_with_retry(&msg));

        manager.mock_driver_send_async(Any).assert_called(2);
    }

    #[test]
    fn test_record_sent_fingerprint_evicts_oldest_when_full() {
        let mut manager = UartManager::default();

        critical_section::with(|_| {
            for value in 0..12u8 {
                manager
                    .sent
                    .push_back([value; CTRL_FINGERPRINT_LEN])
                    .unwrap();
            }
        });

        let new_fingerprint = [0xAA; CTRL_FINGERPRINT_LEN];

        manager.record_sent_fingerprint(new_fingerprint);

        critical_section::with(|_| {
            assert_eq!(manager.sent.len(), 12);
            assert_eq!(manager.sent.front().unwrap(), &[1u8; CTRL_FINGERPRINT_LEN]);
            assert_eq!(manager.sent.back().unwrap(), &new_fingerprint);
        });
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
            data: [0; UART_DATA_LEN],
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
            data: [0; UART_DATA_LEN],
        };

        // Fill the channel to capacity
        let capacity = 12; // This matches the capacity in UartManager (Deque<UartData, 6>)
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
            data: [0; UART_DATA_LEN],
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
            data: [0; UART_DATA_LEN],
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
            data: [0; UART_DATA_LEN],
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
        let packet = create_test_packet(
            [1, 2, 3],
            DEVICE_ADDRESS.get(),
            0x5678,
            0x01,
            0xABCD,
            [0; 10],
        );

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
        let packet = create_test_packet(
            [1, 2, 3],
            0x1234,
            0x5678,
            0x01,
            0xABCD,
            [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0, 0, 0, 0, 0],
        );

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
            assert_eq!(sent_msg.data[2], u8::from(UartMsg::MeshMessage));

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
        let packet1 = create_test_packet(
            [10, 20, 30],
            0x1111,
            0x2222,
            0x02,
            0x1111,
            [0x01, 0x02, 0x03, 0x04, 0x05, 0, 0, 0, 0, 0],
        );

        light_mesh_rx_cb(&packet1);

        // Test case 2: Status request message
        let packet2 = create_test_packet(
            [40, 50, 60],
            0x3333,
            0x4444,
            0x03,
            0x2222,
            [0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0, 0, 0, 0, 0],
        );

        light_mesh_rx_cb(&packet2);

        // Verify both messages were sent to UART
        critical_section::with(|_| {
            assert_eq!(app().uart_manager.send_channel.len(), 2);

            // Check first message by popping and checking
            let msg1 = app().uart_manager.send_channel.pop_front().unwrap();
            assert_eq!(msg1.data[2], u8::from(UartMsg::MeshMessage));
            assert_eq!(msg1.data[10], 0x02); // op code

            // Check second message
            let msg2 = app().uart_manager.send_channel.pop_front().unwrap();
            assert_eq!(msg2.data[2], u8::from(UartMsg::MeshMessage));
            assert_eq!(msg2.data[10], 0x03); // op code
        });
    }
}
