use std::mem::size_of;
use std::ptr::addr_of;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use crate::config::VENDOR_ID;
use crate::sdk::common::crc::crc16;
use crate::sdk::drivers::uart::{UART_DATA_LEN, uart_data_t, UartDriver, UARTIRQMASK};
use crate::sdk::light::{_rf_link_slave_connect, _rf_link_slave_data, app_cmd_value_t, LGT_CMD_LIGHT_ONOFF, LIGHT_OFF_PARAM, LIGHT_ON_PARAM, rf_packet_ll_app_t, rf_packet_ll_data_t, rf_packet_ll_init_t, set_fp_gateway_rx_proc, set_fp_gateway_tx_proc, set_gateway_en};
use crate::sdk::mcu::clock::clock_time;
use crate::sdk::mcu::irq_i::{irq_disable, irq_restore};

pub static mut GATEWAY_CMD_SNO: u32 = 0x123456;

pub struct UartManager {
    pub driver: UartDriver,
    channel: Channel::<CriticalSectionRawMutex, uart_data_t, 5>,
    rcv_count: u32,
}

pub fn ble_master_data_callback (_p: *const u8) {
    // let mut msg = uart_data_t {
    //     len: UART_DATA_LEN as u32 - 4, // - 4 here is to shorted the message by 4 bytes which is what the rx side does
    //     data: [0; UART_DATA_LEN]
    // };
    //
    // msg.data[0] = 0xfe;
    //
    // app().uart_manager.driver.uart_send(&msg);
}

pub fn host_proc() {

}

impl UartManager {
    pub const fn default() -> Self {
        Self {
            driver: UartDriver::default(),
            channel: Channel::<CriticalSectionRawMutex, uart_data_t, 5>::new(),
            rcv_count: 0,
        }
    }

    pub fn init(&mut self) {
        // Set up the driver
        self.driver.init();
    }

    pub fn check_irq(&mut self) {
        let irq_s = UartDriver::uart_irqsource_get();
        if irq_s & UARTIRQMASK::RX as u8 != 0 {
            self.rcv_count += 1;
            let _ = self.channel.try_send(self.driver.rxdata_buf);
        }

        if irq_s & UARTIRQMASK::TX as u8 != 0 {
            self.driver.uart_clr_tx_busy_flag();
        }
    }

    async fn ack_msg(&mut self, msg: &uart_data_t) {
        let mut data: [u8; UART_DATA_LEN] = [0; UART_DATA_LEN];

        // Set the counter
        data[0] = msg.data[0];

        // Ack the counter
        data[1] = 0xff;

        // Set the crc16
        let crc = crc16(&data[0..54]);
        data[54] = (crc & 0xff) as u8;
        data[55] = ((crc >> 8) & 0xff) as u8;

        let msg = uart_data_t {
            len: UART_DATA_LEN as u32 - 4, // - 4 here is to shorted the message by 4 bytes which is what the rx side does
            data,
        };

        self.driver.uart_send_async(&msg).await;
    }

    pub async fn run(&mut self, _spawner: Spawner) {
        loop {
            let msg = self.channel.recv().await;
            self.ack_msg(&msg).await;

            if msg.data[0] == 0x02 {
                let state = irq_disable();

                set_fp_gateway_tx_proc(ble_master_data_callback);
                set_fp_gateway_rx_proc(host_proc);

                set_gateway_en(1);

                let pkt_gateway_init : rf_packet_ll_init_t = rf_packet_ll_init_t {
                        dma_len: size_of::<rf_packet_ll_init_t> as u32 - 4,		// dma_len
                        _type: 5, //FLG_BLE_LIGHT_CONNECT_REQ,				// type
                        rf_len: size_of::<rf_packet_ll_init_t> as u8 - 6,		// rf_len
                        scanA: [0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5],	// scanA
                        advA: [0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5],	// advA
                        aa: [0xd6, 0xbe, 0x89, 0x8e],				// access code
                        crcinit: [0x8d, 0x26, 0xd8],						// crcinit[3]
                        wsize: 0x02,									// wsize
                        woffset: 0x0005,									// woffset
                        interval: 0x0020,									// interval: 32 * 1.25 ms = 40 ms
                        latency: 0x0000,									// latency
                        timeout: 0x0032,									// timeout: 50*10 ms = 500ms
                        chm: [0xff, 0xff, 0xff, 0xff, 0x1f],			// chm[5]
                        hop: 0xac,									// hop: initial channel - 12
                };

                _rf_link_slave_connect(addr_of!(pkt_gateway_init) as *const rf_packet_ll_init_t, clock_time());

                // set_slave_link_connected(true);

                irq_restore(state);
            }

            // Light ctrl
            if msg.data[0] == 0x01 {
                // packet
                let mut pkt_app_data: rf_packet_ll_app_t = rf_packet_ll_app_t {
                    dma_len: 0,
                    _type: 0,
                    rf_len: 0,
                    l2capLen: 0,
                    chanId: 0,
                    opcode: 0,
                    handle: 0,
                    handle1: 0,
                    app_cmd_v: app_cmd_value_t {
                        sno: [0; 3],
                        src: [0; 2],
                        dst: [0; 2],
                        op: 0,
                        vendor_id: 0,
                        par: [0; 10],
                    },
                    rsv: [0; 10],
                };

                let par_len = 10;
                pkt_app_data._type = 0x02;
                pkt_app_data.rf_len = 17 + par_len;
                pkt_app_data.dma_len = pkt_app_data.rf_len as u32 + 2;
                pkt_app_data.l2capLen = pkt_app_data.rf_len as u16 - 4;
                pkt_app_data.chanId = 0x04;
                pkt_app_data.opcode = 0x12; // ATT_OP_WRITE_CMD;
                pkt_app_data.handle = 0x15;
                pkt_app_data.handle1 = 0x00;

                unsafe {
                    GATEWAY_CMD_SNO += 1;
                    pkt_app_data.app_cmd_v.sno[0] = (GATEWAY_CMD_SNO & 0xff) as u8;
                    pkt_app_data.app_cmd_v.sno[1] = ((GATEWAY_CMD_SNO >> 8) & 0xff) as u8;
                    pkt_app_data.app_cmd_v.sno[2] = ((GATEWAY_CMD_SNO >> 16) & 0xff) as u8;
                }
                //memcpy(pkt_app_data.app_cmd_v.src, &device_address, 2);

                pkt_app_data.app_cmd_v.dst[0] = 0xff; // msg.data[1];
                pkt_app_data.app_cmd_v.dst[1] = 0xff; // msg.data[2];

                pkt_app_data.app_cmd_v.op = (LGT_CMD_LIGHT_ONOFF & 0x3F) | 0xC0;
                pkt_app_data.app_cmd_v.vendor_id = VENDOR_ID;

                if msg.data[3] != 0 {
                    pkt_app_data.app_cmd_v.par[0] = LIGHT_ON_PARAM;
                } else {
                    pkt_app_data.app_cmd_v.par[0] = LIGHT_OFF_PARAM;
                }

                // send command
                let r = irq_disable();
                // _rf_link_add_tx_packet(addr_of!(pkt_app_data) as *const u8);
                _rf_link_slave_data(addr_of!(pkt_app_data) as *const rf_packet_ll_data_t, 0);
                irq_restore(r);
            }

            // self.driver.txdata_user = msg;
            // self.driver.uart_send_async().await;

            // let s: heapless::String<43> = heapless::String::from("Data! Time: ");
            // let s = ultoa(clock_time(), s, 10);
            //
            // self.uart_manager.printf_async(s.as_bytes()).await;
        }
    }
}