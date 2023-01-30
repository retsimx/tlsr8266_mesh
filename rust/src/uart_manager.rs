use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use crate::sdk::drivers::uart::{uart_data_t, UartDriver, UARTIRQMASK};

pub struct UartManager {
    pub driver: UartDriver,
    channel: Channel::<NoopRawMutex, uart_data_t, 5>,
}

impl UartManager {
    pub const fn default() -> Self {
        Self {
            driver: UartDriver::default(),
            channel: Channel::<NoopRawMutex, uart_data_t, 5>::new(),
        }
    }

    pub fn init(&mut self) {
        // Set up the driver
        self.driver.init();
    }

    pub fn check_irq(&mut self) {
        let irq_s = UartDriver::uart_irqsource_get();
        if irq_s & UARTIRQMASK::RX as u8 != 0 {
            let _ = self.channel.try_send(self.driver.rxdata_buf);
        }

        if irq_s & UARTIRQMASK::TX as u8 != 0 {
            self.driver.uart_clr_tx_busy_flag();
        }
    }

    pub async fn run(&mut self, spawner: Spawner) {
        loop {
            let msg = self.channel.recv().await;

            self.driver.txdata_user = msg;
            self.driver.uart_send_async().await;

                // let s: heapless::String<43> = heapless::String::from("Data! Time: ");
                // let s = ultoa(clock_time(), s, 10);
                //
                // self.uart_manager.printf_async(s.as_bytes()).await;


        }
    }
}