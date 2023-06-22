use core::cell::RefCell;
use embassy_sync::blocking_mutex::{CriticalSectionMutex};
use crate::embassy::sync::{IrqModeMutex, ThreadModeMutex};
use crate::sdk::light::LightRxBuff;

pub struct SharedState {
    pub light_rx_wptr: usize,
    pub light_rx_buff: [LightRxBuff; 4],
    pub blt_tx_fifo: [[u8; 48]; 8],
    pub blt_tx_wptr: usize,
}

pub struct IrqState {
    pub pair_rands: [u8; 8],
    pub pair_randm: [u8; 8],
}

pub struct AppState {

}

pub static APP_STATE: ThreadModeMutex<RefCell<AppState>> = ThreadModeMutex::new(RefCell::new(AppState {

}));

#[no_mangle]
pub static IRQ_STATE: IrqModeMutex<RefCell<IrqState>> = IrqModeMutex::new(RefCell::new(IrqState {
    pair_rands: [0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7],
    pair_randm: [0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7],
}));

pub static SHARED_STATE: CriticalSectionMutex<RefCell<SharedState>> = CriticalSectionMutex::new(RefCell::new(SharedState {
    light_rx_wptr: 0,
    light_rx_buff: [LightRxBuff{
        dma_len: 0,
        unk1: [0; 3],
        rssi: 0,
        unk2: [0; 3],
        rx_time: 0,
        sno: [0; 3],
        unk3: [0; 5],
        mac: [0; 4],
        unk4: [0; 40]
    }; 4],
    blt_tx_fifo: [[0; 48]; 8],
    blt_tx_wptr: 0
}));