use crate::pub_mut;
use crate::sdk::light::LightRxBuff;

pub_mut!(light_rx_buff, [LightRxBuff; 4], [LightRxBuff{
    dma_len: 0,
    unk1: [0; 3],
    rssi: 0,
    unk2: [0; 3],
    rx_time: 0,
    sno: [0; 3],
    unk3: [0; 5],
    mac: [0; 4],
    unk4: [0; 40]
}; 4]);
pub_mut!(blt_tx_fifo, [u8; 48*8], [0; 48*8]);