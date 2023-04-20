use crate::pub_mut;
use crate::sdk::light::light_rx_buff_t;

pub_mut!(light_rx_buff, [light_rx_buff_t; 4]); //, [0; 256]);
pub_mut!(blt_tx_fifo, [u8; 320]); //, [0; 320]);