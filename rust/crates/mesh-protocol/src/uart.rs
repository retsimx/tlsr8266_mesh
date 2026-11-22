//! UART frame layout: the 44-byte `UartData` buffer, its command opcodes and
//! its CRC-16-MODBUS trailer.
//!
//! The frame is a wire contract shared with the Python host tooling, so the
//! offsets and the CRC coverage are public constants rather than private
//! details of the DMA driver.

use crate::crc::crc16;

/// Maximum length of data that can be stored in a UART data buffer.
///
/// This value must be carefully chosen to balance memory usage against
/// maximum message size requirements. The TLSR8266 has limited memory,
/// so we keep this reasonably small.
pub const UART_DATA_LEN: usize = 44; // data max 252

/// Index of the first payload byte: the frame starts `counter | byte1 | cmd`.
pub const UART_PAYLOAD_OFFSET: usize = 3;

/// Index of the two-byte little-endian CRC-16 trailer.
pub const UART_CRC_OFFSET: usize = 42;

/// Bytes available to a frame payload between the command byte and the CRC.
pub const UART_PAYLOAD_MAX_SIZE: usize = UART_CRC_OFFSET - UART_PAYLOAD_OFFSET; // 39 bytes

/// UART DMA frame.
///
/// This struct must be a multiple of 16 bytes in size to ensure proper
/// DMA alignment for efficient data transfer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct UartData {
    /// Length of valid data in the buffer (in bytes).
    /// Although the field is 32 bits, the maximum length is limited by UART_DATA_LEN.
    pub len: u32,
    /// Buffer containing the actual UART data.
    /// The maximum usable size is defined by UART_DATA_LEN.
    pub data: [u8; UART_DATA_LEN],
}

/// Writes the CRC-16-MODBUS of `data[0..UART_CRC_OFFSET]` into the trailer,
/// little-endian, at `UART_CRC_OFFSET..UART_CRC_OFFSET + 2`.
pub fn compute_crc(msg: &mut UartData) {
    let crc = crc16(&msg.data[0..UART_CRC_OFFSET]);
    let [lo, hi] = crc.to_le_bytes();
    msg.data[UART_CRC_OFFSET] = lo;
    msg.data[UART_CRC_OFFSET + 1] = hi;
}

/// Recomputes the CRC-16-MODBUS over `data[0..UART_CRC_OFFSET]` and compares it
/// with the stored little-endian trailer, mirroring the receive-path check.
pub fn verify_crc(msg: &UartData) -> bool {
    let expected = crc16(&msg.data[0..UART_CRC_OFFSET]).to_le_bytes();
    msg.data[UART_CRC_OFFSET] == expected[0] && msg.data[UART_CRC_OFFSET + 1] == expected[1]
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum UartMsg {
    //EnableUart = 0x01,      // Sent by the client to enable uart comms - not handled, just a dummy message
    LightCtrl = 0x02,    // Sent by the client to control the mesh
    LightStatus = 0x03, // Sent by us to notify when a bulk status message is sent. Sent by the client to force a full refresh of light statuses
    MeshMessage = 0x04, // Sent by us to notify when a mesh message is sent
    PanicMessage = 0x05, // Sent by us to provide details of a panic
    PrintMessage = 0x06, // Sent by us to provide print output
    NodeStatus = 0x07, // Sent by us for individual node online/offline and on/off change events. Payload: groups of [node_id, online, on_off] (3 bytes each, up to 13 per packet)
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
            UartMsg::NodeStatus => 0x07,
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
            0x07 => Ok(UartMsg::NodeStatus),
            0xff => Ok(UartMsg::Ack),
            _ => Err(()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame() -> UartData {
        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };
        msg.data[0] = 0x2a;
        msg.data[2] = u8::from(UartMsg::LightCtrl);
        for (index, byte) in msg.data[UART_PAYLOAD_OFFSET..UART_CRC_OFFSET]
            .iter_mut()
            .enumerate()
        {
            *byte = index as u8;
        }
        msg
    }

    #[test]
    fn compute_crc_round_trips_through_verify_crc() {
        let mut msg = frame();
        compute_crc(&mut msg);

        assert!(
            verify_crc(&msg),
            "compute_crc must produce a trailer verify_crc accepts"
        );

        // The trailer is the little-endian encoding of crc16 over 0..42.
        let expected = crc16(&msg.data[0..UART_CRC_OFFSET]).to_le_bytes();
        assert_eq!(msg.data[UART_CRC_OFFSET], expected[0]);
        assert_eq!(msg.data[UART_CRC_OFFSET + 1], expected[1]);
    }

    #[test]
    fn verify_crc_rejects_a_corrupted_payload_byte() {
        let mut msg = frame();
        compute_crc(&mut msg);
        msg.data[UART_PAYLOAD_OFFSET] ^= 0xff;

        assert!(!verify_crc(&msg), "a flipped payload byte must fail");
    }

    #[test]
    fn verify_crc_rejects_a_corrupted_crc_byte() {
        let mut msg = frame();
        compute_crc(&mut msg);
        msg.data[UART_CRC_OFFSET] ^= 0xff;

        assert!(!verify_crc(&msg), "a flipped CRC byte must fail");
    }

    #[test]
    fn verify_crc_rejects_an_unsealed_frame() {
        let msg = frame();

        assert!(
            !verify_crc(&msg),
            "a zeroed trailer must not pass the CRC check"
        );
    }

    #[test]
    fn uart_msg_round_trips_through_u8() {
        for msg in [
            UartMsg::LightCtrl,
            UartMsg::LightStatus,
            UartMsg::MeshMessage,
            UartMsg::PanicMessage,
            UartMsg::PrintMessage,
            UartMsg::NodeStatus,
            UartMsg::Ack,
        ] {
            let raw = u8::from(msg);
            assert_eq!(UartMsg::try_from(raw), Ok(msg));
        }

        assert_eq!(UartMsg::try_from(0x01), Err(()));
        assert_eq!(UartMsg::try_from(0x00), Err(()));
    }
}
