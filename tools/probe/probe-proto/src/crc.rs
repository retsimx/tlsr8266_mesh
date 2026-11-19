//! CRC16-CCITT for frame integrity.

/// Computes CRC16-CCITT over `data`.
///
/// Polynomial `0x1021`, initial value `0xFFFF`, no input/output reflection and
/// no final XOR. Empty input therefore returns `0xFFFF`.
///
/// This is the bitwise form (one XOR/shift round per input bit) rather than a
/// 512-entry lookup table, which keeps the probe firmware small.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::crc16_ccitt;

    #[test]
    fn known_answer() {
        // CCITT-FALSE (init 0xFFFF) check value.
        assert_eq!(crc16_ccitt(b"123456789"), 0x29B1);
    }

    #[test]
    fn empty_input_returns_init_value() {
        assert_eq!(crc16_ccitt(&[]), 0xFFFF);
    }

    #[test]
    fn all_zero_kib_value_is_stable() {
        // Independently computed with a bitwise CCITT-FALSE (init 0xFFFF)
        // reference.
        assert_eq!(crc16_ccitt(&[0u8; 1024]), 0xB76F);
    }

    #[test]
    fn every_single_bit_flip_changes_the_crc() {
        let mut buf = [0u8; 64];
        for (i, byte) in buf.iter_mut().enumerate() {
            *byte = i as u8;
        }
        let baseline = crc16_ccitt(&buf);
        for (byte_index, _) in buf.iter().enumerate() {
            for bit in 0..8 {
                let mut flipped = buf;
                flipped[byte_index] ^= 1u8 << bit;
                assert_ne!(
                    crc16_ccitt(&flipped),
                    baseline,
                    "bit {bit} of byte {byte_index} was not detected"
                );
            }
        }
    }
}
