use core::cmp::min;
use core::ptr::addr_of;
use crate::sdk::mcu::register::{read_reg_aes_ctrl, read_reg_aes_data, write_reg_aes_ctrl, write_reg_aes_data, write_reg_aes_key};
use crate::state::{PAIR_CONFIG_PWD_ENCODE_SK};

pub fn aes_ll_encryption(key: &[u8], source: &[u8], dest: &mut [u8], direction: u8)
{
    let mut valid_key = [0u8; 16];
    valid_key[0..min(key.len(), 16)].copy_from_slice(&key[0..min(key.len(), 16)]);

    let mut valid_source = [0u8; 16];
    valid_source[0..min(source.len(), 16)].copy_from_slice(&source[0..min(source.len(), 16)]);

    critical_section::with(|_| {
        write_reg_aes_ctrl(direction);

        for i in 0..16 {
            write_reg_aes_key(valid_key[15 - i], i as u32);
        }

        for idx in (0..16).step_by(4).rev() {
            write_reg_aes_data(
                (valid_source[idx] as u32) << 0x18 |
                    ((valid_source[idx + 1] as u32) << 0x10) |
                    ((valid_source[idx + 2] as u32) << 0x8) |
                    (valid_source[idx + 3] as u32)
            );
        }

        while read_reg_aes_ctrl() & 4 == 0 {}

        for idx in (0..16).step_by(4) {
            let tmp = read_reg_aes_data();

            dest[idx] = tmp as u8;
            dest[idx + 1] = (tmp >> 8) as u8;
            dest[idx + 2] = (tmp >> 16) as u8;
            dest[idx + 3] = (tmp >> 24) as u8;
        }
    });
}

pub fn aes_ll_swap(data: &mut [u8])
{
    unsafe {
        let mut src = 15;

        for index in 0..8 {
            let tmp = data[index];
            data[index] = data[src];
            data[src] = tmp;
            src -= 1;
        }
    }
}

pub fn aes_att_encryption(key: &[u8], source: &[u8], dest: &mut [u8])
{
    aes_ll_encryption(key, source, dest, 0);
    aes_ll_swap(dest);
}

pub fn aes_att_decryption(key: &[u8], source: &[u8], dest: &mut [u8])
{
    aes_ll_encryption(key, source, dest, 1);
    aes_ll_swap(dest);
}

pub fn encode_password(password: &mut [u8])
{
    unsafe {
        aes_att_encryption(PAIR_CONFIG_PWD_ENCODE_SK.lock().as_slice(), &*addr_of!(*password), password);
    }
}

pub fn decode_password(password: &mut [u8])
{
    unsafe {
        aes_att_decryption(PAIR_CONFIG_PWD_ENCODE_SK.lock().as_slice(), &*addr_of!(*password), password);
    }
}

pub fn aes_att_decryption_packet(key: &[u8], ivm: &[u8], src: &[u8], packet_data: &mut [u8]) -> bool
{
    let mut ivs = [0u8; 16];
    ivs[1..9].copy_from_slice(&ivm[0..8]);

    if packet_data.len() == 0 {
        ivs = [0; 16];
        ivs[0..8].copy_from_slice(&ivm[0..8]);
        ivs[8] = packet_data.len() as u8;
        ivs[12] = packet_data.len() as u8;

        unsafe {
            aes_att_encryption(key, &*addr_of!(ivs), &mut ivs);
        }
    } else {
        let mut index = 0;
        let mut decoded = [0u8; 16];
        loop {
            let mut breakit = false;
            while index & 0xf != 0 {
                packet_data[index] = packet_data[index] ^ decoded[index & 0xf];
                index = index + 1;
                if packet_data.len() <= index {
                    breakit = true;
                    break;
                };
            }

            if breakit {
                break;
            }

            aes_att_encryption(key, &ivs, &mut decoded);
            ivs[0] += 1;
            packet_data[index] = packet_data[index] ^ decoded[0];
            index = index + 1;

            if index >= packet_data.len() {
                break;
            }
        }

        ivs = [0; 16];
        ivs[0..8].copy_from_slice(&ivm[0..8]);
        ivs[8] = packet_data.len() as u8;

        unsafe {
            aes_att_encryption(key, &*addr_of!(ivs), &mut ivs);
        }
        index = 0;
        loop {
            let mut breakit = false;
            loop {
                ivs[index & 0xf] = packet_data[index] ^ ivs[index & 0xf];

                if index & 0xf != 0xf && packet_data.len() - 1 != index {
                    break;
                }
                unsafe {
                    aes_att_encryption(key, &*addr_of!(ivs), &mut ivs);
                }
                index = index + 1;
                if packet_data.len() <= index {
                    breakit = true;
                    break;
                }
            }
            if breakit {
                break;
            }

            index = index + 1;
            if index >= packet_data.len() {
                break;
            }
        }
    }

    if src.len() != 0 {
        return src[0..src.len()] == ivs[0..src.len()];
    }

    return true;
}

pub fn aes_att_encryption_packet(key: &[u8], ivm: &[u8], src: &mut [u8], packet_data: &mut [u8]) -> bool
{
    let mut encoded = [0u8; 16];
    encoded[0..8].copy_from_slice(&ivm[0..8]);
    encoded[8] = packet_data.len() as u8;

    unsafe {
        aes_att_encryption(key, &*addr_of!(encoded), &mut encoded);
    }

    if packet_data.len() != 0 {
        let mut index = 0;
        loop {
            let mut breakit = false;
            loop {
                encoded[index & 0xf] = packet_data[index] ^ encoded[index & 0xf];
                if index & 0xf != 0xf && packet_data.len() - 1 != index {
                    break;
                }

                unsafe {
                    aes_att_encryption(key, &*addr_of!(encoded), &mut encoded);
                }
                index = index + 1;
                if packet_data.len() <= index {
                    breakit = true;
                    break;
                }
            }
            if breakit {
                break;
            }

            index = index + 1;
            if index >= packet_data.len() {
                break;
            }
        }
    }

    if src.len() != 0 {
        let len = src.len();
        src[0..len].copy_from_slice(&encoded[0..len]);
    }
    encoded = [0; 16];
    encoded[1..9].copy_from_slice(&ivm[0..8]);

    let mut result = [0u8; 16];
    for index in 0..packet_data.len() {
        if index & 0xf == 0 {
            aes_att_encryption(key, &encoded, &mut result);
            encoded[0] += 1;
        }

        packet_data[index] = packet_data[index] ^ result[index & 0xf];
    }
    return true;
}