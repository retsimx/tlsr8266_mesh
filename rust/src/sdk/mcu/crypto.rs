use core::cmp::min;
use core::slice;
use crate::{pub_mut, uprintln};
use crate::sdk::light::{get_pair_config_pwd_encode_sk};
use crate::sdk::mcu::register::{read_reg_aes_ctrl, read_reg_aes_data, write_reg_aes_ctrl, write_reg_aes_data, write_reg_aes_key};

pub fn aes_ll_encryption(key: *const u8, source: *const u8, dest: *mut u8, direction: u8)
{
    critical_section::with(|_| {
        write_reg_aes_ctrl(direction);

        unsafe {
            for i in 0..16 {
                write_reg_aes_key(*key.offset(15 - i), i as u32);
            }

            let mut src_addr = source.offset(0xc);
            while src_addr != source.offset(-4) {
                write_reg_aes_data(
                    (*src_addr as u32) << 0x18 |
                        ((*src_addr.offset(1) as u32) << 0x10) |
                        ((*src_addr.offset(2) as u32) << 0x8) |
                        (*src_addr.offset(3) as u32)
                );

                src_addr = src_addr.offset(-4);
            }
        }

        while read_reg_aes_ctrl() & 4 == 0 {}

        let mut dest_addr = dest as *mut u32;
        unsafe {
            while dest_addr != dest.offset(0x10) as *mut u32 {
                *dest_addr = read_reg_aes_data();
                dest_addr = dest_addr.offset(1);
            }
        }
    });
}

pub fn aes_ll_swap(data: *mut u8)
{
    unsafe {
        let mut data_addr = data.offset(0xf) as *mut u8;
        for index in 0..8 {
            let tmp = *data.offset(index);
            *data.offset(index) = *data_addr;
            *data_addr = tmp;
            data_addr = data_addr.offset(-1);
        }
    }
}

pub fn aes_att_encryption(key: *const u8, source: *const u8, dest: *mut u8)
{
    aes_ll_encryption(key, source, dest, 0);
    aes_ll_swap(dest);
}

pub fn aes_att_decryption(key: *const u8, source: *const u8, dest: *mut u8)
{
    aes_ll_encryption(key, source, dest, 1);
    aes_ll_swap(dest);
}

pub fn encode_password(password: &mut [u8])
{
    unsafe {
        aes_att_encryption(get_pair_config_pwd_encode_sk().as_ptr(), password.as_ptr(), password.as_mut_ptr());
    }
}

pub fn decode_password(password: &mut [u8])
{
    unsafe {
        aes_att_decryption(get_pair_config_pwd_encode_sk().as_ptr(), password.as_ptr(), password.as_mut_ptr());
    }
}

pub unsafe fn aes_att_decryption_packet(key: &[u8], ivm: &[u8], src: &[u8], packet_data: *mut u8, packet_len: u8) -> bool
{
    let mut ivs = [0u8; 16];
    ivs[1..9].copy_from_slice(&ivm[0..8]);

    if packet_len == 0 {
        ivs = [0; 16];
        ivs[0..8].copy_from_slice(&ivm[0..8]);
        ivs[8] = packet_len;
        ivs[12] = packet_len;

        aes_att_encryption(key.as_ptr(), ivs.as_ptr(), ivs.as_mut_ptr());
    } else {
        let mut index = 0;
        let mut decoded = [0u8; 16];
        loop {
            let mut breakit = false;
            while index & 0xf != 0 {
                *packet_data.offset(index) = *packet_data.offset(index) ^ decoded[(index & 0xf) as usize];
                index = index + 1;
                if packet_len <= index as u8 {
                    breakit = true;
                    break;
                };
            }

            if breakit {
                break;
            }

            aes_att_encryption(key.as_ptr(), ivs.as_ptr(), decoded.as_mut_ptr());
            ivs[0] += 1;
            *packet_data.offset(index) = *packet_data.offset(index) ^ decoded[0];
            index = index + 1;

            if index as u8 >= packet_len {
                break;
            }
        }

        ivs = [0; 16];
        ivs[0..8].copy_from_slice(&ivm[0..8]);
        ivs[8] = packet_len;

        aes_att_encryption(key.as_ptr(), ivs.as_ptr(), ivs.as_mut_ptr());
        index = 0;
        loop {
            let mut breakit = false;
            loop {
                ivs[(index & 0xf) as usize] = *packet_data.offset(index) ^ ivs[(index & 0xf) as usize];

                if index & 0xf != 0xf && packet_len - 1 != index as u8 {
                    break;
                }
                aes_att_encryption(key.as_ptr(), ivs.as_ptr(), ivs.as_mut_ptr());
                index = index + 1;
                if packet_len <= index as u8 {
                    breakit = true;
                    break;
                }
            }
            if breakit {
                break;
            }

            index = index + 1;
            if index as u8 >= packet_len {
                break;
            }
        }
    }

    if src.len() != 0 {
        return src[0..src.len()] == ivs[0..src.len()];
    }

    return true;
}

pub unsafe fn aes_att_encryption_packet(key: &[u8], ivm: &[u8], src: &mut [u8], packet_data: *mut u8, packet_len: u8) -> bool
{
    let mut encoded = [0u8; 16];
    encoded[0..8].copy_from_slice(&ivm[0..8]);
    encoded[8] = packet_len;

    aes_att_encryption(key.as_ptr(), encoded.as_ptr(), encoded.as_mut_ptr());

    if packet_len != 0 {
        let mut index = 0;
        loop {
            let mut breakit = false;
            loop {
                encoded[index & 0xf] = *packet_data.offset(index as isize) ^ encoded[index & 0xf];
                if index & 0xf != 0xf && packet_len - 1 != index as u8 {
                    break;
                }

                aes_att_encryption(key.as_ptr(), encoded.as_ptr(), encoded.as_mut_ptr());
                index = index + 1;
                if packet_len <= index as u8 {
                    breakit = true;
                    break;
                }
            }
            if breakit {
                break;
            }

            index = index + 1;
            if index as u8 >= packet_len {
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

    if packet_len != 0 {
        let mut index = 0;
        let mut result = [0u8; 16];
        loop {
            if index & 0xf == 0 {
                aes_att_encryption(key.as_ptr(), encoded.as_ptr(), result.as_mut_ptr());
                encoded[0] += 1;
            }
            *packet_data.offset(index as isize) = *packet_data.offset(index as isize) ^ result[index & 0xf];
            index = index + 1;
            if index as u8 >= packet_len {
                break;
            }
        }
    }
    return true;
}