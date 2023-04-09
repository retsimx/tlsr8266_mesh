use core::slice;
use crate::{no_mangle_fn, pub_mut};
use crate::sdk::light::{get_pair_config_pwd_encode_enable, get_pair_config_pwd_encode_sk};
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
    if *get_pair_config_pwd_encode_enable() {
        unsafe {
            aes_att_encryption(get_pair_config_pwd_encode_sk().as_ptr(), password.as_ptr(), password.as_mut_ptr());
        }
    }
}

pub fn decode_password(password: &mut [u8])
{
    if *get_pair_config_pwd_encode_enable() {
        unsafe {
            aes_att_decryption(get_pair_config_pwd_encode_sk().as_ptr(), password.as_ptr(), password.as_mut_ptr());
        }
    }
}