#[no_mangle] // required by light_ll
pub fn crc16 (mut pD: &[u8]) -> u16
{
    let poly: [u16; 2] = [0, 0xa001];              //0x8005 <==> 0xa001
    let mut crc: u16 = 0xffff;
    // let i,j;

    let mut j = pD.len();
    while j > 0
    {
        let mut ds = pD[pD.len()-j];

        for i in 0..8
        {
            crc = (crc >> 1) ^ poly[((crc ^ (ds as u16)) & 1) as usize];
            ds = ds >> 1;
        }

        j -= 1;
    }

     return crc;
}

#[cfg(test)]
mod tests {
    use sdk::common::crc::crc16;

    #[test]
    fn test_crc16() {
        assert_eq!(crc16(&[0, 1, 2, 3, 4]), 3973);
        assert_eq!(crc16(&[0x66, 0x97, 0xf2, 0x29, 0x5f]), 10238);
    }
}