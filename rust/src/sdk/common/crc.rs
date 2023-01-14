#[no_mangle] // required by light_ll
pub unsafe fn crc16 (mut pD: *const u8, len: u16) -> u16
{
    let poly: [u16; 2] = [0, 0xa001];              //0x8005 <==> 0xa001
    let mut crc: u16 = 0xffff;
    // let i,j;

    let mut j = len;
    while j > 0
    {
        let mut ds = *pD;
        pD = pD.offset(1);

        for i in 0..8
        {
            crc = (crc >> 1) ^ poly[((crc ^ (ds as u16)) & 1) as usize];
            ds = ds >> 1;
        }

        j -= 1;
    }

     return crc;
}