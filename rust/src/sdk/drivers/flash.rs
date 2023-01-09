use std::mem::transmute;
use std::ptr::{null, null_mut};
use sdk::drivers::spi::{mspi_ctrl_write, mspi_get, mspi_high, mspi_low, mspi_read, mspi_wait, mspi_write};
use sdk::mcu::clock::sleep_us;
use sdk::mcu::irq_i::{irq_disable, irq_restore};
use sdk::mcu::watchdog::wd_clear;

static PAGE_SIZE : u32 =			   256;
static PAGE_SIZE_OTP : u32 =		   256;
static FLASH_LOCK_EN : u32 =           0;

/**
 * @brief     flash command definition
 */
enum FLASH_CMD {
	//common cmd
	WRITE_CMD						=	0x02,
	READ_CMD						=	0x03,
	WRITE_SECURITY_REGISTERS_CMD	=	0x42,
	READ_SECURITY_REGISTERS_CMD	=	0x48,

	SECT_ERASE_CMD				=	0x20,
	ERASE_SECURITY_REGISTERS_CMD	=	0x44,

	READ_UID_CMD_GD_PUYA_ZB_UT	=	0x4B,	//Flash Type = GD/PUYA/ZB/UT
	READ_UID_CMD_XTX				=	0x5A,	//Flash Type = XTX

	GET_JEDEC_ID					=	0x9F,

	//special cmd
	WRITE_STATUS_CMD_LOWBYTE		=	0x01,
	WRITE_STATUS_CMD_HIGHBYTE		=	0x31,

	READ_STATUS_CMD_LOWBYTE		=	0x05,
	READ_STATUS_CMD_HIGHBYTE		=	0x35,

	WRITE_DISABLE_CMD 			= 	0x04,
	WRITE_ENABLE_CMD 				= 	0x06,
}

/**
 * @brief     flash status type definition
 */
enum FLASH_STATUS {
	TYPE_8BIT_STATUS   			= 0,
	TYPE_16BIT_STATUS_ONE_CMD  	= 1,
	TYPE_16BIT_STATUS_TWO_CMD  	= 2,
}

/**
 * @brief     flash uid cmd definition
 */
enum FLASH_UID_CMD {
	UID_CMD_GD_PUYA   = 0x4b,
	XTX_READ_UID_CMD	= 0x5A,
}

/**
 * @brief	flash capacity definition
 *			Call flash_read_mid function to get the size of flash capacity.
 *			Example is as follows:
 *			unsigned int mid = flash_read_mid();
 *			The value of (mid&0x00ff0000)>>16 reflects flash capacity.
 */
enum FLASH_CAPACITY {
	SIZE_64K 	= 0x10,
	SIZE_128K = 0x11,
	SIZE_256K = 0x12,
	SIZE_512K = 0x13,
	SIZE_1M 	= 0x14,
	SIZE_2M 	= 0x15,
	SIZE_4M 	= 0x16,
	SIZE_8M 	= 0x17,
}

/**
 * @brief	flash voltage definition
 */
enum FLASH_VOLTAGE{
	VOLTAGE_1V95     = 0x07,
	VOLTAGE_1V9      = 0x06,
	VOLTAGE_1V85     = 0x05,
	VOLTAGE_1V8      = 0x04,
	VOLTAGE_1V75     = 0x03,
	VOLTAGE_1V7      = 0x02,
	VOLTAGE_1V65     = 0x01,
	VOLTAGE_1V6      = 0x00,
}

/**
 * @brief		This function to determine whether the flash is busy..
 * @return		1:Indicates that the flash is busy. 0:Indicates that the flash is free
 */
#[inline(always)]
fn flash_is_busy() -> bool {
	return mspi_read() & 0x01 != 0;		//the busy bit, pls check flash spec
}

/**
 * @brief     This function serves to wait flash done.(make this a asynchorous version).
 * @return    none.
 */
#[inline(never)]
fn flash_wait_done__attribute_ram_code()
{
	sleep_us(100);
	flash_send_cmd__attribute_ram_code(FLASH_CMD::READ_STATUS_CMD_LOWBYTE);

	for i in 0 .. 10000000
	{
		if !flash_is_busy() {
			break;
		}
	}
	mspi_high();
}

/**
 * @brief		This function serves to set flash write command.
 * @param[in]	cmd	- set command.
 * @return		none.
 */
#[inline(never)]
fn flash_send_cmd__attribute_ram_code(cmd: FLASH_CMD){
	mspi_high();
	sleep_us(1);
	mspi_low();
	mspi_write(cmd as u8);
	mspi_wait();
}

/**
 * @brief		This function serves to send flash address.
 * @param[in]	addr	- the flash address.
 * @return		none.
 */
#[inline(never)]
pub fn flash_send_addr__attribute_ram_code(addr: u32){
	mspi_write(((addr>>16) & 0xff) as u8 );
	mspi_wait();
	mspi_write(((addr>>8) & 0xff) as u8);
	mspi_wait();
	mspi_write((addr & 0xff) as u8);
	mspi_wait();
}

/**
 * @brief 		This function is used to read data from flash or read the status of flash.
 * @param[in]   cmd			- the read command.
 * @param[in]   addr		- starting address.
 * @param[in]   addr_en		- whether need to send an address.
 * @param[in]   dummy_cnt	- the length(in byte) of dummy.
 * @param[out]  data		- the start address of the data buffer.
 * @param[in]   data_len	- the length(in byte) of content needs to read out.
 * @return 		none.
 */
#[inline(never)]
fn flash_mspi_read_ram__attribute_ram_code(cmd: FLASH_CMD, addr: u32, addr_en: u8, dummy_cnt: u8, data: *mut u8, data_len: u32)
{
	let r = irq_disable();

	flash_send_cmd__attribute_ram_code(cmd);
	if addr_en != 0
	{
		flash_send_addr__attribute_ram_code(addr);
	}
	for _ in 0..dummy_cnt
	{
		mspi_write(0x00);		/* dummy */
		mspi_wait();
	}
	mspi_write(0x00);			/* to issue clock */
	mspi_wait();
	mspi_ctrl_write(0x0a);		/* auto mode */
	mspi_wait();

	for i in 0..data_len
	{
		unsafe { *data.offset(i as isize) = mspi_get(); }
		mspi_wait();
	}
	mspi_high();

	irq_restore(r);
}

/**
 * @brief 		This function is used to write data or status to flash.
 * @param[in]   cmd			- the write command.
 * @param[in]   addr		- starting address.
 * @param[in]   addr_en		- whether need to send an address.
 * @param[out]  data		- the start address of the data buffer.
 * @param[in]   data_len	- the length(in byte) of content needs to read out.
 * @return 		none.
 * @note		important:  "data" must not reside at flash, such as constant string.If that case, pls copy to memory first before write.
 */
#[inline(never)]
fn flash_mspi_write_ram__attribute_ram_code(cmd: FLASH_CMD, addr: u32, addr_en: u8, data: *const u8, data_len: u32)
{
	let r = irq_disable();

	flash_send_cmd__attribute_ram_code(FLASH_CMD::WRITE_ENABLE_CMD);
	flash_send_cmd__attribute_ram_code(cmd);
	if addr_en != 0
	{
		flash_send_addr__attribute_ram_code(addr);
	}
	for i in 0 .. data_len
	{
		unsafe { mspi_write(*data.offset(i as isize)) }
		mspi_wait();
	}
	mspi_high();
	flash_wait_done__attribute_ram_code();

	irq_restore(r);
}

/**
 * @brief	  	This function serves to read MID of flash(MAC id). Before reading UID of flash,
 * 				you must read MID of flash. and then you can look up the related table to select
 * 				the idcmd and read UID of flash.
 * @return    	MID of the flash(4 bytes).
 * @note        Attention: Before calling the FLASH function, please check the power supply voltage of the chip.
 *              Only if the detected voltage is greater than the safe voltage value, the FLASH function can be called.
 *              Taking into account the factors such as power supply fluctuations, the safe voltage value needs to be greater
 *              than the minimum chip operating voltage. For the specific value, please make a reasonable setting according
 *              to the specific application and hardware circuit.
 *
 *              Risk description: When the chip power supply voltage is relatively low, due to the unstable power supply,
 *              there may be a risk of error in the operation of the flash (especially for the write and erase operations.
 *              If an abnormality occurs, the firmware and user data may be rewritten, resulting in the final Product failure)
 */
pub fn flash_read_mid() -> u32
{
	let mut flash_mid: [u8; 4] = [0; 4];
	flash_mspi_read_ram__attribute_ram_code(FLASH_CMD::GET_JEDEC_ID, 0, 0, 0, flash_mid.as_mut_ptr(), 3);
	let mut flash_mid = unsafe { std::mem::transmute::<[u8; 4], u32>(flash_mid) }.to_be();
	/*
	 * The mids of GD25LD80C and GD25LE80C are both 0x1460c8, but the status register of GD25LD80C is 8 bits,
	 * and the status register of GD25LE80C is 16 bits. The functions of the two chips are different.
	 * The software detection method is to read SFDP Signature. If it is 50444653H, it is GD25LE80C,
	 * if it is all zeros, it is GD25LD80C.
	 */
	if flash_mid == 0x1460c8
	{
		let mut buf: [u8; 4] = [0; 4];
		flash_mspi_read_ram__attribute_ram_code(FLASH_CMD::READ_UID_CMD_XTX, 0x00, 1, 1, buf.as_mut_ptr(), 4);
		if (buf[0] == 0x53) && (buf[1] == 0x46) && (buf[2] == 0x44) && (buf[3] == 0x50)
		{
			flash_mid = 0x011460c8;
		}
	}
	return flash_mid;
}

/**
 * @brief 		This function reads the content from a page to the buf.
 * @param[in]   addr	- the start address of the page.
 * @param[in]   len		- the length(in byte) of content needs to read out from the page.
 * @param[out]  buf		- the start address of the buffer.
 * @return 		none.
 * @note        Attention: Before calling the FLASH function, please check the power supply voltage of the chip.
 *              Only if the detected voltage is greater than the safe voltage value, the FLASH function can be called.
 *              Taking into account the factors such as power supply fluctuations, the safe voltage value needs to be greater
 *              than the minimum chip operating voltage. For the specific value, please make a reasonable setting according
 *              to the specific application and hardware circuit.
 *
 *              Risk description: When the chip power supply voltage is relatively low, due to the unstable power supply,
 *              there may be a risk of error in the operation of the flash (especially for the write and erase operations.
 *              If an abnormality occurs, the firmware and user data may be rewritten, resulting in the final Product failure)
 */
pub fn flash_read_page(addr: u32, len: u32, buf: *mut u8)
{
	flash_mspi_read_ram__attribute_ram_code(FLASH_CMD::READ_CMD, addr, 1, 0, buf, len);
}

/**
 * @brief 		This function writes the buffer's content to the flash.
 * @param[in]   addr	- the start address of the area.
 * @param[in]   len		- the length(in byte) of content needs to write into the flash.
 * @param[in]   buf		- the start address of the content needs to write into.
 * @return 		none.
 * @note        the funciton support cross-page writing,which means the len of buf can bigger than 256.
 *
 *              Attention: Before calling the FLASH function, please check the power supply voltage of the chip.
 *              Only if the detected voltage is greater than the safe voltage value, the FLASH function can be called.
 *              Taking into account the factors such as power supply fluctuations, the safe voltage value needs to be greater
 *              than the minimum chip operating voltage. For the specific value, please make a reasonable setting according
 *              to the specific application and hardware circuit.
 *
 *              Risk description: When the chip power supply voltage is relatively low, due to the unstable power supply,
 *              there may be a risk of error in the operation of the flash (especially for the write and erase operations.
 *              If an abnormality occurs, the firmware and user data may be rewritten, resulting in the final Product failure)
 */
pub fn flash_write_page(mut addr: u32, mut len: u32, buf: *const u8)
{
	let mut ns = PAGE_SIZE - (addr&(PAGE_SIZE - 1));
	let mut nw = 0;
	let mut ptr: u32 = buf as u32;

	loop {
		nw = if len > ns {ns} else {len};
		flash_mspi_write_ram__attribute_ram_code(FLASH_CMD::WRITE_CMD, addr, 1, ptr as *const u8, nw);
		ns = PAGE_SIZE;
		addr += nw;
		ptr += nw;
		len -= nw;
		if len <= 0 {
			break;
		}
	}
}

/**
 * @brief 		This function serves to erase a sector.
 * @param[in]   addr	- the start address of the sector needs to erase.
 * @return 		none.
 * @note        Attention: Before calling the FLASH function, please check the power supply voltage of the chip.
 *              Only if the detected voltage is greater than the safe voltage value, the FLASH function can be called.
 *              Taking into account the factors such as power supply fluctuations, the safe voltage value needs to be greater
 *              than the minimum chip operating voltage. For the specific value, please make a reasonable setting according
 *              to the specific application and hardware circuit.
 *
 *              Risk description: When the chip power supply voltage is relatively low, due to the unstable power supply,
 *              there may be a risk of error in the operation of the flash (especially for the write and erase operations.
 *              If an abnormality occurs, the firmware and user data may be rewritten, resulting in the final Product failure)
 */
pub fn flash_erase_sector(addr: u32)
{
	wd_clear();

	flash_mspi_write_ram__attribute_ram_code(FLASH_CMD::SECT_ERASE_CMD, addr, 1, null_mut(), 0);
}