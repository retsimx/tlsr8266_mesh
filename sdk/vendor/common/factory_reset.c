/********************************************************************************************************
 * @file     factory_reset.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 *
 * @par      Copyright (c) Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *           
 *			 The information contained herein is confidential and proprietary property of Telink 
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms 
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai) 
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in. 
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this 
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided. 
 *           
 *******************************************************************************************************/
#include "../../proj/tl_common.h"
#include "common.h"

FLASH_ADDRESS_EXTERN;

//////////////////Factory Reset///////////////////////////////////////////////////////////////////////
extern void rf_led_ota_ok(void);
void rf_led_ota_error(void);
extern void light_sw_reboot(void);

static int adr_reset_cnt_idx = 0;

#if 1   // org mode
static int reset_cnt = 0;

#define SERIALS_CNT                     (5)   // must less than 7

u8 factory_reset_serials[SERIALS_CNT * 2]   = { 0, 3,    // [0]:must 0
                                                0, 3,    // [2]:must 0
                                                0, 3,    // [4]:must 0
                                                0, 3,
                                                0, 3};

#define RESET_CNT_RECOUNT_FLAG          0
#define RESET_FLAG                      0x80

void	reset_cnt_clean ()
{
	if (adr_reset_cnt_idx < 3840)
	{
		return;
	}
	flash_erase_sector (flash_adr_reset_cnt);
	adr_reset_cnt_idx = 0;
}

void write_reset_cnt (u8 cnt)
{
	flash_write_page (flash_adr_reset_cnt + adr_reset_cnt_idx, 1, (u8 *)(&cnt));
}

void clear_reset_cnt ()
{
    write_reset_cnt(RESET_CNT_RECOUNT_FLAG);
}

int reset_cnt_get_idx ()		//return 0 if unconfigured
{
	u8 *pf = (u8 *)flash_adr_reset_cnt;
	for (adr_reset_cnt_idx=0; adr_reset_cnt_idx<4096; adr_reset_cnt_idx++)
	{
	    u8 restcnt_bit = pf[adr_reset_cnt_idx];
		if (restcnt_bit != RESET_CNT_RECOUNT_FLAG)	//end
		{
        	if(((u8)(~(BIT(0)|BIT(1)|BIT(2)|BIT(3))) == restcnt_bit)  // the fourth not valid
        	 ||((u8)(~(BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5))) == restcnt_bit)){  // the fifth not valid
                clear_reset_cnt();
            }else{
			    break;
			}
		}
	}

    reset_cnt_clean();
    
	return 1;
}

u8 get_reset_cnt_bit ()
{
	if (adr_reset_cnt_idx < 0)
	{
	    reset_cnt_clean();
		return 0;
	}
	
	u8 reset_cnt;
	flash_read_page(flash_adr_reset_cnt + adr_reset_cnt_idx, 1, &reset_cnt);
	return reset_cnt;
}

void increase_reset_cnt ()
{
	u8 restcnt_bit = get_reset_cnt_bit();
	foreach(i,8){
        if(restcnt_bit & BIT(i)){
            if(i < 3){
                reset_cnt = i;
            }else if(i < 5){
                reset_cnt = 3;
            }else if(i < 7){
                reset_cnt = 4;
            }
            
            restcnt_bit &= ~(BIT(i));
            write_reset_cnt(restcnt_bit);
            break;
        }
	}
}

int factory_reset_handle ()
{
    reset_cnt_get_idx();   
    u8 restcnt_bit; 
    restcnt_bit = get_reset_cnt_bit();
	if(restcnt_bit == RESET_FLAG){
        irq_disable();
        factory_reset();
        rf_led_ota_ok();
	    light_sw_reboot();
	}else{
        increase_reset_cnt();
	}
	return 0;
}

int factory_reset_cnt_check ()
{
    static u8 clear_st = 3;
    static u32 reset_check_time;

	if(0 == clear_st) return 0;

	if(3 == clear_st){
        clear_st--;
        reset_check_time = factory_reset_serials[reset_cnt*2];
    }
    
	if((2 == clear_st) && clock_time_exceed(0, reset_check_time*1000*1000)){
	    clear_st--;
	    reset_check_time = factory_reset_serials[reset_cnt*2 + 1];
	    if(3 == reset_cnt || 4 == reset_cnt){
            increase_reset_cnt();
        }
	}
    
	if((1 == clear_st) && clock_time_exceed(0, reset_check_time*1000*1000)){
	    clear_st = 0;
        clear_reset_cnt();
	}
	
	return 0;
}

#else

/****************************************
new factory reset:
user can change any one of factory_reset_serials, and also can change SERIALS_CNT
*****************************************/

#define SERIALS_CNT                     (5*2)   // should less than 100

u8 factory_reset_serials[SERIALS_CNT]   =     { 0, 3,
                                                0, 3,
                                                0, 3,
                                                3, 10,
                                                3, 10,};

#define RESET_CNT_INVALID               0
#define RESET_TRIGGER_VAL               SERIALS_CNT

void	reset_cnt_clean ()
{
	if (adr_reset_cnt_idx < 3840)
	{
		return;
	}
	flash_erase_sector (flash_adr_reset_cnt);
	adr_reset_cnt_idx = 0;
}

void write_reset_cnt (u8 cnt) // reset cnt value from 1 to 254, 0 is invalid cnt
{
    reset_cnt_clean ();
	flash_write_page (flash_adr_reset_cnt + adr_reset_cnt_idx, 1, (u8 *)(&cnt));
}

void clear_reset_cnt ()
{
    write_reset_cnt(RESET_CNT_INVALID);
}

void reset_cnt_get_idx ()		//return 0 if unconfigured
{
	u8 *pf = (u8 *)flash_adr_reset_cnt;
	for (adr_reset_cnt_idx=0; adr_reset_cnt_idx<4096; adr_reset_cnt_idx++)
	{
	    u8 restcnt = pf[adr_reset_cnt_idx];
		if (restcnt != RESET_CNT_INVALID)	//end
		{
		    if(0xFF == restcnt){
		        // do nothing
		    }else if((restcnt > RESET_TRIGGER_VAL) || (restcnt & BIT(0))){   // invalid state
                clear_reset_cnt();
                continue;
            }
			break;
		}
	}
}

u8 get_reset_cnt () // reset cnt value from 1 to 254, 0 is invalid cnt
{
	u8 reset_cnt;
	flash_read_page(flash_adr_reset_cnt + adr_reset_cnt_idx, 1, &reset_cnt);
	return reset_cnt;
}

void increase_reset_cnt ()
{
	u8 reset_cnt = get_reset_cnt();
	if(0xFF == reset_cnt){
	    reset_cnt = 0;
	}else if(reset_cnt){
	    clear_reset_cnt();      // clear current BYTE and then use next BYTE
        adr_reset_cnt_idx++;
	}
	
	reset_cnt++;
	write_reset_cnt(reset_cnt);
}

int factory_reset_handle ()
{
    reset_cnt_get_idx();   
	if(get_reset_cnt() == RESET_TRIGGER_VAL){
        irq_disable();
        factory_reset();
        rf_led_ota_ok();
	    light_sw_reboot();
	}else{
        increase_reset_cnt();
	}
	return 0;
}

int factory_reset_cnt_check ()
{
    static u8 clear_st = 3;
    static u32 reset_check_time;

	if(0 == clear_st) return 0;

	if(3 == clear_st){
        clear_st--;
        reset_check_time = factory_reset_serials[get_reset_cnt() - 1];
    }
    
	if((2 == clear_st) && clock_time_exceed(0, reset_check_time*1000*1000)){
	    clear_st--;
        increase_reset_cnt();
	    reset_check_time = factory_reset_serials[get_reset_cnt() - 1];
	}
    
	if((1 == clear_st) && clock_time_exceed(0, reset_check_time*1000*1000)){
	    clear_st = 0;
        clear_reset_cnt();
	}
	
	return 0;
}

#endif

#if (0 == FLASH_1M_ENABLE) // 512k flash
int factory_reset(){
	u8 r = irq_disable ();
    //clear_reset_cnt();

	for(int i = 1; i < (FLASH_ADR_PAR_MAX - CFG_SECTOR_ADR_MAC_CODE) / 4096; ++i){  // can't use global variate.
	    u32 adr = CFG_SECTOR_ADR_MAC_CODE + i*0x1000;
	    if(FLASH_ADR_RESET_CNT != adr){
		    flash_erase_sector(adr);
		}
	}
	
#if DUAL_MODE_ADAPT_EN
	if(DUAL_MODE_NOT_SUPPORT != dual_mode_state){   // dual_mode_state have been init before
	    set_firmware_type_init();
	}
#endif

    flash_erase_sector(FLASH_ADR_RESET_CNT); // at last should be better, when power off during factory reset erase.
    
    irq_restore(r);
	return 0; 
}
#else // 1 M flash
int factory_reset(){
	u8 r = irq_disable ();
    //clear_reset_cnt();

	for(int i = 0; i < (FLASH_ADR_PAR_MAX - FLASH_ADR_PAIRING) / 4096; ++i){  // can't use global variate.
	    u32 adr = FLASH_ADR_PAIRING + i*0x1000;
	    if(FLASH_ADR_RESET_CNT != adr){
		    flash_erase_sector(adr);
		}
	}
	
#if DUAL_MODE_ADAPT_EN
	if(DUAL_MODE_NOT_SUPPORT != dual_mode_state){   // dual_mode_state have been init before
	    set_firmware_type_init();
	}
#endif

    flash_erase_sector(FLASH_ADR_RESET_CNT); // at last should be better, when power off during factory reset erase.
    
    irq_restore(r);
	return 0; 
}
#endif

enum{
	KICKOUT_OUT_OF_MESH = 0,
	KICKOUT_DEFAULT_NAME,
	KICKOUT_MODE_MAX,
};

void kick_out(u8 par){
    factory_reset();

    if(KICKOUT_OUT_OF_MESH == par){
        u8 buff[16];
        memset (buff, 0, 16);
        extern u8 pair_config_mesh_ltk[];
        memcpy (buff, pair_config_mesh_ltk, 16);
        flash_write_page (flash_adr_pairing + 48, 16, buff);
        memset (buff, 0, 16);
        memcpy (buff, MESH_PWD, strlen(MESH_PWD) > 16 ? 16 : strlen(MESH_PWD));
	    encode_password(buff);
        flash_write_page (flash_adr_pairing + 32, 16, buff);
        memset (buff, 0, 16);
        memcpy (buff, OUT_OF_MESH, strlen(OUT_OF_MESH) > 16 ? 16 : strlen(OUT_OF_MESH));
        flash_write_page (flash_adr_pairing + 16, 16, buff);
        memset (buff, 0, 16);
        buff[0] = PAIR_VALID_FLAG;
        if(pair_config_pwd_encode_enable){
            buff[15] = PAIR_VALID_FLAG;
        }
		if(mesh_pair_enable){
	        get_mac_en = 1;
	        buff[1] = get_mac_en;
        }
        flash_write_page (flash_adr_pairing, 16, buff);
    }
    
    rf_led_ota_ok();
}

