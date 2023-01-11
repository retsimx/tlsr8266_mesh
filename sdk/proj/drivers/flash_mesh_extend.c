/********************************************************************************************************
 * @file	flash_mesh_extend.c
 *
 * @brief	This is the source file for B85
 *
 * @author	Driver Group
 * @date	May 8,2018
 *
 * @par     Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions are met:
 *
 *              1. Redistributions of source code must retain the above copyright
 *              notice, this list of conditions and the following disclaimer.
 *
 *              2. Unless for usage inside a TELINK integrated circuit, redistributions
 *              in binary form must reproduce the above copyright notice, this list of
 *              conditions and the following disclaimer in the documentation and/or other
 *              materials provided with the distribution.
 *
 *              3. Neither the name of TELINK, nor the names of its contributors may be
 *              used to endorse or promote products derived from this software without
 *              specific prior written permission.
 *
 *              4. This software, with or without modification, must only be used with a
 *              TELINK integrated circuit. All other usages are subject to written permission
 *              from TELINK and different commercial license may apply.
 *
 *              5. Licensee shall be solely responsible for any claim to the extent arising out of or
 *              relating to such deletion(s), modification(s) or alteration(s).
 *
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *          ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *          WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *          DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 *          DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *          LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *          ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *          (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *          SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************************************/

#include "../tl_common.h"
#include "../drivers/spi.h"
#include "flash.h"
#include "spi_i.h"
#include "../mcu/watchdog_i.h"


extern u32 flash_adr_mac; // CFG_SECTOR_ADR_MAC_CODE
u32 flash_sector_calibration = CFG_SECTOR_ADR_CALIBRATION_CODE;

#if AUTO_ADAPT_MAC_ADDR_TO_FLASH_TYPE_EN

void blc_readFlashSize_autoConfigCustomFlashSector(void)
{
	u8 *temp_buf;
	unsigned int mid = flash_read_mid();
	temp_buf = (u8 *)&mid;
	u8	flash_cap = temp_buf[2];

    if(CFG_ADR_MAC_512K_FLASH == CFG_SECTOR_ADR_MAC_CODE){
    	if(flash_cap == FLASH_SIZE_1M){
    	    #define MAC_SIZE_CHECK      (6)
    	    u8 mac_null[MAC_SIZE_CHECK] = {0xff,0xff,0xff,0xff,0xff,0xff};
    	    u8 mac_512[MAC_SIZE_CHECK], mac_1M[MAC_SIZE_CHECK];
    	    flash_read_page(CFG_ADR_MAC_512K_FLASH, MAC_SIZE_CHECK, mac_512);
    	    flash_read_page(CFG_ADR_MAC_1M_FLASH, MAC_SIZE_CHECK, mac_1M);
    	    if((0 == memcmp(mac_512,mac_null, MAC_SIZE_CHECK))
    	     &&(0 != memcmp(mac_1M,mac_null, MAC_SIZE_CHECK))){
        		flash_adr_mac = CFG_ADR_MAC_1M_FLASH;
        		flash_sector_calibration = CFG_ADR_CALIBRATION_1M_FLASH;
    		}
    	}
	}else if(CFG_ADR_MAC_1M_FLASH == CFG_SECTOR_ADR_MAC_CODE){
	    if(flash_cap != FLASH_SIZE_1M){
            while(1){ // please check your Flash size
                #if(MODULE_WATCHDOG_ENABLE)
                wd_clear();
                #endif
            }
		}
	}

	flash_set_capacity(flash_cap);
}
#endif



#if (__PROJECT_LIGHT_SWITCH__ || (__PROJECT_MASTER_LIGHT_8266__ || __PROJECT_MASTER_LIGHT_8267__))
#define FLASH_PROTECT_ENABLE    0
#else
#define FLASH_PROTECT_ENABLE    0
#endif

#if (!__PROJECT_OTA_BOOT__)
#if 1
enum{
	FLASH_NONE	                    =	0,
	FLASH_GD25Q40_0x00000_0x1ffff,
	FLASH_GD25Q40_0x00000_0x3ffff,
	FLASH_GD25Q40_0x00000_0x6ffff,
	FLASH_GD25Q40_0x40000_0x7ffff,
	
	FLASH_MD25D40_0x00000_0x3ffff  = 0x11,
	FLASH_MD25D40_0x00000_0x6ffff,
};

enum{
	FLASH_ID_GD25Q40 = 0x001340c8,
	FLASH_ID_MD25D40 = 0x00134051,
};
#endif


FLASH_ADDRESS_EXTERN;
extern u32  ota_program_offset;

static u32 flash_id = 0;

void flash_get_id(){
    flash_id = flash_read_mid();
}

u32 flash_get_jedec_id()    // TODO, to delete later
{
    return flash_read_mid();
}


#if (FLASH_PROTECT_ENABLE)
unsigned char flash_read_status(unsigned char cmd);
void flash_write_status(flash_status_typedef_e type , unsigned short data);


STATIC_ASSERT(CFG_SECTOR_ADR_MAC_CODE >= 0x70000);
static u16 T_flash_status = -1;

u16 flash_status_read(){
	unsigned char status_low = flash_read_status(FLASH_READ_STATUS_CMD_LOWBYTE);
	unsigned char status_high = 0;
	if(FLASH_ID_GD25Q40 == flash_id){
	    status_high = flash_read_status(FLASH_READ_STATUS_CMD_HIGHBYTE);
	}
	return (status_low | (status_high << 8));
}

void flash_status_write(u16 status){
	if(FLASH_ID_GD25Q40 == flash_id){
        flash_write_status(FLASH_TYPE_16BIT_STATUS_ONE_CMD, status);
	}else{
        flash_write_status(FLASH_TYPE_8BIT_STATUS, status);
	}
}

int flash_protect_GD25Q40B(u8 idx){
	u8 r = irq_disable();
	int ret = 0;
	u16 status;
    #if 1   // read status before write
	T_flash_status = status = flash_status_read();
	
	status &= (u16)(~((u16)(0x7c) | (BIT(6) << 8)));
	#else   // use default 0
	status = 0;
	#endif
	
	if(FLASH_NONE == idx){
	
	}else if(FLASH_GD25Q40_0x00000_0x1ffff == idx){
        status |= (u16)(0b01010 << 2);              //0x0028;//
	}else if(FLASH_GD25Q40_0x00000_0x3ffff == idx){
        status |= (u16)(0b01011 << 2);              //0x002c;//
	}else if(FLASH_GD25Q40_0x00000_0x6ffff == idx){
        status |= (u16)((0b00001 << 2) | (BIT(6)<< 8));   //0x4004;//
	}else if(FLASH_GD25Q40_0x40000_0x7ffff == idx){
        status |= (u16)(0b00011 << 2);              //0x000c;//
    }else{
        ret = -1;
    	irq_restore(r);
    	return ret;
    }

    if(T_flash_status == status){
        ret = 0;
    	irq_restore(r);
    	return ret;
    }
    
    flash_status_write(status);

	#if 1   // check
	sleep_us(100);
	T_flash_status = flash_status_read();
	if((T_flash_status & ((u16)((u16)(0x7c) | (BIT(6) << 8))))
	 != (status & ((u16)((u16)(0x7c) | (BIT(6) << 8))))){
        ret = -1;
	}
	#endif
	
	irq_restore(r);
	return ret;
}

int flash_protect_MD25D40(u8 idx){
	u8 r = irq_disable();
	int ret = 0;
	u16 status;
    #if 1   // read status before write
	T_flash_status = status = flash_status_read();
	
	status &= (u16)(~((u16)(0x1c)));
	#else   // use default 0
	status = 0;
	#endif
	
	if(FLASH_NONE == idx){
	
	}else if(FLASH_MD25D40_0x00000_0x3ffff == idx){
        status |= (u16)(0b110 << 2);              //0x0018;//
	}else if(FLASH_MD25D40_0x00000_0x6ffff == idx){
        status |= (u16)(0b100 << 2);              //0x0010;//
	}else{
        ret = -1;
    	irq_restore(r);
    	return ret;
    }

    if(T_flash_status == status){
        ret = 0;
    	irq_restore(r);
    	return ret;
    }
    
    flash_status_write(status);

	#if 1   // check
	sleep_us(100);
	T_flash_status = flash_status_read();
	if((T_flash_status & ((u16)(0x1c)))
	 != (status & ((u16)(0x7c)))){
        ret = -1;
	}
	#endif
	
	irq_restore(r);
	return ret;
}

int flash_protect_cmd(u8 idx){
    int ret = 0;
    if(FLASH_ID_GD25Q40 == flash_id){
        ret = flash_protect_GD25Q40B(idx);
    }else if(FLASH_ID_MD25D40 == flash_id){
        ret = flash_protect_MD25D40(idx);
    }else{
        ret = -1;
    }

    return ret;
}

int flash_protect(u8 idx){
    if(0 == flash_protect_en){
        return 0;
    }
    
    for(int i = 0; i < 5; ++i){
		if(0 == flash_protect_cmd(idx)){
			return 0;
		}
	}

	return -1;
}

int flash_protect_disable(){
    return flash_protect(FLASH_NONE);
}

int flash_protect_up256k(){
    if(FLASH_ID_GD25Q40 == flash_id){
        return flash_protect(FLASH_GD25Q40_0x00000_0x3ffff);
    }else if(FLASH_ID_MD25D40 == flash_id){
        return flash_protect(FLASH_MD25D40_0x00000_0x3ffff);
    }

    return -1;
}

int flash_protect_down256k(){
    if(FLASH_ID_GD25Q40 == flash_id){
        return flash_protect(FLASH_GD25Q40_0x40000_0x7ffff);
    }

    return -1;
}

int flash_protect_8266_normal(){
    if(FLASH_ID_GD25Q40 == flash_id){
        return flash_protect(FLASH_GD25Q40_0x00000_0x1ffff);
    }else if(FLASH_ID_MD25D40 == flash_id){
        if(0x30000 == flash_adr_mac){           // can not be protect
            return flash_protect(FLASH_NONE);
        }else{
            return flash_protect(FLASH_MD25D40_0x00000_0x3ffff);
        }
    }

    return -1;
}

int flash_protect_8267_normal(){    
    if(FLASH_ID_GD25Q40 == flash_id){
        if(0 == ota_program_offset){
            if(0x30000 == flash_adr_mac){
                return flash_protect(FLASH_GD25Q40_0x40000_0x7ffff);
            }else if(0x70000 == flash_adr_mac){
                return flash_protect(FLASH_GD25Q40_0x00000_0x6ffff);
            }
        }else{
            return flash_protect(FLASH_GD25Q40_0x00000_0x1ffff);
        }
    }else if(FLASH_ID_MD25D40 == flash_id){
        return flash_protect(FLASH_MD25D40_0x00000_0x6ffff);
    }

    return -1;
}

#if 0
int flash_protect_8269_normal(){        // same with 8267
    return flash_protect_8267_normal();
}
#endif

int flash_protect_8266_OTA_clr_flag(){    // clear flag of 0x3A000
    if(FLASH_ID_GD25Q40 == flash_id){
        return flash_protect(FLASH_GD25Q40_0x00000_0x3ffff);
    }else if(FLASH_ID_MD25D40 == flash_id){
        return flash_protect(FLASH_NONE);
    }

    return -1;
}

int flash_protect_8267_OTA_clr_flag(){
    if(FLASH_ID_GD25Q40 == flash_id){
        if(0 == ota_program_offset){
            return flash_protect(FLASH_GD25Q40_0x00000_0x3ffff);
        }else{
            return flash_protect(FLASH_GD25Q40_0x40000_0x7ffff);
        }
    }else if(FLASH_ID_MD25D40 == flash_id){
        return flash_protect(FLASH_NONE);
    }

    return -1;
}

int flash_unprotect_OTA_start(){
    if(FLASH_ID_GD25Q40 == flash_id){
        if(0x70000 == flash_adr_mac){
            #if (MCU_CORE_TYPE == MCU_CORE_8266)
            // keep normal
            #else
            return flash_protect(FLASH_NONE);
            #endif
        }else{
            // keep normal
        }
    }else if(FLASH_ID_MD25D40 == flash_id){
        return flash_protect(FLASH_NONE);
    }

    return -1;
}

int flash_protect_OTA_copy(){
    if(FLASH_ID_GD25Q40 == flash_id){
        return flash_protect(FLASH_GD25Q40_0x40000_0x7ffff);
    }else if(FLASH_ID_MD25D40 == flash_id){
        return flash_protect(FLASH_NONE);
    }

    return -1;
}

int flash_protect_OTA_data_erase(){
    #if (MCU_CORE_TYPE == MCU_CORE_8266)
    return flash_protect_8266_normal();
    #else
    return flash_protect(FLASH_NONE);
    #endif

    return -1;
}


void flash_protect_debug(){
#if 1
    static u8 T_protect_idx = 0;
    if(T_protect_idx){
        if(T_protect_idx == 0xff){
            T_protect_idx = 0;       // disprotect
        }
        flash_protect(T_protect_idx);
        T_protect_idx = 0;
    }

    #if (!__PROJECT_LIGHT_SWITCH__ && !__PROJECT_LPN__ && !__PROJECT_8263_SWITCH__ && !__PROJECT_8368_SWITCH__)
	static u32 tick;
	if (clock_time_exceed (tick, 40000))
	{
    	tick = clock_time();
    	static u8 st = 0;
    	u8 s = (!(gpio_read (GPIO_PD4))) || (!(gpio_read(GPIO_PD5))) || (!(gpio_read(GPIO_PC5))) || (!(gpio_read(GPIO_PC6)));
    	if ((!st) & s)
    	{
    	static u32 T_button;  T_button++;
            flash_protect(0);   // disprotect
    	}
    	st = s;
	}
	#endif
#endif    
}
#else
int flash_protect_8266_normal(){return -1;}
int flash_protect_8267_normal(){return -1;}
int flash_protect_8266_OTA_clr_flag(){return -1;}
int flash_protect_8267_OTA_clr_flag(){return -1;}
int flash_unprotect_OTA_start(){return -1;}
int flash_protect_OTA_copy(){return -1;}
int flash_protect_OTA_data_erase(){return -1;}
#endif
#endif



