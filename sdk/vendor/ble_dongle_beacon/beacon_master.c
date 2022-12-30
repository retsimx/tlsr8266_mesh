/********************************************************************************************************
 * @file     beacon_master.c 
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
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../../proj_lib/ble_ll/ble_ll.h"
#include "../../proj_lib/light_ll/light_frame.h"
#include "../common/rf_frame.h"
#include "trace.h"
#include "../common/common.h"


#define UART_ENABLE             1

#if UART_ENABLE
#include "../../proj/drivers/uart.h"

#define UART_DATA_LEN    108      // data max 252
typedef struct{
    u32 len;        // data max 252
    u8 data[UART_DATA_LEN];
}uart_data_t;
STATIC_ASSERT((sizeof(uart_data_t) % 16) == 0);

uart_data_t T_txdata_user;
uart_data_t T_txdata_buf;      // not for user

uart_data_t T_rxdata_user;
uart_data_t T_rxdata_buf;   // data max 252, user must copy rxdata to other Ram,but not use directly
unsigned char uart_rx_true;
#endif

///////////////////////////////////////////////////////////////////////////
rf_custom_master_dat_t 	rf_custom_dat;

int					conn_interval = 24;				// 6 * 1.25 = 7.5 ms
int					conn_timeout = 1000;			// 1 second
int					conn_start = 1;
void irq_timer0(void){
}
void irq_timer1(void){
}

void gpio_irq_user_handle(void){
	return;
}

void gpio_risc0_user_handle(void){
	return;
}

void gpio_risc1_user_handle(void){
	return;
}

void gpio_risc2_user_handle(void){
	return;
}

/////////////////// rf interrupt handler ///////////////////////////
_attribute_ram_code_ void irq_handler(void)
{
	irq_ble_master_handler ();

#if UART_ENABLE
    static unsigned char enterRXIrq,enterTXIrq;
    unsigned char irqS = uart_IRQSourceGet();
    if(irqS & UARTRXIRQ_MASK){
        uart_rx_true = 1;
        enterRXIrq++;
    }

    if(irqS & UARTTXIRQ_MASK){
        uart_clr_tx_busy_flag();
        enterTXIrq++;
    }
#endif

}

void	ble_master_data_callback (u8 *p)
{
	//if connected
}

unsigned char Rf_rssi_filter(u8 tmp_rssi){

	#define RSSI_NUM 5

	//unsigned char tmp_rssi = rssi;////get the RSSI value of this packet

	static unsigned char mid_full_flag = 0,filter_first_flag = 0;
	static unsigned char mid_rssi_value[RSSI_NUM] = {0};

	static unsigned int rssi_filter = 0,mid_cal_data = 0;
	static unsigned char rssi_index = 0;


	if((rssi_index < RSSI_NUM-1)&& (!mid_full_flag)){
		mid_rssi_value[rssi_index] = tmp_rssi;
		mid_cal_data = tmp_rssi;
	}
	else{
		mid_full_flag = 1;////receive 5 packets' flag
		mid_rssi_value[rssi_index] = tmp_rssi;

		unsigned char i = 0,j = 0;
		unsigned char tmp_sort = 0;
		for(i=0;i<RSSI_NUM-1;i++){
			for(j=0;j<RSSI_NUM-1-i;j++){
				if(mid_rssi_value[j] > mid_rssi_value[j+1]){
					tmp_sort = mid_rssi_value[j];
					mid_rssi_value[j] = mid_rssi_value[j+1];
					mid_rssi_value[j+1] = tmp_sort;
				}
			}
		}
		mid_cal_data = mid_rssi_value[RSSI_NUM>>1];
	}

	if(!filter_first_flag){/////filter_first_flag == 0
		filter_first_flag = 1;
		rssi_filter = mid_cal_data;
	}
	else{////filter_first_flag ==  1
		rssi_filter = (3*rssi_filter + 1*mid_cal_data)>>2;
	}

	rssi_index++;
	rssi_index %= RSSI_NUM;////from the header store data
	return rssi_filter;
}



void ble_event_callback (u8 status, u8 *p, u8 rssi)
{
	static u32 bdbg_event = 0;
    u8 rssi_filtered =0;
    u8 p_buff[31];
    u8  tbl_mac [] = {0xef, 0xe1, 0xe2, 0x11, 0x12, 0xc5}; // according to tag side

	//bdbg_event++;
    /*
        p[] =
            u8  rf_len;             //LEN(6)_RFU(2)
            u8  advA[6];            //mac address
            u8  data[31];           //payload 0-31 byte
    */

    //tag MAC for 0xC7E4E3E2E1E0
    if(p[1] == tbl_mac[0] && p[2] == tbl_mac[1] )
    {
        memcpy(p_buff,&p[7],31);

        printf("bdbg_event %d ", bdbg_event++);

        //printf("0x");
        //for(int i=0; i<31; i++) { printf("%02x ", p_buff[i]); }
        rssi_filtered = Rf_rssi_filter(rssi);
        printf("rssi_filtered %d\n", rssi_filtered);

        printf("mac 0x%02x%02x%02x%02x%02x%02x rssi %d", p[6],p[5],p[4],p[3],p[2],p[1],Rf_rssi_filter(rssi)); // check coming in mac address
        printf("\n");
    }

#if UART_ENABLE
	if(p[0] == 33 && (p[1] == 0x34 && p[2] == 0x12))// node filter, just for debug --- mesh light default adv pkt && mac address: 0x3412xxxxxxxx
	{
	    static u8 a_beacon_cnt = 0;
	    ++a_beacon_cnt;

	    T_rxdata_user.data[0] = rssi;
	    memcpy(&(T_rxdata_user.data[1]), p + 1, 6);// mac address
		T_rxdata_user.len = 7; // 1byte rssi + 6bytes mac address
		uart_Send((u8 *)(&T_rxdata_user));
	}
#endif
}


/////////////////////////////////////////////////////////////
//	work with PC software
/////////////////////////////////////////////////////////////
void main_loop(void)
{
	static u32 dbg_st, dbg_m_loop;
	dbg_m_loop ++;


#if UART_ENABLE
    if(uart_rx_true){
        uart_rx_true = 0;
        u32 rx_len = T_rxdata_buf.len + 4 > sizeof(T_rxdata_user) ? sizeof(T_rxdata_user) : T_rxdata_buf.len + 4;
        memcpy(&T_rxdata_user, &T_rxdata_buf, rx_len);
        uart_Send((u8 *)(&T_rxdata_user));
    }
    static u32 uart_check_time;
    if(clock_time_exceed(uart_check_time, 100*1000)){
        uart_check_time = clock_time();
        uart_ErrorCLR();
    }
#endif

#if(STACK_CHECK_ENABLE)
    stack_check();
#endif
#if (BATT_CHECK_ENABLE)
    app_battery_power_check_and_sleep_handle(1);
#endif

	if (! ble_master_status () && conn_start)		// APP(master) emulation
	{
		dbg_st++;

		ble_master_start (
				conn_interval,					//6 * 1.25 = 7.5 ms interval
				conn_timeout,					//1 second time out
				&rf_custom_dat,
				CLOCK_SYS_CLOCK_HZ/1000000		// 32M: 32 tick per us
				);
	}
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void  user_init(void)
{
    #if (BATT_CHECK_ENABLE)
    app_battery_power_check_and_sleep_handle(0); //battery check must do before OTA relative operation
    #endif
	blc_readFlashSize_autoConfigCustomFlashSector();
	usb_log_init ();

	ble_master_init ();

	#if (MCU_CORE_TYPE == MCU_CORE_8267)
	usb_dp_pullup_en (0);    // fix 8267 A0 error:1.5K pull up
	#else
	usb_dp_pullup_en (1);
	#endif

	rf_set_power_level_index (RF_POWER_8dBm);

	ble_set_debug_adv_channel (38);

#if UART_ENABLE
    //Initial IO
    uart_io_init(UART_GPIO_8267_PC2_PC3);

    CLK32M_UART115200;
    uart_BuffInit((u8 *)(&T_rxdata_buf), sizeof(T_rxdata_buf), (u8 *)(&T_txdata_buf));
#endif


#if(STACK_CHECK_ENABLE)
    stack_check_init();
#endif

}
