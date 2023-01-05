
#pragma once
/** @addtogroup  HAL_PWM_Module PWM Module
 *  @{
 *  @details The TLSR8266/TLSR826F512 supports 6-channels PWM output
 */
#include "../tl_common.h"

/** @addtogroup  HAL_PWM_CHANNEL PWM OUTPUT CHANNELS
 *  @{
 *  @details Each PWM#n has its corresponding inverted output at PWM#n_Npin.
 */
enum {
    PWM0 = 0 ,
    PWM0_INV ,
    PWM1 ,
    PWM1_INV ,
    PWM2 ,
    PWM2_INV ,
    PWM3  ,
    PWM3_INV ,
    PWM4 ,
    PWM4_INV ,
    PWM5 ,
    PWM5_INV ,
};

/** @} end of group HAL_PWM_CHANNEL */

/** @addtogroup  HAL_PWM_MODE PWM MODE
 *  @{
 *  @details PWM0 and OWM1 support 3 modes,including normal(continuous) mode,count 	*			mode,and IR mode.PWM2 ~ PWM5 only support normal(continuous) mode.
 */
enum {
    NORMAL_MODE = 0,			//!<PWM0~PWM5 all support normal(continuous)mode.
    //!<In this mode,PWM#n continuously sends out signal frames.
    COUNT_MODE = 1,				//!<Only PWM0 and PWM1 support Counting mode.
    //!<In this mode,PWM#n(n=0,1)sends out specified number of signal frames which defined as a pluse group.
    IR_MODE = 3,				//!<Only PWM0 and PWM1 support Counting mode.
    //!<In this mode,specified number of frames is defined as one pulse group.In contrast to Counting mode  where PWM#n(n=0,1) stop after pulse  group finishes PWM#n will constantly send pulse groups in IR mode.
};
/** @} end of group HAL_PWM_MODE */

/** @addtogroup  PWM_Functions PWM common APIs
 *  @{
 */

/**
 * @brief  		Set the PWM Module clock
 *
 * @details		The PWM module clock should less than system clock.
 *
 * @param[in]	pwm_clk - Specify the clock .
 *
 * @return 		None
 *
 */
void pwm_clk(int pwm_clk);
/**
 * @brief  		Set PWM Frequency And Duty Cycle
 *
 * @details		-the max_tick and cmp_tick is selected base on PWM clock.
 * 				-the max_tick should greater than or equal cmp_tick
 *
 * @param[in]	id - Specify the output channel. This Parameter should select from enumerate PWM_ID.
 *
 * @param[in]	max_tick - Set the output period.
 *
 * @param[in]	cmp_tick - Set the high or low time of max_tick.
 *
 * @return 		None
 *
 */
void pwm_set_duty(int id, u16 max_tick, u16 cmp_tick);

/**
 * @brief  		Set PWM Mode.
 *
 * @details		PWM0 ~ PWM5 all support Normal mode(Continue mode).Only PWM0 and PWM1 support Counting mode and IR mode.
 * 				pulse_num is meaningless in Normal mode.
 *
 * @param[in]	id - Specify the output channel. This Parameter should be one of{PWM0,PWM0_INV,PWM1,PWM1_INV}.
 *
 * @param[in]	mode - Specify PWM mode.PWM mode should selected from enumerate PWM_MODE.
 *
 * @param[in]	pluse_num - Set the corresponding PWM channel pulse number.
 *
 * @return 		None
 */
void pwm_set_mode(int id, u16 mode, int pulse_num);

/**
 * @brief  		Set a PWM channel waveform inverted.
 *
 * @param[in]	id - Specify the output channel. This Parameter should select from enumerate PWM_ID.
 *
 * @param[in]   en - Specify the corresponding PWM channel output inverted enable or disable.
 *
 * @return 		None.
 *
 */
void pwm_set_invert_en(int id, int en);

/**
 * @brief  		Set a PWM channel output enable.
 *
 * @details		Set a channel output enable,not the entire PWM module.
 *
 * @param[in]	id - Specify the output channel. This Parameter should select from enumerate PWM_ID.
 *
 * @return 		None
 *
 */
void pwm_start(int id);

/**
 * @brief  		Set a PWM channel output disable
 *
 * @details		Set a channel output disable,not the entire PWM module
 *
 * @param[in]	id - Specify the output channel. This Parameter should selected from enumerate PWM_ID.
 *
 * @return 		None
 *
 */
void pwm_stop(int id);

/**
 * @brief  		Set Pnum or frame interruption will be generated enable/disable.
 *
 * @details		A frame interruption will be generated(if enabled) after each signal frame is finished in normal(continuous) mode.
 * 				A Pnum interruption will be generated(if enabled) after a pulse group is finished in Counting mode.
 * 				A frame/Pnum interruption will be generated(if enabled) after signal frame/pulse group is finished in IR mode.
 * 				In Normal(Continuous) mode ,id = PWM#n(n=0~5).
 * 				In IR/Counting mode,id = PWM#n(n=0 or 1).
 *
 * @param[in]	id - Specify PWM id.
 * @param[in]	en - 0(FALSE) indicate disable interruption generated.On the contrary,enable interruption generated.
 * @param[in]	is_pnum_int_mask - 0(FALSE) indicate enable or disable a frame interruption generated,and the id could be PWM0~PWM5.
 * 								   !0 (TRUE) indicate enable or disable a Pnum interruption generated,and the id could only be PWM0 or PWM1.
 *
 * @return 		None
 *
 */
void pwm_set_int_mask_enable(int id, int en, int is_pnum_int_mask);

/**
 * @brief  		Get interruption status with corresponding PWM id.
 *
 * @param[in]	id - Specify PWM id.
 * @param[in]	is_pnum_int_mask - Specify interruption type(Frame interruption or Pnum interruption).
 * 									!0(TRUE) indicate Pnum interruption. 0(FALSE) indicate a Frame interruption.
 *
 * @return		TRUE indicate the corresponding interruption has been generated,Or the interruption has not been generated.
 *
 */
bool pwm_int_Status_read(int id, int is_pnum_int_mask);

/**
 * @brief  		Clear corresponding interruption status.
 *
 * @param[in]	id - Specify PWM id.
 * @param[in]	is_pnum_int_mask - Specify interruption type(Frame interruption or Pnum interruption).
 * 									!0(TRUE) indicate Pnum interruption. 0(FALSE) indicate a Frame interruption.
 *
 * @return		None.
 *
 */
void pwm_int_Status_clear(int id, int is_pnum_int_mask);

/** @} end of group PWM_Functions */

/** @} end of group HAL_PWM_Module */


