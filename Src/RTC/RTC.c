/*! \file
*               $Revision: 16923 $
*
*               $Author: mstewart $
*
*               $Date: 2014-01-16 15:40:40 +0000 (Thu, 16 Jan 2014) $
*
*               $HeadURL: https://selacvs01.schrader.local:8443/svn/ECU_Software/LF_TOOL_GEN2/trunk/Src/HAL/HAL_UART/HAL_UART.c $
*
*   \brief      UART interface module
*/
/* COPYRIGHT NOTICE
* ==================================================================================================
*
* The contents of this document are protected under copyright and contain commercially and/or
* technically confidential information. The content of this document must not be used other than
* for the purpose for which it was provided nor may it be disclosed or copied (by the authorised
* recipient or otherwise) without the prior written consent of an authorised officer of Schrader
* Electronics Ltd.
*
*         (C) $Date:: 2014#$ Schrader Electronics Ltd.
*
****************************************************************************************************
*/
/***************************************************************************************************
**                              Includes                                                          **
***************************************************************************************************/
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_conf.h"
#include "misc.h"

#include "C_defs.h"
#include "STDC.h"
#include "main.h"
#include "HAL_BRD.h"
#include "NVM.h"
#include "COMPILER_defs.h"
#include "HAL_I2C.h"
#include "RTC.h"


extern NVM_info_st NVM_info_s;

pass_fail_et RTC_failure_status_s;

/***************************************************************************************************
**                              Data declarations and definitions                                 **
***************************************************************************************************/
/* None */

const u8_t RTC_EXT_default_register_values[ RTC_EXT_MAX_NUM_REGS ] =
{
	0x00,	//Control_status_1
	0x11,	//Control_status_2
	0x00,	//VL_seconds
	0x00,	//Minutes
	0x00,	//Hours
	0x00,	//Days
	0x00,	//Weekdays
	0x00,	//Century_months
	0x00,	//Years
	0x00,	//Minute_alarm
	0x00,	//Hour_alarm
	0x00,	//Day_alarm
	0x00,	//Weekday_alarm
	0x83,	//CLKOUT_control
	0x82,	//Timer_control
	RTC_EXT_DEFAULT_WAKEUP_TIME_SEC
};




void RTC_ext_init( void )
{
	/* Assume no fault until it is detected */
	RTC_failure_status_s = PASS;

	u8_t data_burst[16];
	u8_t time_array[4];
	u8_t test_data[16];

	HAL_I2C_read_multiple_registers( RTC_EXT_I2C_ADDRESS, Control_status_1, data_burst, sizeof( data_burst ) );

	/* Grab the value for time ( in BCD ) as we will write this down again after the configuration has been set */
	time_array[0] = data_burst[VL_seconds];
	time_array[1] = data_burst[Minutes];
	time_array[2] = data_burst[Hours];
	time_array[3] = data_burst[Days];

	/* Write down the default config */
	HAL_I2C_write_multiple_register( RTC_EXT_I2C_ADDRESS, Control_status_1, RTC_EXT_default_register_values, sizeof( RTC_EXT_default_register_values ) );

	/* Now write the time value down again */
	HAL_I2C_write_multiple_register( RTC_EXT_I2C_ADDRESS, VL_seconds, time_array, sizeof( time_array ) );

	/* Now adjust it with the currently stored NVM value */
	RTC_set_wakeup_time( NVM_info_s.NVM_generic_data_blk_s.sleep_time );

	
	/* Now do some checking to ensure that the device is operational as we expect it to be */
	HAL_I2C_read_multiple_registers( RTC_EXT_I2C_ADDRESS, Control_status_1, data_burst, sizeof( data_burst ) );
	
	STDC_memcpy( test_data, RTC_EXT_default_register_values, sizeof( test_data ) );
	STDC_memcpy( &test_data[2], time_array, sizeof( time_array ) );

	if( STDC_memcompare( data_burst, test_data, sizeof(data_burst) ) == FALSE )
	{
		/* Record a failure */
		RTC_failure_status_s = FAIL;
	}
}



void RTC_set_clk_output( disable_enable_et state, RTC_clk_out_setting_et setting )
{
}


void RTC_ext_clear_int( void )
{
	u8_t data;

	/* Read the register first to get the old value */
	HAL_I2C_read_register(RTC_EXT_I2C_ADDRESS, Control_status_2, &data );

	/* Clear the inerrupt active bit */
	data &= !RTC_EXT_TIMER_INT_ACTIVE_BIT;

	/* Write the data back down again :) */
	HAL_I2C_write_single_register( RTC_EXT_I2C_ADDRESS, Control_status_2, &data );
}



void RTC_set_wakeup_time( u32_t seconds )
{
	u8_t data;
	u8_t divider;

	/* Boundary check the wakeup time */
	if( seconds > RTC_EXT_MAX_WAKEUP_TIME_SEC )
	{
		seconds = RTC_EXT_MAX_WAKEUP_TIME_SEC;
	}

	/* Read the register first to get the old value */
	HAL_I2C_read_register(RTC_EXT_I2C_ADDRESS, Timer_control, &data );

	if( seconds > U8_T_MAX )
	{
		divider = seconds/60;

		/* set the correct hz bit*/
		data |= RTC_EXT_ALARM_1_OVER60HZ_BIT;
		data |= RTC_EXT_ALARM_1HZ_BIT;
	}
	else
	{
		divider = seconds;

		/* set the correct hz bit*/
		data &= ~RTC_EXT_ALARM_1_OVER60HZ_BIT;
		data |= RTC_EXT_ALARM_1HZ_BIT;
	}

	HAL_I2C_write_single_register( RTC_EXT_I2C_ADDRESS, Timer_control, &data );

	/* Now set the time register */
	HAL_I2C_write_single_register( RTC_EXT_I2C_ADDRESS, Timer, &divider );
}



void RTC_grab_current_running_time( u8_t* data_p )
{
	HAL_I2C_read_multiple_registers( RTC_EXT_I2C_ADDRESS, VL_seconds, data_p, RTC_TIME_ARRAY_SIZE );
}


pass_fail_et RTC_get_failure_status( void )
{
	return( RTC_failure_status_s );
}


///****************************** END OF FILE *******************************************************/
