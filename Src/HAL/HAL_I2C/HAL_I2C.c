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
#include "misc.h"

#include "C_defs.h"
#include "STDC.h"
#include "NVM.h"
#include "COMPILER_defs.h"
#include "HAL_I2C.h"




extern NVM_info_st NVM_info_s;

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



/***************************************************************************************************
**                              Public Functions                                                  **
***************************************************************************************************/
/*!
****************************************************************************************************
*
*   \brief         Module (re-)initialisation function
*
*   \author        MS
*
*   \return        none
*
*   \note
*
***************************************************************************************************/
void HAL_I2C_init( void )
{
	/* Enable I2C and GPIOA clock, should be enabled anyway but just in case */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );

	I2C_SoftwareResetCmd(I2C1, ENABLE);
	I2C_SoftwareResetCmd(I2C1, DISABLE);

	/* Configure the GPIOs */
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure I2C_EE pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitTypeDef  I2C_InitStructure;

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x38;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C1, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C1, &I2C_InitStructure);

	I2C_AcknowledgeConfig(I2C1,ENABLE);
}



void HAL_I2C_de_init( void )
{
	/* De-init the I2C module */
	I2C_DeInit(I2C1);

	/* I2C Peripheral Disable */
	I2C_Cmd(I2C1, DISABLE);

	/* Disable I2C clock*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);
}




void HAL_I2C_write_single_register( u8_t dev_add, u8_t reg_add, u8_t* data )
{
	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, dev_add, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1, reg_add);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_SendData(I2C1, *data);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTOP(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}




void HAL_I2C_write_multiple_register( u8_t dev_add, u8_t reg_start_add, u8_t* data, u8_t num_bytes )
{
	u8_t i = 0u;

	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, dev_add, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1, reg_start_add );
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	for( i = 0; i < num_bytes; i++ )
	{
		I2C_SendData(I2C1, data[i] );
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}

	I2C_GenerateSTOP(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}




void HAL_I2C_read_register( u8_t dev_add, u8_t reg_add, u8_t* data )
{
	I2C_AcknowledgeConfig(I2C1,ENABLE);

	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, dev_add, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1, reg_add);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, dev_add, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	*data = I2C_ReceiveData(I2C1);

	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_AcknowledgeConfig(I2C1, DISABLE);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	I2C_ReceiveData(I2C1);
}


void HAL_I2C_read_multiple_registers( u8_t dev_add, u8_t reg_start_add, u8_t* data, u8_t num_bytes )
{
	u8_t i = 0u;

	I2C_AcknowledgeConfig(I2C1,ENABLE);

	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, dev_add, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1, reg_start_add);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, dev_add, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	for( i = 0; i < num_bytes ; i++ )
	{
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
		data[i] = I2C_ReceiveData(I2C1);
	}

	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_AcknowledgeConfig(I2C1, DISABLE);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	I2C_ReceiveData(I2C1);
}




void RTC_ext_init( void )
{
	u8_t data_burst[16];

	/* Write down the default config */
	HAL_I2C_write_multiple_register( RTC_EXT_I2C_ADDRESS, Control_status_1, RTC_EXT_default_register_values, sizeof( RTC_EXT_default_register_values ) );

	/* Now adjust it with the currently stored NVM value */
	RTC_set_wakeup_time( NVM_info_s.NVM_generic_data_blk_s.sleep_time );

	HAL_I2C_read_multiple_registers( RTC_EXT_I2C_ADDRESS, Control_status_1, data_burst, sizeof( data_burst ) );
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





//
///***************************************************************************************************
//**                              ISR Handlers                                                      **
//***************************************************************************************************/


///****************************** END OF FILE *******************************************************/
