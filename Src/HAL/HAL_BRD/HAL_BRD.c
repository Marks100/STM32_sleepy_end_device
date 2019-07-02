#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x.h"
#include "misc.h"

#include "autoversion.h"
#include "main.h"
#include "HAL_BRD.h"

false_true_et HAL_BRD_rtc_triggered_s;


/*!
****************************************************************************************************
*
*   \brief         Initialise the Pins,
*   			   lets just do gpio pins here and let other modules handle themselves
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_init( void )
{
	disable_enable_et debug_mode;

	/* Disable the JTAG as this saves us some pins :) */
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* Configure the GPIOs */
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure the DEBUG selector pin, its important that this comes first */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure the power pin for the NRF24l01
	   This is very sensitive, if i configure the pin as output PP here then the PIN
	   goes high and the transistor tries to turn on, and thus powers the NRF chip and
	   This causes problems, so if i init the PIN as input then the external resistor keeps the
	   PIN in the correct state, then when i want the PIN to go high i init it as an output*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure the NRF irq ( Data Sent ) input pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure the Temp sensor enable pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	HAL_BRD_set_onboard_LED( OFF );

	/* small delay to allow the button to settle */
	delay_us(500);

	debug_mode = HAL_BRD_read_debug_pin();

	#if( AUTO_DEBUG_MODE == 1 )
		debug_mode = ENABLE_;
	#endif

	if( debug_mode == ENABLE_ )
	{
		set_operating_mode( DEBUG_MODE );

		/* configure the debug mode led ( this lets us know we are in debug mode and will only be turned
		on in debug mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Configure the wakeup ( or in debug mode interrupt ) pin */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* If we are in debug mode then we can configure the wakeup pin to have an ISR */
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0 );

		EXTI_InitTypeDef EXTI_InitStruct;

		EXTI_InitStruct.EXTI_Line = EXTI_Line0 ;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt ;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_Init(&EXTI_InitStruct);

		NVIC_InitTypeDef NVIC_InitStruct;

		/* Add IRQ vector to NVIC */
		NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
		/* Set priority */
		NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
		/* Set sub priority */
		NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
		/* Enable interrupt */
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
		/* Add to NVIC */
		NVIC_Init(&NVIC_InitStruct);
	}

	/* Configure the NRF24 NCS pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure the NRF24 CE pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable the "RFM69 packet sent" interrupt on pin A1 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1 );

	EXTI_InitTypeDef EXTI_InitStruct;

	EXTI_InitStruct.EXTI_Line = EXTI_Line1 ;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt ;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitTypeDef NVIC_InitStruct;

	/* Add IRQ vector to NVIC */
	NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);

	HAL_BRD_rtc_triggered_s = TRUE;
}








/*!
****************************************************************************************************
*
*   \brief         Reads Any PORT and any PIN
*
*   \author        MS
*
*   \return        low_high_et
*
***************************************************************************************************/
low_high_et HAL_BRD_read_pin_state( GPIO_TypeDef * port, u16_t pin )
{
	low_high_et returnType;

	if( ( port->IDR & pin ) == pin )
	{
		returnType = HIGH;
	}
	else
	{
		returnType = LOW;
	}

	return ( returnType );
}






/*!
****************************************************************************************************
*
*   \brief         SETS Any PIN on any PORT
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_set_pin_state(  GPIO_TypeDef * port, u16_t pin, low_high_et state )
{
	if( state == HIGH)
	{
		port->ODR |= pin;
	}
	else
	{
		port->ODR &= ~pin;
	}
}



/*!
****************************************************************************************************
*
*   \brief         Toggles Any PIN on any PORT
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_toggle_pin_state(  GPIO_TypeDef * port, u16_t pin )
{
    /* Firstly read the PIN state */
    if( ( port->ODR & pin ) == pin )
    {
        HAL_BRD_set_pin_state( port, pin, LOW );
    }
    else
    {
        HAL_BRD_set_pin_state( port, pin, HIGH );
    }
}




/*!
****************************************************************************************************
*
*   \brief         Toggles Any PIN on any PORT
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_set_NRF_power_pin_state( off_on_et state )
{
	if( state == ON )
	{
		/* Configure the GPIOs */
		GPIO_InitTypeDef GPIO_InitStructure;

		/* Configure the power pin for the NRF24l01 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		HAL_BRD_set_pin_state( GPIOA, GPIO_Pin_11, LOW );
	}
	else
	{
	    HAL_BRD_set_pin_state( GPIOA, GPIO_Pin_11, HIGH );
	}
}

/*!
****************************************************************************************************
*
*   \brief         Sets the BMP280 pin state
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_set_BMP280_power_pin_state( off_on_et state )
{
	if( state == ON )
	{
		/* Configure the GPIOs */
		GPIO_InitTypeDef GPIO_InitStructure;

		/* Configure the power pin for the BMP280 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		HAL_BRD_set_pin_state( GPIOB, GPIO_Pin_9, LOW );
	}
	else
	{
	    HAL_BRD_set_pin_state( GPIOB, GPIO_Pin_9, HIGH );
	}
}


/**************************************************************************************************
EXTERNAL API's
***************************************************************************************************/








/*!
****************************************************************************************************
*
*   \brief         Toggles the led
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_toggle_onboard_led( void )
{
    HAL_BRD_toggle_pin_state( GPIOC, GPIO_Pin_13 );
}


void HAL_BRD_set_onboard_LED( off_on_et state )
{
	low_high_et val;

	/* LED is inverse logic so flip the states */
	if( state == OFF )
	{
		val = HIGH;
	}
	else
	{
		val = LOW;
	}
	HAL_BRD_set_pin_state( GPIOC, GPIO_Pin_13, val);
}



/*!
****************************************************************************************************
*
*   \brief         Toggles the debug led
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_toggle_debug_mode_led( void )
{
    HAL_BRD_toggle_pin_state( GPIOA, GPIO_Pin_8 );
}


/*!
****************************************************************************************************
*
*   \brief         Sets the state of the debug mode the led
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_set_debug_mode_LED( off_on_et state )
{
	HAL_BRD_set_pin_state( GPIOA, GPIO_Pin_8, state);
}


/*!
****************************************************************************************************
*
*   \brief         Sets the state of the debug mode the led
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_set_temp_sensor_enable_pin( off_on_et state )
{
	HAL_BRD_set_pin_state( GPIOA, GPIO_Pin_2, state);
}



/*!
****************************************************************************************************
*
*   \brief         Reads the state of the debug pin
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
disable_enable_et HAL_BRD_read_debug_pin( void )
{
	low_high_et state;
	disable_enable_et mode;

	state = HAL_BRD_read_pin_state(GPIOB, GPIO_Pin_5 );

	mode = (( state == HIGH ) ? ENABLE : DISABLE );

	return( mode );
}







/*!
****************************************************************************************************
*
*   \brief         Toggles the heartbeat pin
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_toggle_heartbeat_pin(  void )
{
    HAL_BRD_Toggle_Pin_state( GPIOB, GPIO_Pin_11 );
}



/*!
****************************************************************************************************
*
*   \brief         returns the  status of the RTC trigger
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
false_true_et HAL_BRD_get_rtc_trigger_status( void )
{
	return ( HAL_BRD_rtc_triggered_s );
}




/*!
****************************************************************************************************
*
*   \brief         Sets the trigger state of the RTC
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_set_rtc_trigger_status( false_true_et state )
{
	HAL_BRD_rtc_triggered_s = state;
}




/*!
****************************************************************************************************
*
*   \brief         SETS the rf enable pin
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_NRF24_set_ce_pin_state( low_high_et state )
{
	if( state == HIGH )
	{
		HAL_BRD_set_pin_state( GPIOA, GPIO_Pin_10, HIGH );
	}
	else
	{
		HAL_BRD_set_pin_state( GPIOA, GPIO_Pin_10, LOW );
	}
}



/*!
****************************************************************************************************
*
*   \brief         SETS the SPI chip select pin for the RF module
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_NRF24_spi_slave_select( low_high_et state )
{
	if( state == HIGH )
	{
		HAL_BRD_set_pin_state( GPIOA, GPIO_Pin_9, HIGH );
	}
	else
	{
		HAL_BRD_set_pin_state( GPIOA, GPIO_Pin_9, LOW );
	}
}






low_high_et HAL_BRD_NRF24_read_irq_pin( void )
{
	low_high_et state;

	state = HAL_BRD_read_pin_state(GPIOA, GPIO_Pin_12 );

	return ( state );
}

/*!
****************************************************************************************************
*
*   \brief         Sets the trigger state of the RTC
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_get_SW_version_num( u8_t* version_num_p )
{
	version_num_p[0] = SLEEP_SENSORS_VERSION_MAJOR;
	version_num_p[1] = SLEEP_SENSORS_VERSION_PATCH;
	version_num_p[2] = SLEEP_SENSORS_VERSION_VERIFICATION;
}


/*!
****************************************************************************************************
*
*   \brief         Sets the trigger state of the RTC
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void HAL_BRD_get_HW_version_num( u8_t* version_num_p )
{
	version_num_p[0] = SLEEP_SENSORS_VERSION_MAJOR;
	version_num_p[1] = SLEEP_SENSORS_VERSION_PATCH;
}



/*!
****************************************************************************************************
*
*   \brief         ISR
*
*   \author        MS
*
*   \return        None
*
***************************************************************************************************/
void EXTI0_IRQHandler(void)
{
	/* Make sure that interrupt flag is set */
	if ( EXTI_GetFlagStatus(EXTI_Line0) != RESET )
	{
		/* Now we keep track of the interrupt edge */
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line0);

		HAL_BRD_rtc_triggered_s = TRUE;
	}
}


void EXTI1_IRQHandler(void)
{
	/* Make sure that interrupt flag is set */
	if ( EXTI_GetFlagStatus(EXTI_Line1) != RESET )
	{
		/* Now we keep track of the interrupt edge */
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}







/*!
****************************************************************************************************
*
*   \brief         Interrupt Handler
*
*   \author        MS
*
*   \return        low_high_et
*
***************************************************************************************************/
/* Handle PB12 interrupt */
void EXTI15_10_IRQHandler(void)
{
	/* Make sure that interrupt flag is set */
	if ( EXTI_GetFlagStatus(EXTI_Line15) != RESET )
	{
	    /* Now we keep track of the interrupt edge */
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line15);
	}
}


