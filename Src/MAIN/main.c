/* STM32 specific includes */
#ifndef UNIT_TEST
	#include "misc.h"
    #include "stm32f10x.h"
    #include "stm32f10x_rcc.h"
    #include "stm32f10x_pwr.h"
	#include "stm32f10x_i2c.h"
#endif

#include "C_defs.h"
#include "PROJ_config.h"
#include "COMPILER_defs.h"
#include "COMPILER_config.h"
#include "HAL_BRD.h"
#include "HAL_ADC.h"
#include "HAL_SPI.h"
#include "NVM.h"
#include "RTC.h"
#include "HAL_I2C.h"
#include "HAL_UART.h"
#include "NRF24.h"
#include "BMP280.h"
#include "main.h"

RCC_ClocksTypeDef RCC_Clocks;
u16_t delay_timer = 0u;
u8_t  NRF24_rf_frame_s[12];

MODE_type_et operating_mode;
pass_fail_et nrf_status;


int main(void)
{
	/* Default the mode and then afterwards read the mode switch to update */
	operating_mode = NORMAL_MODE;

	RCC_DeInit();
	SystemInit();

	RCC_GetClocksFreq (&RCC_Clocks);

	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div1);
	RCC_PCLK2Config(RCC_HCLK_Div1);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Init the HW */
	HAL_BRD_init();
	HAL_I2C1_init();
	HAL_SPI_init();
	HAL_ADC_init();
	NVM_init();

	/* Initialise the RTC */
	RTC_ext_init();

	/* Initialise the NRF24 variables */
	NRF24_init();

	if( get_operating_mode() == DEBUG_MODE )
	{
		/* In debug mode lets init the debug usart as this consumes lots of power */
		SERIAL_init();
		HAL_BRD_set_onboard_LED( ON );
	}

	while (1)
	{
		if( get_operating_mode() != DEBUG_MODE )
		{
			BMP280_trigger_meas();

			/* Populate all the data into the RF frame */
			populate_rf_frame();

			/* Disable the ADC before the RF transmission to save as much power as possible */
			HAL_ADC_de_init();

			/* Disable the I2C peripheral and clock to save power  */
			HAL_I2C1_de_init();

			/* Send the data */
			NRF_simple_send( NRF24_rf_frame_s, sizeof( NRF24_rf_frame_s ), 1u );

			/* Check for any failures */
			check_failures();

			/* Disable the SPI peripheral and clock to save power after the RF transmission */
			HAL_SPI_de_init();

			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, DISABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, DISABLE);

			/* Enable the wakeup pin */
			PWR_WakeUpPinCmd(ENABLE);

			/* Enters STANDBY mode */
			PWR_EnterSTANDBYMode();
		}
		else
		{
			/* Handle the serial messages */
			SERIAL_msg_handler();

			/* Check for any failures */
			check_failures();

			if( HAL_BRD_get_rtc_trigger_status() == TRUE )
			{
				BMP280_trigger_meas();

				populate_rf_frame();

				NRF_simple_send( NRF24_rf_frame_s, sizeof( NRF24_rf_frame_s ), 1u );

				/* Set the trigger back to false */
				HAL_BRD_set_rtc_trigger_status( FALSE );
			}
		}
	}
}




/* ISR for systick handler */
void SysTick_Handler( void )
{
	delay_timer--;
}




void MAIN_SYSTICK_init( void )
{
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq (&RCC_Clocks);

	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK_Div8 );

	/* Trigger an interrupt every 1ms */
	SysTick_Config(72000);
}




u8_t generate_random_number( void )
{
	u32_t seed;
	u8_t  random_number;
	u8_t  time_array_s[RTC_TIME_ARRAY_SIZE];

	/* We need to create a "random" value here, and in order to do that we "seed" the rand function with
	 an instantaneous time value comprised of secs, mins, hours and even days */
	RTC_grab_current_running_time( time_array_s );
	seed = time_array_s[0];						   /* The seconds part */
	seed |= ( ( time_array_s[1] & 0x7F ) << 8u );  /* The minutes part */
	seed |= ( ( time_array_s[2] & 0x3F ) << 16u ); /* The hours part */
	seed |= ( ( time_array_s[3] & 0x3F ) << 24u ); /* The days part */

	/* This means that "seed" is now a 32bit number but the max value that srand can accept is 32768,
	   so lets mod the "seed" with 32768 to give us a remainder that can be fed in to create a seed  */
	seed = ( seed % 32768u );

    /* Now Set the seed */
	srand( seed );

    /* Grab the now "random :)" number and limit the values to between 0 and 255 ( 1 byte ) */
    random_number = rand()%255u;

	return ( random_number );
}



void populate_rf_frame( void )
{
	u8_t  data_len = 10u;
	u16_t battery_voltage;

	s32_t temperature = BMP280_get_temperature();

	NRF24_rf_frame_s[0] =  generate_random_number();
	NRF24_rf_frame_s[1] =  SENSOR_TYPE;
	NRF24_rf_frame_s[2] =  ( ( SENSOR_ID & 0xFF00 ) >> 8u );
	NRF24_rf_frame_s[3] =  ( SENSOR_ID & 0x00FF );
	NRF24_rf_frame_s[4] =  PACKET_TYPE;
	NRF24_rf_frame_s[5] =  get_operating_mode();
	NRF24_rf_frame_s[6] =  7u;
	NRF24_rf_frame_s[7] =  ( ( temperature & 0x0000FF00 ) >> 8u );
	NRF24_rf_frame_s[8] =  ( temperature & 0x0000FF );  //round the first part
	NRF24_rf_frame_s[9] =  ( ( NVM_info_s.NVM_generic_data_blk_s.sleep_time & 0xFF00 ) >> 8u );
	NRF24_rf_frame_s[10] = ( NVM_info_s.NVM_generic_data_blk_s.sleep_time & 0x00FF );
}




void delay_ms(u16_t ms)
{
	delay_timer = ms;
	while(delay_timer != 0);
}



void delay_us(u16_t us)
{
	/* If we are just delaying time then we want to do this as efficiently as possible,
	 * Lets reduce the clock speed down as low as we can to do this ( /256 ), this means we are 256
	 * times slower than what we originally were running at */

	RCC_HCLKConfig(RCC_SYSCLK_Div8);
	RCC_GetClocksFreq(&RCC_Clocks);

	asm volatile (	"MOV R0,%[loops]\n\t"\
			"1: \n\t"\
			"SUB R0, #1\n\t"\
			"CMP R0, #0\n\t"\
			"BNE 1b \n\t" : : [loops] "r" (1*us) : "memory"\
		      );

	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_GetClocksFreq (&RCC_Clocks);
}


void set_operating_mode( MODE_type_et mode )
{
	operating_mode = mode;
}

MODE_type_et get_operating_mode( void )
{
	return( operating_mode );
}

void check_failures( void )
{
	RTC_get_failure_status();
	BMP280_get_failure_status();
	NRF_get_failure_status();
}



void assert_failed(u8_t* file, u32_t line)
{
	u32_t lines;
	u8_t files;

	files = *file;
	lines = line;
}





