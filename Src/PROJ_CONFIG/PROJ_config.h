#ifndef PROJ_CONFIG_H
#define PROJ_CONFIG_H

#define STM32 							 1u
#define RFM69W
#define RFM69_DEFAULT_TX_POWER_LEVEL     0u
#define RTC_EXT_DEFAULT_WAKEUP_TIME_SEC  10
#define BATTERY_DIVISION				 17u
#define TEMPERATURE_OFFSET				 20u
#define NUM_ADC_TEMP_SAMPLES			 10u
#define MIN_TEMP_ARRAY_VAL				 33u
#define MAX_TEMP_ARRAY_VAL			 	 918u

#define TEMP_NOT_AVAILABLE				0xFF

#define AUTO_DEBUG_MODE					 0u
#define MULTI_SPI_WRITE_CONFIG			 1u
#define SENSOR_ID 						 0x000A
#define SENSOR_TYPE						 1u
#define PACKET_TYPE						 5u
#define MODE_TYPE						 10u


#if !defined ( RFM69HW ) && !defined ( RFM69W ) && !defined ( RFM69HW_SUPER )
	#error "Please define one of the following RFM69W || RFM69HW || RFM69HW_SUPER"
#endif

#endif /* PROJ_CONFIG_H multiple inclusion guard */
