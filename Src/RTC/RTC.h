#ifndef RTC_H
#define RTC_H


/***************************************************************************************************
**                              Includes                                                          **
***************************************************************************************************/
#include "C_defs.h"
#include "COMPILER_defs.h"

/***************************************************************************************************
**                              Defines                                                           **
***************************************************************************************************/
#define RTC_EXT_MAX_WAKEUP_TIME_SEC 	15300u
#define RTC_EXT_I2C_ADDRESS				0xA2
#define RTC_EXT_MAX_NUM_REGS			16u
#define RTC_TIME_ARRAY_SIZE				4u


#define RTC_EXT_TIMER_INT_ENABLE_BIT (1<<0)
#define RTC_EXT_ALARM_INT_ENABLE_BIT (1<<1)
#define RTC_EXT_TIMER_INT_ACTIVE_BIT (1<<2)
#define RTC_EXT_ALARM_INT_ACTIVE_BIT (1<<3)

#define RTC_EXT_ALARM_1HZ_BIT (1<<1)
#define RTC_EXT_ALARM_1_OVER60HZ_BIT (1<<0)

typedef enum
{
	Control_status_1 = 0u,
	Control_status_2,
	VL_seconds,
	Minutes,
	Hours,
	Days,
	Weekdays,
	Century_months,
	Years,
	Minute_alarm,
	Hour_alarm,
	Day_alarm,
	Weekday_alarm,
	CLKOUT_control,
	Timer_control,
	Timer
} RTC_EXT_registers_et;



typedef enum
{
	ClK_OUT_32768_HZ = 0u,
	ClK_OUT_1024_HZ,
	ClK_OUT_32_HZ,
	ClK_OUT_1_HZ,
} RTC_clk_out_setting_et;


/***************************************************************************************************
**                              Constants                                                         **
***************************************************************************************************/
/* None */


/***************************************************************************************************
**                              Data Types and Enums                                              **
***************************************************************************************************/
/* None */



/***************************************************************************************************
**                              Exported Globals                                                  **
***************************************************************************************************/
/* None */



/***************************************************************************************************
**                              Function Prototypes                                               **
***************************************************************************************************/
void	     RTC_ext_init( void );
void         RTC_set_wakeup_time( u32_t seconds );
void         RTC_grab_current_running_time( u8_t* data_p );
pass_fail_et RTC_get_failure_status( void );


#endif /* RTC_H multiple inclusion guard */

/****************************** END OF FILE *******************************************************/
