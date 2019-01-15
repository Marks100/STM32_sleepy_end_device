#ifndef HAL_BRD_H
#define HAL_BRD_H



/***************************************************************************************************
**                              Includes                                                          **
***************************************************************************************************/
#include "C_defs.h"
#include "COMPILER_defs.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"


/***************************************************************************************************
**                              Defines                                                           **
***************************************************************************************************/
/* None */



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
extern u8_t debug_mode;


/***************************************************************************************************
**                              Function Prototypes                                               **
***************************************************************************************************/

void HAL_BRD_init( void );
low_high_et HAL_BRD_read_pin_state( GPIO_TypeDef * port, u16_t pin );
void HAL_BRD_set_pin_state(  GPIO_TypeDef * port, u16_t pin, low_high_et state );

void HAL_BRD_set_batt_monitor_state( disable_enable_et state );
void HAL_BRD_set_rf_enable_pin( disable_enable_et state );
void HAL_BRD_toggle_led( void );
void HAL_BRD_set_LED( off_on_et state );
void HAL_BRD_set_debug_mode_LED( off_on_et state );
void HAL_BRD_toggle_debug_mode_led( void );
void HAL_BRD_RFM69_set_enable_pin_state( low_high_et state );
void HAL_BRD_RFM69_set_reset_pin_state( low_high_et state );
void HAL_BRD_RFM69_spi_slave_select( low_high_et state );
void HAL_BRD_NRF24_spi_slave_select( low_high_et state );
void HAL_BRD_NRF24_set_ce_pin_state( low_high_et state );
void HAL_BRD_set_NRF_power_pin_state( off_on_et state );

false_true_et HAL_BRD_get_rtc_trigger_status( void );
void HAL_BRD_set_rtc_trigger_status( false_true_et state );
disable_enable_et HAL_BRD_read_debug_pin( void );
low_high_et HAL_BRD_NRF24_read_irq_pin( void );
void HAL_BRD_get_SW_version_num( u8_t* version_num_p );
void HAL_BRD_get_HW_version_num( u8_t* version_num_p );

void HAL_BRD_set_spi_pins_for_lp( void );

void delay_ms(u16_t);
void delay_us(u16_t us);



#endif
