#ifndef HAL_ADC_H
#define HAL_ADC_H


#include "Compiler_defs.h"
#include "C_defs.h"
/***************************************************************************************************
**                              Defines                                                          **
***************************************************************************************************/


#define VCC_REFERENCE_VOLTAGE 3300.0f

/***************************************************************************************************
**                              enumerated Types                                                  **
***************************************************************************************************/
typedef enum
{
    ADC_CHANNEL_0 = 0,
    ADC_CHANNEL_1,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_4,
    ADC_CHANNEL_5,
} ADC_channel_et;







/***************************************************************************************************
**                                 Functions                                                      **
***************************************************************************************************/
void HAL_ADC_init( void ) ;
void HAL_ADC_de_init( void );
u16_t HAL_ADC_measure_joystick_x_axis( void );
u16_t HAL_ADC_measure_joystick_y_axis( void );



#endif
