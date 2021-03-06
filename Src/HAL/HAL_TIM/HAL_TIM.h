#ifndef HAL_TIM_H
#define HAL_TIM_H


/***************************************************************************************************
**                              Includes                                                          **
***************************************************************************************************/
#include "C_defs.h"
#include "COMPILER_defs.h"

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



/***************************************************************************************************
**                              Function Prototypes                                               **
***************************************************************************************************/


void HAL_TIM_init( void );
void HAL_TIM_1_start( void );
void HAL_TIM_1_stop( void );
void HAL_TIM_2_stop( void );
void HAL_TIM_2_start( void );

void HAL_TIM_3_stop( void );
void HAL_TIM_3_start( void );

void HAL_TIM_3_SetAutoreload( u16_t new_time_ms );









#endif
