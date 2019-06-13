
#ifndef HAL_I2C_H
#define HAL_I2C_H

/***************************************************************************************************
**                              Includes                                                          **
***************************************************************************************************/
#include "C_defs.h"
#include "COMPILER_defs.h"

/***************************************************************************************************
**                              Defines                                                           **
***************************************************************************************************/
#define HAL_I2C_LL_TIMEOUT ( 1000u )

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
void HAL_I2C_init( void );
void HAL_I2C_de_init( void );

void HAL_I2C_write_single_register( u8_t dev_address, u8_t register_address, u8_t* data );
void HAL_I2C_write_multiple_register( u8_t dev_address, u8_t register_start_address, u8_t* data, u8_t num_bytes );
void HAL_I2C_read_register(  u8_t dev_add, u8_t reg_add, u8_t* data );
void HAL_I2C_read_multiple_registers( u8_t dev_add, u8_t reg_start_add, u8_t* data, u8_t num_bytes );

void HAL_I2C1_generate_start( void );
void HAL_I2C1_send_7bit_address_TX( u8_t dev_add );
void HAL_I2C1_send_7bit_address_RX( u8_t dev_add );
void HAL_I2C1_send_data( u8_t data );
void HAL_I2C1_receive_data( u8_t* data );
void HAL_I2C1_generate_stop( void );
void HAL_I2C1_finish_read( void );
void HAL_I2C1_finish_write( void );



#endif /* HAL_I2C_H multiple inclusion guard */

/****************************** END OF FILE *******************************************************/
