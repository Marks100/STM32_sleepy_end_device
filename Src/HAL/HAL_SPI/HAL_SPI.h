/*! \file
*               $Revision: 16923 $
*
*               $Author: mstewart $
*
*               $Date: 2014-01-16 15:40:40 +0000 (Thu, 16 Jan 2014) $
*
*               $HeadURL: https://selacvs01.schrader.local:8443/svn/ECU_Software/LF_TOOL_GEN2/trunk/Src/HAL/HAL_SPI/HAL_SPI.h $
*
*   \brief      Public header file for HAL_SPI module.
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
#ifndef HAL_SPI_PUB_H
#define HAL_SPI_PUB_H

/***************************************************************************************************
**                              Includes                                                          **
***************************************************************************************************/
#include "C_defs.h"
#include "COMPILER_defs.h"


/***************************************************************************************************
**                              Defines                                                           **
***************************************************************************************************/
/* None */
#define HAL_SPI_LL_TIMEOUT ( 1000u )



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
void HAL_SPI_init( void );
void HAL_SPI_de_init( void );
false_true_et HAL_SPI_get_init_status( void );
u8_t HAL_SPI_write_and_read_data( u8_t tx_data );
void HAL_SPI_disable_rx_interrupt( void );
void HAL_SPI_enable_rx_interrupt( void );
void HAL_SPI_disable_tx_interrupt( void );
void HAL_SPI_enable_tx_interrupt( void );
void HAL_SPI_reset_lpm_exit_flag( void );
u8_t* HAL_SPI_get_bluetooth_cmd_buffer( void );
void HAL_SPI_clear_receive_spi_buffer( void );


#endif /* HAL_SPI_PUB_H multiple inclusion guard */

/****************************** END OF FILE *******************************************************/
