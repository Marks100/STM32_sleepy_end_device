/*! \file
*               $Revision: 16923 $
*
*               $Author: mstewart $
*
*               $Date: 2014-01-16 15:40:40 +0000 (Thu, 16 Jan 2014) $
*
*               $HeadURL: https://selacvs01.schrader.local:8443/svn/ECU_Software/LF_TOOL_GEN2/trunk/Src/HAL/HAL_SPI/HAL_SPI.c $
*
*   \brief      SPI interface module
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
//#include "stm32f10x_spi.h"

#include "C_defs.h"
#include "COMPILER_defs.h"
#include "HAL_BRD.h"
//#include "HAL_UART.h"
#include "HAL_SPI.h"



/* Module Identification for STDC_assert functionality */
#define STDC_MODULE_ID   STDC_MOD_HAL_SPI

static false_true_et HAL_SPI_initialised = FALSE;







/***************************************************************************************************
**                              Data declaratons and definitions                                 **
***************************************************************************************************/
/* None */



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
*   \note          Fixed baudrate for now at 5MHz 8N1
*
***************************************************************************************************/
void HAL_SPI_init( void )
{
	SPI_InitTypeDef   SPI_InitStructure;

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 0xAAAA;

	SPI_Init(SPI1, &SPI_InitStructure);

	/* Enable SPIz */
	SPI_Cmd(SPI1, ENABLE);

    HAL_SPI_initialised = TRUE;
}




void HAL_SPI_de_init( void )
{
    HAL_SPI_initialised = FALSE;
}



/*!
****************************************************************************************************
*
*   \brief         Return the init status of the SPI module
*
*   \author        MS
*
*   \return        none
*
***************************************************************************************************/
false_true_et HAL_SPI_get_init_status( void )
{
    return ( HAL_SPI_initialised );
}


/*!
****************************************************************************************************
*
*   \brief         Writes a buffer of information out to UART
*
*   \author        MS
*
*   \return        none
*
***************************************************************************************************/
u8_t HAL_SPI_write_and_read_data( u8_t tx_data )
{
    u16_t i;
    u8_t dummy_byte;
    u8_t return_value;

	/* First lets do a dummy read to make sure that the interrupt
	flag is clear and that the buffer is empty*/
	//dummy_byte = SPI_I2S_ReceiveData( SPI1 );

//	/* Send SPI1 data */
//	SPI_I2S_SendData(SPI1, tx_data);
//
//    /* Wait for SPI1 Tx buffer to empty */
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
//
//	/* Wait for SPI1 Rx buffer to not be empty */
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
//
//	/* Wait for the SPI busy flag to clear */
//	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
//
//	/* Read the bute on the SPE receive register */
//	return_value = SPI_I2S_ReceiveData( SPI1 );

	/* Send SPI1 data */
	SPI_I2S_SendData(SPI1, tx_data);

	/* Wait for the SPI busy flag to clear */
	//while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);

	/* Read the buffer on the SPE receive register */
	return_value = SPI_I2S_ReceiveData( SPI1 );







    return ( return_value );
}

/*!
****************************************************************************************************
*
*   \brief
*
*   \author        FK
*
*   \return        none
*
***************************************************************************************************/
void HAL_SPI_disable_rx_interrupt( void )
{

}

/*!
****************************************************************************************************
*
*   \brief
*
*   \author        MS
*
*   \return        none
*
***************************************************************************************************/
void HAL_SPI_enable_rx_interrupt( void )
{

}


/*!
****************************************************************************************************
*
*   \brief
*
*   \author        MS
*
*   \return        none
*
***************************************************************************************************/
void HAL_SPI_enable_tx_interrupt( void )
{
}


/*!
****************************************************************************************************
*
*   \brief
*
*   \author        MS
*
*   \return        none
*
***************************************************************************************************/
void HAL_SPI_disable_tx_interrupt( void )
{
}



/*!
****************************************************************************************************
*
*   \brief
*
*   \author        MS
*
*   \return        none
*
***************************************************************************************************/
void HAL_SPI_clear_receive_spi_buffer( void )
{

}



/***************************************************************************************************
**                              Private Functions                                                 **
***************************************************************************************************/
/* None */


/***************************************************************************************************
**                              ISR Handlers                                                      **
***************************************************************************************************/

/****************************** END OF FILE *******************************************************/
