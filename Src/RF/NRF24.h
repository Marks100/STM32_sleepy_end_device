/*! \file
*               $Revision: 16923 $
*
*               $Author: mstewart $
*
*               $Date: 2014-01-16 15:40:40 +0000 (Thu, 16 Jan 2014) $
*
*               $HeadURL: https://selacvs01.schrader.local:8443/svn/ECU_Software/LF_TOOL_GEN2/trunk/Src/HAL/RF/RF.h $
*
*   \brief      Public header file for RF module.
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
#ifndef RF_H
#define RF_H

/***************************************************************************************************
**                              Includes                                                          **
***************************************************************************************************/
#include "C_defs.h"
#include "COMPILER_defs.h"



/***************************************************************************************************
**                              Defines                                                           **
***************************************************************************************************/

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define MASK_RX_P_NO 3
#define MASK_TX_FULL 0
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      3
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5	    5
#define DPL_P4	    4
#define DPL_P3	    3
#define DPL_P2	    2
#define DPL_P1	    1
#define DPL_P0	    0
#define EN_DPL	    2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0
#define CONT_WAVE   7


/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

#define NRF_TEST_MODE               1u
#define NRF_PA_LNA  				1u

#define NRF24_TICK_RATE_MS          20u
#define NRF24_TX_SCHEDULE_TIME_MS   500u

#define NRF24_TX_SCHEDULE_CNT       ( NRF24_TX_SCHEDULE_TIME_MS / NRF24_TICK_RATE_MS )
#define NRF24_WRITE_BIT             0x20
#define NRF_MAX_CHANNEL_SELECTION   126u
#define NRF_DEF_CHANNEL_SELECTION   108u
#define NRF_MAX_NUM_PIPES           5u
#define NRF_MAX_NUM_RETRIES         15u
#define NRF_DATA_PIPE_OFFSET        10u
#define NRF_MAX_PAYLOAD_SIZE        32u
#define NRF_MAX_STATS_SIZE          1000u
#define NRF_NUM_RX_BUFFERS			3u
#define NRF_PACKET_CTR_SIZE         2u

#define NRF24_DEFAULT_CONFIGURATION_SIZE      ( 27u )



/***************************************************************************************************
**                              Constants                                                         **
***************************************************************************************************/
/* None */


/***************************************************************************************************
**                              Data Types and Enums                                              **
***************************************************************************************************/
/* None */

typedef enum
{
    NRF24_POWERING_UP  = 0,
    NRF24_INITIALISING,
    NRF24_STANDBY,
    NRF24_SETUP_TX,
    NRF24_TX,
    NRF24_TX_TEST_MODE,
    NRF24_SETUP_RX,
    NRF24_RX,
    NRF24_RX_TEST_MODE,
    NRF24_POWER_DOWN,
    NRF24_SETUP_CONST_WAVE,
    NRF24_CONST_WAVE,
} NRF24_state_et;



typedef enum
{
    NRF_POWER_DOWN_MODE = 0u,
    NRF_STANDBY_1_MODE,
    NRF_STANDBY_2_MODE,
    NRF_RX_MODE,
    NRF_TX_MODE
} NRF24_low_level_mode_et;



typedef enum
{
	CONFIG 			= 0,
	EN_AUTO_ACK,
	EN_RXADDR,
	SETUP_AW,
	SETUP_RETR,
	RF_CH,
	RF_SETUP,
	STATUS,
	OBSERVE_TX,
	CD,
	RX_ADDR_P0,
	RX_ADDR_P1,
	RX_ADDR_P2,
	RX_ADDR_P3,
	RX_ADDR_P4,
	RX_ADDR_P5,
	TX_ADDR ,
	RX_PW_P0,
	RX_PW_P1,
	RX_PW_P2,
	RX_PW_P3,
	RX_PW_P4,
	RX_PW_P5,
	FIFO_STATUS,
	DYNPD			= 28,
	FEATURE,
    ADDRESS_NOP,
	REGISTER_MAX
} NRF24_registers_et;

typedef enum
{
    /* Instruction Mnemonics */
    R_REGISTER          = 0x00,
    W_REGISTER          = 0x20,
    REGISTER_MASK       = 0x1F,
    ACTIVATE            = 0x50,
    R_RX_PL_WID         = 0x60,
    R_RX_PAYLOAD        = 0x61,
    W_TX_PAYLOAD        = 0xA0,
    W_ACK_PAYLOAD       = 0xA8,
    FLUSH_TX            = 0xE1,
    FLUSH_RX            = 0xE2,
    REUSE_TX_PL         = 0xE3,
    W_TX_PAYLOAD_NO_ACK = 0xB0,
    NOP                 = 0xFF
} NRF24_instruction_et;


typedef enum
{
    RF_MIN_TX_PWR = 0u,
    RF_MED_1_TX_PWR,
    RF_MED_2_TX_PWR,
    RF_MAX_TX_PWR
} NRF24_power_level_et;

typedef enum
{
  	TX_ENABLE = 0u,
	RX_ENABLE
} NRF24_tx_rx_mode_et;

typedef enum
{
    ADDRESS_ILLEGAL = 0u,
  	ADDRESS_3_BYTES,
  	ADDRESS_4_BYTES,
  	ADDRESS_5_BYTES
} NRF24_address_width_et;

typedef enum
{
    RF24_1MBPS = 0u,
    RF24_2MBPS,
    RF24_250KBPS
} NRF24_air_data_rate_et;

typedef enum
{
    RF24_250_US = 0u,
    RF24_500_US,
    RF24_750_US,
    RF24_1000_US,
    RF24_1250_US,
    RF24_1500_US,
    RF24_1750_US,
    RF24_2000_US,
    RF24_2250_US,
    RF24_2500_US,
    RF24_2750_US,
    RF24_3000_US,
    RF24_3250_US,
    RF24_3500_US,
    RF24_3750_US,
    RF24_4000_US
} NRF24_retransmitt_time_et;

typedef enum
{
  RF24_CRC_8 = 0,
  RF24_CRC_16
} NRF24_crclength_et;

typedef enum
{
    NRF24_DEFAULT_CONFIG,
    NRF24_CFG_MAX
} NRF24_static_configuration_et;


typedef struct
{
   NRF24_registers_et NRF24_register;
   u8_t register_data;
} NRF24_register_data_st;


typedef struct
{
    NRF24_register_data_st* buffer_p;  //! pointer to configuration
    u16_t length;                      //! length of configuration
} NRF24_static_configuration_st;

typedef enum
{
    RF24_RX_DATA_READY = 0u,
    RF24_TX_DATA_SENT,
    RF24_MAX_RETR_REACHED,
    RF24_RX_PIPE_DATA_NUM,
    RF24_TX_FIFO_FULL
} NRF24_status_masks_et;

typedef enum
{
    RF24_RX_EMPTY = 0u,
    RF24_RX_FULL,
    RF24_TX_EMPTY = 4u,
    RF24_TX_FULL,
    RF24_TX_REUSE
} NRF24_fifo_masks_et;


typedef struct
{
    u8_t  NRF24_rx_rf_payload[NRF_MAX_PAYLOAD_SIZE * NRF_NUM_RX_BUFFERS];
    u8_t  NRF24_rx_payload_size;
    u16_t NRF24_rx_failed_ctr;
    u16_t NRF24_rx_missed_packets;
    u8_t  NRF24_tx_rf_payload[NRF_MAX_PAYLOAD_SIZE];
    u8_t  NRF24_tx_payload_size;
    u16_t NRF24_tx_payload_ctr;
    u16_t NRF24_rx_payload_ctr;
    u8_t  NRF24_tx_retry_ctr;
    u16_t NRF24_tx_failed_ctr;
    u8_t  NRF24_tx_high_retry_cnt_s;
} NRF24_tx_rx_payload_info_st;


/***************************************************************************************************
**                              Exported Globals                                                  **
***************************************************************************************************/
/* None */



/***************************************************************************************************
**                              Function Prototypes                                               **
***************************************************************************************************/
pass_fail_et         NRF24_init( void );
void                 NRF24_tick( void );
pass_fail_et         NRF24_set_configuration( NRF24_static_configuration_et config );
pass_fail_et         NRF24_read_registers( NRF24_instruction_et instruction, NRF24_registers_et address, u8_t read_data[], u8_t num_bytes );
pass_fail_et         NRF24_write_registers( NRF24_instruction_et instruction, NRF24_registers_et address, u8_t write_data[], u8_t num_bytes );
pass_fail_et         NRF24_set_low_level_mode( NRF24_low_level_mode_et mode );
pass_fail_et         NRF24_set_channel( u8_t channel );
pass_fail_et         NRF24_flush_rx( void ) ;
pass_fail_et         NRF24_flush_tx( void );
NRF24_power_level_et NRF24_get_PA_TX_power(void);
pass_fail_et         NRF24_set_PA_TX_power( NRF24_power_level_et value);
u8_t                 NRF24_get_status(void);
pass_fail_et         NRF24_set_rf_data_rate( NRF24_air_data_rate_et value );
void                 NRF24_setup_CRC_scheme( disable_enable_et state, NRF24_crclength_et crc_len );
void                 NRF24_setup_constant_wave( disable_enable_et state );
void                 NRF24_setup_pll( disable_enable_et state );
pass_fail_et         NRF24_open_write_data_pipe( u8_t pipe_num, const u8_t* data_pipe_address );
u8_t                 NRF24_get_FIFO_status(void);
pass_fail_et         NRF24_status_register_clr_bit( u8_t bit_mask );
pass_fail_et         NRF24_send_payload( void );
pass_fail_et         NRF24_get_payload( u8_t* buffer );
pass_fail_et         NRF24_toggle_features_register( void );
pass_fail_et         NRF24_set_AA_data_pipe( disable_enable_et state, u8_t pipe_num );
pass_fail_et         NRF24_set_reuse_tx_payload( disable_enable_et state );
pass_fail_et         NRF24_set_dynamic_payloads( disable_enable_et state, u8_t pipe_num );
void                 NRF24_setup_address_widths( NRF24_address_width_et value );
pass_fail_et         NRF24_enable_data_pipe( disable_enable_et state, u8_t pipe_num );
pass_fail_et         NRF24_setup_retries( NRF24_retransmitt_time_et time, u8_t counts );
low_high_et          NRF24_check_status_mask( NRF24_status_masks_et mask, u8_t* data_p );
low_high_et          NRF24_check_fifo_mask( NRF24_fifo_masks_et mask, u8_t* data_p );
pass_fail_et         NRF24_read_data_pipe( u8_t pipe_num, const u8_t* data_p );
u8_t 				 NRF24_get_retry_count( void );
pass_fail_et 		 NRF24_setup_dynamic_ack( disable_enable_et state );

pass_fail_et         NRF24_read_all_registers( u8_t* data_p );
void 				 NRF24_handle_packet_stats( u8_t type );
void                 NRF24_setup_payload( u8_t* data_p, u8_t len );
NRF24_state_et       NRF24_get_state( void );
void                 NRF24_complete_flush( void );
false_true_et        NRF24_scheduled_tx( void );
void                 NRF24_handle_acks_and_tx_failures( void );
void                 NRF24_set_state( NRF24_state_et state );
void                 NRF24_spi_slave_select( low_high_et state );
void                 NRF24_ce_select( low_high_et state );
void 				 NRF_simple_send( u8_t* data_p, u8_t len, u8_t num );
void                 NRF24_power_down( void );
pass_fail_et 		 NRF_get_failure_status( void );



#endif /* RF_H multiple inclusion guard */

/****************************** END OF FILE *******************************************************/
