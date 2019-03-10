#ifndef NVM_H
#define NVM_H

//#include "COMPILER_defs.h"
//#include "COMPILER_config.h"
#include "C_defs.h"
#include "NRF24.h"

/***************************************************************************************************
**                              Defines                                                          **
***************************************************************************************************/
#define NVM_FLASH_PTR_START_ADDR     ((u32_t)0x08008000)
#define NVM_FLASH_PTR_END_ADDR		 ((u32_t)0x080087FF)

#define MAX_NUM_FLASH_BYTES       256u
#define CRC_BYTE                  255u


/***************************************************************************************************
**                              Structure Types                                                  **
***************************************************************************************************/

typedef enum NVM_module_state
{
    NVM_STATE_INITIALISING         = 0,
    NVM_STATE_INITIALISED          = 1,
    NVM_STATE_ERROR                = 2

} NVM_module_state_et;

typedef enum
{
    NO_EE_ERROR = 0,
    SEGMENT_TOO_SMALL,
    MISMATCHED_CRC,
    BYTE_ALLIGNMENT_FAIL,
}ee_error_code_et;


/**************************************************************************************************/
typedef enum
{
    NVM_VERS_GENERIC_DATA         = 78,
} NVM_blk_version_et;
/***************************************************************************************************
This is the version number of the NVM block, This needs to be incremented when changing the layout
the structure of the block or when adding new bytes in
***************************************************************************************************/


typedef struct
{
    u32_t sleep_time;
    NRF24_state_et nrf_startup_tx_rx_mode;

} NVM_generic_data_blk_st;


#pragma pack (1)
typedef struct
{
    u8_t checksum;
    u8_t version;
    u32_t write_count;
    NVM_generic_data_blk_st NVM_generic_data_blk_s;
} NVM_info_st;








/***************************************************************************************************
**                              Exported Globals                                                  **
***************************************************************************************************/
extern NVM_info_st NVM_info_s;


/***************************************************************************************************
**                              Function Prototypes                                               **
***************************************************************************************************/
void NVM_init(void);
void NVM_request_flush(void);
NVM_module_state_et NVM_get_status( void );
void NVM_tick( void );
ok_nok_et NVM_check_blk_crc_and_version( void );
false_true_et NVM_populate_blk_crc_and_version( void );
false_true_et NVM_write_segment ( void );


#endif
