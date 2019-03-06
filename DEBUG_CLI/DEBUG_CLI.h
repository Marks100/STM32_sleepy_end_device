/*! \file       DEBUG_CLI.H
*
*   \brief      Debug cli file for the WES
*/
/* =================================== COPYRIGHT NOTICE ============================================
* The contents of this document are protected under copyright and contain commercially and/or
* technically confidential information. The content of this document must not be used other than
* for the purpose for which it was provided nor may it be disclosed or copied (by the authorised
* recipient or otherwise) without the prior written consent of an authorised officer of Sensata
*
*   Copyright 2016 Sensata Technologies.
*
***************************************************************************************************/

#ifndef DEBUG_CLI_H
#define DEBUG_CLI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "COMPILER_defs.h"
#include "COMPILER_config.h"
#include "C_defs.h"
#include "WESAlgo.h"


/* BIOS module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>

/***************************************************************************************************
**                              Defines                                                          **
***************************************************************************************************/
#define DEBUG_CLI_TASK_TIMEOUT                                  800u   //( 8s timeout ) Value gets Multiplied by the UART read timeout value which is 10ms
#define DEBUG_CLI_MAX_INPUT_CHARS                               40u
#define DEBUG_CLI_CMD_LINE_ARGS_MAX                             6
#define DEBUG_CLI_ACC_TEST_DATA_SIZE	                        256u
#define DEBUG_CLI_SAMPLES_PER_CYCLE		                        100u
#define DEBUG_CLI_MAX_COMMAND_HISTORY                           4
#define DEBUG_CLI_BACKSPACE                                     0x08
#define DEBUG_CLI_CR                                            0x0D
#define DEBUG_CLI_SPACE                                         0x20
#define DEBUG_CLI_DEL                                           0x7F

#define UP_ARROW                                                0x41
#define DOWN_ARROW                                              0x42
#define ERASE_CURRENT_LINE                                      "\r                                   \r\b\b\b"
#define NULL_PARAM_LIST                                         //{0}
#define IGNORE_RANGE_CHECK                                      ((u32_t)~0)

#define SLEEP_DURATION_SECS_MINIMUM                             1
#define SLEEP_DURATION_SECS_MAXIMUM                             10000

#define WES_ID_LOWER_LIMIT                                      0x0000
#define WES_ID_UPPER_LIMIT                                      0xFFFE

#define POWER_LOWER_LIMIT                                       0
#define POWER_UPPER_LIMIT                                       5
#define MAX_USER_CONFIGURABLE_POWER_VALUES                      6

#define MIN_DEFAULT_CHILDTIMEOUT_SECS                           240
#define MIN_PARENT_SEARCH_RETRIES								1u
#define MAX_PARENT_SEARCH_RETRIES								30u
#define MIN_PARENT_SEARCH_INTERVAL_TIME							1u
#define MAX_PARENT_SEARCH_INTERVAL_TIME							60u

#define MAX_ALLOWED_CHILDTIMEOUT_VAL_IN_SECS                    (u32_t)(31536000)       // approx 1 year!!

#define SUPPORTED_FOR_ALL_MODES                                 0xFFFF

#define WES_WAKE_ALGO_WAKE_SCHEDULE_FTM2_MAX                    (255u)


#define HELP_HELP                "help:              help\r\n"
#define HELP_NETWORK             "network:           Reads the current Network Credentials\r\n"
#define HELP_PAN_ID              "panid:             Sets the network pan id (e.g beef, faab, cafe )\r\n"
#define HELP_EXT_PAN_ID          "extpanid:          Sets the network extended pan id (e.g dead00beef00cafe )\r\n"
#define HELP_EUI64               "eui64:             Get 64bit Unique ID\r\n"
#define HELP_MODE                "mode:              Sets the operating mode of the WES\r\n"
#define HELP_HBHA                "hbha:              Prints all of the relevant Bearing Health stats\r\n"
#define HELP_VER                 "ver:               Returns the SW and HW versions of the WES\r\n"
#define HELP_RESET               "reset:             Resets CPU\r\n"
#define HELP_NVM                 "nvm:               Returns the current NVM info\r\n"
#define HELP_NORMAL_SLEEP        "normalcycle:       Sets the sleep/standby duration after power up for normal mode\r\n"
#define HELP_FTM1_SLEEP          "ftm1cycle:         Sets the cycle time for FTM1\r\n"
#define HELP_FTM2_SLEEP          "ftm2cycle:         Sets the cycle time for FTM2\r\n"
#define HELP_FTM2_ALGO_SLEEP     "ftm2algo:          Sets the number of TPMS loops that get executed before the HBHA algo runs\r\n"
                                                     
#define HELP_SPEED_CHECK         "speedcheck:        <0 - Real, 1 - Spoofed> Sets the state of the speed check in FTM2\r\n"
#define HELP_PARENTSEARCHTIME    "parentsearchtime:  <0..100> Sets the time in secs between parent search attempts\r\n"
#define HELP_PARENTSEARCHLIMIT   "parentsearchlimit: <0..100> Sets the number of time that the parent search is attempted\r\n"

#define HELP_ID                  "id:                <0..9999> Sets the 16bit WES ID ( eg 0001 or 1008 )\r\n"
#define HELP_POWER               "power:             <0..5> Sets the TX power of the radio ( eg power 0 - MIN, power 5 - MAX )\r\n"
#define HELP_FRAMETYPE           "frametype:         <0..5> Selects the type of RF frame to be transmitted\r\n"
#define HELP_RFCHAN              "rfchan:            <11..26> Selects the Rf channel to operate on\r\n"
#define HELP_CHILDTIMEOUT        "childtimeout       <240.. 31536000 secs(1 year)> Sets the timeout value which corresponds\r\n"\
                                 "                   to childtimeout on parent\r\n"
#define HELP_DEVELOPER           "developer:         Enable or disable developer console commands(toggle operation)\r\n"

//debug command help
#define HELP_ASIC                "asic:              Returns the current ASIC decoded info from both asics\r\n"
#define HELP_BATT                "batt:              Returns the current WES battery voltage using the AONBATMON\r\n"
#define HELP_TEMP                "temp:              Returns the current WES internal temperature using the AONBATMON\r\n"
#define HELP_FTEST               "ftest:             Runs the EXT Flash chip self test\r\n"
#define HELP_FREAD               "fread:             <0..511><all> Reads the selected flash Sector in HEX, fread all reads all sectors \r\n"
#define HELP_FERASE              "ferase:            <0..511> Erase the selected flash sector( 4K )\r\n"
#define HELP_FCLR                "fclr:              Wipes the entire flash chip memory\r\n"
#define HELP_ACCTEST             "acctest:           Runs the accelerometer self test feature\r\n"

//ACC test commands help
#define HELP_ACC                 "acc:               <x or y or z or xy or yz or xz or xyz> <1..4096> Records and Prints acc samples for the selected axis\r\n"
                                                    
#define HELP_ACCALL              "accall:            Prints a single sample for all the three axis along with supply voltage\r\n"
                                                    
//RF test commands help                             
#define HELP_RFTEST              "rftest:            Test status\r\n"
#define HELP_RFTESTSTART         "rfteststart:       Start RF test mode\r\n"
#define HELP_RFTESTSTOP          "rfteststop:        Stop RF test mode\r\n"
#define HELP_RFTESTCHAN          "rftestchannel:     <11..26> - Set/Get RF test channel \r\n"
#define HELP_RFTESTPOWER         "rftestpower:       <-21,-18,-15,-12,-10,-9,-6,-5,-3,0-5> - Set/Get RF test power (dbm) \r\n"
#define HELP_RFTESTTX            "rftesttx:          <pkt count, pkt length(bytes)> - Tx fixed number of packets with fixed length\r\n"
#define HELP_RFTESTCONT          "rftestcont:        <pkt delay(ms), pkt length(bytes)> - Repeatedly tx packets at a fixed interval\r\n"
//check if the following is supported or not!!! looks like we dont. remove all references to thsi command if so!
#define HELP_RFTESTCONTSTOP      "rftestcontstop:    Stop continuous tx mode\r\n"
#define HELP_RFTESTSTATS         "rfteststats:       Display radio statistics\r\n"
#define HELP_RFTESTRRSSI         "rftestrssi:        Return max and last rssi observed in operation\r\n"
#define HELP_RFTESTSLEEP         "rftestsleep:       Put radio in sleep mode\r\n"
                                                    
//vehicle test RF commands help                     
#define HELP_ACKREQ              "ackreq:            Enable or disable MAC ACK requests\r\n"\
                                 "                   0 - DISABLE, 1 - ENABLE\r\n"
#define HELP_CLEARSCR            "clrscr:            Clears the screen\r\n"
                                                    
// functional test mode 1 commands help             
#define HELP_DV                  "dv:                Prints all the results\r\n"


#define PAN_ID_CMD_PARAM_LIST {\
		HEX,HEX_WORD_16_MIN_NUM_CHARS,(u32_t)0x0001,(u32_t)0xFFFE }

#define EXT_PAN_ID_CMD_PARAM_LIST {\
		HEX,HEX_WORD_64_MIN_NUM_CHARS,IGNORE_RANGE_CHECK,IGNORE_RANGE_CHECK }

#define MODE_CMD_PARAM_LIST {\
        DEC,DEC_BYTE_8_MAX_NUM_CHARS,(u32_t)MODE_MGR_SHIPPING_MODE,(u32_t)(MODE_MGR_MODE_MAX-1)}

#define SLEEP_CMD_PARAM_LIST {\
        DEC,DEC_WORD_16_MAX_NUM_CHARS,(u32_t)SLEEP_DURATION_SECS_MINIMUM,(u32_t)SLEEP_DURATION_SECS_MAXIMUM}

#define WES_ID_CMD_PARAM_LIST {\
        HEX,HEX_WORD_16_MIN_NUM_CHARS,(u32_t)WES_ID_LOWER_LIMIT,(u32_t)WES_ID_UPPER_LIMIT}


#define POWER_CMD_PARAM_LIST {\
        DEC,DEC_BYTE_8_MAX_NUM_CHARS,(u32_t)POWER_LOWER_LIMIT,(u32_t)POWER_UPPER_LIMIT}

#define FRAMETYPE_CMD_PARAM_LIST {\
        DEC,DEC_BYTE_8_MAX_NUM_CHARS,(u32_t)WES_ORIGINAL_DEBUG_TEST_FRAME,(u32_t)WES_TEST_MODE_FRAME}

#define RFCHAN_CMD_PARAM_LIST {\
        DEC,DEC_BYTE_8_MAX_NUM_CHARS,(u32_t)OT_STACK_CHAN_MIN,(u32_t)OT_STACK_CHAN_MAX}

#define CHILDTIMEOUT_CMD_PARAM_LIST {\
        DEC,DEC_WORD_32_MAX_NUM_CHARS,(u32_t)MIN_DEFAULT_CHILDTIMEOUT_SECS,(u32_t)MAX_ALLOWED_CHILDTIMEOUT_VAL_IN_SECS}

#define FREAD_CMD_PARAM_LIST {\
        DEC,DEC_WORD_16_MAX_NUM_CHARS,(u32_t)0,(u32_t)EXT_FLASH_TOP_SECTOR}

#define FERASE_CMD_PARAM_LIST {\
        DEC,DEC_WORD_16_MAX_NUM_CHARS,(u32_t)0,(u32_t)EXT_FLASH_TOP_SECTOR}

#define ACC_CMD_PARAM_LIST {\
        {STR,STR_LEN_MAX_NUM_CHARS,(u32_t)IGNORE_RANGE_CHECK,(u32_t)IGNORE_RANGE_CHECK},\
        {DEC,DEC_WORD_16_MAX_NUM_CHARS,(u32_t)1,(u32_t)WES_WAKE_ALGO_VIB_NUM_DATA_SAMPLES}}

#define RFTEST_CHAN_CMD_PARAM_LIST {\
        STR,STR_LEN_MAX_NUM_CHARS,(u32_t)IGNORE_RANGE_CHECK,(u32_t)IGNORE_RANGE_CHECK}

#define RFTEST_POWER_CMD_PARAM_LIST {\
        STR,STR_LEN_MAX_NUM_CHARS,(u32_t)IGNORE_RANGE_CHECK,(u32_t)IGNORE_RANGE_CHECK}

#define RFTEST_TX_CMD_PARAM_LIST {\
        {STR,STR_LEN_MAX_NUM_CHARS,(u32_t)IGNORE_RANGE_CHECK,(u32_t)IGNORE_RANGE_CHECK},\
        {STR,STR_LEN_MAX_NUM_CHARS,(u32_t)IGNORE_RANGE_CHECK,(u32_t)IGNORE_RANGE_CHECK}}

#define RFTEST_CONT_CMD_PARAM_LIST {\
        {STR,STR_LEN_MAX_NUM_CHARS,(u32_t)IGNORE_RANGE_CHECK,(u32_t)IGNORE_RANGE_CHECK},\
        {STR,STR_LEN_MAX_NUM_CHARS,(u32_t)IGNORE_RANGE_CHECK,(u32_t)IGNORE_RANGE_CHECK}}

#define ACKREQ_CMD_PARAM_LIST {\
        DEC,DEC_BOOLEAN_MAX_NUM_CHARS,(u32_t)SEL_OFF,(u32_t)SEL_ON}

#define SPEEDCHEK_CMD_PARAM_LIST {\
         DEC,DEC_BOOLEAN_MAX_NUM_CHARS,(u32_t)SEL_OFF,(u32_t)SEL_ON}

#define PARENTSEARCHTIME_CMD_PARAM_LIST {\
        DEC,DEC_BYTE_8_MAX_NUM_CHARS,(u32_t)MIN_PARENT_SEARCH_INTERVAL_TIME,(u32_t)MAX_PARENT_SEARCH_INTERVAL_TIME}

#define PARENTSEARCHLIMIT_CMD_PARAM_LIST {\
        DEC,DEC_BYTE_8_MAX_NUM_CHARS,(u32_t)MIN_PARENT_SEARCH_RETRIES,(u32_t)MAX_PARENT_SEARCH_RETRIES}

/***************************************************************************************************
**                              Structure Types                                                  **
***************************************************************************************************/
typedef struct
{
    wesResult        DEBUG_WES_WAKE_ALGO_result_s;
    u32_t            DEBUG_WES_WAKE_ALGO_run_bearing_time_s;
    u32_t            DEBUG_WES_WAKE_ALGO_break_in_ctr_s;
    u32_t            DEBUG_WES_WAKE_ALGO_tpms_count_s;
    processReadiness DEBUG_WES_WAKE_ALGO_process_condition_s;
    u32_t            DEBUG_WES_WAKE_ALGO_run_cnt_s;
    signalStat       DEBUG_WES_WAKE_ALGO_signal_stats_s;
} DEBUG_wes_stats_st;

typedef enum
{
    DEBUG_CLI_ERROR_NONE = 0,
    DEBUG_CLI_ERROR_EMPTY,
    DEBUG_CLI_ERROR_FAILED,
    DEBUG_CLI_ERROR_INVALID_ARGS_MORE,
    DEBUG_CLI_ERROR_INVALID_ARGS,
    DEBUG_CLI_ERROR_NOT_SUPPORTED,
    DEBUG_CLI_ERROR_PROHIBITED,
    DEBUG_CLI_ERROR_NOT_FOUND,
    DEBUG_CLI_ERROR_PARAM_OUT_OF_RANGE,
    DEBUG_CLI_ERROR_INVALID_HEX_PARAM,
    DEBUG_CLI_ERROR_INVALID_DEC_PARAM,
    DEBUG_CLI_ERROR_INVALID_HEX_PARAM_LEN,
    DEBUG_CLI_ERROR_INVALID_DEC_PARAM_LEN,
    DEBUG_CLI_ALREADY_SET,
    DEBUG_CLI_ERROR_INVALID_STR_PARAM,
    DEBUG_CLI_ERROR_MAX
} DEBUG_CLI_error_et;

typedef enum
{
    POWER_MINUS_21_dBm = 0,
    POWER_MINUS_12_dBm,
    POWER_MINUS_6_dBm,
    POWER_0_dBm,
    POWER_3_dBm,
    POWER_5_dBm,
    POWER_MAX_VAL
}TX_POWER_et;

typedef enum
{
    DEC = 0,
    HEX,
    STR
} DEBUG_CLI_param_format_et;

typedef enum
{
    //hex param char lengths
    HEX_BYTE_8_MIN_NUM_CHARS = 2, // 0xFF
    HEX_WORD_16_MIN_NUM_CHARS = 4, // 0xFFFF
    HEX_WORD_32_MIN_NUM_CHARS = 8,  //0xFFFFFFFF
    HEX_WORD_64_MIN_NUM_CHARS = 16,  //4294967295
    //dec param char lens
    DEC_BOOLEAN_MAX_NUM_CHARS = 1,
    DEC_BYTE_8_MAX_NUM_CHARS = 3,
    DEC_WORD_16_MAX_NUM_CHARS = 5, // 65535
    DEC_WORD_32_MAX_NUM_CHARS = 10,  //4294967295
    STR_LEN_MAX_NUM_CHARS     = 8
} DEBUG_CLI_param_char_len_et;

typedef union
{
    u8_t  val_8;
    u16_t val_16;
    u32_t val_32;
    u8_t  val_64[8];
    u8_t  str[STR_LEN_MAX_NUM_CHARS];
} DEBUG_CLI_Param_value_ut;

typedef struct
{
    DEBUG_CLI_param_format_et format;         // parameters are specified in hex or decimal??
    DEBUG_CLI_param_char_len_et num_chars;    // maximum of 8 bytes as of now for extended pan id! but this may be 16 bytes if we are setting any keys.
    u32_t llimit;  // bit map setting of all modes in which this cmd would be supported
    u32_t ulimit;  // if this is TRUE it means this command is exposed to customers otherwise it is for developers
} DEBUG_CLI_Param_st;


typedef struct
{
    const char *mName;
    DEBUG_CLI_error_et (*mHandler)(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list);
    const char *helpinfo;
    u16_t availbility;  // bit map setting of all modes in which this cmd would be supported
    SEL_customer_enable_disable_et public;        // if this is TRUE it means this command is exposed to customers otherwise it is for developers
    u8_t num_params;
    const DEBUG_CLI_Param_st param_list[DEBUG_CLI_CMD_LINE_ARGS_MAX];
} DEBUG_CLI_Command_st;


/***************************************************************************************************
**                              Function Prototypes                                               **
***************************************************************************************************/
void DEBUG_CLI_taskCreate( void );
void DEBUG_CLI_post( void );
void DEBUG_CLI_pend( void );
void DEBUG_CLI_reset_task_timeout( void );
void DEBUG_CLI_handle_received_char( u8_t received_char );

STATIC uint64_t StrToHex(const char* str);
STATIC void DEBUG_CLI_init(void);
STATIC void DEBUG_CLI_close(void);
STATIC void DEBUG_CLI_RX_PIN_interrupt_handler(uint_least8_t index);
STATIC void DEBUG_CLI_setup_rx_wakeup( void );
STATIC void DEBUG_CLI_receive_byte( void );
STATIC void DEBUG_CLI_clear_rx_buffer( void );

// normal mode command handlers
STATIC DEBUG_CLI_error_et  handle_help(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_network(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_panid(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list);
STATIC DEBUG_CLI_error_et  handle_extpanid(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_eui64(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list);
STATIC DEBUG_CLI_error_et  handle_mode(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_hbha(int aArgCount,DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_ver(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_reset(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_nvm(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_normal_sleep(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_ftm1_sleep(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_ftm2_sleep(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_ftm2_algo(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_speedcheck(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_power(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_frametype(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_rfchan(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_childtimeout(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_developer(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_asic(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_batt(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_temp(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_parentsearchtime(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_parentsearchlimit(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );

//Flash streaming mode handlers
STATIC DEBUG_CLI_error_et  handle_ftest(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_fread(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_ferase(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_fclr(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );

//ACC test command handlers
STATIC DEBUG_CLI_error_et  handle_acctest(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_acc(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_accall(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );

// rf test mode command handlers
STATIC DEBUG_CLI_error_et  handle_rftest(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_rftest_start(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_rftest_stop(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_rftest_channel(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_rftest_power(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_rftest_tx(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_rftest_cont(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_rftest_cont_stop(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_rftest_stats(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_rftest_rssi(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_rftest_sleep(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );

//vehicle test RF commands handlers
STATIC DEBUG_CLI_error_et  handle_ackreq(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );

//functional test mode 1 commands handlers
STATIC DEBUG_CLI_error_et  handle_dv(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list );
STATIC DEBUG_CLI_error_et  handle_clearScreen(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list);

STATIC SEL_false_true_et   DEBUG_CLI_get_serial_message_status( void );
STATIC void 		       DEBUG_CLI_print_prompt( SEL_false_true_et newline );
STATIC DEBUG_CLI_error_et  DEBUG_CLI_get_param_value(const DEBUG_CLI_Param_st* p_param_info, char *p_param_str, DEBUG_CLI_Param_value_ut* p_val );
STATIC DEBUG_CLI_error_et  DEBUG_CLI_get_cum_validate_cmd_params(const DEBUG_CLI_Command_st *cmd, int aArgCount, char *aArgVector[], DEBUG_CLI_Param_value_ut* p_param_val_list);
STATIC DEBUG_CLI_error_et  DEBUG_CLI_process_cmd(int aArgCount, char *aArgVector[]);
STATIC DEBUG_CLI_error_et  DEBUG_CLI_parse_cmd(char *aString, uint8_t * aArgc, char **aArgv, uint8_t aArgcMax);
STATIC void 			   DEBUG_CLI_handle_serial_command(void);


#ifdef __cplusplus
}
#endif

#endif /* DEBUG_UTILS_H */

