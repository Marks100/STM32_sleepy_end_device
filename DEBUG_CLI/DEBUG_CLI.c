/******************************************************************************

 @file main.c

 @brief debug utilities


 Target Device: CC2652

 ******************************************************************************

 Copyright (c) 2018, Sensata
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
 its contributors may be used to endorse or promote products derived
 from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: tbc
 Release Date: 2018-07-20
 *****************************************************************************/

/*
 *  ======== DEBUG_CLI.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* POSIX Header files */
#include <sched.h>
#include <pthread.h>
#include <mqueue.h>
#include <semaphore.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* OpenThread public API Header files */
#include <openthread/diag.h>
#include <openthread/openthread.h>

/* OpenThread Internal/Example Header files */
#include "otsupport/otrtosapi.h"
#include "otsupport/otinstance.h"

/* driverlib specific header */
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/aon_batmon.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)
#include DeviceFamily_constructPath(driverlib/systick.h)
#include <openthread/types.h>
#include "otstack.h"

/* BIOS module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

/* Private configuration Header files */
#include "task_config.h"
#include "C_defs.h"
#include "EXT_FLASH.h"
#include "SENSOR_DECODE.h"
#include "WES_WAKE_ALGO.h"
#include "Board.h"
#include "HAL_BRD.h"
#include "HAL_UART.h"
#include "MODE_MGR.h"
#include "NVM.h"
#include "STDC.h"
#include "WESAlgo.h"
#include "DEBUG_CLI.h"



extern NVM_info_st        NVM_info_s;
STATIC DEBUG_wes_stats_st DEBUG_wes_stats_s;

/* Debug display strings */
const char DEBUG_CLI_consoleDisplay_s[] =
        "\n\rWES Debug Console now open, (Type 'help' for list of commands)\r\n";

const char DEBUG_CLI_closing_serial_port_s[] =
        "\r\nClosing serial port due to inactivity to save power, please press the \"Enter\" key to re-open the debug prompt \r\n";

/* Declare the semaphore */
Semaphore_Handle DEBUG_CLI_Sem_Handle;
Semaphore_Struct DEBUG_CLI_Sem_Struct;
Semaphore_Params DEBUG_CLI_Sem_Params;

/* UART handle */
STATIC PIN_Handle  RX_ISR_PinHandle;
STATIC PIN_State   RX_ISR_PinState;

STATIC u32_t             DEBUG_CLI_timeout_s = 0u;
STATIC SEL_false_true_et DEBUG_CLI_open_s;
STATIC SEL_false_true_et DEBUG_CLI_cr_received_s = SEL_FALSE;
STATIC u8_t              DEBUG_CLI_byte_index_s = 0u;
STATIC u8_t              DEBUG_CLI_rx_command_len = 0u;
STATIC char              DEBUG_CLI_msg_read_s[DEBUG_CLI_MAX_INPUT_CHARS];

typedef struct
{
    char cmd[DEBUG_CLI_MAX_INPUT_CHARS];
}DEBUG_CLI_cmd_st;

typedef struct
{
    u8_t index2next;
    DEBUG_CLI_cmd_st cmd_list[DEBUG_CLI_MAX_COMMAND_HISTORY];
}DEBUG_CLI_cmd_history_st;

STATIC DEBUG_CLI_cmd_history_st cmdhistory;

const char* DEBUG_CLI_err_string[ DEBUG_CLI_ERROR_MAX ] =
{
    "\r\n",
    "\r\n",
    "\r\nOperation failed\r\n",
    "\r\nToo many arguments\r\n",
    "\r\nInvalid arguments\r\n",
    "\r\nNot supported\r\n",
    "\r\nCmd prohibited in this mode\r\n",
    "\r\nCmd not found\r\n",
    "\r\nOut of range\r\n",
    "\r\nInvalid Hex param\r\n",
    "\r\nInvalid Dec param\r\n",
    "\r\nInvalid hex param length\r\n",
    "\r\nInvalid Dec param length\r\n",
    "\r\nAlready set\r\n",
    "\r\nInvalid string input\r\n"
};

const char* DEBUG_CLI_frame_type_str[ WES_FRAME_MAX ] =
{
    "Orig dbg test frame",
    "Hardcoded test frame",
    "DV frame",
    "[H] Normal frame",
    "Installation frame",
    "Test mode frame",
};

const char* DEBUG_CLI_power_val_str[ POWER_MAX_VAL ] =
{
 " -21 dBm", " -12 dBm", " -6 dBm", " 0 dBm", " 3 dBm", " 5 dBm"
};

/*! Going to try and do something to save power here, We are going to keep the serial port turned off
( which saves about 1 ma in current consumption ) until we detect a falling edge on the RX pin,
this should signify that a byte is being received, then we will open the serial port, receive the command,
and then turn the serial port back off again */
STATIC PIN_Config RX_ISR_Pin[] =
{
     CC26X2R1_WES_UART0_RX | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
     PIN_TERMINATE                                  /* Terminate list */
};

STATIC bool IsHexChar(char c);
STATIC bool IsDecChar(char c);

STATIC const DEBUG_CLI_Command_st cli_cmds[] =
{
    //15 generic commands
    { "help", &handle_help, HELP_HELP,                                        {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_ENABLE,0,NULL_PARAM_LIST },
    { "network", &handle_network,HELP_NETWORK,                                {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_ENABLE,0,NULL_PARAM_LIST },
	{ "panid", &handle_panid, HELP_PAN_ID,								      {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_ENABLE,1,PAN_ID_CMD_PARAM_LIST },
	{ "extpanid", &handle_extpanid, HELP_EXT_PAN_ID,						  {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_ENABLE,1,EXT_PAN_ID_CMD_PARAM_LIST },
    { "eui64", &handle_eui64, HELP_EUI64,                                     {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_ENABLE,0,NULL_PARAM_LIST },
    { "mode", &handle_mode, HELP_MODE,                                        {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_ENABLE,1,MODE_CMD_PARAM_LIST },
    { "hbha", &handle_hbha, HELP_HBHA,                                        {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_ENABLE,0,NULL_PARAM_LIST  },
    { "ver", &handle_ver,HELP_VER,                                            {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_ENABLE,0,NULL_PARAM_LIST  },
    { "reset", &handle_reset,HELP_RESET,                                      {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_ENABLE,0,NULL_PARAM_LIST  },
    { "nvm", &handle_nvm, HELP_NVM,                                           {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_DISABLE,0,NULL_PARAM_LIST  },
    { "normalcycle", &handle_normal_sleep,HELP_NORMAL_SLEEP,                  {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_DISABLE,1,SLEEP_CMD_PARAM_LIST  },
    { "ftm1cycle", &handle_ftm1_sleep,HELP_FTM1_SLEEP,                        {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_DISABLE,1,SLEEP_CMD_PARAM_LIST  },
    { "ftm2cycle", &handle_ftm2_sleep,HELP_FTM2_SLEEP,                        {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_DISABLE,1,SLEEP_CMD_PARAM_LIST  },
    { "ftm2algo", &handle_ftm2_algo,HELP_FTM2_ALGO_SLEEP,                     {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_DISABLE,1,SLEEP_CMD_PARAM_LIST  },
    { "speedcheck", &handle_speedcheck,HELP_SPEED_CHECK,                      {BV(MODE_MGR_FUNC_TEST_MODE_2)},SEL_CUSTOMER_DISABLE,1,SPEEDCHEK_CMD_PARAM_LIST  },
    { "power", &handle_power,HELP_POWER,                                      {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_DISABLE,1,POWER_CMD_PARAM_LIST  },
    { "rfchan", &handle_rfchan,HELP_RFCHAN,                                   {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_DISABLE,1,RFCHAN_CMD_PARAM_LIST  },
    { "childtimeout", &handle_childtimeout,HELP_CHILDTIMEOUT,                 {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_DISABLE,1,CHILDTIMEOUT_CMD_PARAM_LIST },
    { "developer", &handle_developer,HELP_DEVELOPER,                          {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_ENABLE,0,NULL_PARAM_LIST  },
    { "asic", &handle_asic, HELP_ASIC,                                        {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_DISABLE,0,NULL_PARAM_LIST },
    { "batt", &handle_batt, HELP_BATT,                                        {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_DISABLE,0,NULL_PARAM_LIST },
    { "temp", &handle_temp, HELP_TEMP,                                        {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_DISABLE,0,NULL_PARAM_LIST },
    { "parentsearchtime",  &handle_parentsearchtime,  HELP_PARENTSEARCHTIME,  {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_DISABLE,1,PARENTSEARCHTIME_CMD_PARAM_LIST },
	{ "parentsearchlimit", &handle_parentsearchlimit, HELP_PARENTSEARCHLIMIT, {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_DISABLE,1,PARENTSEARCHLIMIT_CMD_PARAM_LIST },

    { "ftest", &handle_ftest, HELP_FTEST,                                     {BV(MODE_MGR_FLASH_STREAM_MODE)},SEL_CUSTOMER_DISABLE,0,NULL_PARAM_LIST },
    { "fread", &handle_fread, HELP_FREAD,                                     {BV(MODE_MGR_FLASH_STREAM_MODE)},SEL_CUSTOMER_DISABLE,1,FREAD_CMD_PARAM_LIST },
    { "ferase", &handle_ferase, HELP_FERASE,                                  {BV(MODE_MGR_FLASH_STREAM_MODE)},SEL_CUSTOMER_DISABLE,1,FERASE_CMD_PARAM_LIST },
    { "fclr", &handle_fclr, HELP_FCLR,                                        {BV(MODE_MGR_FLASH_STREAM_MODE)},SEL_CUSTOMER_DISABLE,0,NULL_PARAM_LIST },
                                                                             
    //acc test commands                                                      
    { "acctest", &handle_acctest, HELP_ACCTEST,                               {BV(MODE_MGR_ACC_TEST_MODE)},SEL_CUSTOMER_DISABLE,0,NULL_PARAM_LIST },
    { "acc", &handle_acc, HELP_ACC,                                           {BV(MODE_MGR_ACC_TEST_MODE)},SEL_CUSTOMER_DISABLE,2,ACC_CMD_PARAM_LIST },
    { "accall", &handle_accall, HELP_ACCALL,                                  {BV(MODE_MGR_ACC_TEST_MODE)},SEL_CUSTOMER_DISABLE,0,NULL_PARAM_LIST },
                                                                             
    //rf test commands                                                       
    { "rftest", &handle_rftest,     HELP_RFTEST,                              {BV(MODE_MGR_RF_TEST_MODE)},SEL_CUSTOMER_ENABLE,0,NULL_PARAM_LIST  },
    { "rfteststart",&handle_rftest_start,HELP_RFTESTSTART,                    {BV(MODE_MGR_RF_TEST_MODE)},SEL_CUSTOMER_ENABLE,0,NULL_PARAM_LIST  },
    { "rfteststop", &handle_rftest_stop, HELP_RFTESTSTOP,                     {BV(MODE_MGR_RF_TEST_MODE)},SEL_CUSTOMER_DISABLE,0,NULL_PARAM_LIST },
    { "rftestchannel", &handle_rftest_channel,HELP_RFTESTCHAN,                {BV(MODE_MGR_RF_TEST_MODE)},SEL_CUSTOMER_ENABLE,1,RFTEST_CHAN_CMD_PARAM_LIST  },
    { "rftestpower", &handle_rftest_power, HELP_RFTESTPOWER,                  {BV(MODE_MGR_RF_TEST_MODE)},SEL_CUSTOMER_ENABLE,1,RFTEST_POWER_CMD_PARAM_LIST },
    { "rftesttx", &handle_rftest_tx, HELP_RFTESTTX,                           {BV(MODE_MGR_RF_TEST_MODE)},SEL_CUSTOMER_ENABLE,2,RFTEST_TX_CMD_PARAM_LIST  },
    { "rftestcont", &handle_rftest_cont,HELP_RFTESTCONT,                      {BV(MODE_MGR_RF_TEST_MODE)},SEL_CUSTOMER_ENABLE,2,RFTEST_CONT_CMD_PARAM_LIST  },
    { "rftestcontstop", &handle_rftest_cont_stop,HELP_RFTESTCONTSTOP,         {BV(MODE_MGR_RF_TEST_MODE)},SEL_CUSTOMER_ENABLE,0,NULL_PARAM_LIST  },
    { "rfteststats", &handle_rftest_stats, HELP_RFTESTSTATS,                  {BV(MODE_MGR_RF_TEST_MODE)},SEL_CUSTOMER_DISABLE,0,NULL_PARAM_LIST  },
    { "rftestrssi", &handle_rftest_rssi,HELP_RFTESTRRSSI,                     {BV(MODE_MGR_RF_TEST_MODE)},SEL_CUSTOMER_DISABLE,0,NULL_PARAM_LIST  },
    { "rftestsleep", &handle_rftest_sleep,HELP_RFTESTSLEEP,                   {BV(MODE_MGR_RF_TEST_MODE)},SEL_CUSTOMER_DISABLE,0,NULL_PARAM_LIST  },
                                                                             
    //vehicle rf test commands                                               
    { "ackreq", &handle_ackreq,HELP_ACKREQ,                                   {BV(MODE_MGR_VEHICLE_RF_TEST)},SEL_CUSTOMER_DISABLE,1,ACKREQ_CMD_PARAM_LIST  },
                                                                             
    //dv command                                                             
    { "dv", &handle_dv,HELP_DV,                                               {BV(MODE_MGR_FUNC_TEST_MODE_1)},SEL_CUSTOMER_DISABLE,0,NULL_PARAM_LIST  },
    {"clrscr",&handle_clearScreen,HELP_CLEARSCR,                              {SUPPORTED_FOR_ALL_MODES}      ,SEL_CUSTOMER_ENABLE,0,NULL_PARAM_LIST  },
                                                                             
    { NULL, NULL, NULL,                                                       {SUPPORTED_FOR_ALL_MODES},SEL_CUSTOMER_ENABLE,0,NULL_PARAM_LIST }
};



STATIC DEBUG_CLI_error_et handle_clearScreen(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list)
{
  const char *clrscr_str = "\e[1;1H\e[2J";
  HAL_UART_send((u8_t*) clrscr_str, strlen(clrscr_str));
  return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et handle_help(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list)
{
    char debug_string[250] = { 0 };
    const DEBUG_CLI_Command_st *cmd = NULL;

    /* Display the relevant help menu and always display the generic menu */
    for ( cmd = cli_cmds; cmd->mName != NULL; cmd++)
    {
        int k = 0;
        STDC_memset( debug_string, 0x20, sizeof( debug_string ) );

        if (strcmp( cmd->mName,"mode") == 0)
        {
            sprintf(debug_string, cmd->helpinfo); k += strlen(cmd->helpinfo);
            HAL_UART_send((u8_t*) debug_string, strlen(debug_string));
            STDC_memset( debug_string, 0, sizeof( debug_string ) );

            for(int i=0, n=0;i<MODE_MGR_MODE_MAX;i++)
            {
                k=0;
                if( NVM_info_s.NVM_data_store_s.developer_debug_console == SEL_ENABLE || mode_lookup[i].public )
                {
                    sprintf(&debug_string[k], "%s  %d :", (((n+1)%2))?"\t\t":", ", i); k += strlen("       ");
                    sprintf(&debug_string[k], "%s", mode_lookup[i].mName); k += strlen(mode_lookup[i].mName);
                    if(!((n+1)%2))
                        sprintf(&debug_string[k], "%s","\r\n");  k += strlen("\r\n");
                    n++;
                    HAL_UART_send((u8_t*) debug_string, strlen(debug_string));
                    STDC_memset( debug_string, 0, sizeof( debug_string ) );
                }
            }
            sprintf(debug_string, "\r\n");
            HAL_UART_send((u8_t*) debug_string, strlen(debug_string));
            k=0;
            //sprintf(debug_string, "%s","\r\n");k += strlen("\r\n");
            //HAL_UART_send((u8_t*) debug_string, strlen(debug_string));

        }
        else if( (strcmp( cmd->mName,"frametype") == 0) || ( strcmp( cmd->mName,"power") == 0 ) )
        {
            if( (cmd->availbility & BV(MODE_MGR_get_mode())) && (NVM_info_s.NVM_data_store_s.developer_debug_console == SEL_ENABLE || cmd->public) )
            {
                int upperlimit = cmd->param_list[0].ulimit;
                char** enum_str = (char**)(( cmd->mName[0]=='p' )?DEBUG_CLI_power_val_str:DEBUG_CLI_frame_type_str);

                sprintf(debug_string, cmd->helpinfo);
                HAL_UART_send((u8_t*) debug_string, strlen(debug_string));
                STDC_memset( debug_string, 0x20, sizeof( debug_string ) );
                sprintf(debug_string,"\t     ");
                HAL_UART_send((u8_t*) debug_string, strlen(debug_string));

                for(int i=0;i<=upperlimit;i++)
                {
                    sprintf(&debug_string[k], "%s  %d :","   ", i); k += strlen("        ");
                    sprintf(&debug_string[k], "%s ,", enum_str[i]); k += (strlen(enum_str[i])+2);
                }
                sprintf(&debug_string[k], "%s","\r\n"); k += strlen("\r\n");
            }
        }
        else
        {
            if( (cmd->availbility & BV(MODE_MGR_get_mode())) && (NVM_info_s.NVM_data_store_s.developer_debug_console == SEL_ENABLE || cmd->public) )
            {
                sprintf(debug_string, "%s", cmd->helpinfo); k += strlen(cmd->helpinfo);
            }
            else
            {
                continue;
            }
        }

        HAL_UART_send((u8_t*) debug_string, k);
    }

    return DEBUG_CLI_ERROR_NONE;
}


STATIC DEBUG_CLI_error_et handle_network(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char debug_string[80] = { 0 };
    char newline[] = "\r\n";
    sprintf(debug_string, "\r\nChannel - %02d  \r\nPan ID - 0x%04x \r\nExt Pan ID - 0x", NVM_info_s.NVM_data_store_s.channel_selection,NVM_info_s.NVM_data_store_s.pan_id);

    HAL_UART_send((u8_t*) debug_string, strlen(debug_string));

    for (int i = 0; i < 8; i++)
    {
        sprintf(&debug_string[i * 2 + 15], "%02x", NVM_info_s.NVM_data_store_s.extended_pan_id[i]);
    }

    HAL_UART_send((u8_t*) &debug_string[15], 16);
    HAL_UART_send((u8_t*) newline, sizeof(newline));

    return DEBUG_CLI_ERROR_NONE;
}


STATIC DEBUG_CLI_error_et handle_panid(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char debug_string[80] = { 0x20 };

    NVM_info_s.NVM_data_store_s.pan_id = p_param_val_list[0].val_16;
    ot_stack_nwk_pan_id = NVM_info_s.NVM_data_store_s.pan_id;

    NVM_request_flush();

    sprintf(debug_string,  "New PAN ID - 0x%04x\r\n",
            NVM_info_s.NVM_data_store_s.pan_id );

    HAL_UART_send((u8_t*) debug_string, strlen(debug_string));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et handle_extpanid(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char debug_string[80] = { 0x20 };

    STDC_memcpy( NVM_info_s.NVM_data_store_s.extended_pan_id, p_param_val_list[0].val_64, 8);

    NVM_request_flush();

    sprintf(debug_string,  "New Extended PAN ID - %02x%02x%02x%02x%02x%02x%02x%02x\r\n",
            NVM_info_s.NVM_data_store_s.extended_pan_id[0], NVM_info_s.NVM_data_store_s.extended_pan_id[1],
			NVM_info_s.NVM_data_store_s.extended_pan_id[2], NVM_info_s.NVM_data_store_s.extended_pan_id[3],
			NVM_info_s.NVM_data_store_s.extended_pan_id[4], NVM_info_s.NVM_data_store_s.extended_pan_id[5],
			NVM_info_s.NVM_data_store_s.extended_pan_id[6], NVM_info_s.NVM_data_store_s.extended_pan_id[7] );

    HAL_UART_send((u8_t*) debug_string, strlen(debug_string));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et handle_eui64(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char eui64_string[32] = { 0x20 };
    char extaddr[18] ={ 0 };
    otExtAddress extAddress;

    otLinkGetFactoryAssignedIeeeEui64(OtInstance_get(), &extAddress);

    snprintf(&extaddr[0], sizeof(extaddr), "%02x%02x%02x%02x%02x%02x%02x%02x",
    extAddress.m8[0], extAddress.m8[1], extAddress.m8[2],
    extAddress.m8[3], extAddress.m8[4], extAddress.m8[5],
    extAddress.m8[6], extAddress.m8[7]);

    sprintf((char*)eui64_string, "\r\nEUI64 is %s\r\n", extaddr);
    HAL_UART_send((u8_t*) eui64_string, strlen(eui64_string));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et handle_mode(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char debug_string[30] = { 0 };

    NVM_info_s.NVM_data_store_s.mode = (MODE_MGR_operational_mode_et)p_param_val_list[0].val_8;

    sprintf(debug_string, "%s has been selected...\r\n\r\n", mode_lookup[NVM_info_s.NVM_data_store_s.mode].mName);

    NVM_request_flush();

    HAL_UART_send((u8_t*) debug_string, strlen(debug_string));

    //do we need this?? is there a better way? dont know as of now!!

    SysCtrlSystemReset();

    return DEBUG_CLI_ERROR_NONE;;
}

STATIC DEBUG_CLI_error_et  handle_hbha(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char wes_string[150] = { 0x20 };
    SENSOR_DECODE_asic_data_st WES_asic_data_s[2];

    /* grab the asic data  */
    SENSOR_DECODE_get_TPM_data((uint8_t*) WES_asic_data_s);

    STDC_memset(wes_string, 0x20, sizeof(wes_string));

    /* now we would like to get the rest of the WES related data that isnt global */
    WES_WAKE_ALGO_get_stats( &DEBUG_wes_stats_s );

    /* This data needs to be formatted into something that people can understand */
    switch( DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_result_s )
    {
        case BEARING_GOOD:       sprintf(wes_string, "\r\nHBHA Result:\t\tBearing GOOD :)" );   break;
        case BEARING_FAULT:      sprintf(wes_string, "\r\nHBHA Result:\t\tBearing BAD :(" );    break;
        case DATA_DISQUALIFIED:  sprintf(wes_string, "\r\nHBHA Result:\t\tData Disqualified" ); break;
        case NO_INIT:            sprintf(wes_string, "\r\nHBHA Result:\t\tNot Initialised" );   break;
        default:                 sprintf(wes_string, "\r\nHBHA Result:\t\tNot Initialised" );   break;
    }
    HAL_UART_send((u8_t*) wes_string, sizeof(wes_string));
    STDC_memset( wes_string, 0x00, sizeof( wes_string ) );

    switch( DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_process_condition_s )
    {
        case PROCESS_READY:              sprintf(wes_string, "\r\nProcess State:\t\tProcess Ready" );        break;
        case PROCESS_SPEED_NOT_IN_RANGE: sprintf(wes_string, "\r\nProcess State:\t\tSpeed Not In Range" );   break;
        case PROCESS_TEMPERATURE_HIGH:   sprintf(wes_string, "\r\nProcess State:\t\tTemperature Too High" ); break;
        case PROCESS_TEMPERATURE_LOW:    sprintf(wes_string, "\r\nProcess State:\t\tTemperature Too Low" );  break;
        case PROCESS_NOT_BROKEN_IN:      sprintf(wes_string, "\r\nProcess State:\t\tNot Broken In" );        break;
        case PROCESS_NOT_READY:          sprintf(wes_string, "\r\nProcess State:\t\tNot Ready" );            break;
        default:                         sprintf(wes_string, "\r\nProcess State:\t\tNot Ready" );            break;
    }
    HAL_UART_send((u8_t*) wes_string, sizeof(wes_string));
    STDC_memset( wes_string, 0x00, sizeof( wes_string ) );

    switch( DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_run_bearing_time_s )
    {
        case WES_WAKE_ALGO_WAKE_SCHEDULE_15_MIN:   sprintf(wes_string, "\r\nrun bearing time:\t15 mins" );  break;
        case WES_WAKE_ALGO_WAKE_SCHEDULE_1_HOUR:   sprintf(wes_string, "\r\nrun bearing time:\t1 Hour" );   break;
        case WES_WAKE_ALGO_WAKE_SCHEDULE_33_HOURS: sprintf(wes_string, "\r\nrun bearing time:\t33 Hours" ); break;
        default:                                   sprintf(wes_string, "\r\nrun bearing time:\t15 mins" );  break;
    }

    HAL_UART_send((u8_t*) wes_string, sizeof(wes_string));
    STDC_memset( wes_string, 0x00, sizeof( wes_string ) );

    sprintf(wes_string, "\r\ntpms count:\t\t%d\r\nbreak in ctr:\t\t%d\r\nHBHA cycles:\t\t%d\r\nExt Flash Current Add:\t0x%X\r\nEXT Flash write cycles:\t%d\r\n\r\n",

    DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_tpms_count_s,
    DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_break_in_ctr_s,
    DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_run_cnt_s,
    EXT_FLASH_get_current_address_space(),
    EXT_FLASH_get_write_cycles() );

    HAL_UART_send((u8_t*) wes_string, sizeof(wes_string));

    STDC_memset( wes_string, 0x00, sizeof( wes_string ) );
    sprintf(wes_string, "ASIC temp c:\t\t%d\r\nASIC Pressure mb:\t%d\r\n",
            WES_asic_data_s[0].temperature_c,
            WES_asic_data_s[0].pressure_mb );
    HAL_UART_send((u8_t*) wes_string, strlen(wes_string));

    STDC_memset( wes_string, 0x00, sizeof( wes_string ) );

    sprintf(wes_string, "\r\nSignal Stats\r\nwheel freq:\t\t%d\r\nwheel speed:\t\t%d\r\nalgovalue1:\t\t%d\r\nalgovalue2:\t\t%d\r\nmaxpeak:\t\t%d\r\nqualified:\t\t%d\r\n",
            DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_signal_stats_s.wheelFreq,
            DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_signal_stats_s.wheelSpeed,
            DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_signal_stats_s.algoValue1,
            DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_signal_stats_s.algoValue2,
            DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_signal_stats_s.maxPeak,
            DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_signal_stats_s.qualified );

    HAL_UART_send((u8_t*) wes_string, sizeof(wes_string));
    STDC_memset( wes_string, 0x00, sizeof( wes_string ) );

    sprintf(wes_string, "voltage error:\t\t%d\r\ntemperature:\t\t%d\r\nrpm bucket:\t\t%d\r\naccel type:\t\t%d\r\n\r\n",
            DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_signal_stats_s.voltageError,
            DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_signal_stats_s.temperature,
            DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_signal_stats_s.rpmBucket,
            DEBUG_wes_stats_s.DEBUG_WES_WAKE_ALGO_signal_stats_s.accelType );

    HAL_UART_send((u8_t*) wes_string, sizeof(wes_string));
    STDC_memset( wes_string, 0x00, sizeof( wes_string ) );

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_ver(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list)
{
    char vers_string[70] = { 0 };
    char version_num[5];

    HAL_BRD_get_SW_version_number((u8_t*) version_num);

    sprintf(vers_string, "\r\nSensata SW version is %d.%d.%d.%c%c\r\n",
            version_num[0], version_num[1], version_num[2], version_num[3],
            version_num[4]);
    HAL_UART_send((u8_t*) vers_string, strlen(vers_string));
    STDC_memset(vers_string, 0x20, sizeof(vers_string));

    HAL_BRD_get_HW_version_number((u8_t*) version_num);
    sprintf(vers_string, "Sensata HW version is not possible with the current hardware :(\r\n");
    HAL_UART_send((u8_t*) vers_string, strlen(vers_string));

    getWESLibVersion((u8_t*) version_num );
    sprintf(vers_string, "Hendrickson HBHA lib version is %d.%d.%d\r\n",
                    version_num[0], version_num[1], version_num[2] );
    HAL_UART_send((u8_t*) vers_string, strlen(vers_string));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_reset(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char debug_string[25] = { 0x20 };
    sprintf(debug_string, "\r\nResetting core...\r\n\r\n");
    HAL_UART_send((u8_t*) debug_string, strlen(debug_string));

    /* Perform a system reset */
    SysCtrlSystemReset();

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_nvm(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char nvm_data[200];
    NVM_data_store_st* p_nvm_data = &(NVM_info_s.NVM_data_store_s);

    STDC_memset(nvm_data, 0x20, sizeof(nvm_data));

    /* Read out the current NVM data */
    sprintf(nvm_data, "\r\n\r\nchksum\t\t\t\t0x%X\r\nver\t\t\t\t%d\r\nwrite count\t\t\t%d",
         NVM_info_s.checksum, NVM_info_s.version,
         NVM_info_s.write_count);
    HAL_UART_send((u8_t*) nvm_data, strlen(nvm_data));

    sprintf(nvm_data,
         "\r\n\r\n-------------\r\nGENERIC DATA\r\n-------------\r\npan id\t\t\t\t0x%04x\r\next pan id\t\t\t0x%02x%02x%02x%02x%02x%02x%02x%02x\r\n"
         "channel\t\t\t\t%d\r\nMode\t\t\t\t%d\r\n",
         p_nvm_data->pan_id,
         p_nvm_data->extended_pan_id[0],
         p_nvm_data->extended_pan_id[1],
         p_nvm_data->extended_pan_id[2],
         p_nvm_data->extended_pan_id[3],
         p_nvm_data->extended_pan_id[4],
         p_nvm_data->extended_pan_id[5],
         p_nvm_data->extended_pan_id[6],
         p_nvm_data->extended_pan_id[7],
         p_nvm_data->channel_selection,
         (u8_t) p_nvm_data->mode );

    HAL_UART_send((u8_t*) nvm_data, strlen(nvm_data));

    /* Reset the buffer and go again with more data */
    STDC_memset(nvm_data, 0x20, sizeof(nvm_data));

    sprintf(nvm_data,
         "\r\nnormal cyle time(s)\t\t%d\r\nftm1 cycle time(s)\t\t%d\r\nftm2 cycle time(s)\t\t%d\r\nftm2 hbha run bearing time\t%d\r\n",
         p_nvm_data->normal_mode_cycle_time,
         p_nvm_data->ftm1_cycle_time,
         p_nvm_data->ftm2_cycle_time,
         p_nvm_data->ftm2_run_hbha_time );

    HAL_UART_send((u8_t*) nvm_data, strlen(nvm_data));

    /* Reset the buffer and go again with more data */
    STDC_memset(nvm_data, 0x20, sizeof(nvm_data));

    sprintf(nvm_data,"\r\nparentsearch cycle time(s)\t%d\r\nparentsearch retry limit\t%d\r\n\r\n",
    		 NVM_info_s.NVM_data_store_s.parent_search_interval_time,
			 NVM_info_s.NVM_data_store_s.parent_search_cycle_limit );

    HAL_UART_send((u8_t*) nvm_data, strlen(nvm_data));


    /* Reset the buffer and go again with more data */
    STDC_memset(nvm_data, 0x20, sizeof(nvm_data));


    sprintf(nvm_data, "\r\nTX power\t\t\t%d\r\nLF trigger cnt\t\t\t%d\r\nRF frame type\t\t\t%d\r\n",
            p_nvm_data->power_level,
            p_nvm_data->lf_trigger_cnt,
            p_nvm_data->rf_frame_type);

    HAL_UART_send((u8_t*) nvm_data, strlen(nvm_data));


    /* Reset the buffer and go again with more data */
    STDC_memset(nvm_data, 0x20, sizeof(nvm_data));

    sprintf(nvm_data, "\r\n-----------\r\nERROR DATA\r\n-----------\r\nASIC 1\r\nCRC fails:\t\t\t%d\r\nTX fails\t\t\t%d\r\nASIC 2\r\nCRC fails:\t\t\t%d\r\n"
    		"TX Fails:\t\t\t%d\r\n\r\n",
            p_nvm_data->device_self_test_status_s.asic_error_info_s[0].num_crc_fails,
            p_nvm_data->device_self_test_status_s.asic_error_info_s[0].num_tx_fails,
            p_nvm_data->device_self_test_status_s.asic_error_info_s[1].num_crc_fails,
            p_nvm_data->device_self_test_status_s.asic_error_info_s[1].num_tx_fails);

    HAL_UART_send((u8_t*) nvm_data, strlen(nvm_data));


    /* Reset the buffer and go again with more data */
    STDC_memset(nvm_data, 0x20, sizeof(nvm_data));

    sprintf(nvm_data,
         "\r\n----------\r\nWES DATA\r\n----------\r\nspoof speed check\t\t%d\r\nspoof temp check\t\t%d\r\nfault ctr\t\t\t%d\r\n",
         p_nvm_data->spoof_speed_check,
         p_nvm_data->spoof_temp_check,
         p_nvm_data->wes_bearing_fault_cnt);

    HAL_UART_send((u8_t*) nvm_data, strlen(nvm_data));
    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_normal_sleep(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char sleep_string[70];

    DEBUG_CLI_error_et err = DEBUG_CLI_ERROR_NONE;

    if( NVM_info_s.NVM_data_store_s.normal_mode_cycle_time != p_param_val_list[0].val_32 )
    {
        STDC_memset(sleep_string, 0x20, sizeof(sleep_string));

        NVM_info_s.NVM_data_store_s.normal_mode_cycle_time = p_param_val_list[0].val_32;

        /* Request a flush to NVM */
        NVM_request_flush();

        sprintf(sleep_string,"\r\nSleep\\Standby time has been set to %d sec(s)\r\n",NVM_info_s.NVM_data_store_s.normal_mode_cycle_time);

        HAL_UART_send((u8_t*) sleep_string, strlen(sleep_string));

        /* perform a soft reset */
        SysCtrlSystemReset();
    }
    else
    {
        err = DEBUG_CLI_ALREADY_SET;
    }

    return err;
}

STATIC DEBUG_CLI_error_et  handle_ftm1_sleep(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char sleep_string[70];

    DEBUG_CLI_error_et err = DEBUG_CLI_ERROR_NONE;

    if( NVM_info_s.NVM_data_store_s.ftm1_cycle_time != p_param_val_list[0].val_32 )
    {
        STDC_memset(sleep_string, 0x20, sizeof(sleep_string));

        NVM_info_s.NVM_data_store_s.ftm1_cycle_time = p_param_val_list[0].val_32;

        /* Request a flush to NVM */
        NVM_request_flush();

        sprintf(sleep_string,"\r\nCycle time in ftm1 has been set to %d sec(s)\r\n",NVM_info_s.NVM_data_store_s.ftm1_cycle_time);

        HAL_UART_send((u8_t*) sleep_string, sizeof(sleep_string));

        /* perform a soft reset */
        SysCtrlSystemReset();
    }
    else
    {
        err = DEBUG_CLI_ALREADY_SET;
    }

    return err;
}

STATIC DEBUG_CLI_error_et  handle_ftm2_sleep(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{

    char sleep_string[70];

    DEBUG_CLI_error_et err = DEBUG_CLI_ERROR_NONE;

    if( NVM_info_s.NVM_data_store_s.ftm2_cycle_time != p_param_val_list[0].val_32 )
    {
        STDC_memset(sleep_string, 0x20, sizeof(sleep_string));

        NVM_info_s.NVM_data_store_s.ftm2_cycle_time = p_param_val_list[0].val_32;

        /* Request a flush to NVM */
        NVM_request_flush();

        sprintf(sleep_string,"\r\nCycle time in ftm2 has been set to %d sec(s)\r\n",NVM_info_s.NVM_data_store_s.ftm2_cycle_time);

        HAL_UART_send((u8_t*) sleep_string, strlen(sleep_string));

        /* perform a soft reset */
        SysCtrlSystemReset();
    }
    else
    {
        err = DEBUG_CLI_ALREADY_SET;
    }

    return err;
}

STATIC DEBUG_CLI_error_et  handle_ftm2_algo(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char sleep_string[80];

    DEBUG_CLI_error_et err = DEBUG_CLI_ERROR_NONE;

    if( NVM_info_s.NVM_data_store_s.ftm2_run_hbha_time != p_param_val_list[0].val_32 )
    {
        STDC_memset(sleep_string, 0x20, sizeof(sleep_string));

        NVM_info_s.NVM_data_store_s.ftm2_run_hbha_time = p_param_val_list[0].val_32;

        /* Request a flush to NVM */
        NVM_request_flush();

        sprintf(sleep_string,"\r\nHBHA algorithm will now run every %d TPMS loops in FTM2\r\n",NVM_info_s.NVM_data_store_s.ftm2_run_hbha_time);

        HAL_UART_send((u8_t*) sleep_string, strlen(sleep_string));

        /* perform a soft reset */
        SysCtrlSystemReset();
    }
    else
    {
        err = DEBUG_CLI_ALREADY_SET;
    }

    return err;
}

STATIC DEBUG_CLI_error_et  handle_speedcheck(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char speed_check_string[40];

    DEBUG_CLI_error_et err = DEBUG_CLI_ERROR_NONE;

    if( NVM_info_s.NVM_data_store_s.spoof_speed_check != p_param_val_list[0].val_8 )
    {
        STDC_memset(speed_check_string, 0x20, sizeof(speed_check_string));

        NVM_info_s.NVM_data_store_s.spoof_speed_check = (SEL_false_true_et)p_param_val_list[0].val_8;

        /* Request a flush to NVM */
        NVM_request_flush();

        sprintf(speed_check_string, "New Speed Check state - %d\r\n", NVM_info_s.NVM_data_store_s.spoof_speed_check);

        HAL_UART_send((u8_t*) speed_check_string, strlen(speed_check_string));
    }
    else
    {
        err = DEBUG_CLI_ALREADY_SET;
    }

    return err;
}

STATIC DEBUG_CLI_error_et  handle_power(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    DEBUG_CLI_error_et err = DEBUG_CLI_ERROR_NONE;
    char power_string[60];

    if( NVM_info_s.NVM_data_store_s.power_level != p_param_val_list[0].val_8 )
    {
        NVM_info_s.NVM_data_store_s.power_level = p_param_val_list[0].val_8;

        NVM_request_flush();

        STDC_memset( power_string, 0x20, sizeof( power_string ) );

        sprintf(power_string, "New power level is %d ( %s )\r\n", NVM_info_s.NVM_data_store_s.power_level, DEBUG_CLI_power_val_str[NVM_info_s.NVM_data_store_s.power_level]);

        HAL_UART_send((u8_t*) power_string, strlen(power_string));

        /* TODO: Do we need to conside calling SysCtrlSystemReset() to bring the new value into effect?? */
    }
    else
    {
        err = DEBUG_CLI_ALREADY_SET;
    }

    return err;
}

STATIC DEBUG_CLI_error_et  handle_frametype(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    DEBUG_CLI_error_et err = DEBUG_CLI_ERROR_NONE;
    char rf_frame_string[80];

    if( NVM_info_s.NVM_data_store_s.rf_frame_type != p_param_val_list[0].val_8 )
    {

        NVM_info_s.NVM_data_store_s.rf_frame_type = (WES_rf_frame_type)p_param_val_list[0].val_8;

        NVM_request_flush();

        STDC_memset( rf_frame_string, 0x20, sizeof( rf_frame_string ) );

        sprintf(rf_frame_string, "\r\nFrame type %d ( %s ) selected..\r\n", NVM_info_s.NVM_data_store_s.rf_frame_type, DEBUG_CLI_frame_type_str[NVM_info_s.NVM_data_store_s.rf_frame_type]);

        HAL_UART_send((u8_t*) rf_frame_string, strlen(rf_frame_string));
    }
    else

    {
        err = DEBUG_CLI_ALREADY_SET;
    }

    return err;
}

STATIC DEBUG_CLI_error_et  handle_rfchan(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list)
{
    DEBUG_CLI_error_et err = DEBUG_CLI_ERROR_NONE;
    char channel_string[40];

    if( NVM_info_s.NVM_data_store_s.channel_selection != p_param_val_list[0].val_8 )
    {
        NVM_info_s.NVM_data_store_s.channel_selection = ot_stack_nwk_channel = p_param_val_list[0].val_8;

        /* Request a flush to NVM */
        NVM_request_flush();

        STDC_memset( channel_string, 0x20, sizeof( channel_string ) );

        sprintf(channel_string, "New RF Channel - %02d\r\n\r\n", ot_stack_nwk_channel);

        HAL_UART_send((u8_t*) channel_string, strlen(channel_string));

        /* perform a soft reset */
        SysCtrlSystemReset();

    }
    else
    {
        err = DEBUG_CLI_ALREADY_SET;
    }

    return err;
}

STATIC DEBUG_CLI_error_et handle_childtimeout(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list)
{
    DEBUG_CLI_error_et err = DEBUG_CLI_ERROR_NONE;

    char childtimeout_cmd_result[60];

    STDC_memset(childtimeout_cmd_result, 0x20, sizeof(childtimeout_cmd_result));

    if( NVM_info_s.NVM_data_store_s.timeout_tlv_val != p_param_val_list[0].val_32 )
    {
        // store the received timeout value in the NVM
        NVM_info_s.NVM_data_store_s.timeout_tlv_val = p_param_val_list[0].val_32;

        /* Request a flush to NVM */
        NVM_request_flush();

        // bring into effect the new value in the thread stack which would result in
        //ChildUpdateReq over the air to inform the parent about this change
        // This may not be possible before the OT stack task is made to run at least once!!
        //in which case the new value is just stored in the NVM. This setting followed by a
        //NVM reset would immediately bring into effect this value in the thread stack.
        if (OtInstance_get() != NULL)
        {
            otThreadSetChildTimeout( OtInstance_get(), NVM_info_s.NVM_data_store_s.timeout_tlv_val);
        }

        STDC_memset( childtimeout_cmd_result, 0x20, sizeof( childtimeout_cmd_result ) );

        sprintf(childtimeout_cmd_result, "Child timeout set to:%d secs\r\n", NVM_info_s.NVM_data_store_s.timeout_tlv_val);

        HAL_UART_send((u8_t*) childtimeout_cmd_result, strlen(childtimeout_cmd_result));

    }
    else
    {
        err = DEBUG_CLI_ALREADY_SET;
    }

    return err;
}

STATIC DEBUG_CLI_error_et handle_developer(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char debug_string[50] = { 0x20 };

    // toggle the setting
    NVM_info_s.NVM_data_store_s.developer_debug_console ^= 1;

    /* Request a flush to NVM */
    NVM_request_flush();

    sprintf(debug_string, "Developer Debug console has been %s", (NVM_info_s.NVM_data_store_s.developer_debug_console)?"enabled\r\n":"disabled\r\n" );

    HAL_UART_send((u8_t*) debug_string, strlen(debug_string));

    return DEBUG_CLI_ERROR_NONE;

}

//// debug mode commands
STATIC DEBUG_CLI_error_et  handle_asic(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char i = 0;
    char asic_string[200] = { 0x20 };
    SENSOR_DECODE_asic_data_st WES_asic_data_s[2];
    SENSOR_DECODE_asic_data_st* p_asic_data = NULL;

    /* grab the asic data  */
    SENSOR_DECODE_get_TPM_data((uint8_t*) WES_asic_data_s);

    /* We are keeping the payload to 70 bytes as it doesnt seem to like anything longer, this needs to be investigated */
    for (i = 0; i < 2; i++)
    {
        STDC_memset( asic_string, 0x00, sizeof( asic_string ) );
        p_asic_data = &( WES_asic_data_s[i] );
        snprintf((char*) asic_string,sizeof(asic_string),
                "\r\n------ASIC %d\r\n------\r\nID:\t \t0x%02X%02X%02X%02X\r\nPressure:\t%04d mb ( %03d counts )\r\nTemperature:\t%03d %cC  ( %03d counts )\r\n"
                "Status:\t\t0x%02X\r\nPAL data:\t0x%02X\r\nCrc fails:\t%08d\r\nTx fails:\t%08d\r\n\r\n",
                (i),
                ((p_asic_data->id & 0xFF000000) >> 24),
                ((p_asic_data->id & 0x00FF0000) >> 16),
                ((p_asic_data->id & 0x0000FF00) >> 8),
                ((p_asic_data->id & 0x000000FF)),
                (p_asic_data->pressure_mb),
                (p_asic_data->pressure_counts),
                (p_asic_data->temperature_c),
                0xF8,
                (p_asic_data->temperature_counts),
                (p_asic_data->status),
                (p_asic_data->pal_data),
                (p_asic_data->SENSOR_DECODE_asic_error_s.num_crc_fails),
                (p_asic_data->SENSOR_DECODE_asic_error_s.num_tx_fails));

        HAL_UART_send((u8_t*) asic_string, strlen(asic_string));
    }

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_batt(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    u32_t battery_voltage = SENSOR_DECODE_get_battery_voltage_mv();
    char battery_string[50] = { 0x20 };

    snprintf((char*) battery_string, sizeof(battery_string),
             "\r\nCurrent battery voltage is %04d mV\r\n", battery_voltage);

    HAL_UART_send((u8_t*) battery_string, strlen(battery_string));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_temp(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    s32_t temperature = SENSOR_DECODE_get_internal_temp_c();
    char temperature_string[45] = { 0x20 };

    snprintf((char*) temperature_string, sizeof(temperature_string),
             "\r\nCurrent Internal temperature is %d %cC\r\n", temperature,0xF8);

    HAL_UART_send((u8_t*) temperature_string, strlen(temperature_string));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_ftest(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    /* Run the self test of the EXT flash chip */
    char flash_string[200] = { 0x20 };

    sprintf(flash_string,
            "\r\nRunning EXT flash Self test...\r\nStep 1. Read the ID and manufacture code..\r\n"
            "Step 2. Write and read the value 0x%02X to and from a flash sector..",
            EXT_FLASH_SELF_TEST_BYTE);

    HAL_UART_send((u8_t*) flash_string, strlen(flash_string));

    STDC_memset(flash_string, 0x20, sizeof(flash_string));

    if (EXT_FLASH_self_test() == 0)

    {
        sprintf(flash_string, "\r\nSelf Test has passed :)\r\n\r\nThe test byte is:\t0x%X\r\n\r\n",EXT_FLASH_get_test_byte());
    }
    else
    {
        sprintf(flash_string,
                "\r\nSelf Test has failed, perhaps the device is not placed on the board or"
                " the write/read operation failed on the sector\r\n\r\n");
    }

    HAL_UART_send((u8_t*) flash_string, strlen(flash_string));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_fread(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char *sector_string = strstr(DEBUG_CLI_msg_read_s, "fread") + 6;
    char flash_string[100] = { 0x20 };
    char fdata[256];
    int sector_num = 0u;
    char pages = 0u;
    char i = 0u;
    char bytes = 0;
    int fdata_index = 0;

    /* If not look to see if they have requested a specific page */
    sector_num = p_param_val_list[0].val_16;

    sprintf(flash_string, "\r\nReading back sector %d of 511...\r\n",  sector_num);

    HAL_UART_send((u8_t*) flash_string, strlen(flash_string));
    STDC_memset(flash_string, 0x20, sizeof(flash_string));

    /* A sector is 4096 bytes and pages are 256 bytes, so read the data out in pages */
    pages = ( EXT_FLASH_SMALL_SECTOR_SIZE / EXT_FLASH_PAGE_SIZE);

    //TODO: make this below operation thread safe by calling may be Task_Disable();
    for (i = 0; i < pages; i++)
    {
        EXT_FLASH_read_data(
                ((sector_num * EXT_FLASH_SMALL_SECTOR_SIZE)
                        + ( EXT_FLASH_PAGE_SIZE * i)),
                (u8_t*) fdata, EXT_FLASH_PAGE_SIZE);

        sprintf(flash_string, "\r\nPage %02d contents:\r\n", i);
        HAL_UART_send((u8_t*) flash_string, strlen(flash_string));

        for (bytes = 0; bytes < 25; bytes++)
        {
            fdata_index = bytes * 10;
            sprintf(flash_string,
                    "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n",
                    fdata[fdata_index], fdata[fdata_index + 1],
                    fdata[fdata_index + 2], fdata[fdata_index + 3],
                    fdata[fdata_index + 4], fdata[fdata_index + 5],
                    fdata[fdata_index + 6], fdata[fdata_index + 7],
                    fdata[fdata_index + 8], fdata[fdata_index + 9]);

            HAL_UART_send((u8_t*) flash_string, strlen(flash_string));
        }

        fdata_index = bytes * 10;

        sprintf(flash_string,
                "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n\r\n",
                fdata[fdata_index], fdata[fdata_index + 1],
                fdata[fdata_index + 2], fdata[fdata_index + 3],
                fdata[fdata_index + 4], fdata[fdata_index + 5]);


        HAL_UART_send((u8_t*) flash_string, strlen(flash_string));
    }

    sprintf(flash_string, "\r\nEnd of page....\r\n\r\n");
    //TODO does the above print be "end of sector"??!!
    HAL_UART_send((u8_t*) flash_string, strlen(flash_string));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_ferase(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char flash_string[100] = { 0x20 };

    int sector_num = p_param_val_list[0].val_16;

    sprintf(flash_string,"\r\nErasing flash sector %d @ address 0x%08X\r\n",
            sector_num, (sector_num * EXT_FLASH_SMALL_SECTOR_SIZE));

    HAL_UART_send((u8_t*) flash_string, strlen(flash_string));

    STDC_memset(flash_string, 0x20, sizeof(flash_string));

    //TODO: make this operation thread safe by calling may be Task_Disable();
    EXT_FLASH_erase_4K_sector(
            (sector_num * EXT_FLASH_SMALL_SECTOR_SIZE));


    sprintf(flash_string, "\r\nSector Erase complete...\r\n\r\n");

    HAL_UART_send((u8_t*) flash_string, strlen(flash_string));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_fclr(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char flash_string[100] = { 0x20 };

    sprintf(flash_string,
            "\r\nErasing entire flash memory...\r\nThis will take some time, Please be patient....");

    HAL_UART_send((u8_t*) flash_string, strlen(flash_string));

    STDC_memset(flash_string, 0x20, sizeof(flash_string));

    //TODO: make this operation thread safe by calling may be Task_Disable();
    EXT_FLASH_erase_chip();

    sprintf(flash_string, "\r\nFlash Erase complete...\r\n\r\n");

    HAL_UART_send((u8_t*) flash_string, strlen(flash_string));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_acctest(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char acc_self_test_string[80] = { 0x20 };
    SEL_pass_fail_et self_test_status;

    sprintf(acc_self_test_string, "\r\nRunning Accelerometer self test..");

    HAL_UART_send((u8_t*) acc_self_test_string, strlen(acc_self_test_string));

    STDC_memset(acc_self_test_string, 0x20, sizeof(acc_self_test_string));

    self_test_status = WES_WAKE_ALGO_ADXL_perform_self_test();

    if (self_test_status == SEL_PASS)
    {
        sprintf(acc_self_test_string, "\r\n\r\nSelf test passed succesfully\r\n");
    }
    else
    {
        sprintf(acc_self_test_string,
                "\r\n\r\nSelf test Failed!!, This could happen if the device is moving or spinning\r\n");
    }

    HAL_UART_send((u8_t*) acc_self_test_string, strlen(acc_self_test_string));

    return DEBUG_CLI_ERROR_NONE;
}

//acc test command handlers
STATIC DEBUG_CLI_error_et  handle_acc(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    DEBUG_CLI_error_et err = DEBUG_CLI_ERROR_NONE;
    char acc_results[50];
    u8_t acc_test_data[DEBUG_CLI_ACC_TEST_DATA_SIZE];
    u16_t requested_samples;
    u8_t requested_axis=0;
    u8_t cycles = 0;
    u8_t remaining_samples = 0u;
    u16_t battery_voltage = 0u;
    u8_t i = 0;
    u8_t j = 0;
    u16_t sample_counter = 0u;
    u8_t k = 0;

    /* reset the buffer */
    STDC_memset( acc_results, 0x00, sizeof ( acc_results ) );

    if( strlen((char*)p_param_val_list[0].str)>=1 && strlen((char*)p_param_val_list[0].str)<=3 )
    {
        while(k<3 && p_param_val_list[0].str[k] != 0)
        {
            switch(p_param_val_list[0].str[k])
            {
            case 'x':
            case 'X':
                requested_axis |= X_AXIS_BIT_MASK;
                break;
            case 'y':
            case 'Y':
                requested_axis |= Y_AXIS_BIT_MASK;
                break;
            case 'z':
            case 'Z':
                requested_axis |= Z_AXIS_BIT_MASK;
                break;
            default:
                err = DEBUG_CLI_ERROR_INVALID_STR_PARAM;
                break;

            }

            k++;
        }

        if( !err  )
        {

            //requested_axis = p_param_val_list[0].val_8;
            switch(requested_axis)
            {
            case X_AXIS_BIT_MASK:
                sprintf(acc_results, "Selected axis is x\r\n" );
                break;
            case Y_AXIS_BIT_MASK:
                sprintf(acc_results, "Selected axis is y\r\n" );
                break;
            case Z_AXIS_BIT_MASK:
                sprintf(acc_results, "Selected axis is z\r\n\r\n" );
                break;
            case (X_AXIS_BIT_MASK|Y_AXIS_BIT_MASK):
                sprintf(acc_results, "Selected axis are x and y\r\n\r\n" );
                break;
            case (Y_AXIS_BIT_MASK|Z_AXIS_BIT_MASK):
                sprintf(acc_results, "Selected axis are y and z\r\n\r\n");
                break;
            case (X_AXIS_BIT_MASK|Z_AXIS_BIT_MASK):
                sprintf(acc_results, "Selected axis are x and z\r\n\r\n" );
                break;
            case (X_AXIS_BIT_MASK|Y_AXIS_BIT_MASK|Z_AXIS_BIT_MASK):
                sprintf(acc_results, "Selected axis are x,y and z\r\n\r\n" );
                break;
            default:
                break;
            }

            HAL_UART_send((u8_t*) acc_results, strlen( acc_results ) );

            requested_samples = p_param_val_list[1].val_16;

            // TODO:following code is not required and also the print we would not be getting as param validation fails and the control would not arrive here
            if( requested_samples > WES_WAKE_ALGO_VIB_NUM_DATA_SAMPLES )
            {
                requested_samples = WES_WAKE_ALGO_VIB_NUM_DATA_SAMPLES;
                sprintf(acc_results, "0nly %d samples allowed\r\n\r\n", WES_WAKE_ALGO_VIB_NUM_DATA_SAMPLES );
                HAL_UART_send((u8_t*) acc_results, sizeof( acc_results ) );
            }

            /* reset the buffer */
            STDC_memset( acc_results, 0x00, sizeof ( acc_results ) );

            //TODO: do we need to take care of any race conditions here??!!
            WES_WAKE_ALGO_collect_acc_test_samples( requested_axis, requested_samples );

            STDC_memset( acc_results, 0x00, sizeof ( acc_results ) );

            /* We now need to break up the entire buffer into 100 samples at a time ( 16bit ) */
            cycles = (requested_samples / DEBUG_CLI_SAMPLES_PER_CYCLE);
            remaining_samples = (requested_samples % DEBUG_CLI_SAMPLES_PER_CYCLE);

            /* Convert from counts to mv ( found on the T.I forums ) */
            battery_voltage = AONBatMonBatteryVoltageGet();
            battery_voltage = ( ( battery_voltage * 125 ) >> 5 );

            sprintf(acc_results, "Bat V:  %dmv\r\n\r\n", battery_voltage );
            HAL_UART_send((u8_t*) acc_results, strlen( acc_results ) );
            STDC_memset( acc_results, 0x00, sizeof ( acc_results ) );

            /* Now cycle round x times until all blocks of 100 samples have been sent */
            for (i = 0u; i < cycles; i++)
            {
                WES_WAKE_ALGO_get_acc_test_data( acc_test_data, ( DEBUG_CLI_SAMPLES_PER_CYCLE*2u ), ( i*DEBUG_CLI_SAMPLES_PER_CYCLE ) );

                for( j = 0u; j < DEBUG_CLI_SAMPLES_PER_CYCLE; j++ )
                {
                    sample_counter++;
                    sprintf(acc_results, "%04d: %d\r\n", sample_counter, ( acc_test_data[( ( j*2u ) + 1 )] << 8u ) | acc_test_data[j*2u] );
                    HAL_UART_send((u8_t*) acc_results, strlen( acc_results ) );
                }
            }

            /* Send the remainder but only if the remainder isnt 0 :) */
            if (remaining_samples != 0)
            {
                WES_WAKE_ALGO_get_acc_test_data( acc_test_data, ( remaining_samples*2u ), ( i*DEBUG_CLI_SAMPLES_PER_CYCLE ) );

                for( j = 0; j < remaining_samples; j++ )
                {
                    sample_counter++;
                    sprintf(acc_results, "%04d: %d\r\n", sample_counter, ( acc_test_data[( ( j*2u ) + 1 )] << 8u ) | acc_test_data[j*2u] );
                    HAL_UART_send((u8_t*) acc_results, strlen( acc_results ) );
                }
            }

            sprintf(acc_results, "ACC log Completed\r\n" );
            HAL_UART_send((u8_t*) acc_results, strlen( acc_results ) );
        }

    }
    else
    {
        err  = DEBUG_CLI_ERROR_INVALID_STR_PARAM;
    }

    return err;
}

STATIC DEBUG_CLI_error_et  handle_accall(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char acc_results[50];

    u8_t acc_test_data[6];

    u16_t battery_voltage = AONBatMonBatteryVoltageGet();
    /* Convert from counts to mv ( found on the T.I forums ) */
    battery_voltage = ( ( battery_voltage * 125 ) >> 5 );

    //TODO: do we need to take care of any race conditions here??!!
    WES_WAKE_ALGO_collect_acc_test_samples( X_AXIS_BIT_MASK | Y_AXIS_BIT_MASK | Z_AXIS_BIT_MASK , 3u );

    WES_WAKE_ALGO_get_acc_test_data( acc_test_data, 3*2u, 0u );

    sprintf(acc_results, "Bat Voltage:  %d mv\r\nX axis: %d\r\nY axis: %d\r\nZ axis: %d\r\n\r\n",
            battery_voltage,
            ( acc_test_data[1] << 8u | acc_test_data[0] ),
            ( acc_test_data[3] << 8u | acc_test_data[2] ),
            ( acc_test_data[5] << 8u | acc_test_data[4] ));

    HAL_UART_send((u8_t*) acc_results, strlen( acc_results ) );

    return DEBUG_CLI_ERROR_NONE;
}


//RF test command handlers
STATIC DEBUG_CLI_error_et handle_rftest(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char string[40];

    sprintf(string, "\r\nRftest mode %s \r\n\r\n",(otDiagIsEnabled())?"enabled":"disabled");
    HAL_UART_send((u8_t*)string, strlen(string));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et handle_rftest_start(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char string[20];
    const char *resp;
    char newline[] = "\r\n";

    // rftest start
    sprintf(string, "diag start");
    resp = otDiagProcessCmdLine(string);
    HAL_UART_send((u8_t*)newline, strlen(newline));
    HAL_UART_send((u8_t*)resp, strlen(resp));

    // Set power and channel from NV if previously saved we can use the channel value to check this
    if (0u != NVM_info_s.NVM_data_store_s.rftestData.channel)
    {
       // set power
       sprintf(string, "diag power %d ", NVM_info_s.NVM_data_store_s.rftestData.power);
       resp = otDiagProcessCmdLine(string);
       HAL_UART_send((u8_t*)newline, strlen(newline));
       HAL_UART_send((u8_t*)resp, strlen(resp));

       // set channel
       sprintf(string, "diag channel %d", NVM_info_s.NVM_data_store_s.rftestData.channel);
       resp = otDiagProcessCmdLine(string);
       HAL_UART_send((u8_t*)newline, strlen(newline));
       HAL_UART_send((u8_t*)resp, strlen(resp));
    }

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et handle_rftest_stop(int aArgCount,DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char newline[] = "\r\n";

    char string[50];
    // Extract and form command
    sprintf(string, "diag stop");

    // Send command to thread stack
    const char *resp = otDiagProcessCmdLine(string);
    HAL_UART_send((u8_t*)newline, sizeof(newline));

    HAL_UART_send((u8_t*)resp, strlen(resp));
    HAL_UART_send((u8_t*)newline, sizeof(newline));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et handle_rftest_channel(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char string[50];
    char newline[] = "\r\n";

    // Form command
    sprintf(string, "diag channel ");
    // Append cmd data to set new channel
    strncat(string, (char*)p_param_val_list[0].str, 10);

    // Send command to thread stack
    const char *resp = otDiagProcessCmdLine(string);
    if (NULL != strstr(resp, "status 0x00"))
    {
        // Command was successful store channel in NV
        NVM_info_s.NVM_data_store_s.rftestData.channel = atoi((char*)p_param_val_list[0].str);
        NVM_request_flush();
    }

    HAL_UART_send((u8_t*)newline, sizeof(newline));
    HAL_UART_send((u8_t*)resp, strlen(resp));
    HAL_UART_send((u8_t*)newline, sizeof(newline));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_rftest_power(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char string[50];
    char newline[] = "\r\n";

    // Form command
    sprintf(string, "diag power ");

    // Append string to set new power level
    strncat(string, (char*)p_param_val_list[0].str, 10);

    // Send command to thread stack
    const char *resp = otDiagProcessCmdLine(string);
    if (NULL != strstr(resp, "status 0x00"))
    {
        // Command was successful store power level in NV
        NVM_info_s.NVM_data_store_s.rftestData.power = atoi((char*)p_param_val_list[0].str);
        NVM_request_flush();
    }

    HAL_UART_send((u8_t*)newline, sizeof(newline));
    HAL_UART_send((u8_t*)resp, strlen(resp));
    HAL_UART_send((u8_t*)newline, sizeof(newline));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_rftest_tx(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char string[50];
    char newline[] = "\r\n";

    sprintf(string, "diag send ");
    //append the number of packets string param
    strncat(string, (char*)p_param_val_list[0].str, 10);

    strncat(string, " ", 2);

    //append the packet len param
    strncat(string, (char*)p_param_val_list[1].str, 10);

    // Send command to thread stack
    const char *resp = otDiagProcessCmdLine(string);

    HAL_UART_send((u8_t*)newline, sizeof(newline));
    HAL_UART_send((u8_t*)resp, strlen(resp));
    HAL_UART_send((u8_t*)newline, sizeof(newline));


    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_rftest_cont(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char string[50];
    char newline[] = "\r\n";

    sprintf(string, "diag repeat ");

    //append delay param string ( missliseconds )
    strncat(string, (char*)p_param_val_list[0].str, 10);

    strncat(string, " ", 2);

    //append lenth param string ( bytes )
    strncat(string, (char*)p_param_val_list[1].str, 10);

    // Send command to thread stack
    const char *resp = otDiagProcessCmdLine(string);

    HAL_UART_send((u8_t*)newline, sizeof(newline));
    HAL_UART_send((u8_t*)resp, strlen(resp));
    HAL_UART_send((u8_t*)newline, sizeof(newline));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_rftest_cont_stop(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char newline[] = "\r\n";

    char string[50];
    // Extract and form command
    sprintf(string, "diag repeat stop");

    // Send command to thread stack
    const char *resp = otDiagProcessCmdLine(string);

    HAL_UART_send((u8_t*)newline, sizeof(newline));
    HAL_UART_send((u8_t*)resp, strlen(resp));
    HAL_UART_send((u8_t*)newline, sizeof(newline));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_rftest_stats(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char string[100];
    char newline[] = "\r\n";

    // Extract and form command
    sprintf(string, "diag stats");

    // Send command to thread stack
    const char *resp = otDiagProcessCmdLine(string);

    HAL_UART_send((u8_t*)newline, sizeof(newline));
    HAL_UART_send((u8_t*)resp, strlen(resp));
    HAL_UART_send((u8_t*)newline, sizeof(newline));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_rftest_rssi(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char string[50];
    //char newline[] = "\r\n";

    if ((OtInstance_get() != NULL) && otDiagIsEnabled())
    {
        // Get RSSI
        int8_t rssiMax = otPlatRadioGetRssi(OtInstance_get());
        int8_t rssiLast = otPlatRadioGetLastRssi(OtInstance_get());
        sprintf(string, "last msg rssi value: %ddBm, max rssi value: %ddBm\r\n", rssiLast, rssiMax);
    }
    else
    {
        sprintf(string, "RF test not enabled.. ensure its enabled to get rssi\r\n");
    }

    HAL_UART_send((u8_t*)string, strlen(string));

    return DEBUG_CLI_ERROR_NONE;
}

STATIC DEBUG_CLI_error_et  handle_rftest_sleep(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char string[50];
    char newline[] = "\r\n";

    // Extract and form command
    sprintf(string, "diag sleep");

    // Send command to thread stack
    const char *resp = otDiagProcessCmdLine(string);

    HAL_UART_send((u8_t*)newline, sizeof(newline));
    HAL_UART_send((u8_t*)resp, strlen(resp));

    return DEBUG_CLI_ERROR_NONE;
}


//vehicle test RF commands handlers
STATIC DEBUG_CLI_error_et  handle_ackreq(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char ackreq_cmd_result[50] = { 0 };
    DEBUG_CLI_error_et err = DEBUG_CLI_ERROR_NONE;

    if( OT_get_ackreq_config() != p_param_val_list[0].val_8 )
    {
        OT_update_ackreq_config(p_param_val_list[0].val_8);
        sprintf(ackreq_cmd_result, "\r\nmac ack reqs %s\r\n", (p_param_val_list[0].val_8)?"enabled":"disabled");
        HAL_UART_send((u8_t*) ackreq_cmd_result, strlen(ackreq_cmd_result));
    }
    else
    {
        err = DEBUG_CLI_ALREADY_SET;
    }

    return err;
}

//functional test mode 1 commands handlers
STATIC DEBUG_CLI_error_et  handle_dv(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
    char eol_results[56];

    MODE_MGR_get_eol_test_results( (u8_t*)eol_results );

    char debug_string[250] = { 0x20 };
    sprintf(debug_string, "\r\n>>\r\nEOL test results...\r\n\r\n");
    HAL_UART_send((u8_t*) debug_string, strlen(debug_string));
    STDC_memset( debug_string, 0x20, sizeof( debug_string ) );

    sprintf(debug_string, "EUI64:\t\t\t0x%02x%02x%02x%02x%02x%02x%02x%02x\r\n",
            eol_results[0], eol_results[1], eol_results[2], eol_results[3], eol_results[4], eol_results[5], eol_results[6], eol_results[7]);
    HAL_UART_send((u8_t*) debug_string, strlen(debug_string));
    STDC_memset( debug_string, 0x20, sizeof( debug_string ) );

    sprintf(debug_string, "Status 1:\t\t0x%02X\r\nStatus 2:\t\t0x%02X\r\nASIC 1 Pressure:\t%04d mb\r\nASIC 2 Pressure:\t%04d mb\r\nASIC 1 Temperature:\t%03u %cC\r\nASIC 2 Temperature:\t%03u %cC\r\nTI temp:\t\t%03u %cC\r\n\r\n",
            eol_results[8], eol_results[9], ( eol_results[10] << 8u | eol_results[11] ), ( eol_results[12] << 8u | eol_results[13] ),
            0xF8, ( eol_results[14] << 8u | eol_results[15] ), 0xF8, ( eol_results[16] << 8u | eol_results[17] ), 0xF8, ( eol_results[18] << 8u | eol_results[19] ) );
    HAL_UART_send((u8_t*) debug_string, strlen(debug_string));
    STDC_memset( debug_string, 0x20, sizeof( debug_string ) );

    sprintf(debug_string, "Run Bearing count:\t%d\r\nPacket Count:\t\t%d\r\nBattery Voltage:\t%04d mv\r\nBreak in counter:\t%d\r\nBearing Fault ctr:\t%d\r\nAvg wheel speed:\t%d\r\nSelf Check status:\t0x%02X\r\n",
            eol_results[20], ( eol_results[21] << 8u | eol_results[22] ), ( eol_results[23] << 8u | eol_results[24] ), eol_results[25], eol_results[26], eol_results[27], eol_results[28] );
    HAL_UART_send((u8_t*) debug_string, strlen(debug_string));
    STDC_memset( debug_string, 0x20, sizeof( debug_string ) );

    sprintf(debug_string, "ASIC 1 TX Fails:\t%d\r\nASIC 2 TX Fails:\t%d\r\nASIC 1 CRC Fails:\t%d\r\nASIC 2 CRC Fails:\t%d\r\nASIC 1 PAL data:\t0x%02X\r\nASIC 1 Status:\t\t0x%04X\r\nASIC 2 PAL data:\t0x%02X\r\nASIC 2 Status:\t\t0x%04X\r\n\r\n",
            ( eol_results[29] << 8u | eol_results[30] ), ( eol_results[31] << 8u | eol_results[32] ),
            ( eol_results[33] << 8u | eol_results[34] ), ( eol_results[35] << 8u | eol_results[36] ),
              eol_results[37], ( eol_results[38] << 8u | eol_results[39] ),
              eol_results[40], ( eol_results[41] << 8u | eol_results[42] ) );
    HAL_UART_send((u8_t*) debug_string, strlen(debug_string));
    STDC_memset( debug_string, 0x20, sizeof( debug_string ) );

    sprintf(debug_string, "Acc Fault ctr:\t\t%d\r\nX axis:\t\t\t%03d\r\nY axis:\t\t\t%03d\r\nZ axis:\t\t\t%03d\r\nFlash Fault ctr:\t%d\r\nNFC Fault ctr:\t\t%d\r\nCC2652 Reset reason:\t0x%02X\r\n\r\n<<",
             ( eol_results[43] << 8u | eol_results[44] ),
             ( eol_results[45] << 8u | eol_results[46] ), ( eol_results[47] << 8u | eol_results[48] ), ( eol_results[49] << 8u | eol_results[50] ),
             ( eol_results[51] << 8u | eol_results[52] ),
             ( eol_results[53] << 8u | eol_results[54] ),
             ( eol_results[55] ) );
    HAL_UART_send((u8_t*) debug_string, strlen(debug_string));
    STDC_memset( debug_string, 0x20, sizeof( debug_string ) );

    return DEBUG_CLI_ERROR_NONE;
}


STATIC DEBUG_CLI_error_et  handle_parentsearchtime(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
	DEBUG_CLI_error_et err = DEBUG_CLI_ERROR_NONE;

    char parent_search_string[70];
    char *result = strstr(DEBUG_CLI_msg_read_s, "parentsearch time") + 18;
    STDC_memset(parent_search_string, 0x20, sizeof(parent_search_string));

    NVM_info_s.NVM_data_store_s.parent_search_interval_time = p_param_val_list[0].val_32;

    sprintf(parent_search_string,"\r\nParent search interval time has been set to %d secs\r\n\r\n",NVM_info_s.NVM_data_store_s.parent_search_interval_time);
    HAL_UART_send((u8_t*) parent_search_string, sizeof(parent_search_string));

     /* Request a flush to NVM */
    NVM_request_flush();

    return ( err );
}


STATIC DEBUG_CLI_error_et  handle_parentsearchlimit(int aArgCount, DEBUG_CLI_Param_value_ut* p_param_val_list )
{
	DEBUG_CLI_error_et err = DEBUG_CLI_ERROR_NONE;

	char parent_search_string[70];
	char *result = strstr(DEBUG_CLI_msg_read_s, "parentsearch limit") + 19;
	STDC_memset(parent_search_string, 0x20, sizeof(parent_search_string));

	NVM_info_s.NVM_data_store_s.parent_search_cycle_limit = p_param_val_list[0].val_32;

	sprintf(parent_search_string,"\r\nParent search attempt limit has been set to %d\r\n\r\n",NVM_info_s.NVM_data_store_s.parent_search_cycle_limit);
	HAL_UART_send((u8_t*) parent_search_string, sizeof(parent_search_string));

	 /* Request a flush to NVM */
	NVM_request_flush();

	return ( err );
}



/*!
****************************************************************************************************
*
*   \brief         DEBUG_CLI task
*
*   \author        MS
*
*   \return        void
*
***************************************************************************************************/
void *DEBUG_CLI_Thread(void *arg0)
{
    //let the history be retained between CLI sleeps. So initialize this table only once here and not everytime CLI is initialized
    STDC_memset(&cmdhistory,0x00,sizeof(cmdhistory));
    /* Setup the ISR for the falling edge of the RX pin */
    DEBUG_CLI_setup_rx_wakeup();

    while (1)
    {
        if( DEBUG_CLI_timeout_s == 0u )
        {
            /* Close the serial port if its open*/
            if( DEBUG_CLI_open_s == SEL_TRUE )
            {
                DEBUG_CLI_close();
            }
            /* Only Pend the task when we have timed out, and then wait until we receive a UART byte to trigger the start */
            Semaphore_pend( DEBUG_CLI_Sem_Handle, BIOS_WAIT_FOREVER);

            if( DEBUG_CLI_open_s == SEL_FALSE )
            {
                /* Initialise the debug port and send the welcome screen */
                DEBUG_CLI_init();

                /* Display what mode we are in */
                DEBUG_CLI_print_prompt( SEL_TRUE );
            }
        }
        else
        {
            /* Check if we have received a Carriage return as this signifies the end of user input */
            if( DEBUG_CLI_get_serial_message_status() == SEL_TRUE )
            {
                /* This is where we need to analyse the received command as the Carriage Return has been pressed */
                DEBUG_CLI_handle_serial_command();
                /*Clear buffer after analyse as the user may type in other commands before the board goes to sleep */
                DEBUG_CLI_clear_rx_buffer();
            }

            /* Try and Receive another byte */
            DEBUG_CLI_receive_byte();

            /* Decrement the timeout */
            DEBUG_CLI_timeout_s --;
        }
    }
}

/*!
****************************************************************************************************
*
*   \brief         Creates the DEBUG_CLI task
*
*   \author        MS
*
*   \return        void
*
***************************************************************************************************/
void DEBUG_CLI_taskCreate(void)
{
    pthread_t thread;
    pthread_attr_t attrs;
    struct sched_param priParam;

    int retc;
    int detachState;

    /* Set priority and stack size attributes */
    pthread_attr_init(&attrs);
    priParam.sched_priority = 1;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0)
    {
       /* pthread_attr_setdetachstate() failed */
       while (1)
           ;
    }

    pthread_attr_setschedparam(&attrs, &priParam);

    retc |= pthread_attr_setstacksize(&attrs, TASK_CONFIG_WES_DEBUG_TASK_STACK_SIZE);
    if (retc != 0)
    {
       /* pthread_attr_setstacksize() failed */
       while (1)
           ;
    }

    retc = pthread_create(&thread, &attrs, DEBUG_CLI_Thread, NULL);
    if (retc != 0)
    {
       /* pthread_create() failed */
       while (1)
           ;
    }

    Semaphore_construct(&DEBUG_CLI_Sem_Struct, 0, &DEBUG_CLI_Sem_Params);
    DEBUG_CLI_Sem_Handle = Semaphore_handle(&DEBUG_CLI_Sem_Struct);
}

/*!
****************************************************************************************************
*
*   \brief         Init the DEBUG_CLI
*
*   \author        MS
*
*   \return        void
*
***************************************************************************************************/
STATIC void DEBUG_CLI_init(void)
{
    /* Init the Uart module */
    HAL_UART_init();

    /* Clear the buffer */
    DEBUG_CLI_clear_rx_buffer();

    /* reset the received byte index flag */
    DEBUG_CLI_byte_index_s = 0u;
    DEBUG_CLI_rx_command_len=0;

    /* Reset  Carriage return flag */
    DEBUG_CLI_cr_received_s = SEL_FALSE;

    /* Flag to let the DEBUG_CLI interface know that the serial port is now open */
    DEBUG_CLI_open_s = SEL_TRUE;

    /* Send the welcome screen */
    HAL_UART_send((u8_t*) DEBUG_CLI_consoleDisplay_s, sizeof(DEBUG_CLI_consoleDisplay_s) );
}


/*!
****************************************************************************************************
*
*   \brief         Close the debug CLI
*
*   \author        MS
*
*   \return        void
*
***************************************************************************************************/
STATIC void DEBUG_CLI_close(void)
{
    HAL_UART_send((u8_t*) DEBUG_CLI_closing_serial_port_s, sizeof(DEBUG_CLI_closing_serial_port_s) );

    /* De-Init the Uart module */
    HAL_UART_de_init();

    /* Setup the wakeup state for the next time we receive a charachter over the serial port */
    DEBUG_CLI_setup_rx_wakeup();

    DEBUG_CLI_open_s = SEL_FALSE;
}


/*!
****************************************************************************************************
*
*   \brief         Clears the RX buffer
*
*   \author        MS
*
*   \return        void
*
***************************************************************************************************/
STATIC void DEBUG_CLI_clear_rx_buffer( void )
{
     STDC_memset( DEBUG_CLI_msg_read_s, 0x00, sizeof(DEBUG_CLI_msg_read_s) );
     DEBUG_CLI_byte_index_s = 0u;
     DEBUG_CLI_rx_command_len=0;
}

/*!
****************************************************************************************************
*
*   \brief         Receive 1 byte from the UART buffer
*
*   \author        MS
*
*   \return        void
*
***************************************************************************************************/
STATIC void DEBUG_CLI_receive_byte( void )
{
    HAL_UART_receive_byte();
}

/*!
****************************************************************************************************
*
*   \brief         Check for a valid serial message being received
*
*   \author        MS
*
*   \return        void
*
***************************************************************************************************/
STATIC SEL_false_true_et DEBUG_CLI_get_serial_message_status( void )
{
    return ( DEBUG_CLI_cr_received_s );
}

STATIC bool IsSpaceOrNewLine(char aChar)
{
    return (aChar == ' ') || (aChar == '\t') || (aChar == '\r') || (aChar == '\n');
}

STATIC DEBUG_CLI_error_et DEBUG_CLI_parse_cmd(char *aString, uint8_t * aArgc, char **aArgv, uint8_t aArgcMax)
{
    DEBUG_CLI_error_et error = DEBUG_CLI_ERROR_NONE;
    char *  cmd;

    *aArgc = 0;

    for (cmd = aString; IsSpaceOrNewLine(*cmd) && *cmd; cmd++)
        ;

    if (*cmd)
    {
        aArgv[(*aArgc)++] = cmd++; // the first argument

        for (; *cmd; cmd++)
        {
            if (IsSpaceOrNewLine(*cmd))
            {
                *cmd = '\0';
            }
            else if (*(cmd - 1) == '\0')
            {
                if( (*aArgc) >= aArgcMax )
                {
                    error = DEBUG_CLI_ERROR_INVALID_ARGS_MORE;
                    break;
                }
                aArgv[(*aArgc)++] = cmd;
            }
        }
    }
    else
    {
        // a null charater encountered!!
        // a simple new line entered by user. So just print the prompt
        error = DEBUG_CLI_ERROR_EMPTY;
    }

    return error;
}



STATIC DEBUG_CLI_error_et DEBUG_CLI_get_param_value(const DEBUG_CLI_Param_st* p_param_info, char *p_param_str, DEBUG_CLI_Param_value_ut* p_val )
{
    DEBUG_CLI_error_et error = DEBUG_CLI_ERROR_NONE;
    u8_t char_index = 0;

    char param_str[9]={0};
    char byte_str[3]={0};

    if( p_param_info->format == HEX )
    {
        if( strlen(p_param_str) == p_param_info->num_chars )
        {
            while(char_index<p_param_info->num_chars)
            {
                if(!IsHexChar(p_param_str[char_index]))
                {
                    error = DEBUG_CLI_ERROR_INVALID_HEX_PARAM;
                    break;
                }
                char_index++;
            }

            if( error == DEBUG_CLI_ERROR_NONE )
            {
                STDC_memset( p_val->val_64, 0, sizeof( p_val->val_64 ) );
                // all good with the chars used to specify the param value
                //param with hex value
                switch( p_param_info->num_chars )
                {
                    case HEX_BYTE_8_MIN_NUM_CHARS:
                        sprintf(byte_str, "%c%c", p_param_str[0], p_param_str[1]); byte_str[HEX_BYTE_8_MIN_NUM_CHARS]=0;
                        p_val->val_8 =  StrToHex(byte_str);
                        break;

                    case HEX_WORD_16_MIN_NUM_CHARS:
                        sprintf(param_str, "%c%c%c%c", p_param_str[0], p_param_str[1], p_param_str[2], p_param_str[3]); param_str[HEX_WORD_16_MIN_NUM_CHARS]=0;
                        p_val->val_16 = StrToHex(param_str);
                        break;

                    case HEX_WORD_32_MIN_NUM_CHARS:
                        sprintf(param_str, "%c%c%c%c%c%c%c%c", p_param_str[0], p_param_str[1], p_param_str[2], p_param_str[3],p_param_str[4], p_param_str[5], p_param_str[6], p_param_str[7]); param_str[HEX_WORD_16_MIN_NUM_CHARS]=0;
                        p_val->val_32 =  StrToHex(param_str);
                        break;

                    case HEX_WORD_64_MIN_NUM_CHARS:

                        for (int i = 0; i < HEX_WORD_64_MIN_NUM_CHARS; i++)
                        {
                            sprintf(byte_str, "%c%c", p_param_str[i*2], p_param_str[(i*2)+1]); byte_str[2]=0;
                            p_val->val_64[i] = StrToHex(byte_str);
                        }

                        break;
                    default:
                        error = DEBUG_CLI_ERROR_INVALID_HEX_PARAM_LEN;
                        break;
                }
            }
        }
        else
        {
            error = DEBUG_CLI_ERROR_INVALID_HEX_PARAM_LEN;
        }
    }
    else if (p_param_info->format == DEC)
    {

        while(p_param_str[char_index])
        {
            if(!IsDecChar(p_param_str[char_index]))
            {
                error = DEBUG_CLI_ERROR_INVALID_DEC_PARAM;
                break;
            }
            char_index++;
        }

        if ( error == DEBUG_CLI_ERROR_NONE )
        {
            if( p_param_info->num_chars >= strlen(p_param_str) )
            {
                STDC_memset( p_val->val_64, 0, sizeof( p_val->val_64 ) );
                switch( p_param_info->num_chars )
                {
                case DEC_BOOLEAN_MAX_NUM_CHARS:
                    p_val->val_8 = atoi(p_param_str);
                    break;

                case DEC_BYTE_8_MAX_NUM_CHARS:
                    p_val->val_8 = atoi(p_param_str);
                    break;

                case DEC_WORD_16_MAX_NUM_CHARS:
                    p_val->val_16 = atoi(p_param_str);
                    break;

                case DEC_WORD_32_MAX_NUM_CHARS:
                    p_val->val_32 = atoi(p_param_str);
                    break;
                default:
                    error = DEBUG_CLI_ERROR_INVALID_DEC_PARAM_LEN;
                    break;
                }
            }
            else
            {
                //problem
                error = DEBUG_CLI_ERROR_INVALID_DEC_PARAM_LEN;
            }
        }
    }
    else
    {
        // its a string param
        STDC_memset( p_val->str, 0, sizeof( p_val->str ) );
        strcpy((char*)p_val->str,p_param_str);
    }

    // if it is required to do param value range checking , do it now to see if the received value is well within the expected range as configred in the table
    if( !(error) && !( p_param_info->llimit == IGNORE_RANGE_CHECK && p_param_info->ulimit == IGNORE_RANGE_CHECK))
    {
        // range checking required!! if range checking is not required for eg for ext panid, the table is configured with 0xFFFFFFF for both upper and lower limit
        if( ( p_val->val_32 < p_param_info->llimit ) || ( p_val->val_32 > p_param_info->ulimit) )
        {
            error = DEBUG_CLI_ERROR_PARAM_OUT_OF_RANGE;
        }

    }
    return error;
}

STATIC DEBUG_CLI_error_et DEBUG_CLI_get_cum_validate_cmd_params(const DEBUG_CLI_Command_st *cmd, int aArgCount, char *aArgVector[], DEBUG_CLI_Param_value_ut* p_param_val_list)
{
    DEBUG_CLI_error_et error = DEBUG_CLI_ERROR_NONE;
    u8_t index = 0;

    if( ( cmd != NULL ) && ( cmd->num_params == aArgCount ) )
    {
        if( ( aArgCount ) && ( aArgVector != NULL ) )
        {
            do
            {
                error = DEBUG_CLI_get_param_value(&(cmd->param_list[index]), aArgVector[index], &(p_param_val_list[index]));

                index++;
            }while( ( error == DEBUG_CLI_ERROR_NONE ) && ( index < aArgCount) );
        }
    }
    else

    {
        error = DEBUG_CLI_ERROR_INVALID_ARGS;
    }

    return error;
}

STATIC DEBUG_CLI_error_et DEBUG_CLI_process_cmd(int aArgCount, char *aArgVector[])
{
    DEBUG_CLI_error_et error = DEBUG_CLI_ERROR_NONE;
    const DEBUG_CLI_Command_st *cmd = NULL;
    DEBUG_CLI_Param_value_ut    val[DEBUG_CLI_CMD_LINE_ARGS_MAX];

    for ( cmd = cli_cmds; cmd->mName != NULL; cmd++)
    {
        if ((strcmp(aArgVector[0], cmd->mName) == 0))
        {
            // check if in the current mode this command is allowed or not
            if( (cmd->availbility & BV(MODE_MGR_get_mode())))
            {
                if(NVM_info_s.NVM_data_store_s.developer_debug_console == SEL_ENABLE || cmd->public  )
                {
                    //the command can be executed. Now get into validating the parameters and their values provided by the user
                    error = DEBUG_CLI_get_cum_validate_cmd_params(cmd, (aArgCount-1), ((aArgCount-1)?&aArgVector[1]:NULL),((aArgCount-1)?val:NULL));
                }
                else
                {
                    error = DEBUG_CLI_ERROR_NOT_FOUND;
                }
            }
            else
            {
                error = DEBUG_CLI_ERROR_PROHIBITED;
            }
            // there is a matching entry in the table. just come out of the loop!
            break;
        }
    }

    if(cmd->mName == NULL)
    {
        // looks like we have looped through the entire table and we did not get see the command.
        error = DEBUG_CLI_ERROR_NOT_FOUND;
    }

    if( error == DEBUG_CLI_ERROR_NONE  )
    {
        error = cmd->mHandler((aArgCount-1),((aArgCount-1)?val:NULL));
    }

    return error;
}


STATIC void DEBUG_CLI_handle_serial_command(void)
{
    DEBUG_CLI_error_et error = DEBUG_CLI_ERROR_NONE;

    char *argVector[DEBUG_CLI_CMD_LINE_ARGS_MAX];
    uint8_t argCount = 0;

    char  string[80] = { 0x20 };

    STDC_memset( string, 0, sizeof( string ) );

    error = DEBUG_CLI_parse_cmd(DEBUG_CLI_msg_read_s, &argCount, argVector, DEBUG_CLI_CMD_LINE_ARGS_MAX);

    if(error==DEBUG_CLI_ERROR_NONE)
        error = DEBUG_CLI_process_cmd(argCount,argVector);
    strcpy(string,DEBUG_CLI_err_string[error]);

    HAL_UART_send((u8_t*) string, strlen(string));
    DEBUG_CLI_print_prompt( SEL_TRUE );
    DEBUG_CLI_cr_received_s = SEL_FALSE;
}

/*!
****************************************************************************************************
*
*   \brief         Sets the correct PINS ( UART RX ) to detect a data transfer edge to trigger the
*                  initialisation of the serial port
*
*   \author        MS
*
*   \return        void
*
***************************************************************************************************/
STATIC void DEBUG_CLI_setup_rx_wakeup( void )
{
    /* Now open the ISR pin so that a NEG interrupt edge will trigger it */
    RX_ISR_PinHandle = PIN_open( &RX_ISR_PinState, RX_ISR_Pin);
    if(!RX_ISR_PinHandle)
    {
        /* Error initializing button pins */
        while(1);
    }

    /* Setup callback for the ISR pin */
    if (PIN_registerIntCb((PIN_Handle)RX_ISR_PinHandle, (PIN_IntCb)&DEBUG_CLI_RX_PIN_interrupt_handler) != 0)
    {
        /* Error registering button callback function */
        while(1);
    }
}




/*!
****************************************************************************************************
*
*   \brief         Check each charachter received
*
*   \author        MS
*
*   \return        void
*
***************************************************************************************************/

void DEBUG_CLI_handle_received_char( u8_t received_char )
{
    u8_t index = 0;
    if( ( ( received_char >= DEBUG_CLI_SPACE ) && ( received_char <= DEBUG_CLI_DEL ) ) || ( received_char <= DEBUG_CLI_CR ) )
    {
        if ( received_char == '\r')
        {
            DEBUG_CLI_rx_command_len=DEBUG_CLI_byte_index_s;
            DEBUG_CLI_byte_index_s = 0;
            /* We have received a Carriage Return, so lets set this flag to let the task know that a valid
            command has been received */
            DEBUG_CLI_cr_received_s = SEL_TRUE;
            HAL_UART_send( "\n", sizeof(received_char) );

            if(DEBUG_CLI_rx_command_len!=0)
            {
                STDC_memset( cmdhistory.cmd_list[cmdhistory.index2next].cmd, 0, sizeof( cmdhistory.cmd_list[cmdhistory.index2next].cmd ) );

                strcpy(cmdhistory.cmd_list[cmdhistory.index2next].cmd,DEBUG_CLI_msg_read_s);

                cmdhistory.index2next = (cmdhistory.index2next+1)%DEBUG_CLI_MAX_COMMAND_HISTORY;
            }

        }
        else if(received_char == DEBUG_CLI_DEL)
        {
            if (DEBUG_CLI_byte_index_s>0)
            {
                DEBUG_CLI_byte_index_s--;
            }
            DEBUG_CLI_msg_read_s[ DEBUG_CLI_byte_index_s] = 0;
        }
        else if ( received_char == DEBUG_CLI_BACKSPACE )
        {
            if (DEBUG_CLI_byte_index_s>0)
            {
                DEBUG_CLI_byte_index_s--;
                DEBUG_CLI_msg_read_s[ DEBUG_CLI_byte_index_s] = 0;
                HAL_UART_send( "\b ", strlen("\b ") );
                HAL_UART_send( &received_char, 1 );

            }

            return;
        }
        else if( ( received_char == UP_ARROW || received_char == DOWN_ARROW ) &&(  DEBUG_CLI_msg_read_s[ DEBUG_CLI_byte_index_s-1 ] == '[' ) )
        {
            //received 'A' and may be we had received a '[' previously which means a up arrow has been hit.
            //load the previous command and print it and wait for the carriage return
            //DEBUG_CLI_msg_read_s[ DEBUG_CLI_byte_index_s++ ] = received_char;
            if(received_char == UP_ARROW  )
            {
                index = (cmdhistory.index2next)?cmdhistory.index2next-1:DEBUG_CLI_MAX_COMMAND_HISTORY-1;
            }
            else
            {
                index = (cmdhistory.index2next+1)%DEBUG_CLI_MAX_COMMAND_HISTORY;
            }

            cmdhistory.index2next = index;

            strcpy(DEBUG_CLI_msg_read_s,cmdhistory.cmd_list[index].cmd);

            DEBUG_CLI_byte_index_s = strlen(cmdhistory.cmd_list[index].cmd);

            HAL_UART_send( (u8_t*)ERASE_CURRENT_LINE, strlen(ERASE_CURRENT_LINE));

            DEBUG_CLI_print_prompt(SEL_FALSE);

            HAL_UART_send( (u8_t*)cmdhistory.cmd_list[index].cmd, strlen(cmdhistory.cmd_list[index].cmd) );
            return;
        }
        else if( received_char == '[' )
        {
            //received 'A' and may be we had received a '[' previously which means a up arrow has been hit.
            //load the previous command and print it and wait for the carriage return
            DEBUG_CLI_msg_read_s[ DEBUG_CLI_byte_index_s++ ] = received_char;
            return;
        }
        else
        {
            if( DEBUG_CLI_byte_index_s > DEBUG_CLI_MAX_INPUT_CHARS )
            {
                DEBUG_CLI_clear_rx_buffer();
            }

            /* Record every byte that gets received */
            DEBUG_CLI_msg_read_s[ DEBUG_CLI_byte_index_s++ ] = received_char;
        }

        HAL_UART_send( &received_char, sizeof(received_char) );
    }
    else
    {
        DEBUG_CLI_clear_rx_buffer();
    }
}

STATIC void DEBUG_CLI_print_prompt(SEL_false_true_et newline )
{
    MODE_MGR_operational_mode_et mode;
    char mode_string[100];

    STDC_memset( mode_string, 0x20, sizeof( mode_string ) );
    mode = MODE_MGR_get_mode();

    if( newline == SEL_TRUE )
    {
        sprintf( mode_string, "\r\n");
    }
    sprintf(  &mode_string[(newline==1)?2:0], "%s>", mode_lookup[mode].mName);

    HAL_UART_send( (u8_t*)mode_string, strlen(mode_string));
}

/*!
****************************************************************************************************
*
*   \brief         Resets the task timeout that handles closing the serial port
*
*   \author        MS
*
*   \return        void
*
***************************************************************************************************/
void DEBUG_CLI_reset_task_timeout( void )
{
    DEBUG_CLI_timeout_s = DEBUG_CLI_TASK_TIMEOUT;
}




/*!
****************************************************************************************************
*
*   \brief         The handler that gets called after the PIN interrup has been generated
*
*   \author        MS
*
*   \return        void
*
***************************************************************************************************/
STATIC void DEBUG_CLI_RX_PIN_interrupt_handler(uint_least8_t index)
{
    /* Disable the interrupt */
    PIN_setInterrupt( RX_ISR_PinHandle, PIN_ID( Board_TX ) | PIN_IRQ_DIS );
    PIN_clrPendInterrupt( RX_ISR_PinHandle, PIN_ID( Board_TX ) );

    /* The first thing we need to do is close the instance of the PIn that we used as the ISR,
    as we now need to assign it to the serial port */
    PIN_close( RX_ISR_PinHandle );

    /* Reset the timeout */
    DEBUG_CLI_reset_task_timeout();

    /* Start the SERIAL task by posting the SERIAL semaphore */
    Semaphore_post( DEBUG_CLI_Sem_Handle );
}




///*!
//****************************************************************************************************
//*
//*   \brief         The will wake the serial port from another task if data needs to be sent
//*
//*   \author        MS
//*
//*   \return        void
//*
//***************************************************************************************************/
//STATIC void DEBUG_CLI_external_wakeup( u8_t* data_p, u16_t len )
//{
//    /* Reset the timeout */
//    DEBUG_CLI_reset_task_timeout();
//
//    /* Lets simulate a 'CR' being enters as this keeps the DEBUG CLI state machine as it was before */
//    DEBUG_CLI_cr_received_s = SEL_TRUE;
//
//    /* Copy the data into the message buffer, simulating user interaction */
//    //STDC_memcpy( DEBUG_CLI_msg_read_s, data_p, len );
//
//    /* Start the SERIAL task by posting the SERIAL semaphore */
//    Semaphore_post( DEBUG_CLI_Sem_Handle );
//}
//


/*!
 ****************************************************************************************************
 *
 *   \brief         Posts the DEBUG_CLI task
 *
 *   \author        MS
 *
 *   \return        None
 ***************************************************************************************************/
void DEBUG_CLI_post(void)
{
    Semaphore_post( DEBUG_CLI_Sem_Handle );
}



/*!
 ****************************************************************************************************
 *
 *   \brief         Pends the DEBUG_CLI task
 *
 *   \author        MS
 *
 *   \return        None
 ***************************************************************************************************/
void DEBUG_CLI_pend(void)
{
    Semaphore_pend( DEBUG_CLI_Sem_Handle, BIOS_WAIT_FOREVER);
}

STATIC bool IsHexChar(char c)
{
    return ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||  (c >= 'A' && c <= 'F'));
}

STATIC bool IsDecChar(char c)
{
    return (c >= '0' && c <= '9');
}

/*!
****************************************************************************************************
*
*   \brief         Function to translate a string to HEX
*
*   \author        MS
*
*   \return        void
*
***************************************************************************************************/
STATIC uint64_t StrToHex(const char* str)
{
    return (uint64_t) strtoull(str, 0, 16);
}

