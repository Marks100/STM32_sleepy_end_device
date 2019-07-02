#ifndef BMP280_H
#define BMP280_H


/***************************************************************************************************
**                              Includes                                                          **
***************************************************************************************************/
#include "C_defs.h"
#include "COMPILER_defs.h"

#define BMP280_DEVICE_ID	0x58
#define BMP280_I2C_ADDR 	0xED
#define BMP280_RESET_CMD	0xB6

#define BMP280_MEASURING	        0x08
#define BMP280_MEASURE_COMPLETE  	0x00

#define	BMP280_OVERSAMPLING_T1		0x20
#define	BMP280_OVERSAMPLING_T2		0x40
#define	BMP280_OVERSAMPLING_T4		0x60
#define	BMP280_OVERSAMPLING_T8		0x80
#define	BMP280_OVERSAMPLING_T16		0xA0

#define	BMP280_OVERSAMPLING_P1		0x04
#define	BMP280_OVERSAMPLING_P2		0x08
#define	BMP280_OVERSAMPLING_P4		0x0C
#define	BMP280_OVERSAMPLING_P8		0x10
#define	BMP280_OVERSAMPLING_P16		0x14

#define	BMP280_TSB_0_5				0x00
#define	BMP280_TSB_62_5				0x20
#define	BMP280_TSB_125				0x40
#define	BMP280_TSB_250				0x60
#define	BMP280_TSB_500				0x80
#define	BMP280_TSB_1000				0xA0
#define	BMP280_TSB_2000				0xC0
#define	BMP280_TSB_4000				0xE0

#define	BMP280_FILTER_OFF			0x00
#define	BMP280_FILTER_COEFFICIENT2	0x04
#define	BMP280_FILTER_COEFFICIENT4	0x08
#define	BMP280_FILTER_COEFFICIENT8	0x0C
#define	BMP280_FILTER_COEFFICIENT16	0x10

#define	BMP280_MODE_SLEEP			0x00
#define	BMP280_MODE_FORCED			0x01
#define	BMP280_MODE_NORMAL			0x03

#define	BMP280_MEAS_BIT_MASK			(BMP280_OVERSAMPLING_T1 | BMP280_OVERSAMPLING_P1 | BMP280_MODE_SLEEP)
#define	BMP280_CONFIG_BIT_MASK		    (BMP280_TSB_0_5 | BMP280_FILTER_COEFFICIENT16 )

#define BMP280_READ_TIMEOUT			20u

/***************************************************************************************************
**                              Defines                                                           **
***************************************************************************************************/

typedef enum
{
	BMP280_SLEEP_MODE = 0u,
	BMP280_NORMAL_MODE,
	BMP280_FORCED_MODE = 3,
} BMP280_operating_modes_et;


typedef enum
{
	BMP280_TEMP_XLSB  = 0xFC,
	BMP280_TEMP_LSB   = 0xFB,
	BMP280_TEMP_MSB   = 0xFA,
	BMP280_PRESS_XLSB = 0xF9,
	BMP280_PRESS_LSB  = 0xF8,
	BMP280_PRESS_MSB  = 0xF7,
	BMP280_CONFIG     = 0xF5,
	BMP280_CTRL_MEAS  = 0xF4,
	BMP280_STATUS     = 0xF3,
	BMP280_RESET      = 0xE0,
	BMP280_ID		  = 0xD0,
	BMP280_CALIB_START = 0x88,

} BMP280_registers_et;


/***************************************************************************************************
**                              Constants                                                         **
***************************************************************************************************/
/* None */


/***************************************************************************************************
**                              Data Types and Enums                                              **
***************************************************************************************************/
/* None */
typedef struct
{
	u16_t calib_T1;
	s16_t calib_T2;
	s16_t calib_T3;

	u16_t calib_P1;
	s16_t calib_P2;
	s16_t calib_P3;
	s16_t calib_P4;
	s16_t calib_P5;
	s16_t calib_P6;
	s16_t calib_P7;
	s16_t calib_P8;
	s16_t calib_P9;

} BMP280_calib_st;



/***************************************************************************************************
**                              Exported Globals                                                  **
***************************************************************************************************/
/* None */



/***************************************************************************************************
**                              Function Prototypes                                               **
***************************************************************************************************/
void BMP280_init( void );
u8_t BMP280_read_id( void );
void BMP280_reset( void );
u8_t BMP280_read_status( void );

void  BMP280_set_mode ( BMP280_operating_modes_et mode );
u32_t BMP280_read_raw_pressure_counts( void );
u32_t BMP280_read_raw_temp_counts( void );
void  BMP280_read_calib_values( void );
void  BMP280_convert( u32_t* temperature, u32_t* pressure);
void  BMP280_trigger_meas( void );

s32_t BMP280_get_temperature( void );
pass_fail_et BMP280_get_failure_status( void );

#endif /* BMP280 multiple inclusion guard */

///****************************** END OF FILE *******************************************************/



//
//
//
//
//#ifndef _BMP280_LIB_H_
//#define _BMP280_LIB_H_
//
//#define BMP280_addr 0xED
//
//#define	BMP280_REG_CONTROL 0xF4
//#define	BMP280_REG_CONFIG 0xF5
//
//#define	BMP280_REG_RESULT_PRESSURE 0xF7			// 0xF7(msb) , 0xF8(lsb) , 0xF9(xlsb) : stores the pressure data.
//#define BMP280_REG_RESULT_TEMPRERATURE 0xFA		// 0xFA(msb) , 0xFB(lsb) , 0xFC(xlsb) : stores the temperature data.
//
//#define	BMP280_OVERSAMPLING_T1		0x20
//#define	BMP280_OVERSAMPLING_T2		0x40
//#define	BMP280_OVERSAMPLING_T4		0x60
//#define	BMP280_OVERSAMPLING_T8		0x80
//#define	BMP280_OVERSAMPLING_T16		0xA0
//
//#define	BMP280_OVERSAMPLING_P1		0x04
//#define	BMP280_OVERSAMPLING_P2		0x08
//#define	BMP280_OVERSAMPLING_P4		0x0C
//#define	BMP280_OVERSAMPLING_P8		0x10
//#define	BMP280_OVERSAMPLING_P16		0x14
//
//#define	BMP280_MODE_SLEEP			0x00
//#define	BMP280_MODE_FORCED			0x01
//#define	BMP280_MODE_NORMAL			0x03
//
//#define	BMP280_TSB_0_5				0x00
//#define	BMP280_TSB_62_5				0x20
//#define	BMP280_TSB_125				0x40
//#define	BMP280_TSB_250				0x60
//#define	BMP280_TSB_500				0x80
//#define	BMP280_TSB_1000				0xA0
//#define	BMP280_TSB_2000				0xC0
//#define	BMP280_TSB_4000				0xE0
//
//#define	BMP280_FILTER_OFF			0x00
//#define	BMP280_FILTER_COEFFICIENT2	0x04
//#define	BMP280_FILTER_COEFFICIENT4	0x08
//#define	BMP280_FILTER_COEFFICIENT8	0x0C
//#define	BMP280_FILTER_COEFFICIENT16	0x10
//
//#define	BMP280_SPI_OFF	0x00
//#define	BMP280_SPI_ON	0x01
//
//#define	BMP280_MEAS			(BMP280_OVERSAMPLING_T16 | BMP280_OVERSAMPLING_P16 | BMP280_MODE_NORMAL)
//#define	BMP280_CONFIG		(BMP280_TSB_0_5 | BMP280_FILTER_COEFFICIENT16 | BMP280_SPI_OFF)
//
//void BMP280_Init(void);
//void bmp280Convert(long* temperature, long* pressure);
//long bmp280CalcAltitude(long pressure);
