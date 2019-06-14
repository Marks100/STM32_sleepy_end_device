/*! \file
*
*               $Author: mstewart $
*
*               $Date: 2014-01-16 15:40:40 +0000 (Thu, 16 Jan 2014) $
*
*   \brief      BMP280
*/

/***************************************************************************************************
**                              Includes                                                          **
***************************************************************************************************/
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_gpio.h"
#include "misc.h"

#include "C_defs.h"
#include "STDC.h"
#include "HAL_BRD.h"
#include "NVM.h"
#include "COMPILER_defs.h"
#include "HAL_I2C.h"
#include "BMP280.h"



extern NVM_info_st NVM_info_s;

BMP280_calib_st BMP280_calib_s;
s32_t BMP280_pressure_reading_s;
s32_t BMP280_temperature_reading_s;
false_true_et BMP280_init_s;

pass_fail_et BMP280_failure_status_s;

/***************************************************************************************************
**                              Data declarations and definitions                                 **
***************************************************************************************************/
/* None */


void BMP280_init( void )
{
	BMP280_init_s = FALSE;
	BMP280_failure_status_s = PASS;

	BMP280_pressure_reading_s = 0;
	BMP280_temperature_reading_s = 0;

	u8_t register_data;
	u8_t id = 0u;

	STDC_memset( &BMP280_calib_s, 0x00, sizeof( BMP280_calib_s ) );

	id = BMP280_read_id();

	register_data = BMP280_MEAS_BIT_MASK;
	HAL_I2C_write_single_register( BMP280_I2C_ADDR, BMP280_CTRL_MEAS, &register_data );

	register_data = BMP280_CONFIG_BIT_MASK;
	HAL_I2C_write_single_register( BMP280_I2C_ADDR, BMP280_CONFIG, &register_data );

	register_data = 0u;
	HAL_I2C_read_register( BMP280_I2C_ADDR, BMP280_CONFIG, &register_data );

	BMP280_read_calib_values();

	BMP280_init_s = TRUE;

	if( id != BMP280_DEVICE_ID )
	{
		BMP280_failure_status_s = FAIL;
	}
}



u8_t BMP280_read_id( void )
{
	u8_t id;

	HAL_I2C_read_register( BMP280_I2C_ADDR, BMP280_ID, &id );

	return ( id );
}



void BMP280_reset( void )
{
	u8_t register_data = BMP280_RESET_CMD;

	HAL_I2C_write_single_register( BMP280_I2C_ADDR, BMP280_RESET, &register_data );
}


u8_t BMP280_read_status( void )
{
	u8_t status;

	HAL_I2C_read_register( BMP280_I2C_ADDR, BMP280_STATUS, &status );

	return( status );
}


void BMP280_read_calib_values( void )
{
	u8_t calib_data[24];

	HAL_I2C_read_multiple_registers( BMP280_I2C_ADDR, BMP280_CALIB_START, calib_data, sizeof( calib_data ) );

	BMP280_calib_s.calib_T1 = ( ( calib_data[1] << 8u ) | calib_data[0] );
	BMP280_calib_s.calib_T2 = ( ( calib_data[3] << 8u ) | calib_data[2] );
	BMP280_calib_s.calib_T3 = ( ( calib_data[5] << 8u ) | calib_data[4] );

	BMP280_calib_s.calib_P1 = ( ( calib_data[7] << 8u ) | calib_data[6] );
	BMP280_calib_s.calib_P2 = ( ( calib_data[9] << 8u ) | calib_data[8] );
	BMP280_calib_s.calib_P3 = ( ( calib_data[11] << 8u ) | calib_data[10] );
	BMP280_calib_s.calib_P4 = ( ( calib_data[13] << 8u ) | calib_data[12] );
	BMP280_calib_s.calib_P5 = ( ( calib_data[15] << 8u ) | calib_data[14] );
	BMP280_calib_s.calib_P6 = ( ( calib_data[17] << 8u ) | calib_data[16] );
	BMP280_calib_s.calib_P7 = ( ( calib_data[19] << 8u ) | calib_data[18] );
	BMP280_calib_s.calib_P8 = ( ( calib_data[21] << 8u ) | calib_data[20] );
	BMP280_calib_s.calib_P9 = ( ( calib_data[23] << 8u ) | calib_data[22] );
}


u32_t BMP280_read_raw_temp_counts( void )
{
	u8_t temp_data[3];
	u32_t temp_value;

	HAL_I2C_read_multiple_registers( BMP280_I2C_ADDR, BMP280_TEMP_MSB, temp_data, sizeof( temp_data ) );

	temp_value = ( ( temp_data[0] << 16u ) | ( temp_data[1] << 8 ) | temp_data[2] );

	return ( temp_value >> 4u );
}


u32_t BMP280_read_raw_pressure_counts( void )
{
	u8_t pressure_data[3];
	u32_t pressure_value;

	HAL_I2C_read_multiple_registers( BMP280_I2C_ADDR, BMP280_PRESS_MSB, pressure_data, sizeof( pressure_data ) );

	pressure_value = ( ( pressure_data[0] << 16u ) | ( pressure_data[1] << 8 ) | pressure_data[2] );

	return ( pressure_value >> 4u );
}



void BMP280_convert( u32_t* temperature, u32_t* pressure)
{
  unsigned long adc_T;
  unsigned long adc_P;

  adc_P = BMP280_read_raw_pressure_counts();
  adc_T = BMP280_read_raw_temp_counts();

  double var1, var2, p, t_fine;
  var1 = (((double)adc_T)/16384.0 - ((double)BMP280_calib_s.calib_T1)/1024.0) * ((double)BMP280_calib_s.calib_T2);
  var2 = ((((double)adc_T)/131072.0 - ((double)BMP280_calib_s.calib_T1)/8192.0) * (((double)adc_T)/131072.0 - ((double) BMP280_calib_s.calib_T1)/8192.0)) * ((double)BMP280_calib_s.calib_T3);
  t_fine = (var1 + var2);
  *temperature = (int32_t) (t_fine*10 / 5120.0);

  var1 = ((double)t_fine/2.0) - 64000.0;
  var2 = var1 * var1 * ((double)BMP280_calib_s.calib_P6) / 32768.0;
  var2 = var2 + var1 * ((double)BMP280_calib_s.calib_P5) * 2.0;
  var2 = (var2/4.0)+(((double)BMP280_calib_s.calib_P4) * 65536.0);
  var1 = (((double)BMP280_calib_s.calib_P3) * var1 * var1 / 524288.0 + ((double)BMP280_calib_s.calib_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0)*((double)BMP280_calib_s.calib_P1);
  if (var1 == 0.0)
  {
    return; // avoid exception caused by division by zero
  }
  p = 1048576.0 - (double)adc_P;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double)BMP280_calib_s.calib_P9) * p * p / 2147483648.0;
  var2 = p * ((double)BMP280_calib_s.calib_P8) / 32768.0;
  p = (p + (var1 + var2 + ((double)BMP280_calib_s.calib_P7)) / 16.0);

  *pressure = (long)p;
}



void BMP280_trigger_meas( void )
{
	u8_t status;
	u16_t timeout = 0u;

	BMP280_init();

	delay_us( 1000u );

	BMP280_set_mode( BMP280_FORCED_MODE );

	delay_us( 20000u );

	status = BMP280_read_status();

	while( ( ( status & BMP280_MEASURING ) == BMP280_MEASURING ) && ( timeout < BMP280_READ_TIMEOUT ) )
	{
		status = BMP280_read_status();
		delay_us( 20000u );
	}

	if( status != BMP280_MEASURE_COMPLETE )
	{
		BMP280_failure_status_s = FAIL;
	}

	BMP280_convert( &BMP280_temperature_reading_s, &BMP280_pressure_reading_s );

	BMP280_set_mode( BMP280_SLEEP_MODE );
}



void BMP280_set_mode ( BMP280_operating_modes_et mode )
{
  u8_t register_data = 0u;

  HAL_I2C_read_register( BMP280_I2C_ADDR, BMP280_CTRL_MEAS, &register_data );

  register_data &= ~( 0x03 );
  register_data |= mode;

  HAL_I2C_write_single_register( BMP280_I2C_ADDR, BMP280_CTRL_MEAS, &register_data );
}



s32_t BMP280_get_temperature( void )
{
	return ( BMP280_temperature_reading_s );
}

pass_fail_et BMP280_get_failure_status( void )
{
	return ( BMP280_failure_status_s );
}


///****************************** END OF FILE *******************************************************/














//
//
//
//#include "stm32f10x_i2c.h"
//#include "bmp280.h"
//#include <math.h>
//
//unsigned short dig_T1;
//short dig_T2;
//short dig_T3;
//unsigned short dig_P1;
//
//short dig_P2;
//short dig_P3;
//short dig_P4;
//short dig_P5;
//short dig_P6;
//short dig_P7;
//short dig_P8;
//short dig_P9;
//
//short bmp280ReadShort(unsigned char address)
//{
//  short msb=0;
//  short lsb=0;
//
//  I2C_AcknowledgeConfig(I2C1,ENABLE);
//  I2C_GenerateSTART(I2C1,ENABLE);
//
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
//  I2C_Send7bitAddress(I2C1, BMP280_addr, I2C_Direction_Transmitter);
//
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//
//  I2C_SendData(I2C1,address);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//
//  I2C_GenerateSTART(I2C1,ENABLE);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
//
//  I2C_Send7bitAddress(I2C1, BMP280_addr, I2C_Direction_Receiver);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
//
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//  lsb = I2C_ReceiveData(I2C1);
//
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//  msb = I2C_ReceiveData(I2C1);
//
//  I2C_GenerateSTOP(I2C1,ENABLE);
//  I2C_AcknowledgeConfig(I2C1,DISABLE);
//
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//  I2C_ReceiveData(I2C1);
//
//  return (msb << 8) | lsb;
//}
//
//unsigned long bmp280ReadLong(unsigned char address)
//{
//  unsigned long result=0;
//
//  unsigned long msb=0;
//  unsigned long lsb=0;
//  unsigned long xsb=0;
//
//  I2C_AcknowledgeConfig(I2C1,ENABLE);
//  I2C_GenerateSTART(I2C1,ENABLE);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
//
//  I2C_Send7bitAddress(I2C1, BMP280_addr, I2C_Direction_Transmitter);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//
//  I2C_SendData(I2C1,address);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//
//  I2C_GenerateSTART(I2C1,ENABLE);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
//
//  I2C_Send7bitAddress(I2C1, BMP280_addr, I2C_Direction_Receiver);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
//
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//  msb = I2C_ReceiveData(I2C1);
//
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//  lsb = I2C_ReceiveData(I2C1);
//
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//  xsb = I2C_ReceiveData(I2C1);
//
//  I2C_GenerateSTOP(I2C1,ENABLE);
//  I2C_AcknowledgeConfig(I2C1,DISABLE);
//
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//  I2C_ReceiveData(I2C1);
//
//  result = (msb << 16) | (lsb << 8) | xsb;
//
//  return (result >> 4);
//}
//
//void bmp280WriteByte(unsigned char address, unsigned char data)
//{
//  I2C_GenerateSTART(I2C1,ENABLE);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
//
//  I2C_Send7bitAddress(I2C1, BMP280_addr, I2C_Direction_Transmitter);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//
//  I2C_SendData(I2C1,address);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//
//  I2C_SendData(I2C1,data);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//
//  I2C_GenerateSTOP(I2C1,ENABLE);
//
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//}
////----------------------------------------
//
//void bmp280Convert(long* temperature, long* pressure)
//{
//  unsigned long adc_T;
//  unsigned long adc_P;
//  adc_T = bmp280ReadLong(BMP280_REG_RESULT_TEMPRERATURE);
//  adc_P = bmp280ReadLong(BMP280_REG_RESULT_PRESSURE);
//
//  double var1, var2, p, t_fine;
//  var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
//  var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) * (((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
//  t_fine = (var1 + var2);
//  *temperature = (int32_t) (t_fine*10 / 5120.0);
//
//  var1 = ((double)t_fine/2.0) - 64000.0;
//  var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
//  var2 = var2 + var1 * ((double)dig_P5) * 2.0;
//  var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
//  var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
//  var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
//  if (var1 == 0.0)
//  {
//    return; // avoid exception caused by division by zero
//  }
//  p = 1048576.0 - (double)adc_P;
//  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
//  var1 = ((double)dig_P9) * p * p / 2147483648.0;
//  var2 = p * ((double)dig_P8) / 32768.0;
//  p = (p + (var1 + var2 + ((double)dig_P7)) / 16.0);
//
//  *pressure = (long)p;
//}
////----------------------------------------
//
//void BMP280_Init(void)
//{
//  dig_T1 = bmp280ReadShort(0x88);//dig_T1
//  dig_T2 = bmp280ReadShort(0x8A);//dig_T2
//  dig_T3 = bmp280ReadShort(0x8C);//dig_T3
//  dig_P1 = bmp280ReadShort(0x8E);//dig_P1
//  dig_P2 = bmp280ReadShort(0x90);//dig_P2
//  dig_P3 = bmp280ReadShort(0x92);//dig_P3
//  dig_P4 = bmp280ReadShort(0x94);//dig_P4
//  dig_P5 = bmp280ReadShort(0x96);//dig_P5
//  dig_P6 = bmp280ReadShort(0x98);//dig_P6
//  dig_P7 = bmp280ReadShort(0x9A);//dig_P7
//  dig_P8 = bmp280ReadShort(0x9C);//dig_P8
//  dig_P9 = bmp280ReadShort(0x9E);//dig_P9
//
//  bmp280WriteByte(BMP280_REG_CONFIG, BMP280_CONFIG);
//  bmp280WriteByte(BMP280_REG_CONTROL, BMP280_MEAS);
//}
