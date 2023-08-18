/*
 * MS5611_STM32.c
 *
 *  Created on: 30 Mar 2023
 *      Author: burak
 */


/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  BME280_STM32.c
  Author:     ControllersTech.com
  Updated:    Dec 14, 2021

  ***************************************************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ***************************************************************************************************************
*/

#include "MS5611_STM32.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c1;
Ms5611_t ms5611_t = {0};

#define SUPPORT_64BIT 0
#define SUPPORT_32BIT 1

#define I2C_TIMEOUT ( 100 )
#define I2C_NUMBER_OF_TRIALS ( 4 )
#define I2C_REG_ADD_SIZE_1_BYTE ( 1 )
#define I2C_DATA_SIZE_1_BYTE  ( 1 )

float Temperature_MS5611, Pressure_MS5611, Altitude_MS5611;


////////////////////////////////////////////

struct calibrationParameters
{

	uint16_t C1;	// Pressure sensitivity | SENS(T1)
	uint16_t C2;	// Pressure offset | OFF(T1)
	uint16_t C3;	// Temperature coefficient of pressure sensitivity | TCS
	uint16_t C4;	// Temperature coefficient of pressure offset | TCO
	uint16_t C5;	// Reference temperature | T(REF)
	uint16_t C6;	// Temperature coefficient of the temperature | TEMPSENS

}CalibrationParameters;

struct digitalValues
{

	uint32_t D1;	// Digital pressure value
	uint32_t D2;	// Digital temperature value

}DigitalValues;

struct calculationParameters
{
	int32_t dT;		// Difference between actual and reference temperature
	int32_t TEMP;	// Actual temperature

	int64_t OFF;	// Offset at actual temperature
	int64_t SENS;	// Sensitivity at actual temperature
	int32_t P;		// Temperature compensated pressure

}CalculationParameters;

///////////////////////////////////////////////////////////////////////////////



static void MS5611_Reset(void)
{
	uint8_t I2C_COMMAND[1] = {0};
	I2C_COMMAND[0] = MS5611_RESET_REG;

	HAL_I2C_Master_Transmit(MS5611_I2C, (MS5611_ADDR_CSB_LOW<<1), I2C_COMMAND, sizeof(I2C_COMMAND), HAL_MAX_DELAY);
}


static void MS5611_ReadCalibrationData(void)
{
	uint8_t rx_Buffer[2] = {0};

	uint8_t I2C_COMMAND[6] = {0};

	I2C_COMMAND[0] = MS5611_PROM_READ_C1;
	I2C_COMMAND[1] = MS5611_PROM_READ_C2;
	I2C_COMMAND[2] = MS5611_PROM_READ_C3;
	I2C_COMMAND[3] = MS5611_PROM_READ_C4;
	I2C_COMMAND[4] = MS5611_PROM_READ_C5;
	I2C_COMMAND[5] = MS5611_PROM_READ_C6;

	HAL_I2C_Master_Transmit(MS5611_I2C, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[0], sizeof(I2C_COMMAND[0]), HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(MS5611_I2C, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
	CalibrationParameters.C1 = ((rx_Buffer[0] << 8) | (rx_Buffer[1]));

	HAL_I2C_Master_Transmit(MS5611_I2C, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[1], sizeof(I2C_COMMAND[1]), HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(MS5611_I2C, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
	CalibrationParameters.C2 = ((rx_Buffer[0] << 8) | (rx_Buffer[1]));

	HAL_I2C_Master_Transmit(MS5611_I2C, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[2], sizeof(I2C_COMMAND[2]), HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(MS5611_I2C, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
	CalibrationParameters.C3 = ((rx_Buffer[0] << 8) | (rx_Buffer[1]));

	HAL_I2C_Master_Transmit(MS5611_I2C, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[3], sizeof(I2C_COMMAND[3]), HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(MS5611_I2C, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
	CalibrationParameters.C4 = ((rx_Buffer[0] << 8) | (rx_Buffer[1]));

	HAL_I2C_Master_Transmit(MS5611_I2C, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[4], sizeof(I2C_COMMAND[4]), HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(MS5611_I2C, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
	CalibrationParameters.C5 = ((rx_Buffer[0] << 8) | (rx_Buffer[1]));

	HAL_I2C_Master_Transmit(MS5611_I2C, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[5], sizeof(I2C_COMMAND[5]), HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(MS5611_I2C, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
	CalibrationParameters.C6 = ((rx_Buffer[0] << 8) | (rx_Buffer[1]));
}




static void MS5611_ReadDigitalValues(int osr)
{
	uint8_t rx_Buffer[3] = {0};  // dizinin ismi aynı zamanda adresi anlamına da gelir. O yüzden receive'de & kullanmadık

	uint8_t I2C_COMMAND[11] = {0};

	I2C_COMMAND[0] = MS5611_PRESSURE_OSR_256;
	I2C_COMMAND[1] = MS5611_PRESSURE_OSR_512;
	I2C_COMMAND[2] = MS5611_PRESSURE_OSR_1024;
	I2C_COMMAND[3] = MS5611_PRESSURE_OSR_2048;
	I2C_COMMAND[4] = MS5611_PRESSURE_OSR_4096;
	I2C_COMMAND[5] = MS5611_TEMP_OSR_256;
	I2C_COMMAND[6] = MS5611_TEMP_OSR_512;
	I2C_COMMAND[7] = MS5611_TEMP_OSR_1024;
	I2C_COMMAND[8] = MS5611_TEMP_OSR_2048;
	I2C_COMMAND[9] = MS5611_TEMP_OSR_4096;
	I2C_COMMAND[10] = MS5611_READ_ADC;

	switch(osr)
	{
	case 256:

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[0], sizeof(I2C_COMMAND[0]), HAL_MAX_DELAY);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[10], sizeof(I2C_COMMAND[10]), HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
		DigitalValues.D1 = ((rx_Buffer[0] << 16) | (rx_Buffer[1] << 8) | (rx_Buffer[2]));

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[5], sizeof(I2C_COMMAND[5]), HAL_MAX_DELAY);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[10], sizeof(I2C_COMMAND[10]), HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
		DigitalValues.D2 = ((rx_Buffer[0] << 16) | (rx_Buffer[1] << 8) | (rx_Buffer[2]));
		break;

	case 512:

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[1], sizeof(I2C_COMMAND[1]), HAL_MAX_DELAY);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[10], sizeof(I2C_COMMAND[10]), HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
		DigitalValues.D1 = ((rx_Buffer[0] << 16) | (rx_Buffer[1] << 8) | (rx_Buffer[2]));

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[6], sizeof(I2C_COMMAND[6]), HAL_MAX_DELAY);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[10], sizeof(I2C_COMMAND[10]), HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
		DigitalValues.D2 = ((rx_Buffer[0] << 16) | (rx_Buffer[1] << 8) | (rx_Buffer[2]));
		break;

	case 1024:

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[2], sizeof(I2C_COMMAND[2]), HAL_MAX_DELAY);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[10], sizeof(I2C_COMMAND[10]), HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
		DigitalValues.D1 = ((rx_Buffer[0] << 16) | (rx_Buffer[1] << 8) | (rx_Buffer[2]));

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[7], sizeof(I2C_COMMAND[7]), HAL_MAX_DELAY);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[10], sizeof(I2C_COMMAND[10]), HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
		DigitalValues.D2 = ((rx_Buffer[0] << 16) | (rx_Buffer[1] << 8) | (rx_Buffer[2]));
		break;

	case 2048:

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[3], sizeof(I2C_COMMAND[3]), HAL_MAX_DELAY);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[10], sizeof(I2C_COMMAND[10]), HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
		DigitalValues.D1 = ((rx_Buffer[0] << 16) | (rx_Buffer[1] << 8) | (rx_Buffer[2]));

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[8], sizeof(I2C_COMMAND[8]), HAL_MAX_DELAY);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[10], sizeof(I2C_COMMAND[10]), HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
		DigitalValues.D2 = ((rx_Buffer[0] << 16) | (rx_Buffer[1] << 8) | (rx_Buffer[2]));
		break;

	case 4096:

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[4], sizeof(I2C_COMMAND[4]), HAL_MAX_DELAY);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[10], sizeof(I2C_COMMAND[10]), HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
		DigitalValues.D1 = ((rx_Buffer[0] << 16) | (rx_Buffer[1] << 8) | (rx_Buffer[2]));

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[9], sizeof(I2C_COMMAND[9]), HAL_MAX_DELAY);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[10], sizeof(I2C_COMMAND[10]), HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
		DigitalValues.D2 = ((rx_Buffer[0] << 16) | (rx_Buffer[1] << 8) | (rx_Buffer[2]));
		break;

	default:

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[2], sizeof(I2C_COMMAND[2]), HAL_MAX_DELAY);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[10], sizeof(I2C_COMMAND[10]), HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
		DigitalValues.D1 = ((rx_Buffer[0] << 16) | (rx_Buffer[1] << 8) | (rx_Buffer[2]));

		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[7], sizeof(I2C_COMMAND[7]), HAL_MAX_DELAY);
		HAL_Delay(50);
		HAL_I2C_Master_Transmit(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), &I2C_COMMAND[10], sizeof(I2C_COMMAND[10]), HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (MS5611_ADDR_CSB_LOW<<1), rx_Buffer, sizeof(rx_Buffer), HAL_MAX_DELAY);
		DigitalValues.D2 = ((rx_Buffer[0] << 16) | (rx_Buffer[1] << 8) | (rx_Buffer[2]));
		break;
	}
}


static void MS5611_DoCalculations(void)
{
	/* For dT, OFF, and SENS max and min values should be defined */

	/* dT */
	CalculationParameters.dT = DigitalValues.D2 - (CalibrationParameters.C5 * pow(2,8));

	if(CalculationParameters.dT < -16776960)
		CalculationParameters.dT = -16776960;

	if(CalculationParameters.dT > 16777216)
		CalculationParameters.dT = 16777216;

	/* TEMP */
	CalculationParameters.TEMP = 2000 + (CalculationParameters.dT * CalibrationParameters.C6 /pow(2,26));

	/* OFF */
	CalculationParameters.OFF = (CalibrationParameters.C2 * pow(2,16)) +
			((CalibrationParameters.C4 * CalculationParameters.dT) / pow(2,7));

	if(CalculationParameters.OFF < -8589672450)
		CalculationParameters.OFF = -8589672450;

	if(CalculationParameters.OFF > 12884705280)
		CalculationParameters.OFF = 12884705280;

	/* SENS */
	CalculationParameters.SENS = (CalibrationParameters.C1 * pow(2,15)) +
			((CalibrationParameters.C3 * CalculationParameters.dT) / pow(2,8));

	if(CalculationParameters.SENS < -4294836225)
		CalculationParameters.SENS = -4294836225;

	if(CalculationParameters.SENS > 6442352640)
		CalculationParameters.SENS = 6442352640;

	/* P */
	CalculationParameters.P = ((DigitalValues.D1 * CalculationParameters.SENS / pow(2,21) - CalculationParameters.OFF)/ pow(2,15));
}


float MS5611_GetTemperature(int osr)
{

	MS5611_ReadDigitalValues(osr);
	MS5611_DoCalculations();

	Temperature_MS5611 = CalculationParameters.TEMP / 100.00;

	return Temperature_MS5611;

}


float MS5611_GetPressure(int osr)
{

	MS5611_ReadDigitalValues(osr);
	MS5611_DoCalculations();

	Pressure_MS5611 = CalculationParameters.P / 100.00;
	Altitude_MS5611 = (1 - pow(Pressure_MS5611 / 1013.25, 0.190284)) * 44307.7;

	return Altitude_MS5611;


}

void MS5611_Init(void)
{
	MS5611_Reset();
	HAL_Delay(50);
	MS5611_ReadCalibrationData();
}


