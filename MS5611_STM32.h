/*
 * MS5611_STM32.h
 *
 *  Created on: 30 Mar 2023
 *      Author: burak
 */

#ifndef INC_MS5611_STM32_H_
#define INC_MS5611_STM32_H_

#include <stdint.h> //standart tipleri kullanmak için (uint16_t gibi)
#include "stm32f1xx_hal.h"


#define MS5611_I2C 								&hi2c1

#define MS5611_ADDR_CSB_HIGH				    0x76
#define MS5611_ADDR_CSB_LOW   					0x77 // device address olarak bunu kullanıyoz


#define MS5611_CONV_D1							0x48


#define MS5611_RESET_REG 						0x1E

#define MS5611_PROM_READ_C1						0xA2
#define MS5611_PROM_READ_C2						0xA4
#define MS5611_PROM_READ_C3						0xA6
#define MS5611_PROM_READ_C4						0xA8
#define MS5611_PROM_READ_C5						0xAA
#define MS5611_PROM_READ_C6						0xAC

#define MS5611_READ_ADC							0x00

#define MS5611_PRESSURE_OSR_256  				0x40
#define MS5611_PRESSURE_OSR_512  				0x42
#define MS5611_PRESSURE_OSR_1024 				0x44
#define MS5611_PRESSURE_OSR_2048 				0x46
#define MS5611_PRESSURE_OSR_4096 				0x48

#define MS5611_TEMP_OSR_256      				0x50
#define MS5611_TEMP_OSR_512  	  				0x52
#define MS5611_TEMP_OSR_1024 	  				0x54
#define MS5611_TEMP_OSR_2048     				0x56
#define MS5611_TEMP_OSR_4096     				0x58


/* Function Prototypes */

void MS5611_Init(void);
float MS5611_GetTemperature(int osr);
float MS5611_GetPressure(int osr);
int32_t MS5611_Oku(int osr);


#endif /* INC_MS5611_STM32_H_ */
