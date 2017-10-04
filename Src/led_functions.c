/*
 * led_functions.c
 *
 *  Created on: 2 okt. 2017
 *      Author: ASW-MNO-Admin
 */


#include <stdint.h>
#include "stm32f4xx_hal.h"
//#include "led_conf.h"

/* Configure the number of Pixels used in your LED-strip */
#define NR_OF_PIXELS_IN_LED_STRIP 10

#define SPI_TIMEOUT
/*  */







void SetAllLeds(SPI_HandleTypeDef *hspi, uint8_t *pData)
{
//	int i;
//	for (i = 0; i < NR_OF_PIXELS_IN_LED_STRIP; ++i) {
//		HAL_SPI_Transmit(hspi, pData, sizeof(pData), 10);
//	}
	HAL_SPI_Transmit(hspi, pData, sizeof(pData), 10);
	HAL_SPI_Transmit(hspi, pData, sizeof(pData), 10);
	HAL_SPI_Transmit(hspi, pData, sizeof(pData), 10);
	HAL_SPI_Transmit(hspi, pData, sizeof(pData), 10);
	HAL_SPI_Transmit(hspi, pData, sizeof(pData), 10);
	HAL_SPI_Transmit(hspi, pData, sizeof(pData), 10);
	HAL_SPI_Transmit(hspi, pData, sizeof(pData), 10);
	HAL_SPI_Transmit(hspi, pData, sizeof(pData), 10);
	HAL_SPI_Transmit(hspi, pData, sizeof(pData), 10);
	HAL_SPI_Transmit(hspi, pData, sizeof(pData), 10);
}

