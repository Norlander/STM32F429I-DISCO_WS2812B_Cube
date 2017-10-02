#include "testframework.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "main.h"
/*
 #define TEST_CASE_WITH_ALL(testname, setup, teardown, timeout_ms) void __ats__case_##testname(void);
 #define TEST_CASE(testname) TEST_CASE_WITH_ALL(testname, 0, 0, 0)
 #define ASSERT(test_expression) if (!(test_expression)) { __ats__framework_test_fail(0); }
 #define ASSERT_MSG(fail_message, test_expression) if (!(test_expression)) { __ats__framework_test_fail(fail_message); }
*/

volatile HAL_StatusTypeDef SPI_Status;
extern SPI_HandleTypeDef hspi3;
extern uint8_t SPIBufRed[9];
extern uint8_t SPIBufClear[9];

void setup_ws2812b(void)
{
	HAL_StatusTypeDef HalStatus;
	HalStatus = HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_SPI3_Init();
	MX_ADC1_Init();

	int i;
	for (i = 0; i < 9; ++i) {
		SPI_Status = HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
		//ASSERT(SPI_Status == HAL_OK);
	}
}


TEST_CASE_WITH_ALL(BLINK_LED1_RED, setup_ws2812b, 0, 1000)
{
	ASSERT(1);
	SPI_Status = HAL_SPI_Transmit(&hspi3, SPIBufRed, sizeof(SPIBufRed), 10);
	SPI_Status = HAL_SPI_Transmit(&hspi3, SPIBufRed, sizeof(SPIBufRed), 10);
	SPI_Status = HAL_SPI_Transmit(&hspi3, SPIBufRed, sizeof(SPIBufRed), 10);
	SPI_Status = HAL_SPI_Transmit(&hspi3, SPIBufRed, sizeof(SPIBufRed), 10);
	SPI_Status = HAL_SPI_Transmit(&hspi3, SPIBufRed, sizeof(SPIBufRed), 10);
	SPI_Status = HAL_SPI_Transmit(&hspi3, SPIBufRed, sizeof(SPIBufRed), 10);
	SPI_Status = HAL_SPI_Transmit(&hspi3, SPIBufRed, sizeof(SPIBufRed), 10);
	ASSERT(SPI_Status == HAL_OK);

}
