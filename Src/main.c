/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery.h"
#include "led_functions.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;
TIM_HandleTypeDef htim3;
TIM_OC_InitTypeDef hoctim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */



/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void SetAllLedsGreenMain()
{
//	int i;
//	for (i = 0; i < NR_OF_PIXELS_IN_LED_STRIP; ++i) {
//		HAL_SPI_Transmit(hspi, pData, sizeof(pData), 10);
//	}
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
}


volatile uint32_t clocks[7];
void fillSPIBuf(void);

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */
	uint8_t LEDState = OffState;
	//HAL_StatusTypeDef HAL_Status = HAL_OK;
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	clocks[0] = SystemCoreClock;
	clocks[1] = HAL_RCC_GetSysClockFreq();
	clocks[2] = HAL_RCC_GetHCLKFreq();
	clocks[3] = HAL_RCC_GetPCLK1Freq();
	clocks[4] = HAL_RCC_GetPCLK2Freq();
	clocks[5] = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_TIM);


	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI3_Init();
	MX_ADC1_Init();



	/* USER CODE BEGIN 2 */
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
	BSP_LED_Init(LED3);
	MX_TIM3_Init();
	/* SPI3 running at 2.5MHz. This gives means a bit will be shifted out on MOSI-line every 0,4us.
	 * A WS2812B ONE is  0,8us HI + 0,4us LO
	 * A WS2812B ZERO is  0,4us HI + 0,8us LO
	 * In other words to send 255 to the 8-bit GREEN-LED use 8 * 3 = 24-bits:
	 * 	0b11011011	-->
	 * 	0b01101101	-->
	 * 	0b10110110	-->
	 * This means that to send ONE  2 HI + 1 LO bit must be sent. 0b110
	 * This means that to send ZERO 1 HI + 2 LO bit must be sent. 0b100
	 * A LED-Pixel contains RGB LEDs. Each LED has a resolution of "8-bits", [0 - 255].
	 * To control one LED in the RGB-pixel 8 bits are need constituted of 3 ONE/ZERO bits --> 24bits
	 * To control all colors (R-G-B) in the pixel 3 * 24bits = 72bits are needed.
	 * uint8_t *SPIBuf[9] declared as global variable...
	 */
#define DELAY 100
	//volatile HAL_StatusTypeDef SPI_Status;

#ifdef SPI_VIA_DMA
	HAL_Status = HAL_SPI_Transmit_DMA(&hspi3, SPIBufClear, sizeof(SPIBufClear));
	HAL_Status = HAL_SPI_Transmit_DMA(&hspi3, SPIBufClear, sizeof(SPIBufClear));
	HAL_Status = HAL_SPI_Transmit_DMA(&hspi3, SPIBufClear, sizeof(SPIBufClear));
	HAL_Status = HAL_SPI_Transmit_DMA(&hspi3, SPIBufClear, sizeof(SPIBufClear));
	HAL_Status = HAL_SPI_Transmit_DMA(&hspi3, SPIBufClear, sizeof(SPIBufClear));
	HAL_Status = HAL_SPI_Transmit_DMA(&hspi3, SPIBufClear, sizeof(SPIBufClear));
	HAL_Status = HAL_SPI_Transmit_DMA(&hspi3, SPIBufClear, sizeof(SPIBufClear));
	HAL_Status = HAL_SPI_Transmit_DMA(&hspi3, SPIBufClear, sizeof(SPIBufClear));
	HAL_Status = HAL_SPI_Transmit_DMA(&hspi3, SPIBufClear, sizeof(SPIBufClear));
	HAL_Status = HAL_SPI_Transmit_DMA(&hspi3, SPIBufClear, sizeof(SPIBufClear));
	HAL_Delay(DELAY);
	HAL_SPI_Transmit_DMA(&hspi3, SPIBufRed, sizeof(SPIBufRed));
	HAL_SPI_Transmit_DMA(&hspi3, SPIBufRed, sizeof(SPIBufRed));
	HAL_SPI_Transmit_DMA(&hspi3, SPIBufRed, sizeof(SPIBufRed));
	HAL_SPI_Transmit_DMA(&hspi3, SPIBufRed, sizeof(SPIBufRed));
	HAL_SPI_Transmit_DMA(&hspi3, SPIBufRed, sizeof(SPIBufRed));
	HAL_SPI_Transmit_DMA(&hspi3, SPIBufGreen, sizeof(SPIBufGreen));
	HAL_SPI_Transmit_DMA(&hspi3, SPIBufGreen, sizeof(SPIBufGreen));
	HAL_SPI_Transmit_DMA(&hspi3, SPIBufGreen, sizeof(SPIBufGreen));
	HAL_SPI_Transmit_DMA(&hspi3, SPIBufGreen, sizeof(SPIBufGreen));
	HAL_SPI_Transmit_DMA(&hspi3, SPIBufGreen, sizeof(SPIBufGreen));
#else

	SetAllLedsGreenMain();

	HAL_Delay(DELAY);

	SetAllLeds10callsRed();

	HAL_Delay(DELAY);

	SetAllLedsGreen();

	HAL_Delay(DELAY);

	ClearAllLeds();

	HAL_Delay(DELAY);

	SetAllLedsColor(SPIBufGreen);

	HAL_Delay(DELAY);

	SetAllLedsColor(SPIBufRed);

	HAL_Delay(DELAY);

	SetAllLedsColor(SPIBufBlue);

	HAL_Delay(DELAY);

	SetAllLedsColor(SPIBufClear);

	HAL_Delay(DELAY);

	ClearAllLeds();

	HAL_Delay(DELAY);

	LEDSiren();

	HAL_Delay(DELAY);

	ClearAllLeds();

	HAL_Delay(DELAY);

	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);

	HAL_Delay(DELAY);

	HAL_SPI_Transmit(&hspi3, SPIBufRed, sizeof(SPIBufRed), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufRed, sizeof(SPIBufRed), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufRed, sizeof(SPIBufRed), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufRed, sizeof(SPIBufRed), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufRed, sizeof(SPIBufRed), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);

	HAL_Delay(DELAY);

	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);

	HAL_Delay(DELAY);

	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufGreen, sizeof(SPIBufGreen), 10);

	HAL_Delay(DELAY);

	HAL_SPI_Transmit(&hspi3, SPIBufBlue, sizeof(SPIBufBlue), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufBlue, sizeof(SPIBufBlue), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufBlue, sizeof(SPIBufBlue), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufBlue, sizeof(SPIBufBlue), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufBlue, sizeof(SPIBufBlue), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufBlue, sizeof(SPIBufBlue), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufBlue, sizeof(SPIBufBlue), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufBlue, sizeof(SPIBufBlue), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufBlue, sizeof(SPIBufBlue), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufBlue, sizeof(SPIBufBlue), 10);

	HAL_Delay(DELAY);

	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);
	HAL_SPI_Transmit(&hspi3, SPIBufClear, sizeof(SPIBufClear), 10);

#endif

	//	HAL_Delay(DELAY);
	//
	//	HAL_SPI_Transmit(&hspi3, &SPIBufReset, 1, 10);

	HAL_Delay(DELAY);
	//HAL_SPI_Transmit(&hspi3, SPIBufRed, sizeof(SPIBufRed), 10);
	// Ger hard fault...
	SetAllLedsGreenMain();


	while (1)
	{

		while( 0 == BSP_PB_GetState(BUTTON_KEY) )
		{
			HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);
		}

		HAL_Delay(100);

		if (LEDState < NROFSTATES) {
			LEDState++;

		}
		else
		{
			LEDState = 0;
		}

		switch (LEDState) {
		case RedState:
			SetAllLedsGreenMain(&hspi3, SPIBufGreen);
			break;
		default:
			break;
		}
	}
}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 160;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/**
 * Enable DMA controller clock
 */
void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	}
}

void MX_TIM3_Init(void)
{
	__HAL_RCC_TIM3_CLK_ENABLE();

	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);


	htim3.Instance = TIM3;

	/* TIM3 is attached to the TIMER APB1 bus/clock --> 80MHz */
	/* TIM3 is counting at 20MHz when using 80/4... */
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	/* Alarm sounds should be running at ~3kHz
	 * During each Hz a full period must be completed. That is the GPIO port must toggle on off.
	 * To achieve this the timer should run twice the speed of 3kHz --> 6 kHz.
	 * 20MHz / 6kHz --> D05 (+-1?) */
	htim3.Init.Period = 0xa;
	htim3.Init.Prescaler = 0xD05;/*? ...No need to divide the prescaler, leave it at default! */
	/* TIM3_InitStruct.RepetitionCounter = ? ...No need for a repetition counter event - yet! */

	  if(HAL_TIM_Base_Init(&htim3) != HAL_OK)
	  {
	    /* Initialization Error */
	    Error_Handler();
	  }

	  if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
	  {
	    /* Starting Error */
	    Error_Handler();
	  }

	  clocks[6] = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_TIM);
//	hoctim3.OCMode = TIM_OCMODE_TOGGLE;
//	hoctim3.OCFastMode = TIM_OCFAST_DISABLE;
//	hoctim3.OCPolarity = TIM_OCPOLARITY_HIGH;
//	hoctim3.Pulse = 0xD04;
//	HAL_TIM_OC_Init(&hoctim3);
}


void HAL_SYSTICK_Callback(void)
{
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
}

/* SPI3 init function */
void MX_SPI3_Init(void)
{
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
     PF0   ------> FMC_A0
     PF1   ------> FMC_A1
     PF2   ------> FMC_A2
     PF3   ------> FMC_A3
     PF4   ------> FMC_A4
     PF5   ------> FMC_A5
     PF7   ------> SPI5_SCK
     PF8   ------> SPI5_MISO
     PF9   ------> SPI5_MOSI
     PF10   ------> LTDC_DE
     PC0   ------> FMC_SDNWE
     PA3   ------> LTDC_B5
     PA4   ------> LTDC_VSYNC
     PA6   ------> LTDC_G2
     PB0   ------> LTDC_R3
     PB1   ------> LTDC_R6
     PF11   ------> FMC_SDNRAS
     PF12   ------> FMC_A6
     PF13   ------> FMC_A7
     PF14   ------> FMC_A8
     PF15   ------> FMC_A9
     PG0   ------> FMC_A10
     PG1   ------> FMC_A11
     PE7   ------> FMC_D4_DA4
     PE8   ------> FMC_D5_DA5
     PE9   ------> FMC_D6_DA6
     PE10   ------> FMC_D7_DA7
     PE11   ------> FMC_D8_DA8
     PE12   ------> FMC_D9_DA9
     PE13   ------> FMC_D10_DA10
     PE14   ------> FMC_D11_DA11
     PE15   ------> FMC_D12_DA12
     PB10   ------> LTDC_G4
     PB11   ------> LTDC_G5
     PB12   ------> USB_OTG_HS_ID
     PB13   ------> USB_OTG_HS_VBUS
     PB14   ------> USB_OTG_HS_DM
     PB15   ------> USB_OTG_HS_DP
     PD8   ------> FMC_D13_DA13
     PD9   ------> FMC_D14_DA14
     PD10   ------> FMC_D15_DA15
     PD14   ------> FMC_D0_DA0
     PD15   ------> FMC_D1_DA1
     PG4   ------> FMC_A14_BA0
     PG5   ------> FMC_A15_BA1
     PG6   ------> LTDC_R7
     PG7   ------> LTDC_CLK
     PG8   ------> FMC_SDCLK
     PC6   ------> LTDC_HSYNC
     PC7   ------> LTDC_G6
     PC9   ------> I2C3_SDA
     PA8   ------> I2C3_SCL
     PA11   ------> LTDC_R4
     PA12   ------> LTDC_R5
     PC10   ------> LTDC_R2
     PD0   ------> FMC_D2_DA2
     PD1   ------> FMC_D3_DA3
     PD3   ------> LTDC_G7
     PD6   ------> LTDC_B2
     PG10   ------> LTDC_G3
     PG11   ------> LTDC_B3
     PG12   ------> LTDC_B4
     PG15   ------> FMC_SDNCAS
     PB5   ------> FMC_SDCKE1
     PB6   ------> FMC_SDNE1
     PB8   ------> LTDC_B6
     PB9   ------> LTDC_B7
     PE0   ------> FMC_NBL0
     PE1   ------> FMC_NBL1
 */
void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : A0_Pin A1_Pin A2_Pin A3_Pin
                           A4_Pin A5_Pin SDNRAS_Pin A6_Pin 
                           A7_Pin A8_Pin A9_Pin */
	GPIO_InitStruct.Pin = A0_Pin|A1_Pin|A2_Pin|A3_Pin
			|A4_Pin|A5_Pin|SDNRAS_Pin|A6_Pin
			|A7_Pin|A8_Pin|A9_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI5_SCK_Pin SPI5_MISO_Pin SPI5_MOSI_Pin */
	GPIO_InitStruct.Pin = SPI5_SCK_Pin|SPI5_MISO_Pin|SPI5_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : ENABLE_Pin */
	GPIO_InitStruct.Pin = ENABLE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
	HAL_GPIO_Init(ENABLE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SDNWE_Pin */
	GPIO_InitStruct.Pin = SDNWE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	HAL_GPIO_Init(SDNWE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
	GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : B1_Pin MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
	GPIO_InitStruct.Pin = B1_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : B5_Pin VSYNC_Pin G2_Pin R4_Pin
                           R5_Pin */
	GPIO_InitStruct.Pin = B5_Pin|VSYNC_Pin|G2_Pin|R4_Pin
			|R5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ACP_RST_Pin */
	GPIO_InitStruct.Pin = ACP_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OC_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : R3_Pin R6_Pin */
	GPIO_InitStruct.Pin = R3_Pin|R6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : A10_Pin A11_Pin BA0_Pin BA1_Pin
                           SDCLK_Pin SDNCAS_Pin */
	GPIO_InitStruct.Pin = A10_Pin|A11_Pin|BA0_Pin|BA1_Pin
			|SDCLK_Pin|SDNCAS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin
                           D8_Pin D9_Pin D10_Pin D11_Pin 
                           D12_Pin NBL0_Pin NBL1_Pin */
	GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin
			|D8_Pin|D9_Pin|D10_Pin|D11_Pin
			|D12_Pin|NBL0_Pin|NBL1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : G4_Pin G5_Pin B6_Pin B7_Pin */
	GPIO_InitStruct.Pin = G4_Pin|G5_Pin|B6_Pin|B7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
	GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : VBUS_FS_Pin */
	GPIO_InitStruct.Pin = VBUS_FS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : D13_Pin D14_Pin D15_Pin D0_Pin
                           D1_Pin D2_Pin D3_Pin */
	GPIO_InitStruct.Pin = D13_Pin|D14_Pin|D15_Pin|D0_Pin
			|D1_Pin|D2_Pin|D3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : TE_Pin */
	GPIO_InitStruct.Pin = TE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
	GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : R7_Pin DOTCLK_Pin B3_Pin */
	GPIO_InitStruct.Pin = R7_Pin|DOTCLK_Pin|B3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : HSYNC_Pin G6_Pin R2_Pin */
	GPIO_InitStruct.Pin = HSYNC_Pin|G6_Pin|R2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : I2C3_SDA_Pin */
	GPIO_InitStruct.Pin = I2C3_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
	HAL_GPIO_Init(I2C3_SDA_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : I2C3_SCL_Pin */
	GPIO_InitStruct.Pin = I2C3_SCL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
	HAL_GPIO_Init(I2C3_SCL_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : G7_Pin B2_Pin */
	GPIO_InitStruct.Pin = G7_Pin|B2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : G3_Pin B4_Pin */
	GPIO_InitStruct.Pin = G3_Pin|B4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : LD3_Pin LD4_Pin */
	GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : SDCKE1_Pin SDNE1_Pin */
	GPIO_InitStruct.Pin = SDCKE1_Pin|SDNE1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	//GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
