/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stm32l4xx_nucleo.h"
#include "hd44780.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef enum {
	SAMPLING_OFF,
	SAMPLING_ON
}SAMPLING_STATE;

#define TXBUFFERSIZE		128
#define RXBUFFERSIZE        128

#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))



/*
 * I2C LCD display parameters
 */
#define 		LCD1_I2C_ADDRESS		0x3F


/*
 * UART 2 Settings
 */
#define 		TXBUFFERSIZE			128
#define 		RXBUFFERSIZE        	128
__IO ITStatus UartReady = RESET;
uint8_t 		aTxBuffer[TXBUFFERSIZE]; /* Buffer used for transmission */
uint8_t 		aRxBuffer[RXBUFFERSIZE]; /* Buffer used for reception */


/*
 * SD settings
 */
uint32_t 		stored;
uint8_t 		filename[20];
uint8_t 		write_buffer[200];
uint8_t 		read_buffer[200];
FRESULT 		res;	/* FatFs function common result code */
uint32_t 		byteswritten, bytesread;
uint8_t 		uSD_Inserted = 0;
/*
 *  ADC parameters
 */
#define 		VDD_APPLI				((uint32_t) 3300)    		/* Value of analog voltage supply Vdda (unit: mV) */
#define 		RANGE_12BITS			((uint32_t) 4095)    		/* Max value with a full range of 12 bits */
#define 		ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t)    7)  	/* Size of array containing ADC converted values: set to ADC sequencer number of ranks converted, to have a rank in each address */

/* Variable containing ADC conversions results */
__IO uint16_t   aADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];
uint16_t   		uhADCChannelToDAC_mVolt = 0;
uint16_t   		uhVrefInt_mVolt = 0;
uint8_t 		ADC_result;
__IO uint16_t 	uhADCxConvertedData = 0;

#define 		COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(ADC_DATA)    \
						( ((ADC_DATA) * VDD_APPLI) / RANGE_12BITS)

/* I2C display */
#define 		LCD1_I2C_ADDRESS		0x3F
LCD_PCF8574_HandleTypeDef	lcd;
#define 		LCD_CLEAR_DISPLAY 				1

/*
 * Misc variables
 */
/* counter for pushbutton function increment*/
uint8_t 		PB_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM5_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void callBack(DMA_HandleTypeDef * hdma);
void do_ADC_conversion(void);
int size(char *ptr);
void AlternateAdcStart(void);
void initLCD(void);
void errorHandleLCD(LCD_RESULT result);
void errorHandleI2C(PCF8574_RESULT result);
void writeTestLCD(void);
int fputc(int ch, FILE *f);
HAL_StatusTypeDef SendData(void);
void writeLCDmessage(char* message, uint8_t clear);
void adcSampling(SAMPLING_STATE sampling);

FRESULT User_SD_Init(void);
FRESULT User_SD_File_Read(char* filename, uint8_t* read_buffer);
FRESULT User_SD_File_Write(char* filename, uint8_t* write_buffer);
void errorHandleFRESULT(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SDMMC1_SD_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();
  MX_TIM5_Init();

  /* USER CODE BEGIN 2 */
	uint8_t len;
	HAL_StatusTypeDef HAL_Result;

/*
 * UART Message
 */
	memset(aTxBuffer, '\0',TXBUFFERSIZE-1);
	len = sprintf((char*)aTxBuffer,"%s","UART READY\n");
	HAL_Result = HAL_UART_Transmit(&huart2, aTxBuffer, len, 200);
	switch (HAL_Result) {
		case HAL_OK:
			break;
		case HAL_ERROR:
			_Error_Handler("UART Transmit Error", __LINE__);
			break;
		default:
			break;
	}

	/*
	* LCD start and test
	*/
	initLCD();
	LCD_ClearDisplay(&lcd);

/*
 * Setup SD
 */

#define USE_SD

#if  defined  (USE_SD)

	HAL_SD_MspInit(&hsd1);
	hsd1.State = HAL_SD_STATE_RESET; // force HAL_SD_MspInit(&hsd1) to be called
	uint8_t MSD_Result = BSP_SD_Init(); //calls HAL_SD_MspInit(&hsd1); and Hal_SD_Init();
	//change to enable the SD function (or not)
	switch (MSD_Result) {
		case MSD_OK:
			break;
		case MSD_ERROR:
			writeLCDmessage("MSD_ERROR", LCD_CLEAR_DISPLAY);
			_Error_Handler("MSD_ERROR", __LINE__);
			break;
		case MSD_ERROR_SD_NOT_PRESENT:
			writeLCDmessage("SD_NOT_PRESENT", LCD_CLEAR_DISPLAY);
			_Error_Handler("MSD_ERROR_SD_NOT_PRESENT", __LINE__);
			break;
		default:
			break;
	}

	len = sprintf((char*)aTxBuffer,"%s","Stuff for the card\n"); //SD_WRITE uses size to find /0 to get length to send

	if( User_SD_Init() !=  FR_OK ) _Error_Handler("SD Initialization Error", __LINE__);
	res = User_SD_File_Write("job.txt", aTxBuffer);
	errorHandleFRESULT();

#endif

	memset(aTxBuffer, '\0',TXBUFFERSIZE-1);
	len = sprintf((char*)aTxBuffer,"%s","****The SD card was initialized successfully****\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)aTxBuffer, len, 200);


	memset(aTxBuffer, '\0',TXBUFFERSIZE-1);
	len = sprintf((char*)aTxBuffer,"%s","****Project Init Success, entering endless loop ...****\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)aTxBuffer, len, 200);

	len = sprintf((char*)aTxBuffer,"%s","****Project Init Success, SNED by DMA ...****\n");
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)aTxBuffer, len);
	//HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, RXBUFFERSIZE);
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)aRxBuffer, RXBUFFERSIZE);
	//if(HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)



	/*this needs work*/
	//printf("\n\r UART Printf Example: retarget the C library printf function to the UART\n\r");


	/*
	* Start periphals
	*/
	/** TIM for base timer, timer for internal trigger */
	HAL_TIM_Base_Start_IT(&htim4);

	/**TIM for Output Compare, PinToggle */
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_4);

	/** TIM for base timer, timer for internal trigger */
	HAL_TIM_Base_Start_IT(&htim3);

	/**TIM for Output Compare, PinToggle */
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);

	/* Run the ADC calibration in single-ended mode */
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
		_Error_Handler("ADC Calibration Start error", __LINE__);

	adcSampling(SAMPLING_OFF);

	//HAL_ADC_Start_IT(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_SDMMC1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 10;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV8;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 7;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_CC4;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SDMMC1 init function */
static void MX_SDMMC1_SD_Init(void)
{

  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 39999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 3999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_detect_Pin */
  GPIO_InitStruct.Pin = uSD_detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(uSD_detect_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */


FRESULT User_SD_Init(void) //HAL_StatusTypeDef
{

		/*##-1- Link the micro SD disk I/O driver ##################################*/
		//res = FATFS_LinkDriver(&SD_Driver, SDPath);
		//if(res != FR_OK) return res;

		/*##-2- Register the file system object to the FatFs module ##############*/
		res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);



		return res;
}


FRESULT User_SD_File_Write(char *filename, uint8_t *write_buffer) //HAL_StatusTypeDef
{
//	FRESULT f_open (
//	FIL* fp,			/* Pointer to the blank file object */
//	const TCHAR* path,	/* Pointer to the file name */
//	BYTE mode			/* Access mode and file open mode flags */
//)

	res  = f_open(&SDFile, filename, FA_WRITE | FA_CREATE_ALWAYS);
	errorHandleFRESULT();

	/*##-5- Write data to the text file ################################*/
	res = f_write(&SDFile, write_buffer, (UINT)size((char*)write_buffer), (void *)&byteswritten);
	errorHandleFRESULT();

	/*##-6- Close the open text file #################################*/
	res = f_close(&SDFile);
	errorHandleFRESULT();

	return res;
}


FRESULT User_SD_File_Read(char *filename, uint8_t* read_buffer)
{
	res  = f_open(&SDFile,filename, FA_READ);
		if(res != FR_OK)
				{
					/* 'STM32.TXT' file Open for write Error */
					_Error_Handler(" File Open Error", __LINE__);
				}
		else
		{
				/*##-5- Write data to the text file ################################*/
				res = f_read(&SDFile, read_buffer, sizeof(*read_buffer), (void *)&bytesread);

				if((bytesread == 0) || (res != FR_OK))
				{
					/* 'STM32.TXT' file Write or EOF Error */
					_Error_Handler(" File Read Error", __LINE__);
				}
				else
				{
						/*##-6- Close the open text file #################################*/
						return res ;
						f_close(&SDFile);
				}
			}
		return res;
}

FRESULT open_append (
    FIL* fp,            /* [OUT] File object to create */
    const char* path    /* [IN]  File name to be opened */
)
{
    FRESULT fr;

    /* Opens an existing file. If not exist, creates a new file. */
    fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (fr == FR_OK) {
        /* Seek to end of the file to append data */
        fr = f_lseek(fp, f_size(fp));
        if (fr != FR_OK)
            f_close(fp);
    }
    return fr;
}

FRESULT User_SD_File_Append(char *filename, uint8_t *write_buffer)
{
 	res = open_append(&SDFile, filename);
	if (res != FR_OK) return 1;

	/* Append a line */
	//f_printf(&SDFile, "%02u/%02u/%u, %2u:%02u\n", 10, 12, 456, 22, 55);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); //RED LED
	f_puts((char*)write_buffer,&SDFile);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); //RED LED

	/* Close the file */
	res = f_close(&SDFile);

	return res;

}

/**
  * @brief  Conversion complete callback in non blocking mode.... DMA comes here too.
  * @param  AdcHandle : ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	uint8_t len;
	//uhADCxConvertedData = HAL_ADC_GetValue(hadc);

	memset(aTxBuffer, '\0',TXBUFFERSIZE-1);
	len = sprintf((char*)aTxBuffer,"%u %u %u\n",
			aADCxConvertedValues[1],aADCxConvertedValues[2],aADCxConvertedValues[3]);

	/*
	len = sprintf((char*)aTxBuffer,"%u %u %u %u %u %u\n",
			aADCxConvertedValues[1],aADCxConvertedValues[2],aADCxConvertedValues[3],
			aADCxConvertedValues[4],aADCxConvertedValues[5],aADCxConvertedValues[6]);
	*/


	//find last char in file
	res = f_lseek(&SDFile, f_size(&SDFile));
	//errorHandleFRESULT();

	//Write data to the text file
	//f_puts((char*)write_buffer,&SDFile);
	res = f_write(&SDFile, (char*)aTxBuffer, len, (void *)&byteswritten);
	//errorHandleFRESULT();


	HAL_UART_Transmit(&huart2, (uint8_t*)aTxBuffer, len, 200);


	/* do something with the data*/
   // uhVrefInt_mVolt = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(aADCxConvertedValues[2]);

    BSP_LED_Toggle(LED2);
}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	__HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);

  /* In case of ADC error, call main error handler */
	//_Error_Handler("HAL_ADC_ErrorCallback", __LINE__);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    BSP_LED_Toggle(LED2);  /*using nucleo file*/
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

//returns the size of a character array using a pointer to the first element of the character array
int size(char *ptr)
{
    //variable used to access the subsequent array elements.
    int offset = 0;
    //variable that counts the number of elements in your array
    int count = 0;

    //While loop that tests whether the end of the array has been reached
    while (*(ptr + offset) != '\0')
    {
        //increment the count variable
        ++count;
        //advance to the next element of the array
        ++offset;
    }
    //return the size of the array
    return count;
}


void do_ADC_conversion(void)
{
	/*
	 * this is playing code only.
	 * Enable sequencer in discontinuous mode, this will perform
	 * the conversion of the next rank in sequencer.
	 * Note: 	For this example, conversion is triggered by software start,
				therefore "HAL_ADC_Start()" must be called for each conversion.
				Since DMA transfer has been initiated previously by function
				"HAL_ADC_Start_DMA()", this function will keep DMA transfer active.
	 */
	if (HAL_ADC_Start(&hadc1) != HAL_OK)
	{
	  Error_Handler();
	}

    /* After each intermediate conversion,
       - EOS remains reset (EOS is set only every third conversion)
       - EOC is set then immediately reset by DMA reading of DR register.
     Therefore, the only reliable flag to check the conversion end is EOSMP
     (end of sampling flag).
     Once EOSMP is set, the end of conversion will be reached when the successive
     approximations are over.
     RM indicates 12.5 ADC clock cycles for the successive approximations
     duration with a 12-bit resolution, or 185.ns at 80 MHz.
     Therefore, it is known that the conversion is over at
     HAL_ADC_PollForConversion() API return */
    if (HAL_ADC_PollForEvent(&hadc1, ADC_EOSMP_EVENT, 10) != HAL_OK)
    {
      Error_Handler();
    }
}


HAL_StatusTypeDef SendData(void)
{
	HAL_StatusTypeDef result = HAL_OK;
	if(huart2.gState == HAL_UART_STATE_READY)
	{
		//at 115200, at 500hz this is about as much as can send between triggers
		memset(aTxBuffer, '\0',TXBUFFERSIZE);
		uint8_t len = sprintf((char*)aTxBuffer,"Other Text %i\n",454);

		//transmit using the HAL
		if (0) {
			result = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)aTxBuffer, len);
		}
		else
		{

			//try just using the registers.... with HAL macros
				__HAL_DMA_DISABLE(&hdma_usart2_tx);
			//Configure the total number of bytes to be transferred to the DMA control register.
				hdma_usart2_tx.Instance->CNDTR = len;
			//4. Configure the channel priority in the DMA register
				//hdma_usart2_tx.Instance->CCR = DMA_PRIORITY_LOW ; ///this is clearing lost of other flag
			//5. Configure DMA interrupt generation after half/ full transfer as required by the application.
				__HAL_DMA_ENABLE_IT(&hdma_usart2_tx, DMA_IT_TC|DMA_IT_HT|DMA_IT_TE);
			//6. Clear the TC flag in the USART_ISR register by setting the TCCF bit in the USART_ICR register.
				__HAL_UART_CLEAR_FLAG(&huart2,UART_CLEAR_TCF);
				/* Clear all flags */
				//hdma_usart2_tx.DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << hdma_usart2_tx.ChannelIndex);
				__HAL_DMA_CLEAR_FLAG(&hdma_usart2_tx, DMA_FLAG_HT7|DMA_FLAG_TE7|DMA_FLAG_GL7);
			//7. Activate the channel in the DMA register.
				// Activate the channel by setting the ENABLE bit in the DMA_CCRx register.
				__HAL_DMA_ENABLE(&hdma_usart2_tx);
				SET_BIT(huart2.Instance->CR3, USART_CR3_DMAT);
		}
	}
	return result;
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3 && htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
	{
	//For serial testing, send stuff
	//SendData();
	}

	if(htim->Instance==TIM4 && htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4)
	{
	//For serial testing, send stuff
	//SendData();
	}
}

void writeLCDmessage(char* message, uint8_t clear)
{
	if (clear)	LCD_ClearDisplay(&lcd);
	LCD_SetLocation(&lcd, 0, 0);
	LCD_WriteString(&lcd, message);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == B1_Pin)
	{
		if (PB_counter == 0){
			adcSampling(SAMPLING_ON);
			PB_counter ++;
		} else if(PB_counter == 1){
			adcSampling(SAMPLING_OFF);
			PB_counter = 0;
		}
	}

	if(GPIO_Pin == uSD_detect_Pin)
	{
		uSD_Inserted =! HAL_GPIO_ReadPin(uSD_detect_GPIO_Port, GPIO_Pin);
		//should re-run SD-Init and set ok to use SD
	}


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE))
	{
    	__HAL_UART_CLEAR_IDLEFLAG(huart);

    	//this is a read only field only
    	if(DMA1->ISR &  DMA_ISR_TCIF6)
    		{
    		 //ATTEMPT TO SET TC FLAG
    		}
    	//clear idle flag

    //IDLE flag set
	}
}

void callBack(DMA_HandleTypeDef * hdma)
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	//here
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	//stop DMA
	__HAL_DMA_DISABLE(&hdma_usart2_rx);
	HAL_UART_AbortReceive(&huart2);
	//reastart it
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)aRxBuffer, RXBUFFERSIZE);
	//not used
}







void initLCD(void)
{
	lcd.pcf8574.PCF_I2C_ADDRESS = LCD1_I2C_ADDRESS;
	lcd.pcf8574.PCF_I2C_TIMEOUT = 1000;
	lcd.pcf8574.i2c = hi2c1;
	//	lcd.pcf8574.i2c.Instance = I2C1;
		//lcd.pcf8574.i2c.Init.Timing= 400000;
		//this was code from example but ClockSpeed is not there
		//lcd.pcf8574.i2c.Init.ClockSpeed = 400000;
	lcd.NUMBER_OF_LINES = NUMBER_OF_LINES_2;
	lcd.errorCallback = errorHandleLCD;
	lcd.pcf8574.errorCallback = errorHandleI2C;
	//lcd.errorCallback = errorHandleLCD; //added this to see if can handle error
	lcd.type = TYPE0;
	lcd.backLight = 1;

	if(LCD_Init(&lcd)!=LCD_OK){ //LCD_Init set the pins as defined in hd44780.c!
		// error occured
		_Error_Handler(__FILE__, __LINE__);
	}
}


/**
  * @brief  Error Callback from LCD library, set in initLCD
  * @param  None
  * @note   Just goes to standard error handle
  * @retval None
  */
void errorHandleLCD(LCD_RESULT result)
{
	_Error_Handler(__FILE__, __LINE__);
}

/**
  * @brief  Error Callback from LCD library, set in initLCD
  * @param  None
  * @note   Just goes to standard error handle
  * @retval None
  */
void errorHandleI2C(PCF8574_RESULT result)
{
	_Error_Handler(__FILE__, __LINE__);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */

int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
 // HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

//write some stuff to the LCD
void writeTestLCD(void)
{
	LCD_SetLocation(&lcd, 0, 0);
	LCD_WriteString(&lcd, "pi:");
	LCD_SetLocation(&lcd, 0, 1);
	LCD_WriteString(&lcd, "e:");
	LCD_SetLocation(&lcd, 6, 0);
	LCD_WriteFloat(&lcd,3.1415926,7);
	LCD_SetLocation(&lcd, 6, 1);
	LCD_WriteFloat(&lcd,2.71,2);
}

void adcSampling(SAMPLING_STATE sampling)
{

	if(sampling)
	{
		res  = f_open(&SDFile, "data.txt" , FA_WRITE);
		res = f_lseek(&SDFile, f_size(&SDFile));
		uint16_t len =0;
		len = sprintf((char*)aTxBuffer,"%s","Start of Sample\n");

		res = f_write(&SDFile, (char*)aTxBuffer, len, (void *)&byteswritten);
		errorHandleFRESULT();
		/* Start ADC conversion on regular group with transfer by DMA */
		if (HAL_ADC_Start_DMA(&hadc1,
							(uint32_t *)aADCxConvertedValues,
							ADCCONVERTEDVALUES_BUFFER_SIZE
						   ) != HAL_OK)
		_Error_Handler("ADC conversion start error", __LINE__);
		writeLCDmessage("SAMPLING", LCD_CLEAR_DISPLAY);
	}
	else
	{
		/*##-6- Close the open text file #################################*/
		res = f_close(&SDFile);
		errorHandleFRESULT();

		HAL_ADC_Stop_DMA(&hadc1);
		writeLCDmessage("NOT_SAMPLING", LCD_CLEAR_DISPLAY);
	}
}

void AlternateAdcStart(void)
{
    /* Start ADC conversion */
    /* Since sequencer is enabled in discontinuous mode, this will perform    */
    /* the conversion of the next rank in sequencer.                          */
    /* Note: For this example, conversion is triggered by software start,     */
    /*       therefore "HAL_ADC_Start()" must be called for each conversion.  */
    /*       Since DMA transfer has been initiated previously by function     */
    /*       "HAL_ADC_Start_DMA()", this function will keep DMA transfer      */
    /*       active.                                                          */
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
      Error_Handler();
    }

    /* After each intermediate conversion,
       - EOS remains reset (EOS is set only every third conversion)
       - EOC is set then immediately reset by DMA reading of DR register.
     Therefore, the only reliable flag to check the conversion end is EOSMP
     (end of sampling flag).
     Once EOSMP is set, the end of conversion will be reached when the successive
     approximations are over.
     RM indicates 12.5 ADC clock cycles for the successive approximations
     duration with a 12-bit resolution, or 185.ns at 80 MHz.
     Therefore, it is known that the conversion is over at
     HAL_ADC_PollForConversion() API return */
    if (HAL_ADC_PollForEvent(&hadc1, ADC_EOSMP_EVENT, 10) != HAL_OK)
    {
      Error_Handler();
    }

}

void errorHandleFRESULT(void)
{
	switch (res) {
		case FR_OK:
			break;
		case FR_DISK_ERR:
			while(1);
			break;
		case FR_NOT_READY:
			while(1);
			break;
		default:
			break;
	}
	return;

		//FR_OK = 0,				/* (0) Succeeded */
		//FR_DISK_ERR,			/* (1) A hard error occurred in the low level disk I/O layer */
		//FR_INT_ERR,				/* (2) Assertion failed */
		//FR_NOT_READY,			/* (3) The physical drive cannot work */
		//FR_NO_FILE,				/* (4) Could not find the file */
		//FR_NO_PATH,				/* (5) Could not find the path */
		//FR_INVALID_NAME,		/* (6) The path name format is invalid */
		//FR_DENIED,				/* (7) Access denied due to prohibited access or directory full */
		//FR_EXIST,				/* (8) Access denied due to prohibited access */
		//FR_INVALID_OBJECT,		/* (9) The file/directory object is invalid */
		//FR_WRITE_PROTECTED,		/* (10) The physical drive is write protected */
		//FR_INVALID_DRIVE,		/* (11) The logical drive number is invalid */
		//FR_NOT_ENABLED,			/* (12) The volume has no work area */
		//FR_NO_FILESYSTEM,		/* (13) There is no valid FAT volume */
		//FR_MKFS_ABORTED,		/* (14) The f_mkfs() aborted due to any problem */
		//FR_TIMEOUT,				/* (15) Could not get a grant to access the volume within defined period */
		//FR_LOCKED,				/* (16) The operation is rejected according to the file sharing policy */
		//FR_NOT_ENOUGH_CORE,		/* (17) LFN working buffer could not be allocated */
		//FR_TOO_MANY_OPEN_FILES,	/* (18) Number of open files > _FS_LOCK */
		//FR_INVALID_PARAMETER	/* (19) Given parameter is invalid */
}
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
