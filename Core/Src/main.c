/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "crc.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ads131m0x.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t process_10Ms_Timer(void);
void AllertHandler(void);
uint8_t check_AllertThrescholds(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//extern DMA_HandleTypeDef hdma_usart2_rx;
//extern DMA_HandleTypeDef hdma_spi1_rx;
uint8_t main_task_scheduler;
uint8_t adc_enable_mask;

uint8_t alive_timer;
uint16_t timer_10ms;

__attribute__((__section__(".board_info"))) const unsigned char BOARD_NAME[16] = "BMS-MEASURE-APP";

__attribute__((__section__(".sw_info"))) const _SW_INFO_REGS sw_info_regs = {
		__SW_RELEASE__,
		__SW_RELEASE_DATE__,
		0x0000000000000000,
		"no tag"
};

//alles was persistend (im Flash) gespeichert werden soll, z.b. Kalibration, ...
__attribute__((__section__(".app_config"))) const _BMS_MEASURE_CONFIG_REGS app_cfg_regs = {
	//enable mask
	((0x00<<ADC_CH1) | (0x01<<ADC_CH2) | (0x01<<ADC_CH3) | (0x01<<ADC_CH4) | (0x01<<ADC_CH5) | (0x00<<ADC_CH6)),

	{//calibration
		{-128500, 0x800000},	//adc channel 01, current measurement, unused at the moment
#if (CHANNEL_COUNT > 1)
		{-128500, 0x800000},	//adc channel 02, current measurement
#endif
#if (CHANNEL_COUNT > 2)
		{-128500, 0x800000},	//adc channel 03, current measurement
#endif
#if (CHANNEL_COUNT > 3)
		{-128500, 0x800000},	//adc channel 04, current measurement
#endif
#if (CHANNEL_COUNT > 4)
		{-77000, 0x800000},		//adc channel 05, voltage measurement
#endif
#if (CHANNEL_COUNT > 5)
		{0, 0x800000},			//adc channel 06, unused at the moment, current measurement based on shunt resistor
#endif
	},
	{ //allert Thresholds
		{ -30000.0, 30000.0, (ENABLE_MIN_THRESHOLD|ENABLE_MAX_THRESHOLD)},
#if (CHANNEL_COUNT > 1)
		{ -30000.0, 30000.0, (ENABLE_MIN_THRESHOLD|ENABLE_MAX_THRESHOLD)},
#endif
#if (CHANNEL_COUNT > 2)
		{ -30000.0, 30000.0, (ENABLE_MIN_THRESHOLD|ENABLE_MAX_THRESHOLD)},
#endif
#if (CHANNEL_COUNT > 3)
		{ -30000.0, 30000.0, (ENABLE_MIN_THRESHOLD|ENABLE_MAX_THRESHOLD)},
#endif
#if (CHANNEL_COUNT > 4)
		{ 342.0, 475.0, (ENABLE_MIN_THRESHOLD|ENABLE_MAX_THRESHOLD)},
#endif
#if (CHANNEL_COUNT > 5)
		{ 0.0, 0.0, 0x00}
#endif
	}
};

const _DEV_CONFIG_REGS* pDevConfig = (const _DEV_CONFIG_REGS*)DEV_CONFIG_FL_ADDRESS;


/**
 * @var		main_regs
 * @brief	Registersatz im Ram (Arbeitsregister)
 * @see		MAIN_REGS
 * @see		main_ee_regs
 *
 */
_MAIN_REGS main_regs = {
	//!<RW CTRL Ein-/Ausschalten usw.  (1 BYTE )
	((1<<REG_CTRL_ACTIVATE) | (1<<REG_CTRL_CRIT_ALLERT)),

	SYS_OK,
	STATE_OFF,

	ALIVE_TIMEOUT_10MS,

	0,
	0,
	0,
	0,
	0,

	{}

};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
 {
	 int DataIdx;
	 for (DataIdx = 0; DataIdx < len; DataIdx++)
	 {
		 ITM_SendChar(*ptr++);
	 }
	 return len;
 }

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	main_task_scheduler = 0;
	alive_timer = 0;
	timer_10ms = 0;

	memcpy(&main_regs.cfg_regs, &app_cfg_regs, sizeof(_BMS_MEASURE_CONFIG_REGS));

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_CAN_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_CRC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan);

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }

  ADS131M08_init(&hspi1);

  // Start timer
  HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (main_task_scheduler & PROCESS_ADS131M08)
	  {
		  if (!process_ADS131M08())
		  {
			  check_AllertThrescholds();
			  main_task_scheduler &= ~PROCESS_ADS131M08;
		  }
	  }

	  if (main_task_scheduler & PROCESS_CAN)
	  {
		  if (!process_CAN())
			  main_task_scheduler &= ~PROCESS_CAN;
	  }

	  if (main_task_scheduler & PROCESS_10_MS_TASK)
	  {
		  if (!process_10Ms_Timer())
			  main_task_scheduler &= ~PROCESS_10_MS_TASK;
	  }

	  if (main_task_scheduler & PROCESS_100_MS_TASK)
	  {
		  main_task_scheduler &= ~PROCESS_100_MS_TASK;
		  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	  }


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */


//*****************************************************************************
//
//! wird alle 10ms aufgerufen, getriggert durch den Timer4-overrun
//!
//! \fn uint8_t process_10Ms_Timer(void)
//!
//!
//! \return None.
//
//*****************************************************************************
uint8_t check_AllertThrescholds(void)
{
	uint8_t ch;
	float abs_value;

	for (ch=0; ch<NUMB_ADC_CH; ch++)
	{
		if (main_regs.cfg_regs.adc_enable_mask & (1<<ch) )
		{
			//abs_value = abs();
			abs_value = adcConfM->chData[ch].v;

			//upper threschold?
			if (main_regs.cfg_regs.allert_thresholds[ch].enable_mask & ENABLE_MAX_THRESHOLD)
			{

				if (abs_value >= main_regs.cfg_regs.allert_thresholds[ch].max_threshold )
				{
					AllertHandler();
				}
			}
			//lower threschold?
			if (main_regs.cfg_regs.allert_thresholds[ch].enable_mask & ENABLE_MIN_THRESHOLD)
			{

				if (abs_value <= main_regs.cfg_regs.allert_thresholds[ch].min_threshold )
				{
					AllertHandler();
				}
			}
		}
	}

	return 0;
}


//*****************************************************************************
//
//! wird alle 10ms aufgerufen, getriggert durch den Timer4-overrun
//!
//! \fn uint8_t process_10Ms_Timer(void)
//!
//!
//! \return None.
//
//*****************************************************************************
uint8_t process_10Ms_Timer(void)
{
	if (alive_timer)
	{
		if (--alive_timer == 0)
		{
			//kritisch
			alive_timer = main_regs.alive_timeout;
		}
	}

	if (!(++timer_10ms % 10))
	{
		main_task_scheduler |= PROCESS_100_MS_TASK;
	}

	return 0;
}


//*****************************************************************************
//
//! wird im Fehlerfall aufgerufen und aktivert die vollständige Abschaltung
//!
//! \fn void AllertHandler(void)
//!
//!
//! \return None.
//
//*****************************************************************************
void AllertHandler(void)
{
	//send Something?

	if (main_regs.ctrl & (1<<REG_CTRL_ACTIVATE))
	{
		HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_SET);

		while(1){}
	}
}


//*****************************************************************************
//
//! Callback: timer has rolled over
//!
//! \fn void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//!
//!
//! \return None.
//
//*****************************************************************************
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Check which version of the timer triggered this callback and toggle LED
	if (htim == &htim4 )
	{
		main_task_scheduler |= PROCESS_10_MS_TASK;
	}
}


/* Save checksum and length to flash crc area ----------------------------------------------------------*/
/*
uint8_t SaveCfgParams2Fl(void){

	HAL_StatusTypeDef returnedERR=HAL_OK;
	//_BMS_MEASURE_CONFIG_REGS app_cfg_regs;
	uint32_t* p_app_cfg_regs=(uint32_t*)&main_regs.cfg_regs;
	uint32_t length = 0;
	uint32_t calculatedCrc;
	int i;

	//copy crc flash mem to temp ram variable before ereasing flash
	//memcpy((void*)&dev_crc_regs, (void*)pDevCRCRegs, sizeof(_BMS_MEASURE_CONFIG_REGS));

	//get and calc the new parameters for the spezefic flash area
	length =  (uint32_t)(bl_ctrl_reg.flash_ptr - bl_ctrl_reg.flash_start_addr);
	calculatedCrc=btld_CalcChecksum(bl_ctrl_reg.flash_start_addr, length);

	//set the new params in the temp ram struct
 	switch(bl_ctrl_reg.flash_area) {

		case BOOTLOADER:
			dev_crc_regs.bl_length = length;
			dev_crc_regs.bl_crc32 = calculatedCrc;
			break;

		case APPLICATION:
			dev_crc_regs.app_length = length;
			dev_crc_regs.app_crc32 = calculatedCrc;
			break;

		case APPLICATION_CONFIG:
			dev_crc_regs.app_config_length = length;
			dev_crc_regs.app_config_crc32 = calculatedCrc;
			break;

		case DEVICE_CONFIG:
			dev_crc_regs.dev_config_length = length;
			dev_crc_regs.dev_config_crc32 = calculatedCrc;
			break;

		case BL_DEV_CRC:
			//flash_ptr = DEV_CRC_FL_ADDRESS;
			//break;
		default:
			return HAL_ERROR;
	}

 	//erease the flash page that holds the dev crc and length params
 	btld_EraseFlash(BL_DEV_CRC);

	HAL_FLASH_Unlock();

	//write back the updated, temp ram struct to dev crc flash
	//@note: be aware when modifying the dev crc struct, we need a 4 Byte alignment, otherwise the copycode below will fail
	for (i=0; i<(sizeof(_DEV_CRC_REGS)/4); i++) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)((uint32_t*)pDevCRCRegs+i), (uint64_t)p_dev_crc_regs[i] ) != HAL_OK) {
			returnedERR=HAL_ERROR;
			break;
		}
	}

	HAL_FLASH_Lock();

	return returnedERR;
}
*/

/* Jump to application -------------------------------------------------------------*/
void JumpToBtld(void){
    uint32_t  JumpAddress = *(__IO uint32_t*)(DEV_BL_ADDRESS + 4);
    pFunction Jump = (pFunction)JumpAddress;


    HAL_RCC_DeInit();
    HAL_DeInit();

    //HAL_NVIC_DisableIRQ();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

#if (SET_VECTOR_TABLE)
    SCB->VTOR = DEV_BL_ADDRESS;
#endif

    __set_MSP(*(__IO uint32_t*)DEV_BL_ADDRESS);
    Jump();
}

/* Jump to application -------------------------------------------------------------*/
void JumpToApp(void){
    uint32_t  JumpAddress = *(__IO uint32_t*)(DEV_APP_ADDRESS + 4);
    pFunction Jump = (pFunction)JumpAddress;


    HAL_RCC_DeInit();
    HAL_DeInit();

    //HAL_NVIC_DisableIRQ();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

#if (SET_VECTOR_TABLE)
    SCB->VTOR = DEV_APP_ADDRESS;
#endif

    __set_MSP(*(__IO uint32_t*)DEV_APP_ADDRESS);
    Jump();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
