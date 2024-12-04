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

#if __has_include("gitcommit.h")
	#include "gitcommit.h"
#else
	#define __GIT_SHORT_HASH__ 0x0000000
	#define __GIT_BRANCH__ "none"
	#define __GIT_DATE_STR__ "2024-11-25"
	#define __GIT_DATE_UT__ 1732571756
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t process_10Ms_Timer(void);
void AlertHandler(void);
uint8_t check_Alert_and_Warn_Thresholds(void);
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

#ifdef __DEBUG__
__attribute__((__section__(".dev_config"))) const _DEV_CONFIG_REGS dev_config_regs = {
		__DEV_ID__,
		__BOARD_TYPE__,
		__BOARD_NAME__,
		__BOARD_VERSION__,
		__BOARD_MF_DATE__,
		DEAULT_BL_CAN_BITRATE,
		DEAULT_APP_CAN_BITRATE,
		DEAULT_TRIPP_CAN_ID,
		DEAULT_BROADCAST_CAN_ID,
};
#endif


__attribute__((__section__(".board_info"))) const unsigned char BOARD_NAME[20] = __BOARD_NAME__;

__attribute__((__section__(".sw_info"))) const _SW_INFO_REGS sw_info_regs = {
		__SW_NAME__,
	#ifdef __DEBUG__
		"Debug",
	#else
		"Release",
	#endif
		__SW_RELEASE__,
		__SW_RELEASE_DATE__,
		{	//git-info aus gitcommit.h
			__GIT_SHORT_HASH__,
			__GIT_DATE_UT__,
			__GIT_DATE_STR__,
			__GIT_BRANCH__,
		}
};

//alles was persistend (im Flash) gespeichert werden soll, z.b. Kalibration, ...
__attribute__((__section__(".app_config"))) const _BMS_MEASURE_CONFIG_REGS app_cfg_regs = {

	{//calibration
		{-128400, 0x800000},	//adc channel 01, current measurement, unused at the moment
#if (CHANNEL_COUNT > 1)
		{-114200, 0x800000},	//adc channel 02, current measurement
#endif
#if (CHANNEL_COUNT > 2)
		{-87500, 0x800000},		//adc channel 03, current measurement
#endif
#if (CHANNEL_COUNT > 3)
		{-125100, 0x800000},		//adc channel 04, current measurement
#endif
#if (CHANNEL_COUNT > 4)
		{-77200, 0x7E1AD8},		//adc channel 05, voltage measurement
#endif
#if (CHANNEL_COUNT > 5)
		{0, 0x800000},			//adc channel 06, unused at the moment, current measurement based on shunt resistor
#endif
	},
	{ //alert Thresholds
		{ 0.0, 30000.0, (ENABLE_MAX_THRESHOLD)},
#if (CHANNEL_COUNT > 1)
		{ 0.0, 30000.0, (ENABLE_MAX_THRESHOLD)},
#endif
#if (CHANNEL_COUNT > 2)
		{ 0.0, 30000.0, (ENABLE_MAX_THRESHOLD)},
#endif
#if (CHANNEL_COUNT > 3)
		{ 0.0, 50000.0, (ENABLE_MAX_THRESHOLD)},
#endif
#if (CHANNEL_COUNT > 4)
 #ifdef __DEBUG__
		{ 400.0, 555.0, (0)},//ENABLE_MIN_THRESHOLD|ENABLE_MAX_THRESHOLD
 #else
		{ 400.0, 555.0, (ENABLE_MIN_THRESHOLD|ENABLE_MAX_THRESHOLD)},
 #endif
#endif
#if (CHANNEL_COUNT > 5)
		{ 0.0, 0.0, 0x00}
#endif
	},
	{ //Warn Thresholds
		{ 0.0, 25000.0, (ENABLE_MAX_THRESHOLD)},
#if (CHANNEL_COUNT > 1)
		{ 0.0, 25000.0, (ENABLE_MAX_THRESHOLD)},
#endif
#if (CHANNEL_COUNT > 2)
		{ 0.0, 25000.0, (ENABLE_MAX_THRESHOLD)},
#endif
#if (CHANNEL_COUNT > 3)
		{ 0.0, 45000.0, (ENABLE_MAX_THRESHOLD)},
#endif
#if (CHANNEL_COUNT > 4)
#ifdef __DEBUG__
		{ 420.0, 545.0, (0)},//ENABLE_MIN_THRESHOLD|ENABLE_MAX_THRESHOLD
#else
		{ 420.0, 545.0, (ENABLE_MIN_THRESHOLD|ENABLE_MAX_THRESHOLD)},
#endif
#endif
#if (CHANNEL_COUNT > 5)
		{ 0.0, 0.0, 0x00}
#endif
	},
	{ //Current Fast Trip Threshold
		{ (DEFAULT_CFT_CURRENT*UINT_FS_CURRENT_A), (ENABLE_MAX_THRESHOLD), (CFT_RELAY1)},
#if (CHANNEL_COUNT > 1)
		{ (DEFAULT_CFT_CURRENT*UINT_FS_CURRENT_A), (ENABLE_MAX_THRESHOLD), (CFT_RELAY2)},
#endif
#if (CHANNEL_COUNT > 2)
		{ (DEFAULT_CFT_CURRENT*UINT_FS_CURRENT_A), (ENABLE_MAX_THRESHOLD), (CFT_RELAY3)},
#endif
#if (CHANNEL_COUNT > 3)
		{ (50*UINT_FS_CURRENT_A), (ENABLE_MAX_THRESHOLD), (CFT_RELAY4)},
#endif
#if (CHANNEL_COUNT > 4)
		{ 0, 0x00},
#endif
#if (CHANNEL_COUNT > 5)
		{ 0, 0x00}
#endif
	},
	//allert_mask mask
	((0x01<<ADC_CH1) | (0x01<<ADC_CH2) | (0x01<<ADC_CH3) | (0x01<<ADC_CH4) | (0x01<<ADC_CH5) | (0x00<<ADC_CH6)),

	//warn_mask mask
	((0x01<<ADC_CH1) | (0x01<<ADC_CH2) | (0x01<<ADC_CH3) | (0x01<<ADC_CH4) | (0x01<<ADC_CH5) | (0x00<<ADC_CH6)),

	//current_fast_trip_mask mask
	((0x01<<ADC_CH1) | (0x01<<ADC_CH2) | (0x01<<ADC_CH3) | (0x00<<ADC_CH4) | (0x00<<ADC_CH5) | (0x00<<ADC_CH6)),

	//crit_allert_mask mask
	((0x01<<ADC_CH1) | (0x01<<ADC_CH2) | (0x01<<ADC_CH3) | (0x01<<ADC_CH4) | (0x01<<ADC_CH5) | (0x00<<ADC_CH6)),

	//enable mask
	((0x01<<ADC_CH1) | (0x01<<ADC_CH2) | (0x01<<ADC_CH3) | (0x01<<ADC_CH4) | (0x01<<ADC_CH5) | (0x00<<ADC_CH6))
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
	((1<<REG_CTRL_ACTIVATE) | (1<<REG_CTRL_CRIT_ALERT) | (1<<REG_CTRL_ENABLE_TRIP) |
	(0<<REG_CTRL_ENABLE_ADC_AUTO_SEND) | (0<<REG_CTRL_ENABLE_CF_TRIP) | (1<<REG_CTRL_WARN_ENABLE)),

	SYS_OK,
	ERR_NONE,
	STATE_OFF,

	ALIVE_TIMEOUT_10MS,

	0,
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



/****************************************************************************
  * @brief  trip one or more high current relays in case of overcurrent
  * @retval nore
  * @fn TripCFTRelay
  */
void TripCFTRelay (uint8_t cft_relay_mask)
{
	//CFT_RELAY1
	if (cft_relay_mask & CFT_RELAY1)	{
		HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_RESET);
	}

	//CFT_RELAY2
	if (cft_relay_mask & CFT_RELAY2)	{
		HAL_GPIO_WritePin(RELAY_3_GPIO_Port, RELAY_3_Pin, GPIO_PIN_RESET);
	}

#if __BOARD_VERSION__ >= 0x0200
	//CFT_RELAY3
	if (cft_relay_mask & CFT_RELAY3)	{
		HAL_GPIO_WritePin(RELAY_4_GPIO_Port, RELAY_4_Pin, GPIO_PIN_RESET);
	}

	//CFT_RELAY4
	if (cft_relay_mask & CFT_RELAY4)	{
		HAL_GPIO_WritePin(RELAY_5_GPIO_Port, RELAY_5_Pin, GPIO_PIN_RESET);
	}
#endif

}

_LED_SIGNAL_STATE	led_state={1,SLOW_FLASH,OFF};

/****************************************************************************
  * @brief  The application entry point.
  * @retval int
  */
void set_signal_led(uint8_t led, _LED_SIGNAL_MASK mask)
{
	if(led & GREEN_LED)
		led_state.green_led_mask=mask;

	if(led & CAN_LED) {
		led_state.can_led_mask=mask;
		led_state.mask=1;
	}

	return;
}


/****************************************************************************
  * @brief  The application entry point.
  * @retval int
  */
void signal_led_task(void)
{

	if(led_state.green_led_mask & led_state.mask) {
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	}else {
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	}

	if(led_state.can_led_mask & led_state.mask) {
		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		led_state.can_led_mask &= + ~led_state.mask;
	}

	led_state.mask<<=1;
	if (!led_state.mask) {
		led_state.mask=1;
	}
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

	//copy app-config from flash to ram (we will use it from ram)
	memcpy(&main_regs.cfg_regs, &app_cfg_regs, sizeof(_BMS_MEASURE_CONFIG_REGS));

	//copy dev-config from flash to ram (we will use it from ram)
	memcpy(&main_regs.dev_config, pDevConfig, sizeof(_DEV_CONFIG_REGS));

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

  // Start CAN
  HAL_CAN_Start(&hcan);

  //Init and Start ADC
  ADS131M08_init(&hspi1);

  // Start timer
  HAL_TIM_Base_Start_IT(&htim4);

  //activate incomming notifications on CAN-Bus
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }

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
			  check_Alert_and_Warn_Thresholds();
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

		  signal_led_task();

		  //main_task_scheduler |= PROCESS_CAN;
		  //can_task_scheduler |= PROCESS_CAN_SEND_NEW_ALIVE_DATA;
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
//! \fn uint8_t check_Alert_and_Warn_Thresholds(void)
//!
//!
//! \return None.
//
//*****************************************************************************
uint8_t check_Alert_and_Warn_Thresholds(void)
{
	uint8_t ch;
	float abs_value;
	uint8_t alert_msg[7];
	uint8_t ch_type;
	uint8_t led_mask;

	for (ch=0; ch<NUMB_ADC_CH; ch++)
	{
		if (main_regs.cfg_regs.adc_enable_mask & (1<<ch) )
		{
			//abs_value = abs();
			abs_value = adcConfM->chData[ch].v;

			//upper alert threschold?
			if (main_regs.cfg_regs.alert_thresholds[ch].enable_mask & ENABLE_MAX_THRESHOLD)
			{
				if (abs_value >= main_regs.cfg_regs.alert_thresholds[ch].max )
				{
					//AllertHandler();
					alert_msg[1]=ENABLE_MAX_THRESHOLD;
					goto crit_err_mark;
				}
			}

			//upper warn threschold?
			if (main_regs.cfg_regs.warn_thresholds[ch].enable_mask & ENABLE_MAX_THRESHOLD)
			{
				if (abs_value >= main_regs.cfg_regs.warn_thresholds[ch].max )
				{
					//AllertHandler();
					alert_msg[1]=ENABLE_MAX_THRESHOLD;
					goto warn_mark;
				}
			}

			//lower alert threschold?
			if (main_regs.cfg_regs.alert_thresholds[ch].enable_mask & ENABLE_MIN_THRESHOLD)
			{
				if (abs_value <= main_regs.cfg_regs.alert_thresholds[ch].min )
				{
					//AllertHandler();
					alert_msg[1]=ENABLE_MIN_THRESHOLD;
					goto crit_err_mark;
				}
			}

			//lower warn threschold?
			if (main_regs.cfg_regs.warn_thresholds[ch].enable_mask & ENABLE_MIN_THRESHOLD)
			{
				if (abs_value <= main_regs.cfg_regs.warn_thresholds[ch].min )
				{
					//AllertHandler();
					alert_msg[1]=ENABLE_MIN_THRESHOLD;
					goto warn_mark;
				}
			}
		}
	}

	main_regs.sys_state=STATE_OK;
	return 0;

crit_err_mark:
	ch_type = adcConfM->chData[ch].measure_type;

	if (ch_type & 0x02) { //Current
		alert_msg[0]=ERR_CURRENT;
		led_mask=LED_3_FLASH;
	}else if (ch_type & 0x04){ //Voltage
		alert_msg[0]=ERR_VOLTAGE;
		led_mask=LED_2_FLASH;
	}else {
		alert_msg[0]=ERR_UNKNOWN;
		led_mask=LED_4_FLASH;
	}

	set_signal_led(GREEN_LED, led_mask);

	*((float*)(alert_msg+2)) = adcConfM->chData[adcConfM->ch].v;

	DoAlert(alert_msg, 7);

	return 1; //we will not reach this (but just to be sure)

warn_mark:
	ch_type = adcConfM->chData[ch].measure_type;

	if (ch_type & 0x02) { //Current
		alert_msg[0]=ERR_CURRENT;
		led_mask=LED_3_FLASH;
	}else if (ch_type & 0x04){ //Voltage
		alert_msg[0]=ERR_VOLTAGE;
		led_mask=LED_2_FLASH;
	}else {
		alert_msg[0]=ERR_UNKNOWN;
		led_mask=LED_4_FLASH;
	}

	set_signal_led(GREEN_LED, led_mask);

	*((float*)(alert_msg+2)) = adcConfM->chData[adcConfM->ch].v;

	if (main_regs.ctrl & (1<<REG_CTRL_WARN_ENABLE))
		can_send_warn_msg(alert_msg, 6);

	main_regs.sys_state=STATE_WARN;
	return 1;
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
	uint8_t alert_msg;

	if (alive_timer)
	{
		if (--alive_timer == 0)
		{
			//kritisch
			alive_timer = main_regs.alive_timeout;

			alert_msg = ERR_ALIVE;
			DoAlert(&alert_msg, 1);
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
//! \fn void AlertHandler(void)
//!
//!
//! \return None.
//
//*****************************************************************************
void AlertHandler(void)
{
	//send Something?

	if (main_regs.ctrl & (1<<REG_CTRL_ACTIVATE))
	{
		HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_SET);

		Error_Handler();
	}
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
void DoAlert(uint8_t* p_msg, uint8_t len)
{
	//send Something?
	can_send_alert_msg(p_msg, len);

	if (main_regs.ctrl & (1<<REG_CTRL_CRIT_ALERT))
	{
		//Time to say goodbye, trip SYSTEM_TRIP_RELAY2
		HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_SET);

		AlertHandler();
	}

	main_regs.sys_err=(_SYS_ERR_CODES)p_msg[0];
	main_regs.sys_state=SYS_ERROR;

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


/* Jump to Bootoader -------------------------------------------------------------*/
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
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
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
