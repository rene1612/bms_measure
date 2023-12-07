/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern uint8_t main_task_scheduler;

extern uint8_t adc_enable_mask;
extern uint8_t alive_timer;

typedef enum
{
	CURRENT_MA_FLOAT,
	VOLTAGE_MV_FLOAT,
	TEMPERATURE_GC_FLOAT
}_MEASURE_TYPE;


typedef enum
{
	NO_CMD,
	READ_REG_CMD,
	WRITE_REG_CMD,
	SYS_RESET_CMD,
	SYS_BOOT_CMD,
	SET_RELAY_CMD,
	ALIVE_CMD,
	ADC_OFFSET_CAL_CMD,
	END_CMD
}_CAN_CMD;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCU_DRDY_Pin GPIO_PIN_2
#define MCU_DRDY_GPIO_Port GPIOA
#define MCU_SYNC_RESET_Pin GPIO_PIN_3
#define MCU_SYNC_RESET_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOB
#define RELAY_2_Pin GPIO_PIN_14
#define RELAY_2_GPIO_Port GPIOB
#define RELAY_1_Pin GPIO_PIN_15
#define RELAY_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define RELAY_3_Pin GPIO_PIN_13
#define RELAY_3_GPIO_Port GPIOB

#define PROCESS_NO_TASK			0x00
#define PROCESS_ADS131M08		0x01
#define PROCESS_CAN				0x02
#define PROCESS_10_MS_TASK		0x04
#define PROCESS_100_MS_TASK		0x08

#define ALIVE_TIMEOUT_10MS		13

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
