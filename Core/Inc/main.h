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
#include <dev_config.h>

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
extern uint8_t alive_timer;


//#define CHANNEL_COUNT ((uint8_t)NUMB_ADC_CH + 0)   // ADS131M04 -> 4 Channels
#define CHANNEL_COUNT (6)   // ADS131M04 -> 4 Channels

#if ((CHANNEL_COUNT < 1) || (CHANNEL_COUNT > 8))
    #error Invalid channel count configured in 'main.h'.
#endif

typedef enum
{
	CURRENT_MA_FLOAT,
	VOLTAGE_MV_FLOAT,
	TEMPERATURE_GC_FLOAT
}_MEASURE_TYPE;


typedef enum
{
	ACK = 0x11,
	NACK= 0x13
}_REPLAY_TYPE;


typedef struct
{
	float	min_threshold;
	float	max_threshold;
	uint8_t enable_mask;
}_ALLERT_THRESHOLDS;

#define ENABLE_MIN_THRESHOLD (0x01<<1)
#define ENABLE_MAX_THRESHOLD (0x01<<2)

typedef struct
{
	uint32_t	offset;
	uint32_t	gain;
}_ADC_CH_CALIBRATION;


typedef enum
{
	NO_CMD,
	SYS_READ_REG_CMD,
	SYS_WRITE_REG_CMD,
	SYS_RESET_CMD,
	SYS_APP_RESET_CMD,
	SYS_BOOT_CMD,
	SET_RELAY_CMD,
	ALIVE_CMD,
	ADC_OFFSET_CAL_CMD,
	ADC_GAIN_CAL_CMD,
	ADC_READ_REG_CMD,
	ADC_WRITE_REG_CMD,
	REPLAY_AKC_NACK_CMD,
	REPLAY_DATA_CMD,
	END_CMD
}_CAN_CMD;


typedef enum
{
	SYS_OK,
	SYS_ACTIVE_START,
	SYS_ACTIVE_FC,
	SYS_ACTIVE_SBC,
	SYS_ACTIVE_STOP,
	SYS_ERROR,
}_SYS_STATE;


/**
 * @struct	REG
 * @brief	Registersatz des Controllers.
 *
 * @note	Der Registersatz wird im RAM und im EEProm gehalten
 */
 typedef struct
 {
  uint8_t 				adc_enable_mask;
  _ADC_CH_CALIBRATION	adc_calibration[CHANNEL_COUNT];
  _ALLERT_THRESHOLDS	allert_thresholds[CHANNEL_COUNT];
 }_BMS_MEASURE_CONFIG_REGS;


/**
 * @struct	REG
 * @brief	Registersatz des Controllers.
 *
 * @note	Der Registersatz wird im RAM und im EEProm gehalten
 */
 typedef struct
 {
  uint8_t						ctrl;

  _SYS_STATE					sys_state;

  uint8_t						monitor_led_state;

  uint8_t						alive_timeout;

  _BMS_MEASURE_CONFIG_REGS		cfg_regs;
 }_MAIN_REGS;


 /**
  * @struct	REG
  * @brief	Registersatz des Controllers.
  *
  * @note	Der Registersatz wird im RAM und im EEProm gehalten
  */
  typedef struct
  {
 /**
  * @var	unsigned int sw_release
  * @brief	Register mit der Softwareversion
  * @see	__SW_RELEASE__
  * @see	SW_REL_REG
  * @see	config.h
  */
  uint16_t		sw_release;

 /**
  * @var	unsigned int sw_release_date
  * @brief	Register mit dem Datum der Softwareversion
  * Formatierung:
  *	- Byte 0 -> Tag
  *	- BYTE 1 -> Monat
  *	- BYTE 2 -> Jahr
  *	- BYTE 3 -> Jahr
  * @see	__SW_RELEASE_DATE__
  * @see	SW_REL_DATE_REG
  * @see	config.h
  */
  uint32_t		sw_release_date;

  uint64_t		sw_git_short_hash;

  const char	sw_git_tag[16];
 }_SW_INFO_REGS;


 /**
  * @def		REG_ADDR_AUTO_INC
  * @brief	Flag im Registeradressbyte, welches eine automatische Incrementierung
  *			der Registeradresse nach einer Lese- oder Schreibaktion bewirkt.
  *
  * @note	Die Registeradresse wird vom Master bei jeder Schreibaktion als erstes Byte gesendet.
  * @see		fco_reg_addr
  * @see		TWI-WRITE-Mode
  */
  #define REG_ADDR_AUTO_INC	0x80


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // Funktionen(Prototypes)
  //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void set_sys_state (_SYS_STATE sys_state);
 void JumpToBtld(void);
 void JumpToApp(void);

 /* Private typedef -----------------------------------------------------------*/
 typedef void (*pFunction)(void);

 extern  _MAIN_REGS main_regs;


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
#define RELAY_1_Pin GPIO_PIN_14
#define RELAY_1_GPIO_Port GPIOB
#define RELAY_2_Pin GPIO_PIN_15
#define RELAY_2_GPIO_Port GPIOB
#define RELAY_3_Pin GPIO_PIN_9
#define RELAY_3_GPIO_Port GPIOA
#define RELAY_4_Pin GPIO_PIN_10
#define RELAY_4_GPIO_Port GPIOA
#define RELAY_5_Pin GPIO_PIN_11
#define RELAY_5_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

#define PROCESS_NO_TASK			0x00
#define PROCESS_ADS131M08		0x01
#define PROCESS_CAN				0x02
#define PROCESS_10_MS_TASK		0x04
#define PROCESS_100_MS_TASK		0x08
#define PROCESS_STATUS			0x10

#define ALIVE_TIMEOUT_10MS		15
#define APP_CAN_BITRATE			500000UL

#define __DEV_SIGNATURE__			0x12
#define __SW_RELEASE__				0x0100
#define SW_RELEASE_DAY				8
#define SW_RELEASE_MONTH			12
#define SW_RELEASE_YEAR				2023
#define __SW_RELEASE_DATE__			((SW_RELEASE_DAY<<24 ) | (SW_RELEASE_MONTH<<18) | SW_RELEASE_YEAR)


 /**
  * Bit-Defines für das Controllregister
  */
  #define REG_CTRL_ACTIVATE			0
  #define REG_CTRL_DEACTIVATE		1
  #define REG_CTRL_CRIT_ALLERT		2
  #define REG_CTRL_RESET			7	//!<Reset des Controllers auslösen

 /**
  * Zustände
  */
#define STATE_OFF					0x00	//!<Keine Blinken (LED aus)

#define STATE_OK					0x4F	//!<Zustand alles OK (gleichmäßiges "langsames" Blinken Tastverhältnis 50/50)

#define STATE_WARN					0xCC	//!<Zustand Warnung (gleichmäßiges "schnelles" Blinken Tastverhältnis 50/50)

#define STATE_ERR_UNKNOWN			0x1F	//!<Zustand unbekannter Fehler (gleichmäßiges "sehr schnelles" Blinken Tastverhältnis 50/50)

#define STATE_ERR_HEADSINK_TEMP		0x11	//!<Zustand Fehler Kühlkörper-Temperatur zu hoch (einmal kurzes blinken)



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
