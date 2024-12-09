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



#ifndef __BOARD_TYPE__
	#define __BOARD_TYPE__				BMS_MEASURE_BOARD
//	#define __BOARD_TYPE__				BMS_BALANCE_BOARD
//	#define __BOARD_TYPE__				BMS_BLK_BOARD
#endif


#define __BOARD_NAME__ 					"BMS_MEASURE_BOARD"

//
#define __BRD_ID__						0x01

#ifndef __DEV_ID__
	#define __DEV_ID__					(__BOARD_TYPE__ + __BRD_ID__)
#endif

#ifndef __BOARD_VERSION__
	#define __BOARD_VERSION__			(0x0100)
//	#define __BOARD_VERSION__			(0x0201)
#endif


#ifndef __BOARD_MF_DATE__
	#define BOARD_MF_DAY				7
	#define BOARD_MF_MONTH				9
	#define BOARD_MF_YEAR				2023
	#define __BOARD_MF_DATE__			((BOARD_MF_DAY<<24 ) | (BOARD_MF_MONTH<<16) | BOARD_MF_YEAR)
#endif



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


/**
 * Zustände
 */
typedef enum
{
STATE_OFF	=				0x00,	//!<Keine Blinken (LED aus)
STATE_OK	=				0x4F,	//!<Zustand alles OK (gleichmäßiges "langsames" Blinken Tastverhältnis 50/50)
STATE_WARN	=				0xCC,	//!<Zustand Warnung (gleichmäßiges "schnelles" Blinken Tastverhältnis 50/50)
STATE_ERR_UNKNOWN=			0x1F,	//!<Zustand unbekannter Fehler (gleichmäßiges "sehr schnelles" Blinken Tastverhältnis 50/50)
STATE_ERR_VOLTAGE=			0x02,	//!<Zustand Fehler Kühlkörper-Temperatur zu hoch (einmal kurzes blinken)
STATE_ERR_ERR_CURRENT=		0x03,	//!<Zustand Fehler Kühlkörper-Temperatur zu hoch (einmal kurzes blinken)
STATE_ERR_ALIVE=			0x04,	//!<Zustand Fehler Kühlkörper-Temperatur zu hoch (einmal kurzes blinken)
}_LED_STATE;


typedef enum
{
ERR_NONE	=			0x00,	//!<Keine Blinken (LED aus)
ERR_UNKNOWN=			0x8F,	//!<Zustand unbekannter Fehler (gleichmäßiges "sehr schnelles" Blinken Tastverhältnis 50/50)
ERR_VOLTAGE=			0x02,	//!<Zustand Fehler  (einmal kurzes blinken)
ERR_CURRENT=			0x03,	//!<Zustand Fehler  (einmal kurzes blinken)
ERR_ALIVE=				0x04,
}_SYS_ERR_CODES;

typedef enum
{
	NO_TYPE					=0,
	CURRENT_MA_FLOAT		=0x02,
	CURRENT_MA_FP_INT16,
	VOLTAGE_MV_FLOAT		=0x04,
	VOLTAGE_MV_FP_INT16,
	TEMPERATURE_GC_FLOAT	=0x08,
	TEMPERATURE_GC_FP_INT16
}_MEASURE_TYPE;


typedef struct
{
	float	min;
	float	max;
	uint8_t enable_mask;
}_ADC_THRESHOLDS;

typedef struct
{
	uint32_t	max;			//threshold value to trip
	uint8_t 	enable_mask;	//activated or not
	uint8_t		cft_relays;		//associated relay
}_ADC_CFT_THRESHOLDS; //Current Fast Trip Threshold

#define ENABLE_MIN_THRESHOLD (0x01<<1)
#define ENABLE_MAX_THRESHOLD (0x01<<2)
#define UINT_FS_CURRENT_MA		(uint32_t)179	//Umrechnungsfaktor adc-register zu Stromwert (siehe ADC)
#define UINT_FS_CURRENT_A		(uint32_t)178481	//Umrechnungsfaktor für Strom in Ampere zu  adc-register-wert zu Stromwert (siehe ADC)


#ifdef __DEBUG__
 #define DEFAULT_CFT_CURRENT		1.0
#else
 #define DEFAULT_CFT_CURRENT		32.0
#endif

#define SYSTEM_TRIP_RELAY2		0x01
#define CFT_RELAY1				0x02
#define CFT_RELAY2				0x04
#if __BOARD_VERSION__ >= 0x0200
 #define CFT_RELAY3				0x08
 #define CFT_RELAY4				0x10
#else
 #define CFT_RELAY3				0x00
 #define CFT_RELAY4				0x00
#endif

typedef struct
{
	uint32_t	offset;
	uint32_t	gain;
}_ADC_CH_CALIBRATION;


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
	_ADC_CH_CALIBRATION		adc_calibration[CHANNEL_COUNT];
	_ADC_THRESHOLDS			alert_thresholds[CHANNEL_COUNT];
	_ADC_THRESHOLDS			warn_thresholds[CHANNEL_COUNT];
	_ADC_CFT_THRESHOLDS		current_fast_trip_thresholds[CHANNEL_COUNT];
	uint8_t					allert_mask;
	uint8_t					warn_mask;
	uint8_t					current_fast_trip_mask;
	uint8_t					crit_allert_mask;
	uint8_t 				adc_enable_mask;
}_BMS_MEASURE_CONFIG_REGS;


#pragma pack(push,1)


typedef struct
{
uint8_t			bms_data_type;
uint8_t			flags_ch_number;
float			value;
}_BMSM_ADC_DATA1;

#pragma pack(pop)


#define NO_LED		0x00
#define GREEN_LED	0x01
#define CAN_LED		0x02
#define ALL_LED		(GREEN_LED+CAN_LED)

 typedef enum
 {
 	OFF=0x0000,
 	SLOW_FLASH=0x00FF,
 	FAST_FLASH=0x0F0F,
 	FVERY_FAST_FLASH=0x3333,
 	HYPER_FAST_FLASH=0x5555,
 	LED_100MS_FLASH=0x0001,
 	LED_200MS_FLASH=0x0003,
 	LED_300MS_FLASH=0x0007,
 	LED_1_FLASH=0x0001,
	LED_2_FLASH=0x0005,
	LED_3_FLASH=0x0015,
	LED_4_FLASH=0x0055,
	LED_5_FLASH=0x0155,
	ON=0xFFFF
 }_LED_SIGNAL_MASK;

 /**
  * @struct	REG
  * @brief	Registersatz des Controllers.
  *
  * @note	Der Registersatz wird im RAM und im EEProm gehalten
  */
  typedef struct
  {
 	uint16_t					mask;
 	_LED_SIGNAL_MASK			green_led_mask;
 	_LED_SIGNAL_MASK			can_led_mask;
  }_LED_SIGNAL_STATE;

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
	_SYS_ERR_CODES				sys_err;
	_LED_STATE					monitor_led_state;

	uint8_t						alive_timeout;

	uint32_t					can_rx_cmd_id;
	uint32_t					can_tx_data_id;
	uint32_t					can_tx_heartbeat_id;
	uint32_t					can_rx_brdc_cmd_id;
	uint32_t 					can_filterMask;
	uint32_t 					can_filterID; // Only accept bootloader CAN message ID

	_BMS_MEASURE_CONFIG_REGS	cfg_regs;	//copy of app-config
	_DEV_CONFIG_REGS			dev_config;	//copy of dev-config
	_SW_INFO_REGS				sw_info;	//copy of sw_info
	_BOARD_INFO_STRUCT			board_info;	//copy of board_info
 }_MAIN_REGS;




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
 void DoAlert(uint8_t* p_msg, uint8_t len);
 void set_signal_led(uint8_t led, _LED_SIGNAL_MASK mask);
 void TripCFTRelay (uint8_t cft_relay_mask);

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

#if __BOARD_VERSION__ == 0x0100

#undef LED_RED_Pin
#undef LED_RED_GPIO_Port
#undef RELAY_1_Pin
#undef RELAY_1_GPIO_Port
#undef RELAY_2_Pin
#undef RELAY_2_GPIO_Port
#undef RELAY_3_Pin
#undef RELAY_3_GPIO_Port
#undef RELAY_4_Pin
#undef RELAY_4_GPIO_Port
#undef RELAY_5_Pin
#undef RELAY_5_GPIO_Port

#define LED_RED_Pin 0
#define LED_RED_GPIO_Port GPIOB
#define RELAY_1_Pin GPIO_PIN_13
#define RELAY_1_GPIO_Port GPIOB
#define RELAY_2_Pin GPIO_PIN_14
#define RELAY_2_GPIO_Port GPIOB
#define RELAY_3_Pin GPIO_PIN_15
#define RELAY_3_GPIO_Port GPIOB

#define RELAY_4_Pin 0
#define RELAY_5_Pin 0

#elif __BOARD_VERSION__ >= 0x0200
 	 //Stuff for next Board-Version
#endif

#define PROCESS_NO_TASK			0x00
#define PROCESS_ADS131M08		0x01
#define PROCESS_CAN				0x02
#define PROCESS_10_MS_TASK		0x04
#define PROCESS_100_MS_TASK		0x08
#define PROCESS_STATUS			0x10

#define ALIVE_TIMEOUT_10MS		15
#define APP_CAN_BITRATE			500000UL

#define __DEV_SIGNATURE__			0x12
#define __SW_RELEASE__				0x0102
#define SW_RELEASE_DAY				01
#define SW_RELEASE_MONTH			12
#define SW_RELEASE_YEAR				2024
#define __SW_RELEASE_DATE__			((SW_RELEASE_DAY<<24 ) | (SW_RELEASE_MONTH<<16) | SW_RELEASE_YEAR)
#define __SW_NAME__					"BMS_MEASURE_APP"


 /**
* Bit-Defines für das Controllregister
*/
#define REG_CTRL_ACTIVATE				0
#define REG_CTRL_DEACTIVATE				1
#define REG_CTRL_ENABLE_TRIP			2
#define REG_CTRL_ENABLE_CF_TRIP			3
#define REG_CTRL_ENABLE_ADC_AUTO_SEND	4
#define REG_CTRL_WARN_ENABLE			5
#define REG_CTRL_CRIT_ALERT				6
#define REG_CTRL_RESET					7	//!<Reset des Controllers auslösen

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
