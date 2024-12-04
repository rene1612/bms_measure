/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

#define CANRX_SA 0x01
#define CANTX_SA 0x02
#define CANTX_HA 0x03
#define RXFILTERMASK 0xFFFFFFFC

//#define BMS_MEASURE_CAN_ID		0x448



extern uint8_t				can_task_scheduler;


#define PROCESS_CAN_SEND_AUTO_ADC_DATA 		0x01
#define PROCESS_CAN_ON_MSG					0x02
#define PROCESS_CAN_SEND_REPLAY				0x04
#define PROCESS_CAN_SEND_NEW_ALIVE_DATA		0x08
#define PROCESS_CAN_SEND_ADC_DATA			0x10
#define PROCESS_CAN_ON_BRDC_MSG				0x80


/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
void can_send_brdc_msg(uint8_t* p_msg, uint8_t len);
void can_send_alert_msg(uint8_t* p_alert_msg, uint8_t len);
void can_send_warn_msg(uint8_t* p_warn_msg, uint8_t len);

uint8_t		process_CAN(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

