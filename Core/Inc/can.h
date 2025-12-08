/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

typedef enum
{
	CAN_M3508=0,//M3508
	CAN_M2006,//M2006
	CAN_GM6020,//GM6020
	CAN_RoboMasterC,//C板
}CAN_MotorModel;//CAN总线设备分类枚举

extern uint8_t CAN_CAN1DeviceNumber;//CAN1总线上设备数量
extern uint8_t CAN_CAN2DeviceNumber;//CAN2总线上设备数量
extern uint8_t CAN_DeviceNumber;//CAN总线上设备数量
extern int8_t CAN_IDSelect;//CAN总线上ID列表选择位

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */

void CAN_CANIDReset(void);//CAN接收ID列表复位
void CAN_CAN_GetRefereeSystemData(void);//CAN接收获取裁判系统状态

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

