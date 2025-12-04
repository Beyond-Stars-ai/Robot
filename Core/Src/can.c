/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "main.h"

/* 外部变量声明 */
extern uint8_t CAN_CAN1DeviceNumber;
extern uint8_t CAN_CAN2DeviceNumber; 
extern uint8_t CAN_DeviceNumber;
extern int8_t CAN_IDSelect;

/* 电机ID定义 - 在 can.h 中已定义，避免重复定义 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
// 1. 创建过滤器配置结构体
CAN_FilterTypeDef sFilterConfig;

// 2. 设置过滤器编号（CAN1使用0-14）
sFilterConfig.FilterBank = 0;

// 3. 选择掩码模式（可以接收多个ID）
sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;

// 4. 选择32位模式
sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

// 5. 设置基础ID（以标准ID左移5位写入高16位）
// 例如接收 0x200-0x20F：以 0x200 为基准，使用掩码匹配高11位
sFilterConfig.FilterIdHigh = (uint16_t)((0x200U << 5) >> 16);
sFilterConfig.FilterIdLow = (uint16_t)((0x200U << 5) & 0xFFFFU);

// 6. 设置掩码（对标准ID的高11位进行掩码，使 0x200-0x20F 区间通过）
// 对应 32 位滤波寄存器，需要左移5位再分割
uint32_t mask = (0x7F0U << 5); // 0x7F0 = 掩码位，保留高11位（示例）
sFilterConfig.FilterMaskIdHigh = (uint16_t)((mask >> 16) & 0xFFFFU);
sFilterConfig.FilterMaskIdLow = (uint16_t)(mask & 0xFFFFU);

// 7. 分配到FIFO0
sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

// 8. 激活过滤器
sFilterConfig.FilterActivation = ENABLE;

// 9. 设置CAN1的过滤器范围
sFilterConfig.SlaveStartFilterBank = 14;

// 10. 应用过滤器配置
if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
{
    Error_Handler();
}

// 11. 启动CAN1接收中断
if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
{
    Error_Handler();
}

// 12. 启动CAN1
if (HAL_CAN_Start(&hcan1) != HAL_OK)
{
    Error_Handler();
}
  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
// 1. 创建第一个过滤器配置 - 用于GM6020_1 (Yaw轴)
CAN_FilterTypeDef sFilterConfig_CAN2_Yaw;
sFilterConfig_CAN2_Yaw.FilterBank = 15;           // CAN2使用15-27
sFilterConfig_CAN2_Yaw.FilterMode = CAN_FILTERMODE_IDLIST;  // 精确匹配
sFilterConfig_CAN2_Yaw.FilterScale = CAN_FILTERSCALE_32BIT;
{
  uint32_t id32 = (0x205U << 5); // 标准ID 左移 5 位写入 32 位寄存器格式
  sFilterConfig_CAN2_Yaw.FilterIdHigh = (uint16_t)((id32 >> 16) & 0xFFFFU);
  sFilterConfig_CAN2_Yaw.FilterIdLow  = (uint16_t)(id32 & 0xFFFFU);
  /* 列表模式下，Mask 字段通常不被使用，但保持与 Id 相同以保证一致性 */
  sFilterConfig_CAN2_Yaw.FilterMaskIdHigh = (uint16_t)((id32 >> 16) & 0xFFFFU);
  sFilterConfig_CAN2_Yaw.FilterMaskIdLow  = (uint16_t)(id32 & 0xFFFFU);
}
sFilterConfig_CAN2_Yaw.FilterFIFOAssignment = CAN_RX_FIFO1;
sFilterConfig_CAN2_Yaw.FilterActivation = ENABLE;
sFilterConfig_CAN2_Yaw.SlaveStartFilterBank = 14;

if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig_CAN2_Yaw) != HAL_OK)
{
    Error_Handler();
}

// 2. 创建第二个过滤器配置 - 用于板间通信
CAN_FilterTypeDef sFilterConfig_CAN2_CToC;
sFilterConfig_CAN2_CToC.FilterBank = 16;           // 下一个过滤器
sFilterConfig_CAN2_CToC.FilterMode = CAN_FILTERMODE_IDLIST;  // 精确匹配
sFilterConfig_CAN2_CToC.FilterScale = CAN_FILTERSCALE_32BIT;
{
  uint32_t id32 = (0x189U << 5); // 标准ID 左移 5 位
  sFilterConfig_CAN2_CToC.FilterIdHigh = (uint16_t)((id32 >> 16) & 0xFFFFU);
  sFilterConfig_CAN2_CToC.FilterIdLow  = (uint16_t)(id32 & 0xFFFFU);
  sFilterConfig_CAN2_CToC.FilterMaskIdHigh = (uint16_t)((id32 >> 16) & 0xFFFFU);
  sFilterConfig_CAN2_CToC.FilterMaskIdLow  = (uint16_t)(id32 & 0xFFFFU);
}
sFilterConfig_CAN2_CToC.FilterFIFOAssignment = CAN_RX_FIFO1;
sFilterConfig_CAN2_CToC.FilterActivation = ENABLE;
sFilterConfig_CAN2_CToC.SlaveStartFilterBank = 14;

if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig_CAN2_CToC) != HAL_OK)
{
    Error_Handler();
}

// 3. 启动CAN2接收中断
if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
{
    Error_Handler();
}

// 4. 启动CAN2
if (HAL_CAN_Start(&hcan2) != HAL_OK)
{
    Error_Handler();
}
  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
