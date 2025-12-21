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

#include "CToC.h"
#include "GM6020.h"
#include "M2006.h"
#include "M3508.h"

uint8_t CAN_CAN1DeviceNumber = 4;                                                                                               // CAN1总线上设备数量
uint8_t CAN_CAN2DeviceNumber = 2;                                                                                               // CAN2总线上设备数量
uint8_t CAN_DeviceNumber = 6;                                                                                                   // CAN总线上设备数量
uint32_t CAN_CAN1IDList[10][2] = {{CAN_GM6020, GM6020_2}, {CAN_M2006, M2006_7}, {CAN_M3508, M3508_1}, {CAN_M3508, M3508_2}, 0}; // CAN1总线上设备ID列表
uint32_t CAN_CAN2IDList[10][2] = {{CAN_GM6020, GM6020_1}, {CAN_RoboMasterC, CToC_MasterID1}, 0};                                // CAN2总线上设备ID列表
int8_t CAN_IDSelect = 0;                                                                                                        // CAN总线上ID列表选择位

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
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = ENABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN1_Init 2 */

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
    hcan2.Init.AutoBusOff = ENABLE;
    hcan2.Init.AutoWakeUp = ENABLE;
    hcan2.Init.AutoRetransmission = DISABLE;
    hcan2.Init.ReceiveFifoLocked = DISABLE;
    hcan2.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN2_Init 2 */

    /* USER CODE END CAN2_Init 2 */
}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED = 0;

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (canHandle->Instance == CAN1)
    {
        /* USER CODE BEGIN CAN1_MspInit 0 */

        /* USER CODE END CAN1_MspInit 0 */
        /* CAN1 clock enable */
        HAL_RCC_CAN1_CLK_ENABLED++;
        if (HAL_RCC_CAN1_CLK_ENABLED == 1)
        {
            __HAL_RCC_CAN1_CLK_ENABLE();
        }

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**CAN1 GPIO Configuration
        PD0     ------> CAN1_RX
        PD1     ------> CAN1_TX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
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
    else if (canHandle->Instance == CAN2)
    {
        /* USER CODE BEGIN CAN2_MspInit 0 */

        /* USER CODE END CAN2_MspInit 0 */
        /* CAN2 clock enable */
        __HAL_RCC_CAN2_CLK_ENABLE();
        HAL_RCC_CAN1_CLK_ENABLED++;
        if (HAL_RCC_CAN1_CLK_ENABLED == 1)
        {
            __HAL_RCC_CAN1_CLK_ENABLE();
        }

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**CAN2 GPIO Configuration
        PB5     ------> CAN2_RX
        PB6     ------> CAN2_TX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
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

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{

    if (canHandle->Instance == CAN1)
    {
        /* USER CODE BEGIN CAN1_MspDeInit 0 */

        /* USER CODE END CAN1_MspDeInit 0 */
        /* Peripheral clock disable */
        HAL_RCC_CAN1_CLK_ENABLED--;
        if (HAL_RCC_CAN1_CLK_ENABLED == 0)
        {
            __HAL_RCC_CAN1_CLK_DISABLE();
        }

        /**CAN1 GPIO Configuration
        PD0     ------> CAN1_RX
        PD1     ------> CAN1_TX
        */
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);

        /* CAN1 interrupt Deinit */
        HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
        /* USER CODE BEGIN CAN1_MspDeInit 1 */

        /* USER CODE END CAN1_MspDeInit 1 */
    }
    else if (canHandle->Instance == CAN2)
    {
        /* USER CODE BEGIN CAN2_MspDeInit 0 */

        /* USER CODE END CAN2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_CAN2_CLK_DISABLE();
        HAL_RCC_CAN1_CLK_ENABLED--;
        if (HAL_RCC_CAN1_CLK_ENABLED == 0)
        {
            __HAL_RCC_CAN1_CLK_DISABLE();
        }

        /**CAN2 GPIO Configuration
        PB5     ------> CAN2_RX
        PB6     ------> CAN2_TX
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);

        /* CAN2 interrupt Deinit */
        HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
        /* USER CODE BEGIN CAN2_MspDeInit 1 */

        /* USER CODE END CAN2_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/*
 *函数简介:CAN1总线接收报文
 *参数说明:报文存储数组
 *返回类型:报文ID
 *备注:默认8字节标准数据帧
 *备注:没有接收到数据,直接退出,返回0
 */
uint32_t CAN_CAN1Receive(uint8_t *Data)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        return 0; // 没有接收到数据，直接退出
    }

    // 复制接收到的数据
    for (uint8_t i = 0; i < 8; i++)
    {
        Data[i] = RxData[i];
    }

    return RxHeader.StdId; // 返回标准ID
}

/*
 *函数简介:CAN2总线接收报文
 *参数说明:报文存储数组
 *返回类型:报文ID
 *备注:默认8字节数据
 *备注:没有接收到数据,直接退出,返回0
 */
uint32_t CAN_CAN2Receive(uint8_t *Data)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK)
    {
        return 0;
    }

    for (uint8_t i = 0; i < 8; i++)
    {
        Data[i] = RxData[i];
    }

    return RxHeader.StdId;
}

/*
 *函数简介:CAN1总线更改接收ID
 *参数说明:接收ID
 *返回类型:无
 *备注:无
 */
void CAN_CAN1ChangeID(uint32_t ID)
{
    // CAN_FilterTypeDef sFilterConfig;

    // // 配置过滤器
    // sFilterConfig.FilterBank = 0;  // 使用过滤器0
    // sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码模式
    // sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  // 32位过滤器

    // // ID配置：标准ID左移5位
    // sFilterConfig.FilterIdHigh = (ID << 5) >> 16;      // ID高16位
    // sFilterConfig.FilterIdLow = (ID << 5) & 0xFFFF;    // ID低16位

    // // 掩码配置：0xFFE3直接赋值
    // sFilterConfig.FilterMaskIdHigh = 0xFFE3;  // 掩码高16位
    // sFilterConfig.FilterMaskIdLow = 0xFFE3;   // 掩码低16位

    // sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;  // 分配到FIFO0
    // sFilterConfig.FilterActivation = ENABLE;  // 激活过滤器
    // sFilterConfig.SlaveStartFilterBank = 14;  // 从CAN2开始使用的过滤器编号
    CAN_FilterTypeDef sFilterConfig;

    // 配置过滤器
    sFilterConfig.FilterBank = 0;                      // 使用过滤器0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位过滤器

    // ID配置：标准ID左移5位
    sFilterConfig.FilterIdHigh = (ID << 5) >> 16;   // ID高16位
    sFilterConfig.FilterIdLow = (ID << 5) & 0xFFFF; // ID低16位

    // 掩码配置：0xFFE3直接赋值
    sFilterConfig.FilterMaskIdHigh = 0xFFE3; // 掩码高16位-
    sFilterConfig.FilterMaskIdLow = 0xFFE3;  // 掩码低16位
    // 配置过滤器（使用 16-bit scale，保持与原寄存器行为一致）
    sFilterConfig.FilterBank = 0;                      // 使用过滤器0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT; // 16位过滤器（与原实现一致）

    // 16-bit 尺度下，把 (ID << 5) 放入 16 位字段
    sFilterConfig.FilterIdHigh = (uint16_t)((ID << 5) & 0xFFFF);
    sFilterConfig.FilterIdLow = (uint16_t)((ID << 5) & 0xFFFF);

    // 掩码配置（16 位尺度下放到相应字段）
    sFilterConfig.FilterMaskIdHigh = 0xFFE3;
    sFilterConfig.FilterMaskIdLow = 0xFFE3;

    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // 分配到FIFO0
    sFilterConfig.FilterActivation = ENABLE;           // 激活过滤器
    sFilterConfig.SlaveStartFilterBank = 14;           // 从CAN2开始使用的过滤器编号
    // 配置过滤器
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 *函数简介:CAN2总线更改接收ID
 *参数说明:接收ID
 *返回类型:无
 *备注:无
 */
void CAN_CAN2ChangeID(uint32_t ID)
{
    CAN_FilterTypeDef sFilterConfig;

    // 配置过滤器（使用 16-bit scale）
    sFilterConfig.FilterBank = 15;                     // 使用过滤器15 (CAN2 区域)
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT; // 16位过滤器

    sFilterConfig.FilterIdHigh = (uint16_t)((ID << 5) & 0xFFFF);
    sFilterConfig.FilterIdLow = (uint16_t)((ID << 5) & 0xFFFF);

    sFilterConfig.FilterMaskIdHigh = 0xFFE3;
    sFilterConfig.FilterMaskIdLow = 0xFFE3;

    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1; // 分配到FIFO1
    sFilterConfig.FilterActivation = ENABLE;           // 激活过滤器
    sFilterConfig.SlaveStartFilterBank = 14;           // 从CAN2开始使用的过滤器编号
    // CAN_FilterTypeDef sFilterConfig;

    // // 配置过滤器
    // sFilterConfig.FilterBank = 15;                     // 使用过滤器15
    // sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码模式
    // sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位过滤器

    // // ID配置：标准ID左移5位
    // sFilterConfig.FilterIdHigh = (ID << 5) >> 16;   // ID高16位
    // sFilterConfig.FilterIdLow = (ID << 5) & 0xFFFF; // ID低16位

    // // 掩码配置：0xFFE3直接赋值
    // sFilterConfig.FilterMaskIdHigh = 0xFFE3; // 掩码高16位
    // sFilterConfig.FilterMaskIdLow = 0xFFE3;  // 掩码低16位

    // sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1; // 分配到FIFO1
    // sFilterConfig.FilterActivation = ENABLE;           // 激活过滤器
    // sFilterConfig.SlaveStartFilterBank = 14;           // 从CAN2开始使用的过滤器编号
    // 配置过滤器
    if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 *函数简介:CAN接收ID列表复位
 *参数说明:无
 *返回类型:无
 *备注:复位CAN_IDSelect,重新从CAN1的1号设备开始接收
 */
void CAN_CANIDReset(void)
{
    CAN_IDSelect = 0;
    CAN_CAN1ChangeID(CAN_CAN1IDList[0][1]);
    CAN_CAN2ChangeID(0x000);
}

/*
 *函数简介:CAN接收获取裁判系统状态
 *参数说明:无
 *返回类型:无
 *备注:跳转到接收底盘C板的回传数据,主要用于发射机构掉电时的CAN设备隔离
 */
void CAN_CAN_GetRefereeSystemData(void)
{
    CAN_IDSelect = 5;
    CAN_CAN1ChangeID(0x000);
    CAN_CAN2ChangeID(CAN_CAN2IDList[1][1]);
}

/*
 *函数简介:CAN1_FIFO0接收中断函数
 *参数说明:无
 *返回类型:无
 *备注:进入中断时关闭连接检测计时,离开中断时重新打开连接检测计时
 *备注:某一设备掉线时,CAN_IDSelect会停留在当前设备在ID列表的索引
 *备注:从掉线到重新连接时会重启遥控器
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        uint8_t Data[8];
        uint32_t ID = CAN_CAN1Receive(Data);

        // 暂时注释掉连接检测相关功能
        /*
        LinkCheck_OFF();
        if(LinkCheck_ErrorID >= 0 && LinkCheck_ErrorID < CAN_CAN1DeviceNumber)
        {
            if(ID == CAN_CAN1IDList[LinkCheck_ErrorID][1])
            {
                LinkCheck_ErrorID = -1;
                LinkCheck_Error = 0;
                Warming_BuzzerClean();
            }
        }
        */

        // if(CAN_CAN1IDList[CAN_IDSelect][0] == CAN_M3508)
        //     M3508_CANDataProcess(ID, Data);
        // else if(CAN_CAN1IDList[CAN_IDSelect][0] == CAN_M2006)
        //     M2006_CANDataProcess(ID, Data);
        // else
        //     GM6020_CANDataProcess(ID, Data);

        // 保留ID切换逻辑
        CAN_IDSelect = (CAN_IDSelect + 1) % CAN_DeviceNumber;
        if (CAN_IDSelect >= 0 && CAN_IDSelect < CAN_CAN1DeviceNumber)
        {
            CAN_CAN1ChangeID(CAN_CAN1IDList[CAN_IDSelect][1]);
            CAN_CAN2ChangeID(0x000);
        }
        else
        {
            CAN_CAN1ChangeID(0x000);
            CAN_CAN2ChangeID(CAN_CAN2IDList[CAN_IDSelect - CAN_CAN1DeviceNumber][1]);
        }

        // 暂时注释掉发射机构相关功能
        /*
        if(RefereeSystem_ShooterOpenFlag == 1)
        {
            RefereeSystem_ShooterOpenCounter++;
            if(RefereeSystem_ShooterOpenCounter >= 5000)
            {
                RefereeSystem_ShooterOpenCounter = 0;
                RefereeSystem_ShooterOpenFlag = 0;
                CAN_CAN1DeviceNumber = 4;
                CAN_DeviceNumber = 6;
            }
        }
        */

        // 暂时注释掉连接检测
        // LinkCheck_ON();
    }
}

/*
 *函数简介:CAN2_FIFO1接收中断函数
 *参数说明:无
 *返回类型:无
 *备注:进入中断时关闭连接检测计时,离开中断时重新打开连接检测计时
 *备注:某一设备掉线时,CAN_IDSelect会停留在当前设备在ID列表的索引
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN2)
    {
        uint8_t Data[8];
        uint32_t ID = CAN_CAN2Receive(Data);

        // 暂时注释掉连接检测相关功能
        /*
        LinkCheck_OFF();
        if(LinkCheck_ErrorID >= CAN_CAN1DeviceNumber && LinkCheck_ErrorID < CAN_DeviceNumber)
        {
            if(ID == CAN_CAN2IDList[LinkCheck_ErrorID-CAN_CAN1DeviceNumber][1])
            {
                LinkCheck_ErrorID = -1;
                LinkCheck_Error = 0;
                Warming_BuzzerClean();
            }
        }
        */

        // 暂时注释掉数据处理
        /*
        if(CAN_CAN2IDList[CAN_IDSelect-CAN_CAN1DeviceNumber][0] == CAN_GM6020)
            GM6020_CANDataProcess(ID, Data);
        else if(CAN_CAN2IDList[CAN_IDSelect-CAN_CAN1DeviceNumber][0] == CAN_RoboMasterC)
            CToC_CANDataProcess(ID, Data);
        */

        // 保留ID切换逻辑
        CAN_IDSelect = (CAN_IDSelect + 1) % CAN_DeviceNumber;
        if (CAN_IDSelect >= 0 && CAN_IDSelect < CAN_CAN1DeviceNumber)
        {
            CAN_CAN1ChangeID(CAN_CAN1IDList[CAN_IDSelect][1]);
            CAN_CAN2ChangeID(0x000);
        }
        else
        {
            CAN_CAN1ChangeID(0x000);
            CAN_CAN2ChangeID(CAN_CAN2IDList[CAN_IDSelect - CAN_CAN1DeviceNumber][1]);
        }

        // 暂时注释掉发射机构相关功能
        /*
        if(RefereeSystem_ShooterOpenFlag == 1)
        {
            RefereeSystem_ShooterOpenCounter++;
            if(RefereeSystem_ShooterOpenCounter >= 5000)
            {
                RefereeSystem_ShooterOpenCounter = 0;
                RefereeSystem_ShooterOpenFlag = 0;
                CAN_CAN1DeviceNumber = 4;
                CAN_DeviceNumber = 6;
            }
        }
        */

        // 暂时注释掉连接检测
        // LinkCheck_ON();
    }
}

/* USER CODE END 1 */
