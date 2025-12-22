/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

// #include "usart.h"

#include "remote_control.h"
#include "CToC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t receiveData[18];

// RC_ctrl_t rc_control = {0};

uint8_t num = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
  .name = "DebugTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MoveTask */
osThreadId_t MoveTaskHandle;
const osThreadAttr_t MoveTask_attributes = {
  .name = "MoveTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TurnTask */
osThreadId_t TurnTaskHandle;
const osThreadAttr_t TurnTask_attributes = {
  .name = "TurnTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for StatuTask */
osThreadId_t StatuTaskHandle;
const osThreadAttr_t StatuTask_attributes = {
  .name = "StatuTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Move_R_Queue */
osMessageQueueId_t Move_R_QueueHandle;
const osMessageQueueAttr_t Move_R_Queue_attributes = {
  .name = "Move_R_Queue"
};
/* Definitions for Turn_Queue */
osMessageQueueId_t Turn_QueueHandle;
const osMessageQueueAttr_t Turn_Queue_attributes = {
  .name = "Turn_Queue"
};
/* Definitions for Statu_Queue */
osMessageQueueId_t Statu_QueueHandle;
const osMessageQueueAttr_t Statu_Queue_attributes = {
  .name = "Statu_Queue"
};
/* Definitions for Move_L_Queue */
osMessageQueueId_t Move_L_QueueHandle;
const osMessageQueueAttr_t Move_L_Queue_attributes = {
  .name = "Move_L_Queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDebugTask(void *argument);
void StartMoveTask(void *argument);
void StartTurnTask(void *argument);
void StartStatuTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Move_R_Queue */
  Move_R_QueueHandle = osMessageQueueNew (16, sizeof(MOVE_BUFFER_SIZE), &Move_R_Queue_attributes);

  /* creation of Turn_Queue */
  Turn_QueueHandle = osMessageQueueNew (16, sizeof(TURN_BUFFER_SIZE), &Turn_Queue_attributes);

  /* creation of Statu_Queue */
  Statu_QueueHandle = osMessageQueueNew (16, sizeof(STATUS_BUFFER_SIZE), &Statu_Queue_attributes);

  /* creation of Move_L_Queue */
  Move_L_QueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &Move_L_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DebugTask */
  DebugTaskHandle = osThreadNew(StartDebugTask, NULL, &DebugTask_attributes);

  /* creation of MoveTask */
  MoveTaskHandle = osThreadNew(StartMoveTask, NULL, &MoveTask_attributes);

  /* creation of TurnTask */
  TurnTaskHandle = osThreadNew(StartTurnTask, NULL, &TurnTask_attributes);

  /* creation of StatuTask */
  StatuTaskHandle = osThreadNew(StartStatuTask, NULL, &StatuTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDebugTask */
/**
 * @brief  Function implementing the DebugTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument)
{
  /* USER CODE BEGIN StartDebugTask */
    osDelay(10);
    printf("Start Remote Task\r\n");
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receiveData, sizeof(receiveData));
    /* Infinite loop */
    for (;;)
    {
        printf("hello world\r\n");
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        osDelay(750);
        HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
        osDelay(750);
        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
        osDelay(750);
    }
  /* USER CODE END StartDebugTask */
}

/* USER CODE BEGIN Header_StartMoveTask */
/**
 * @brief Function implementing the MoveTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMoveTask */
__weak void StartMoveTask(void *argument)
{
  /* USER CODE BEGIN StartMoveTask */
    uint8_t moveBuffer[MOVE_BUFFER_SIZE];
    uint8_t turnBuffer[TURN_BUFFER_SIZE];
    int16_t moveData[4];

    /* Infinite loop */
    for (;;)
    {
        if (osMessageQueueGet(Move_R_QueueHandle, moveBuffer, NULL, osWaitForever) == osOK)
        {
            memcpy(&moveData[0], moveBuffer, sizeof(int16_t));
            memcpy(&moveData[1], moveBuffer + sizeof(int16_t), sizeof(int16_t));

        };
        if (osMessageQueueGet(Move_L_QueueHandle, turnBuffer, NULL, osWaitForever) == osOK)
        {
            memcpy(&moveData[2], turnBuffer, sizeof(int16_t));
            memcpy(&moveData[3], turnBuffer + sizeof(int16_t), sizeof(int16_t));

        };
        CToC_MasterSendData(moveData[0], moveData[1], moveData[2], moveData[3]);
    }
  /* USER CODE END StartMoveTask */
}

/* USER CODE BEGIN Header_StartTurnTask */
/**
 * @brief Function implementing the TurnTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTurnTask */
__weak void StartTurnTask(void *argument)
{
  /* USER CODE BEGIN StartTurnTask */
    uint8_t turnBuffer[TURN_BUFFER_SIZE];
    int16_t turnData[2];
    /* Infinite loop */
    for (;;)
    {
        if (osMessageQueueGet(Turn_QueueHandle, turnBuffer, NULL, osWaitForever) == osOK)
        {
            memcpy(&turnData[0], turnBuffer, sizeof(int16_t));
            memcpy(&turnData[1], turnBuffer + sizeof(int16_t), sizeof(int16_t));
            // 处理转向数据
            // printf("Turn Data: %d, %d\r\n", turnData[0], turnData[1]);
        };
        osDelay(10);
    }
  /* USER CODE END StartTurnTask */
}

/* USER CODE BEGIN Header_StartStatuTask */
/**
 * @brief Function implementing the StatuTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartStatuTask */
__weak void StartStatuTask(void *argument)
{
  /* USER CODE BEGIN StartStatuTask */
    uint8_t statusBuffer[STATUS_BUFFER_SIZE];
    uint8_t statusData[2];
    /* Infinite loop */
    for (;;)
    {
        if (osMessageQueueGet(Statu_QueueHandle, statusBuffer, NULL, osWaitForever) == osOK)
        {
            memcpy(&statusData[0], statusBuffer, sizeof(uint8_t));
            memcpy(&statusData[1], statusBuffer + sizeof(uint8_t), sizeof(uint8_t));
            // 处理状态数据
            // printf("Status Data: %d, %d\r\n", statusData[0], statusData[1]);
        };
    }
  /* USER CODE END StartStatuTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint8_t Message_Remote[18] = {0};
    RC_ctrl_t rc_control = {0};
    if (huart->Instance == USART3 && Size > 0 && Size <= sizeof(receiveData))
    {
        memcpy(Message_Remote, receiveData, Size);
        Message_Remote_to_rc(Message_Remote, &rc_control);

        // 调试显示
        // num++;
        // if (num >= 25)
        // {
        //     printf("Raw data: ");
        //     for (int i = 0; i < Size; i++)
        //     {
        //         printf("%02X ", receiveData[i]);
        //     }
        //     printf("\r\n");
        //     printf("RC Channels: %d,%d,%d,%d,%d\n",
        //            rc_control.rc.ch[0], rc_control.rc.ch[1],
        //            rc_control.rc.ch[2], rc_control.rc.ch[3], rc_control.rc.ch[4]);
        //     printf("Switch: %d,%d\n",
        //            rc_control.rc.s[0], rc_control.rc.s[1]);
        //     num = 0;
        // }

        // 创建三个独立的缓冲区
        uint8_t moveBuffer[MOVE_BUFFER_SIZE];
        uint8_t turnBuffer[TURN_BUFFER_SIZE];
        uint8_t statusBuffer[STATUS_BUFFER_SIZE];

        // 移动数据
        memcpy(moveBuffer, &rc_control.rc.ch[0], sizeof(int16_t));
        memcpy(moveBuffer + sizeof(int16_t), &rc_control.rc.ch[1], sizeof(int16_t));
        osMessageQueuePut(Move_R_QueueHandle, moveBuffer, 0, 0);

        // 转向数据
        memcpy(turnBuffer, &rc_control.rc.ch[2], sizeof(int16_t));
        memcpy(turnBuffer + sizeof(int16_t), &rc_control.rc.ch[3], sizeof(int16_t));
        osMessageQueuePut(Turn_QueueHandle, turnBuffer, 0, 0);
        osMessageQueuePut(Move_L_QueueHandle, turnBuffer, 0, 0);

        // 状态数据
        memcpy(statusBuffer, &rc_control.rc.s[0], sizeof(uint8_t));
        memcpy(statusBuffer + sizeof(uint8_t), &rc_control.rc.s[1], sizeof(uint8_t));
        osMessageQueuePut(Statu_QueueHandle, statusBuffer, 0, 0);

        // 清除 IDLE 中断标志
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        // 重新启动IT接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receiveData, sizeof(receiveData));

        // 禁止半传送中断
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
}

int _write(int fd, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END Application */

