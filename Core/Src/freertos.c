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
RC_ctrl_t global_rc_control; // 全局遥控器数据
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void RC_Data_Print(RC_ctrl_t *rc_data);
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
/* Definitions for RemoteTask */
osThreadId_t RemoteTaskHandle;
const osThreadAttr_t RemoteTask_attributes = {
  .name = "RemoteTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CtoCTask */
osThreadId_t CtoCTaskHandle;
const osThreadAttr_t CtoCTask_attributes = {
  .name = "CtoCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for rcDataQueue */
osMessageQueueId_t rcDataQueueHandle;
const osMessageQueueAttr_t rcDataQueue_attributes = {
  .name = "rcDataQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDebugTask(void *argument);
void StartRemoteTask(void *argument);
void StartCtoCTask(void *argument);

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
  /* creation of rcDataQueue */
  rcDataQueueHandle = osMessageQueueNew (1, sizeof(RC_ctrl_t), &rcDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DebugTask */
  DebugTaskHandle = osThreadNew(StartDebugTask, NULL, &DebugTask_attributes);

  /* creation of RemoteTask */
  RemoteTaskHandle = osThreadNew(StartRemoteTask, NULL, &RemoteTask_attributes);

  /* creation of CtoCTask */
  CtoCTaskHandle = osThreadNew(StartCtoCTask, NULL, &CtoCTask_attributes);

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
    osDelay(100);
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

/* USER CODE BEGIN Header_StartRemoteTask */
/**
* @brief Function implementing the RemoteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRemoteTask */
void StartRemoteTask(void *argument)
{
  /* USER CODE BEGIN StartRemoteTask */
  RC_ctrl_t current_rc_data = {0};
  uint8_t num = 0;
  osDelay(100);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receiveData, sizeof(receiveData));
  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(rcDataQueueHandle, &current_rc_data, NULL, osWaitForever) == osOK)
    {
    // 处理遥控器数据
    if (num>20)
    {
    RC_Data_Print(&current_rc_data);
    printf("Remote Control Data Received\n");
    num = 0;
    }
    num++;
    }
  }
  /* USER CODE END StartRemoteTask */
}

/* USER CODE BEGIN Header_StartCtoCTask */
/**
* @brief Function implementing the CtoCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCtoCTask */
void StartCtoCTask(void *argument)
{
  /* USER CODE BEGIN StartCtoCTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCtoCTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        // 直接解析到全局变量
        Message_Remote_to_rc(receiveData, &global_rc_control);
        
        // 通过队列发送数据
        if (rcDataQueueHandle != NULL)
        {
            osMessageQueuePut(rcDataQueueHandle, &global_rc_control, 0, 0);
        }
        
        // 清除IDLE中断标志
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        // 重新启动DMA接收
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

void RC_Data_Print(RC_ctrl_t *rc_data)
{
    if (rc_data == NULL) return;
    
    printf("RC Channels: %d,%d,%d,%d,%d\n",
           rc_data->rc.ch[0], rc_data->rc.ch[1],
           rc_data->rc.ch[2], rc_data->rc.ch[3], rc_data->rc.ch[4]);
    printf("Switch: %d,%d\n",
           rc_data->rc.s[0], rc_data->rc.s[1]);
    // printf("Mouse: x=%d,y=%d,z=%d,press=%d,%d\n",
    //        rc_data->mouse.x, rc_data->mouse.y, rc_data->mouse.z,
    //        rc_data->mouse.press_l, rc_data->mouse.press_r);
    printf("\n");
}
/* USER CODE END Application */

