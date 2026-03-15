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
#include "Motor.h"
#include "bsp_can.h"

#include "Gimbal_CtoC.h"
#include "Gimbal_Control.h"
// #include "Gimbal_SmallYaw.h"
// #include "Gimbal_Pitch.h"
// #include "Gimbal_Yaw.h"
// #include "new_Gimbal_Yaw.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern M6020_Motor Can2_M6020_MotorStatus[7];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t receiveData[18];
RC_ctrl_t global_rc_control; // 全局遥控器数据

int16_t origin_BigYaw_count = 7434;
int16_t origin_SmallYaw_count = 2364;

int16_t now_BigYaw_count = 0;
int16_t now_SmallYaw_count = 0;

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
/* Definitions for CanTask */
osThreadId_t CanTaskHandle;
const osThreadAttr_t CanTask_attributes = {
  .name = "CanTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TOTask */
osThreadId_t TOTaskHandle;
const osThreadAttr_t TOTask_attributes = {
  .name = "TOTask",
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
void StartCanTask(void *argument);
void StartTOTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  Can_Filter_Init();

  // 清除接收缓冲区并开始接收
  memset(receiveData, 0, sizeof(receiveData));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receiveData, sizeof(receiveData));

  //延时确保Can数据接收稳定
  osDelay(50); 
  Gimbal_Control_Init();
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

  /* creation of CanTask */
  CanTaskHandle = osThreadNew(StartCanTask, NULL, &CanTask_attributes);

  /* creation of TOTask */
  TOTaskHandle = osThreadNew(StartTOTask, NULL, &TOTask_attributes);

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
    /* Infinite loop */
    for (;;)
    {
        // printf("hello world\r\n");
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
  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(rcDataQueueHandle, &current_rc_data, NULL, osWaitForever) == osOK)
    {
    // 处理遥控器数据
    if (num>200)
    // if (num>20)
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

/* USER CODE BEGIN Header_StartCanTask */
/**
* @brief Function implementing the CanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTask */
void StartCanTask(void *argument)
{
  /* USER CODE BEGIN StartCanTask */
  osDelay(200);
  /* Infinite loop */
  for(;;)
  {
    Gimbal_CtoC_Remote();
    Motor_6020_Voltage1(0, 0, 0, 0, &hcan2);
    Gimbal_Control_Loop();
    // Gimbal_SmallYaw_Control();

    // Gimbal_Pitch_Control();
    // Motor_6020_Voltage1((int16_t)BigYaw_SpeedPID.OUT, (int16_t)SmallYaw_SpeedPID.OUT, 0, 0, &hcan2);
    osDelay(10);
  }
  /* USER CODE END StartCanTask */
}

/* USER CODE BEGIN Header_StartTOTask */
/**
* @brief Function implementing the TOTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTOTask */
void StartTOTask(void *argument)
{
  /* USER CODE BEGIN StartTOTask */
  osDelay(20);

  /* Infinite loop */
  for(;;)
  {
    now_BigYaw_count = Can2_M6020_MotorStatus[0].Angle;
    now_SmallYaw_count = Can2_M6020_MotorStatus[1].Angle;
    printf("now_BigYaw_count: %d, now_SmallYaw_count: %d\r\n", now_BigYaw_count, now_SmallYaw_count); //最好的外部调试窗口
    osDelay(200);
  }
  /* USER CODE END StartTOTask */
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

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        // 打印错误信息
        printf("UART Error: 0x%02lX\n", huart->ErrorCode);

        // 清除错误标志
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE);
        
        // 重新初始化UART接收
        memset(receiveData, 0, sizeof(receiveData));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receiveData, sizeof(receiveData));
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    // 读取接收到的消息
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        return; // 安全检查

    // 只处理标准帧
    if (rx_header.IDE != CAN_ID_STD)
        return;

    // 根据 CAN 外设实例区分总线
    if (hcan == &hcan1)
    {
        switch (rx_header.StdId)
			{
					case 0x201:
							CAN1_M3508_DataProcess(0x201,rx_data);break;
					case 0x202:
							CAN1_M3508_DataProcess(0x202,rx_data);break;
					case 0x203:
							break;
					case 0x204:
							break;
					case 0x205:
							break;
					case 0x206:
							CAN1_M6020_DataProcess(0x206,rx_data);break;
					case 0x207:
							CAN1_M2006_DataProcess(0x207,rx_data);break;
					case 0x208:
							break;
					default:
					{
							break;
					}
			}
    }
    else if (hcan == &hcan2)
    {
				switch (rx_header.StdId)
			{
					case 0x201:
							break;
					case 0x202:
							break;
					case 0x203:
							break;
					case 0x204:
							break;
					case 0x205:
							CAN2_M6020_DataProcess(0x205,rx_data);break;
					case 0x206:
							CAN2_M6020_DataProcess(0x206,rx_data);break;
					case 0x207:
							break;
					case 0x208:
							break;
					case 0x146:
							// CToC_AngleProcess(0x146,rx_data,&Can_BMI088_Data);
              break;
					default:
					{
							break;
					}
			}
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

