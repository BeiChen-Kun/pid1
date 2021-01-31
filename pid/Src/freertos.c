/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "usart.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
QueueHandle_t DataQueue;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void DataSend_Task(void const * argument);
void DataRecv_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	DataQueue = xQueueCreate(2,9);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, DataSend_Task, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, DataRecv_Task, osPriorityIdle, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	
	
	PID_Init();
	
  /* Infinite loop */
  for(;;)
  {
		float speed = PID_Calculate(150.0);
		printf("pid: %f\r\n",speed);
		//printf("\r\n%d\r\n",i);
		//i = i + 1;
    osDelay(200);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_DataSend_Task */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataSend_Task */
void DataSend_Task(void const * argument)
{
  /* USER CODE BEGIN DataSend_Task */
	//uint8_t txdata[8] = {0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38};
	uint8_t len;	
  uint16_t times=0;
	//uint8_t i = 0;
  /* Infinite loop */
  for(;;)
  {
		/*
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,0);
    osDelay(500);
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,1);
    osDelay(500);
		*/
		/* 判断是否接收成功 */
		
		//printf("%d\r\n",i);
		//i = i + 1;
		//osDelay(500);
		
    if(USART_RX_STA&0x8000)
			{					   
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			//printf("串口接受发送的消息为:\r\n");
			//HAL_UART_Transmit(&huart1,(uint8_t*)USART_RX_BUF,len,1000);	//发送接收到的数据
			while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		//等待发送结束
			//printf("\r\n\r\n");//插入换行
			USART_RX_STA=0;
			}
			else
			{
			times++;  
			} 
			
		
		xQueueSend(DataQueue,USART_RX_BUF,10);
		memset(USART_RX_BUF,0,200);
	  osDelay(1000);
  }
  /* USER CODE END DataSend_Task */
}

/* USER CODE BEGIN Header_DataRecv_Task */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataRecv_Task */
void DataRecv_Task(void const * argument)
{
  /* USER CODE BEGIN DataRecv_Task */
	uint8_t rxdata[200] = {0};
  /* Infinite loop */
  for(;;)
  {

		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,0);
    osDelay(500);
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,1);
    osDelay(500);

		if(pdPASS == xQueueReceive(DataQueue,rxdata,10))
		{
			  if(rxdata[0] != 0x00)
				{
			  printf("\r\n从消息队列中接收到的消息为:\r\n");
		    HAL_UART_Transmit(&huart1,rxdata,sizeof(rxdata),1000);
        printf("\r\n");
				}
		}
		
		
		//osDelay(1000);
  }
  /* USER CODE END DataRecv_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
