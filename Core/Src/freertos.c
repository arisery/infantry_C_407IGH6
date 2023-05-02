/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gimbal.h"
#include"function.h"
#include "INS_task.h"
#include "gimbal.h"
#include "Shoot.h"
#include <chassis.h>
#include "stdio.h"
#include "key.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern float speed_low ;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern chassis_struct_t chassis;
extern gimbal_t gimbal;
float PITCH,ROLL,YAW;
extern TIM_HandleTypeDef htim6,htim8;
extern RC_ctrl_t rc_ctrl;
osThreadId ChassisHandle;
osThreadId ShootHandle;
int temp1,temp2;
 char ptrTaskList[500];
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId GimbalHandle;
uint32_t GimbalBuffer[ 1024 ];
osStaticThreadDef_t GimbalControlBlock;
osThreadId ChassisHandle;
uint32_t myTask04Buffer[ 1024 ];
osStaticThreadDef_t myTask04ControlBlock;
osThreadId myTask05Handle;
uint32_t myTask05Buffer[ 1024 ];
osStaticThreadDef_t myTask05ControlBlock;
osThreadId imuTaskHandle;
uint32_t imuTaskBuffer[ 2048 ];
osStaticThreadDef_t imuTaskControlBlock;
osThreadId Lost_DetectHandle;
osThreadId KEYHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Gimbal_TASK(void const * argument);
void Chassis_Tsak(void const * argument);
void Shoot_TASK(void const * argument);
void INS_task(void const * argument);
void Detect_Task(void const * argument);
extern void Key_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Gimbal */
  osThreadStaticDef(Gimbal, Gimbal_TASK, osPriorityNormal, 0, 1024, GimbalBuffer, &GimbalControlBlock);
  GimbalHandle = osThreadCreate(osThread(Gimbal), NULL);

  /* definition and creation of Chassis */
  osThreadStaticDef(Chassis, Chassis_Tsak, osPriorityNormal, 0, 1024, myTask04Buffer, &myTask04ControlBlock);
  ChassisHandle = osThreadCreate(osThread(Chassis), NULL);

  /* definition and creation of myTask05 */
  osThreadStaticDef(myTask05, Shoot_TASK, osPriorityNormal, 0, 1024, myTask05Buffer, &myTask05ControlBlock);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of imuTask */
  osThreadStaticDef(imuTask, INS_task, osPriorityAboveNormal, 0, 2048, imuTaskBuffer, &imuTaskControlBlock);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* definition and creation of Lost_Detect */
  osThreadDef(Lost_Detect, Detect_Task, osPriorityIdle, 0, 256);
  Lost_DetectHandle = osThreadCreate(osThread(Lost_Detect), NULL);

  /* definition and creation of KEY */
  osThreadDef(KEY, Key_Task, osPriorityAboveNormal, 0, 512);
  KEYHandle = osThreadCreate(osThread(KEY), NULL);

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
  /* Infinite loop */
  for(;;)
  {
//	  osDelay(10);
//	   vTaskList(ptrTaskList);
//	    printf("**********************************************\r\n");
//	    osDelay(10);
//	    printf("Task        State    Prio    Stack    Num\r\n");
//	   // printf("%s",ptrTaskList);
//	    osDelay(10);
//	    HAL_UART_Transmit_DMA(&huart1, ptrTaskList, 500);
//	    osDelay(10);
//	    printf("**********************************************\r\n");
//    osDelay(30000);
//     speed_low = 25;
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Gimbal_TASK */
/**
* @brief Function implementing the Gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_TASK */
void Gimbal_TASK(void const * argument)
{
  /* USER CODE BEGIN Gimbal_TASK */
	printf("Gimbal Task Start...\r\n");
	osDelay(1000);
	gimbal_init();

  /* Infinite loop */
  for(;;)
  {
	//  Toggle_LED_R;
	  gimbal_task();
    osDelay(5);
  }
  /* USER CODE END Gimbal_TASK */
}

/* USER CODE BEGIN Header_Chassis_Tsak */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Tsak */
__weak void Chassis_Tsak(void const * argument)
{
  /* USER CODE BEGIN Chassis_Tsak */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Tsak */
}

/* USER CODE BEGIN Header_Shoot_TASK */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_TASK */
void Shoot_TASK(void const * argument)
{
  /* USER CODE BEGIN Shoot_TASK */
	osDelay(5);
	printf("Shoot Task Start...\r\n");
	shoot_init();
  /* Infinite loop */
  for(;;)
  {
	  shoot_task();
		  osDelay(5);

  }
  /* USER CODE END Shoot_TASK */
}

/* USER CODE BEGIN Header_INS_task */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_task */
__weak void INS_task(void const * argument)
{
  /* USER CODE BEGIN INS_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_task */
}

/* USER CODE BEGIN Header_Detect_Task */
/**
* @brief Function implementing the Lost_Detect thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
__weak void Detect_Task(void const * argument)
{
  /* USER CODE BEGIN Detect_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Detect_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
