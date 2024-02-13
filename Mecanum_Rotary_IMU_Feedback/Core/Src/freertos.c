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
#include "tim.h"
#include "i2c.h"
#include "ssd1306.h"
#include "bno055_stm32.h"
#include "bno055.h"
#include "fonts.h"
#include "math.h"
#include "IIRFilter.h"
#include <stdio.h>
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	double Roll;
	double Pitch;
	double Yaw;
}Rotation;


Rotation Angle;
bno055_vector_t V;
bno055_vector_t Q;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


//Wheel
#define R 0.05
#define lx 0.165  //m
#define ly 0.225  //m
#define d1 0.05
#define d2 0.15

//IMU

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


// IMU
float Radian;
double sinr_cosp;
double cosr_cosp;
double sinp;
double siny_cosp;
double cosy_cosp;
double Heading;
double previous_heading =0.0;
float heading;
double theta;
//float theta;
double angle; // Angle in degrees
double radians;

/*Apply Method IIRfilter*/

float gyroFiltered[3];
float accelFiltered[3];

IIRFilter gyroADC[3];
IIRFilter accelADC[3];

float acc[3];
float gyr[3];
uint8_t Data;
uint8_t Check;


/* USER CODE END Variables */
osThreadId RotaryIMU_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
float Map(float Input, float Min_Input , float Max_Input ,float Min_Output, float Max_Output){

	return (float) ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}

//void read_encoder(Encoder *enc, TIM_HandleTypeDef* timer){
//	enc->new_counter = __HAL_TIM_GET_COUNTER(timer);
//	enc->counter_status = __HAL_TIM_IS_TIM_COUNTING_DOWN(timer);
//	int16_t count_change = enc->new_counter - enc->counter;
//	if(enc->counter_status && count_change <0){
//		count_change += 65536;
//	}else if (!enc->counter_status && count_change > 0){
//		count_change -= 65536;
//	}
//	enc->counter = enc->new_counter;
//	enc->counter_status = (count_change >=0);
//	enc->speed = (float)count_change*1000.0f/(CPR_X * sampling_time);
//	enc->rdps = (float)count_change*2*PI*1000.0f/(CPR_X * sampling_time);
//}

/* USER CODE END FunctionPrototypes */

void RotaryIMU_Init(void const * argument);

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of RotaryIMU_Task */
  osThreadDef(RotaryIMU_Task, RotaryIMU_Init, osPriorityNormal, 0, 2100);
  RotaryIMU_TaskHandle = osThreadCreate(osThread(RotaryIMU_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_RotaryIMU_Init */
/**
  * @brief  Function implementing the RotaryIMU_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_RotaryIMU_Init */
void RotaryIMU_Init(void const * argument)
{
  /* USER CODE BEGIN RotaryIMU_Init */
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();

  /* Infinite loop */
  for(;;)
  {
	  // Qauternion_to_Euler(Angle);
		Q = bno055_getVectorQuaternion();
		// yaw (z-axis rotation)
		siny_cosp = 2 * (Q.w * Q.z + Q.x * Q.y);
		cosy_cosp = 1 - 2 * (Q.y * Q.y + Q.z * Q.z);
		Angle.Yaw = atan2(siny_cosp, cosy_cosp);
		theta = Angle.Yaw;
    osDelay(10);
  }
  /* USER CODE END RotaryIMU_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
