/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;
typedef struct{
	uint32_t counter;
	uint32_t new_counter;
	uint8_t counter_status;
	float speed;
	float rdps;
	double distant;
}Encoder;
Encoder encoder0;
Encoder encoder1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1456
#define r 0.03 //[Radius m]
#define CPR_X 1440
#define CPR_Y 1440
#define sampling_time  10 //[10ms]
#define dt 0.01
#define rotary1 0
#define rotary2 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//----------CAN----------------//
uint8_t TxData[6];
uint32_t TxMailbox;
uint16_t V1_out = 0;
uint16_t V2_out = 0;
uint16_t V3_out = 0;
uint16_t V4_out = 0;
uint16_t Vx = 0;
uint16_t Vy = 0;
uint16_t Omega = 0;
float Speed;

uint8_t RxData[8];
//float V;
float W1;
float W2;
float X_enR;
float Y_enR;
float Yaw_enR;
float yaw_enR;
float X_old_enR;
float Y_old_enR;
float Vx_enR;
float Vy_enR;

// CAN RX
float RxData1 = 0;
float RxData2 = 0;
float RxData3 = 0;
float RxData4 = 0;
uint8_t datacheck = 0;
uint8_t cntt;
float V1_back;
float V2_back;
float V3_back;
float V4_back;


//Varible to read Encoder through External Interrupt
float nowA[4];
float nowB[4];
float lastA[4];
float lastB[4];
float dir[4];
float cnt[4];
float Enc_count[4];
float X_distance;
float Y_distance;
float Encoder_distance;
float old_count[2];
float new_count[2];
float wheel_velocity_encoder[2];

//laser
uint16_t AD_RES[2];
uint16_t cal_adc = 0;
extern double theta;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
float map(float Input, float Min_Input , float Max_Input ,float Min_Output, float Max_Output){

	return (float) ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	cntt++;
	while (cntt - 100 > 0) {
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		cntt = 0;
	}

	if (RxHeader.StdId == 0x215) {
			RxData1 = (RxData[0] << 8) | RxData[1];
			RxData2 = (RxData[2] << 8) | RxData[3];
			V1_back = map(RxData1, 0, 65535, -30.0, 30.0);
			V2_back = map(RxData2, 0, 65535, -30.0, 30.0);
			datacheck = 1;
	}
	else if (RxHeader.StdId == 0x211) {
			RxData3 = (RxData[0] << 8) | RxData[1];
			RxData4 = (RxData[2] << 8) | RxData[3];
			V3_back = map(RxData3, 0, 65535, -30.0, 30.0);
			V4_back = map(RxData4, 0, 65535, -30.0, 30.0);
			datacheck = 1;
	}
}



float encoder(int i)
{
	if (nowA[i] != lastA[i])
	{
		lastA[i] = nowA[i];
		if (lastA[i] == 0)
		{
			if (nowB[i] == 0)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
		else
		{
			if (nowB[i] == 1)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
	}
	if (nowB[i] != lastB[i])
	{
		lastB[i] = nowB[i];
		if (lastB[i] == 0)
		{
			if (nowA[i] == 1)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
		else
		{
			if (nowA[i] == 0)
			{
				dir[i] = 0;
				cnt[i]--;
			}
			else
			{
				dir[i] = 1;
				cnt[i]++;
			}
		}
	}
	return cnt[i];
}

float Speed_encoder(int i, float CPR)
{
	new_count[i] = Enc_count[i];
	wheel_velocity_encoder[i] = (2.0 *PI *(new_count[i] - old_count[i]) * r) / (CPR * dt);
	old_count[i] = new_count[i];
	return wheel_velocity_encoder[i];
}
void read_encoder(Encoder *enc, TIM_HandleTypeDef* timer){
	enc->new_counter = __HAL_TIM_GET_COUNTER(timer);
	enc->counter_status = __HAL_TIM_IS_TIM_COUNTING_DOWN(timer);
	int16_t count_change = enc->new_counter - enc->counter;
	if(enc->counter_status && count_change <0){
		count_change += 65536;
	}else if (!enc->counter_status && count_change > 0){
		count_change -= 65536;
	}
	enc->counter = enc->new_counter;
	enc->counter_status = (count_change >=0);
	enc->speed = (float)count_change*1000.0f/(CPR_X * sampling_time);
	enc->rdps = (float)count_change*2*PI*1000.0f/(CPR_X * sampling_time);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // CAN _Transmition
  	HAL_CAN_Start(&hcan1);
  	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  	TxHeader.DLC =6; // data length
  	TxHeader.IDE = CAN_ID_STD;
  	TxHeader.RTR = CAN_RTR_DATA;
  	TxHeader.StdId = 0x407; //Id 0x7FF
  	// TIMER Internal clock
  	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
  	HAL_TIM_Base_Start_IT(&htim3);

  	//ADC
//  	HAL_ADCEx_(&hadc1);
//  	HAL_ADC_Start_DMA(&hadc1, &AD_RES, 2);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// ENCODER Rotary 1
	if (GPIO_Pin == R1_C1_Pin || R1_C2_Pin)
	{
		nowA[0] = HAL_GPIO_ReadPin(R1_C1_GPIO_Port, R1_C1_Pin);
		nowB[0] = HAL_GPIO_ReadPin(R1_C2_GPIO_Port, R1_C2_Pin);
		Enc_count[0] = encoder(0);

	}
	// ENCODER Rotary 2
//	if (GPIO_Pin == E2_C1_Pin || E2_C2_Pin)
//	{
//		nowA[1] = HAL_GPIO_ReadPin(E2_C1_GPIO_Port, E2_C1_Pin);
//		nowB[1] = HAL_GPIO_ReadPin(E2_C2_GPIO_Port, E2_C2_Pin);
//		Enc_count[1] = encoder(1);
//	}

}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM3)
	  {
		read_encoder(&encoder0, &htim1);
		W1 = -1*Speed_encoder(rotary1, CPR_X);
		W2 = encoder0.rdps * r;
		Vx_enR = W1 * cosf(theta) - W2 * sinf(theta);
		Vy_enR = W1 * sinf(theta) + W2 * cosf(theta);

		X_enR = X_old_enR + Vx_enR * dt;
		Y_enR = Y_old_enR + Vy_enR * dt;
		X_old_enR = X_enR;
		Y_old_enR = Y_enR;


		Vx = map(Vx_enR, -30.0, 30.0, 0, 65535);
		Vy = map(Vy_enR, -30.0, 30.0, 0, 65535);
		Omega = map(theta, -6.28, 6.28, 0, 65535);

		TxData[0] = ((Vx & 0xFF00) >> 8);
		TxData[1] = (Vx & 0x00FF);
		TxData[2] = ((Vy & 0xFF00) >> 8);
		TxData[3] = (Vy & 0x00FF);
		TxData[4] = ((Omega & 0xFF00) >> 8);
		TxData[5] = (Omega & 0x00FF);

		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
	  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
