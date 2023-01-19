/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"
#include "M_PID.h"
#include "M_CAN_FilterSetup.h"
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

/* USER CODE BEGIN PV */
uint32_t adcBuff[5];
uint16_t adcRead[5];
int16_t adcDiff;

float voltage[5];



float scale = 0.5;
float scale2 = 0.2;

float VSupply = 28;
float MaxVmotor = 12.0;

int pwmMax = 400;

float diffFilter,diffOffset;

float maxVoltage = 0;
float potDeg,potDegFilt;

float spPos=0,error_pos;
float kp=0.5,propo;
float PID_eps;

int mode;

M_PID_Typedef PID_Posisi = { .Kp = 1.5, .Ki = 0.15, .Kd = 0.0, .TimeSampling =
		0.001, .ModePID = M_PID_MODE_NORMAL, .Max_Output = 250 };


M_CAN_Filter_Typedef CAN_Rx_Filter;

short int steer_encoder;
short int steer_SP_Pos;

CAN_RxHeaderTypeDef RxMsg;
CAN_TxHeaderTypeDef TxMsg;

uint8_t CAN_RxData[8];
uint8_t CAN_TxData[8];


//uint32_t DeviceID = 3; // Steering
uint32_t DeviceID = 1; // MKiri

uint32_t TxMailbox;

uint32_t tickEPS;

int maxDeg = 230;
int minDeg = 90;

uint8_t epsEMGC;

float middleSteerADC = 1980;
float adctoEnc;
float posSteerADC;
float posSteerFusion;

int testPWM=250;

int PWM_out;

uint32_t timeS;

int16_t ENC_Abs;

int16_t middle_ADC = 1600;
int16_t ADC_To_ENC;
int16_t ENC_ABS_Filt;

int16_t ENC_Fusion;

uint32_t time_fuse;

int16_t MaxPos = 8000;

int16_t PWM_CAN;

// Maximal 8000

int64_t Encoder_Jarak;
int16_t prev_encoder;
int16_t speed_encoder;

uint8_t status_kalib;
uint8_t cnt_kalib;

int16_t MaxAtas = 1650;

int16_t Encoder_Scale;

uint8_t Kalib_CAN;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void motor_pwm(int pwm);
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, adcBuff, 5);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);


  M_CAN_Filter_Init(&CAN_Rx_Filter, &hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  TxMsg.DLC = 8;
  TxMsg.ExtId = 0;
  TxMsg.IDE = CAN_ID_STD;
  TxMsg.RTR = CAN_RTR_DATA;
  TxMsg.StdId = 0x580 + DeviceID;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(HAL_GetTick() - time_fuse > 20)
	  {



		  time_fuse = HAL_GetTick();

		  speed_encoder = __HAL_TIM_GET_COUNTER(&htim1);
		  TIM1->CNT = 0;

		  Encoder_Jarak += speed_encoder;
		  Encoder_Scale = Encoder_Jarak  * 0.001;

		  if(status_kalib==0)
		  {
			  motor_pwm(-100);
			  if(abs(speed_encoder)<50)
				  cnt_kalib++;
			  else
				  cnt_kalib=0;

			  if(cnt_kalib>50)
			  {
				  cnt_kalib=0;
				  status_kalib=1;
				  Encoder_Jarak = 0;
				  motor_pwm(0);
			  }

		  }
		  else if(status_kalib == 1)
		  {


			  if(Kalib_CAN == 1)
			  {
//				  status_kalib = 0;
			  }

			  if(Encoder_Scale > 1650)
			  {
				  if(PWM_CAN>0)
				  {
					  PWM_out = 0;
				  }
				  else
				  {
					  PWM_out = PWM_CAN;
				  }
			  }
			  else if(Encoder_Scale < -10)
			  {
				  if(PWM_CAN<0)
				  {
					  PWM_out = 0;
				  }
				  else
				  {
					  PWM_out = PWM_CAN;
				  }
			  }
			  else
			  {
				  PWM_out = PWM_CAN;
			  }









			  if(PWM_out>pwmMax)
				  PWM_out = pwmMax;
			  else if(PWM_out<-pwmMax)
				  PWM_out = -pwmMax;




			  motor_pwm(PWM_out);
		  }

	  }



	  if(HAL_GetTick()-timeS>=200)
	  {
		  timeS = HAL_GetTick();


		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);

	  }



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adcRead[0] = (adcBuff[0] >> 0)  & 0xFFFF;
	adcRead[1] = (adcBuff[0] >> 16) & 0xFFFF;
	adcRead[2] = (adcBuff[1] >> 0)  & 0xFFFF;
	adcRead[3] = (adcBuff[1] >> 16) & 0xFFFF;
	adcRead[4] = (adcBuff[2] >> 0)  & 0xFFFF;


}

void motor_pwm(int pwm)
{
	if (pwm > 0) {
		HAL_GPIO_WritePin(Dir1_GPIO_Port, Dir1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Dir2_GPIO_Port, Dir2_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(Dir1_GPIO_Port, Dir1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Dir2_GPIO_Port, Dir2_Pin, GPIO_PIN_RESET);
	}

	TIM3->CCR1 = abs(pwm);

}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMsg, CAN_RxData);
		if(RxMsg.StdId == DeviceID+0x600)
		{
			if(CAN_RxData[0]==0)
			{
//				memset(CAN_TxData,0,8);
//				memcpy(CAN_TxData,adcRead,8);
//				HAL_CAN_AddTxMessage(hcan, &TxMsg, CAN_TxData, &TxMailbox);
			}
			else if(CAN_RxData[0]==1)
			{
//				memset(CAN_TxData,0,8);
//				memcpy(CAN_TxData,&adcRead[4],2);
//				memcpy(CAN_TxData+2,&steer_encoder,2);
//				HAL_CAN_AddTxMessage(hcan, &TxMsg, CAN_TxData, &TxMailbox);
			}
			else if(CAN_RxData[0]==2)
			{
				memcpy(&PWM_CAN,CAN_RxData+1,2);
				Kalib_CAN = CAN_RxData[2];

				memset(CAN_TxData,0,8);
				memcpy(CAN_TxData,&status_kalib,1);
				memcpy(CAN_TxData+2,&Encoder_Scale,2);
				HAL_CAN_AddTxMessage(hcan, &TxMsg, CAN_TxData, &TxMailbox);
			}

		}

	}


}




/* USER CODE END 4 */

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
