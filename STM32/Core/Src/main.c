/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "software_timer.h"
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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM2_Init(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void display7SEG(int num);
void update7SEG(int index);

const int MAX_LED = 4;
int index_led = 0;
int led_buffer[4] = {1, 2, 3, 4};
int hour = 15, min = 8, sec = 50;

void updateClockBuffer();

const int MAX_LED_MATRIX = 8;
int index_matrix = 0;
uint8_t matrix_buffer[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

void enableColM(int index);
void updateLEDMatrix(int index);
void displayLetter(uint8_t *arr);
void enableRowM(int index);


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
  MX_TIM2_Init();
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  setTimer1(100);
  setTimer2(25);
  setTimer3(50);

  uint8_t letterA[8] = {0x00, 0x3F, 0x7F, 0xCC, 0xCC, 0x7F, 0x3F, 0x00};

  displayLetter(letterA);

  int index_col = 0;
  while (1)
  {
	  if (timer1_flag)
	  {
		  setTimer1(100);
		  sec++;
		  HAL_GPIO_TogglePin(DOT_GPIO_Port, DOT_Pin);
		  if (sec >= 60) {
			  sec = 0;
			  min++;
		  }

		  if(min >= 60) {
			  min = 0;
			  hour++;
		  }

		  if(hour >=24) {
			  hour = 0;
		  }

		  updateClockBuffer();
	  }

	  if (timer2_flag)
	  {
		  setTimer2(25);
		  if (index_led >= MAX_LED) index_led = 0;
		  update7SEG(index_led++);
	  }

	  if (timer3_flag)
	  {
		  setTimer3(50);
		  if (index_col >= MAX_LED_MATRIX) index_col = 0;
		  if (index_matrix >= MAX_LED_MATRIX) index_matrix = 0;
		  enableColM(index_col++);
		  updateLEDMatrix(index_matrix++);
	  }


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ENM0_Pin|ENM1_Pin|DOT_Pin|RED_LED_Pin
                          |EN0_Pin|EN1_Pin|EN2_Pin|EN3_Pin
                          |ENM2_Pin|ENM3_Pin|ENM4_Pin|ENM5_Pin
                          |ENM6_Pin|ENM7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, sm_a_Pin|sm_b_Pin|sm_c_Pin|ROW2_Pin
                          |ROW3_Pin|ROW4_Pin|ROW5_Pin|ROW6_Pin
                          |ROW7_Pin|sm_d_Pin|sm_e_Pin|sm_f_Pin
                          |sm_g_Pin|ROW0_Pin|ROW1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ENM0_Pin ENM1_Pin DOT_Pin RED_LED_Pin
                           EN0_Pin EN1_Pin EN2_Pin EN3_Pin
                           ENM2_Pin ENM3_Pin ENM4_Pin ENM5_Pin
                           ENM6_Pin ENM7_Pin */
  GPIO_InitStruct.Pin = ENM0_Pin|ENM1_Pin|DOT_Pin|RED_LED_Pin
                          |EN0_Pin|EN1_Pin|EN2_Pin|EN3_Pin
                          |ENM2_Pin|ENM3_Pin|ENM4_Pin|ENM5_Pin
                          |ENM6_Pin|ENM7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : sm_a_Pin sm_b_Pin sm_c_Pin ROW2_Pin
                           ROW3_Pin ROW4_Pin ROW5_Pin ROW6_Pin
                           ROW7_Pin sm_d_Pin sm_e_Pin sm_f_Pin
                           sm_g_Pin ROW0_Pin ROW1_Pin */
  GPIO_InitStruct.Pin = sm_a_Pin|sm_b_Pin|sm_c_Pin|ROW2_Pin
                          |ROW3_Pin|ROW4_Pin|ROW5_Pin|ROW6_Pin
                          |ROW7_Pin|sm_d_Pin|sm_e_Pin|sm_f_Pin
                          |sm_g_Pin|ROW0_Pin|ROW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	timerRun();
}

void display7SEG(int num)
{
	switch (num)
	{
	case 0:
		HAL_GPIO_WritePin(sm_a_GPIO_Port, sm_a_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_b_GPIO_Port, sm_b_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_c_GPIO_Port, sm_c_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_d_GPIO_Port, sm_d_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_e_GPIO_Port, sm_e_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_f_GPIO_Port, sm_f_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_g_GPIO_Port, sm_g_Pin, GPIO_PIN_SET);
		break;

	case 1:
		HAL_GPIO_WritePin(sm_a_GPIO_Port, sm_a_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_b_GPIO_Port, sm_b_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_c_GPIO_Port, sm_c_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_d_GPIO_Port, sm_d_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_e_GPIO_Port, sm_e_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_f_GPIO_Port, sm_f_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_g_GPIO_Port, sm_g_Pin, GPIO_PIN_SET);
		break;

	case 2:
		HAL_GPIO_WritePin(sm_a_GPIO_Port, sm_a_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_b_GPIO_Port, sm_b_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_c_GPIO_Port, sm_c_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_d_GPIO_Port, sm_d_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_e_GPIO_Port, sm_e_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_f_GPIO_Port, sm_f_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_g_GPIO_Port, sm_g_Pin, GPIO_PIN_RESET);
		break;

	case 3:
		HAL_GPIO_WritePin(sm_a_GPIO_Port, sm_a_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_b_GPIO_Port, sm_b_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_c_GPIO_Port, sm_c_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_d_GPIO_Port, sm_d_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_e_GPIO_Port, sm_e_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_f_GPIO_Port, sm_f_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_g_GPIO_Port, sm_g_Pin, GPIO_PIN_RESET);
		break;

	case 4:
		HAL_GPIO_WritePin(sm_a_GPIO_Port, sm_a_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_b_GPIO_Port, sm_b_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_c_GPIO_Port, sm_c_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_d_GPIO_Port, sm_d_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_e_GPIO_Port, sm_e_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_f_GPIO_Port, sm_f_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_g_GPIO_Port, sm_g_Pin, GPIO_PIN_RESET);
		break;

	case 5:
		HAL_GPIO_WritePin(sm_a_GPIO_Port, sm_a_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_b_GPIO_Port, sm_b_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_c_GPIO_Port, sm_c_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_d_GPIO_Port, sm_d_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_e_GPIO_Port, sm_e_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_f_GPIO_Port, sm_f_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_g_GPIO_Port, sm_g_Pin, GPIO_PIN_RESET);
		break;

	case 6:
		HAL_GPIO_WritePin(sm_a_GPIO_Port, sm_a_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_b_GPIO_Port, sm_b_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_c_GPIO_Port, sm_c_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_d_GPIO_Port, sm_d_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_e_GPIO_Port, sm_e_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_f_GPIO_Port, sm_f_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_g_GPIO_Port, sm_g_Pin, GPIO_PIN_RESET);
		break;

	case 7:
		HAL_GPIO_WritePin(sm_a_GPIO_Port, sm_a_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_b_GPIO_Port, sm_b_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_c_GPIO_Port, sm_c_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_d_GPIO_Port, sm_d_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_e_GPIO_Port, sm_e_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_f_GPIO_Port, sm_f_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_g_GPIO_Port, sm_g_Pin, GPIO_PIN_SET);
		break;

	case 8:
		HAL_GPIO_WritePin(sm_a_GPIO_Port, sm_a_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_b_GPIO_Port, sm_b_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_c_GPIO_Port, sm_c_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_d_GPIO_Port, sm_d_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_e_GPIO_Port, sm_e_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_f_GPIO_Port, sm_f_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_g_GPIO_Port, sm_g_Pin, GPIO_PIN_RESET);
		break;

	case 9:
		HAL_GPIO_WritePin(sm_a_GPIO_Port, sm_a_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_b_GPIO_Port, sm_b_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_c_GPIO_Port, sm_c_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_d_GPIO_Port, sm_d_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_e_GPIO_Port, sm_e_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_f_GPIO_Port, sm_f_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sm_g_GPIO_Port, sm_g_Pin, GPIO_PIN_RESET);
		break;

	default:
		HAL_GPIO_WritePin(sm_a_GPIO_Port, sm_a_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_b_GPIO_Port, sm_b_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_c_GPIO_Port, sm_c_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_d_GPIO_Port, sm_d_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_e_GPIO_Port, sm_e_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_f_GPIO_Port, sm_f_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(sm_g_GPIO_Port, sm_g_Pin, GPIO_PIN_SET);
		break;
	}
}

void update7SEG(int index){
    switch (index){
        case 0:
        	// enable led 0 and disable led 1, 2, 3
        	HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_SET);
            display7SEG(led_buffer[0]);
            break;

        case 1:
        	// enable led 1 and disable led 0, 2, 3
        	HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_SET);
            display7SEG(led_buffer[1]);
            break;

        case 2:
        	// enable led 2 and disable led 0, 1, 3
        	HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_SET);
            display7SEG(led_buffer[2]);
            break;

        case 3:
        	// enable led 3 and disable led 0, 1, 2
        	HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_RESET);
            display7SEG(led_buffer[3]);
            break;

        default:
            break;
    }
}


void updateClockBuffer()
{
	led_buffer[0] = hour/10;
	led_buffer[1] = hour%10;
	led_buffer[2] = min/10;
	led_buffer[3] = min%10;
}

void enableColM(int index)
{
	switch (index)
	{
	case 0:
		HAL_GPIO_WritePin(ENM0_GPIO_Port, ENM0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ENM1_GPIO_Port, ENM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM2_GPIO_Port, ENM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM3_GPIO_Port, ENM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM4_GPIO_Port, ENM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM5_GPIO_Port, ENM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM6_GPIO_Port, ENM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM7_GPIO_Port, ENM7_Pin, GPIO_PIN_RESET);
		break;

	case 1:
		HAL_GPIO_WritePin(ENM0_GPIO_Port, ENM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM1_GPIO_Port, ENM1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ENM2_GPIO_Port, ENM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM3_GPIO_Port, ENM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM4_GPIO_Port, ENM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM5_GPIO_Port, ENM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM6_GPIO_Port, ENM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM7_GPIO_Port, ENM7_Pin, GPIO_PIN_RESET);
		break;

	case 2:
		HAL_GPIO_WritePin(ENM0_GPIO_Port, ENM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM1_GPIO_Port, ENM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM2_GPIO_Port, ENM2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ENM3_GPIO_Port, ENM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM4_GPIO_Port, ENM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM5_GPIO_Port, ENM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM6_GPIO_Port, ENM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM7_GPIO_Port, ENM7_Pin, GPIO_PIN_RESET);
		break;

	case 3:
		HAL_GPIO_WritePin(ENM0_GPIO_Port, ENM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM1_GPIO_Port, ENM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM2_GPIO_Port, ENM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM3_GPIO_Port, ENM3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ENM4_GPIO_Port, ENM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM5_GPIO_Port, ENM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM6_GPIO_Port, ENM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM7_GPIO_Port, ENM7_Pin, GPIO_PIN_RESET);
		break;

	case 4:
		HAL_GPIO_WritePin(ENM0_GPIO_Port, ENM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM1_GPIO_Port, ENM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM2_GPIO_Port, ENM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM3_GPIO_Port, ENM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM4_GPIO_Port, ENM4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ENM5_GPIO_Port, ENM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM6_GPIO_Port, ENM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM7_GPIO_Port, ENM7_Pin, GPIO_PIN_RESET);
		break;

	case 5:
		HAL_GPIO_WritePin(ENM0_GPIO_Port, ENM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM1_GPIO_Port, ENM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM2_GPIO_Port, ENM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM3_GPIO_Port, ENM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM4_GPIO_Port, ENM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM5_GPIO_Port, ENM5_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ENM6_GPIO_Port, ENM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM7_GPIO_Port, ENM7_Pin, GPIO_PIN_RESET);
		break;

	case 6:
		HAL_GPIO_WritePin(ENM0_GPIO_Port, ENM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM1_GPIO_Port, ENM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM2_GPIO_Port, ENM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM3_GPIO_Port, ENM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM4_GPIO_Port, ENM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM5_GPIO_Port, ENM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM6_GPIO_Port, ENM6_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ENM7_GPIO_Port, ENM7_Pin, GPIO_PIN_RESET);
		break;

	case 7:
		HAL_GPIO_WritePin(ENM0_GPIO_Port, ENM0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM1_GPIO_Port, ENM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM2_GPIO_Port, ENM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM3_GPIO_Port, ENM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM4_GPIO_Port, ENM4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM5_GPIO_Port, ENM5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM6_GPIO_Port, ENM6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ENM7_GPIO_Port, ENM7_Pin, GPIO_PIN_SET);
		break;
	}
}

void updateLEDMatrix(int index)
{
	switch (index)
	{
	case 0:
		HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, (GPIO_PinState)((matrix_buffer[0] >> 7) & 0x01));
		HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, (GPIO_PinState)((matrix_buffer[0] >> 6) & 0x01));
		HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, (GPIO_PinState)((matrix_buffer[0] >> 5) & 0x01));
		HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, (GPIO_PinState)((matrix_buffer[0] >> 4) & 0x01));
		HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, (GPIO_PinState)((matrix_buffer[0] >> 3) & 0x01));
		HAL_GPIO_WritePin(ROW5_GPIO_Port, ROW5_Pin, (GPIO_PinState)((matrix_buffer[0] >> 2) & 0x01));
		HAL_GPIO_WritePin(ROW6_GPIO_Port, ROW6_Pin, (GPIO_PinState)((matrix_buffer[0] >> 1) & 0x01));
		HAL_GPIO_WritePin(ROW7_GPIO_Port, ROW7_Pin, (GPIO_PinState)((matrix_buffer[0]) & 0x01));
		break;

	case 1:
		HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, (GPIO_PinState)((matrix_buffer[1] >> 7) & 0x01));
		HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, (GPIO_PinState)((matrix_buffer[1] >> 6) & 0x01));
		HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, (GPIO_PinState)((matrix_buffer[1] >> 5) & 0x01));
		HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, (GPIO_PinState)((matrix_buffer[1] >> 4) & 0x01));
		HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, (GPIO_PinState)((matrix_buffer[1] >> 3) & 0x01));
		HAL_GPIO_WritePin(ROW5_GPIO_Port, ROW5_Pin, (GPIO_PinState)((matrix_buffer[1] >> 2) & 0x01));
		HAL_GPIO_WritePin(ROW6_GPIO_Port, ROW6_Pin, (GPIO_PinState)((matrix_buffer[1] >> 1) & 0x01));
		HAL_GPIO_WritePin(ROW7_GPIO_Port, ROW7_Pin, (GPIO_PinState)((matrix_buffer[1]) & 0x01));
		break;

	case 2:
		HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, (GPIO_PinState)((matrix_buffer[2] >> 7) & 0x01));
		HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, (GPIO_PinState)((matrix_buffer[2] >> 6) & 0x01));
		HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, (GPIO_PinState)((matrix_buffer[2] >> 5) & 0x01));
		HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, (GPIO_PinState)((matrix_buffer[2] >> 4) & 0x01));
		HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, (GPIO_PinState)((matrix_buffer[2] >> 3) & 0x01));
		HAL_GPIO_WritePin(ROW5_GPIO_Port, ROW5_Pin, (GPIO_PinState)((matrix_buffer[2] >> 2) & 0x01));
		HAL_GPIO_WritePin(ROW6_GPIO_Port, ROW6_Pin, (GPIO_PinState)((matrix_buffer[2] >> 1) & 0x01));
		HAL_GPIO_WritePin(ROW7_GPIO_Port, ROW7_Pin, (GPIO_PinState)((matrix_buffer[2]) & 0x01));
		break;

	case 3:
		HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, (GPIO_PinState)((matrix_buffer[3] >> 7) & 0x01));
		HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, (GPIO_PinState)((matrix_buffer[3] >> 6) & 0x01));
		HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, (GPIO_PinState)((matrix_buffer[3] >> 5) & 0x01));
		HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, (GPIO_PinState)((matrix_buffer[3] >> 4) & 0x01));
		HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, (GPIO_PinState)((matrix_buffer[3] >> 3) & 0x01));
		HAL_GPIO_WritePin(ROW5_GPIO_Port, ROW5_Pin, (GPIO_PinState)((matrix_buffer[3] >> 2) & 0x01));
		HAL_GPIO_WritePin(ROW6_GPIO_Port, ROW6_Pin, (GPIO_PinState)((matrix_buffer[3] >> 1) & 0x01));
		HAL_GPIO_WritePin(ROW7_GPIO_Port, ROW7_Pin, (GPIO_PinState)((matrix_buffer[3]) & 0x01));
		break;

	case 4:
		HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, (GPIO_PinState)((matrix_buffer[4] >> 7) & 0x01));
		HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, (GPIO_PinState)((matrix_buffer[4] >> 6) & 0x01));
		HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, (GPIO_PinState)((matrix_buffer[4] >> 5) & 0x01));
		HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, (GPIO_PinState)((matrix_buffer[4] >> 4) & 0x01));
		HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, (GPIO_PinState)((matrix_buffer[4] >> 3) & 0x01));
		HAL_GPIO_WritePin(ROW5_GPIO_Port, ROW5_Pin, (GPIO_PinState)((matrix_buffer[4] >> 2) & 0x01));
		HAL_GPIO_WritePin(ROW6_GPIO_Port, ROW6_Pin, (GPIO_PinState)((matrix_buffer[4] >> 1) & 0x01));
		HAL_GPIO_WritePin(ROW7_GPIO_Port, ROW7_Pin, (GPIO_PinState)((matrix_buffer[4]) & 0x01));
		break;

	case 5:
		HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, (GPIO_PinState)((matrix_buffer[5] >> 7) & 0x01));
		HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, (GPIO_PinState)((matrix_buffer[5] >> 6) & 0x01));
		HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, (GPIO_PinState)((matrix_buffer[5] >> 5) & 0x01));
		HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, (GPIO_PinState)((matrix_buffer[5] >> 4) & 0x01));
		HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, (GPIO_PinState)((matrix_buffer[5] >> 3) & 0x01));
		HAL_GPIO_WritePin(ROW5_GPIO_Port, ROW5_Pin, (GPIO_PinState)((matrix_buffer[5] >> 2) & 0x01));
		HAL_GPIO_WritePin(ROW6_GPIO_Port, ROW6_Pin, (GPIO_PinState)((matrix_buffer[5] >> 1) & 0x01));
		HAL_GPIO_WritePin(ROW7_GPIO_Port, ROW7_Pin, (GPIO_PinState)((matrix_buffer[5]) & 0x01));
		break;

	case 6:
		HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, (GPIO_PinState)((matrix_buffer[6] >> 7) & 0x01));
		HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, (GPIO_PinState)((matrix_buffer[6] >> 6) & 0x01));
		HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, (GPIO_PinState)((matrix_buffer[6] >> 5) & 0x01));
		HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, (GPIO_PinState)((matrix_buffer[6] >> 4) & 0x01));
		HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, (GPIO_PinState)((matrix_buffer[6] >> 3) & 0x01));
		HAL_GPIO_WritePin(ROW5_GPIO_Port, ROW5_Pin, (GPIO_PinState)((matrix_buffer[6] >> 2) & 0x01));
		HAL_GPIO_WritePin(ROW6_GPIO_Port, ROW6_Pin, (GPIO_PinState)((matrix_buffer[6] >> 1) & 0x01));
		HAL_GPIO_WritePin(ROW7_GPIO_Port, ROW7_Pin, (GPIO_PinState)((matrix_buffer[6]) & 0x01));
		break;

	case 7:
		HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, (GPIO_PinState)((matrix_buffer[7] >> 7) & 0x01));
		HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, (GPIO_PinState)((matrix_buffer[7] >> 6) & 0x01));
		HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, (GPIO_PinState)((matrix_buffer[7] >> 5) & 0x01));
		HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, (GPIO_PinState)((matrix_buffer[7] >> 4) & 0x01));
		HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, (GPIO_PinState)((matrix_buffer[7] >> 3) & 0x01));
		HAL_GPIO_WritePin(ROW5_GPIO_Port, ROW5_Pin, (GPIO_PinState)((matrix_buffer[7] >> 2) & 0x01));
		HAL_GPIO_WritePin(ROW6_GPIO_Port, ROW6_Pin, (GPIO_PinState)((matrix_buffer[7] >> 1) & 0x01));
		HAL_GPIO_WritePin(ROW7_GPIO_Port, ROW7_Pin, (GPIO_PinState)((matrix_buffer[7]) & 0x01));
		break;
	}
}

void displayLetter(uint8_t *arr)
{
	for (int i = 0; i < MAX_LED_MATRIX; i++)
	{
		matrix_buffer[i] = arr[i];
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
