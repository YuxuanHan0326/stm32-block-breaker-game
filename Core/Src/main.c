/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "screen.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PADDLE_PWM_INIT 0x14
#define JOYSTICK_LEFT_THR 1748
#define JOYSTICK_RIGHT_THR 2348
#define JOYSTICK_UP_THR 500
#define JOYSTICK_DOWN_THR 3596
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void initialise_game(void);
void render_frame(void);
void game_main(void);
void update_paddle(void);
void update_ball(void);
void user_input_joystick(void);
void user_input_pushbutton(void);
void calculate_blocks(void);
void path_change_joystick(void);
void auto_play(void);


int8_t random_initial_speed(void);
int8_t random_initial_pos(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t NULL_PWM = INIT_PWM;
uint8_t BOUNDARY_PWM =  INIT_PWM;
uint8_t BLOCK_PWM = INIT_PWM;
uint8_t PADDLE_PWM = 0x02;
uint8_t PADDLE_PWM_HIT = 0x5A;
uint8_t PADDLE_PWM_FAIL = 0x02;
uint8_t BALL_PWM = 0x80;

int8_t main_CLK_autoreload = 21;
uint8_t main_CLK_counter = 0;
uint8_t long_press_CLK_autoreload = 30;
uint8_t long_press_CLK_counter = 0;

uint8_t MSG[50] = {'\0'};  // Message
uint16_t VRX;
uint16_t VRY;
uint16_t RNG;
uint16_t rawValues[3] = {2048, 2048};
uint8_t sendBuffer[2];
uint8_t frameBuffer[11] = {0x00};
uint8_t blinkBuffer[11] = {0x00};
uint8_t PWMBuffer[11][8] = {{INIT_PWM}};
uint8_t testBuffer[11] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t pointProperties[13][9] = {0}; // 0: NULL, 1: Boundary, 2: Block, 3: Paddle, 4: Ball, 5: Bottom Line (1, 1) Top Left
uint8_t paddle_loc[2];
uint8_t ball_loc[2];
uint8_t ball_blink_enable = 0;
uint8_t pause_game = 1;
uint8_t auto_play_enable = 0;
uint8_t current_press_state = 1;
uint8_t prev_press_state = 1;
uint8_t prev_press_state_1 = 1;
uint8_t prev_press_state_2 = 1;
uint8_t disable_single_click = 0;
uint8_t score = 0;
int8_t bonus = 0;
int8_t blocks_left = 28;

int8_t Vx;
int8_t Vy;

int8_t fail_flag = 0;
uint8_t breath_request = 0;

uint8_t convCompleted = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	for (uint8_t i=0; i<hadc1.Init.NbrOfConversion; i++){
		VRX = (uint16_t) rawValues[0];
		VRY = (uint16_t) rawValues[1];
		RNG = (uint16_t) rawValues[2];
	}
}


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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) rawValues, 3);


  ISSI_init(&hi2c1);

  initialise_game();
  render_frame();
  ISSI_send_buffer(&hi2c1, frameBuffer);
  ISSI_send_buffer_PWM(&hi2c1, PWMBuffer);
  ISSI_send_buffer_BLINK(&hi2c1, blinkBuffer);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  render_frame();
	  ISSI_send_buffer(&hi2c1, frameBuffer);
	  ISSI_send_buffer_PWM(&hi2c1, PWMBuffer);
	  ISSI_send_buffer_BLINK(&hi2c1, blinkBuffer);

	  if (breath_request == 1){
		  ISSI_set_frame(&hi2c1, 0x01);
		  ISSI_set_frame(&hi2c1, 0x00);
		  breath_request = 0;
	  }


	  HAL_Delay(5);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 838;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1846;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM2) // Push Button Sample CLK and main CLK, T=10ms
	{
		if (main_CLK_autoreload <= 0){
			main_CLK_autoreload = 0;
		}
		if (main_CLK_counter > main_CLK_autoreload){
			game_main();
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			main_CLK_counter = 0;
		}
		if (long_press_CLK_counter > long_press_CLK_autoreload){
			if ((prev_press_state_2 == 1) && (prev_press_state_1 == 0) && (prev_press_state == 0)){
				auto_play_enable = !auto_play_enable;
				breath_request = 1;
				disable_single_click = 1;
			}
			prev_press_state_2 = prev_press_state_1;
			prev_press_state_1 = current_press_state;
			long_press_CLK_counter = 0;
		}

		long_press_CLK_counter++;
		main_CLK_counter++;
		user_input_pushbutton();
	}

	else if (htim->Instance == TIM3) // User Input CLK, T=0.333s
	{
		user_input_joystick();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_13)  // On board push button, only for external interrupt testing purposes
    {
    	memset(MSG, '\0', sizeof(MSG));
    	sprintf(MSG, "Test\r\n");
    	HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
    	pause_game = !pause_game;
    }
}

void initialise_game(void) {
	// 0: NULL, 1: Boundary, 2: Block, 3: Paddle, 4: Ball, 5: Bottom Line, (1, 1) Top Left
	bonus = 1;
	blocks_left = 21;
	for (uint8_t i = 0; i <= 11; i++){
		for (uint8_t j = 0; j <= 8; j++){
			pointProperties[i][j] = 0;
		}
	}
	// Initialise Boundaries
	Vx = random_initial_speed(); Vy = random_initial_speed();
	ball_blink_enable = 0;
	for (uint8_t i = 0; i <= 11; i++) {
		pointProperties[i][0] = 1;
		pointProperties[i][8] = 1;
	}
	for (uint8_t i = 1; i <= 7; i++) {
		pointProperties[0][i] = 1;
	}

	// Initialise Bottom Line
	for (uint8_t i = 0; i <= 8; i++) {
		pointProperties[12][i] = 5;
	}

	// Place Paddle
	paddle_loc[0] = 11; paddle_loc[1] = 4;
	update_paddle();

	// Place Ball
	ball_loc[0] = 7 + random_initial_pos(); ball_loc[1] = 4 + random_initial_pos();
	pointProperties[ball_loc[0]][ball_loc[1]] = 4;

	// Place Blocks
	for (uint8_t i = 1; i <= 7; i++) {
		pointProperties[1][i] = 2;
		pointProperties[2][i] = 2;
		pointProperties[3][i] = 2;
	}
}

void render_frame(void) {
	// 0: NULL, 1: Boundary, 2: Block, 3: Paddle, 4: Ball, 5: Bottom Line, (1, 1) Top Left
	uint8_t write_intermediate = 0x00;
	uint8_t blink_intermediate = 0x00;

	for (uint8_t row = 1; row <= 11; row++) {
		write_intermediate = 0x00;
		blink_intermediate = 0x00;
		for (uint8_t col = 1; col <= 7; col++) {
			switch (pointProperties[row][col]) {
			case 2:
				write_intermediate = write_intermediate + pow(2, col - 1);
				PWMBuffer[row - 1][col - 1] = BLOCK_PWM;
				break;
			case 3:
				write_intermediate = write_intermediate + pow(2, col - 1);
				PWMBuffer[row - 1][col - 1] = PADDLE_PWM;
				break;
			case 4:
				write_intermediate = write_intermediate + pow(2, col - 1);
				PWMBuffer[row - 1][col - 1] = BALL_PWM;
				if (ball_blink_enable){
					blink_intermediate = blink_intermediate + pow(2, col - 1);
				}
				break;
			default:
				PWMBuffer[row - 1][col - 1] = INIT_PWM;
			}
		}
		frameBuffer[row - 1] = write_intermediate;
		blinkBuffer[row - 1] = blink_intermediate;
	}
}

void game_main(void){
	if (pause_game != 1){
		auto_play();
		update_ball();
		calculate_blocks();
	}
}

void update_paddle(void){
	if(pointProperties[paddle_loc[0]][paddle_loc[1] + 2] == 3){
		pointProperties[paddle_loc[0]][paddle_loc[1] + 2] = 0;
	}
	else if (pointProperties[paddle_loc[0]][paddle_loc[1] - 2] == 3){
		pointProperties[paddle_loc[0]][paddle_loc[1] - 2] = 0;
	}
	pointProperties[paddle_loc[0]][paddle_loc[1]] = 3;
	pointProperties[paddle_loc[0]][paddle_loc[1] - 1] = 3;
	pointProperties[paddle_loc[0]][paddle_loc[1] + 1] = 3;
}

void update_ball(void){
	uint8_t flag = 0;
	PADDLE_PWM = PADDLE_PWM_INIT;
	if(pointProperties[ball_loc[0]][ball_loc[1] + Vx] != 0){
		if (pointProperties[ball_loc[0]][ball_loc[1] + Vx] == 2){
			pointProperties[ball_loc[0]][ball_loc[1] + Vx] = 0;  // Clear block
		}
		else if(pointProperties[ball_loc[0]][ball_loc[1] + Vx] == 3){
			PADDLE_PWM = PADDLE_PWM_HIT;
			flag = 1;
		}
		Vx = -Vx;
	}

	if (flag == 1){
		path_change_joystick();
	}

	flag = 0;
	if(pointProperties[ball_loc[0] + Vy][ball_loc[1]] != 0){
		if (pointProperties[ball_loc[0] + Vy][ball_loc[1]] == 2){
			pointProperties[ball_loc[0] + Vy][ball_loc[1]] = 0;  // Clear Block
			Vy = -Vy;
		}
		else if (pointProperties[ball_loc[0] + Vy][ball_loc[1]] == 3) {
			PADDLE_PWM = PADDLE_PWM_HIT;
			Vy = -Vy;
			flag = 1;
		}
		else if (pointProperties[ball_loc[0] + Vy][ball_loc[1]] == 1) {
			Vy = -Vy;
		}
		else if (pointProperties[ball_loc[0] + Vy][ball_loc[1]] == 5) {
			pause_game = 1;
			Vy = 0; Vx = 0;
			ball_blink_enable = 1;
			PADDLE_PWM = PADDLE_PWM_FAIL;
			fail_flag = 1;
			memset(MSG, '\0', sizeof(MSG));
			sprintf(MSG, "You LOOSE ! Score: %d\r\nPress Button to RESTART\r\n\r\n", score);
			HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		}

	}
	else if (pointProperties[ball_loc[0] + Vy][ball_loc[1] + Vx] != 0){
		if (pointProperties[ball_loc[0] + Vy][ball_loc[1] + Vx] == 2){
			pointProperties[ball_loc[0] + Vy][ball_loc[1] + Vx] = 0;  // Clear Block
		}
		else if (pointProperties[ball_loc[0] + Vy][ball_loc[1] + Vx] == 3){
			PADDLE_PWM = PADDLE_PWM_HIT;
			flag = 1;
		}
		Vx = -Vx;
		Vy = -Vy;
	}

	if (flag == 1){
		path_change_joystick();
	}

	// Path not available
	if (pointProperties[ball_loc[0] + Vy][ball_loc[1] + Vx] != 0){
		return;
	}
	pointProperties[ball_loc[0]][ball_loc[1]] = 0;
	ball_loc[0] = ball_loc[0] + Vy;
	ball_loc[1] = ball_loc[1] + Vx;
	pointProperties[ball_loc[0]][ball_loc[1]] = 4;
}

void user_input_joystick(void){
	if (VRX <= JOYSTICK_LEFT_THR){
		if ((paddle_loc[1] > 2) && (pause_game != 1) && (auto_play_enable != 1)){
			paddle_loc[1]--;
			update_paddle();
		}
	}
	else if ((VRX >= JOYSTICK_RIGHT_THR) && (pause_game != 1) && (auto_play_enable != 1)){
		if (paddle_loc[1] < 6){
			paddle_loc[1]++;
			update_paddle();
		}
	}

	if (VRY <= JOYSTICK_UP_THR){
		main_CLK_autoreload--;
		if (main_CLK_autoreload > 0){
			bonus++;
			memset(MSG, '\0', sizeof(MSG));
			sprintf(MSG, "CLK Speed Increased, bonus = %d\r\n\r\n", bonus);
			HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		}
		else{
			memset(MSG, '\0', sizeof(MSG));
			sprintf(MSG, "MAX CLK Speed !!!\r\n\r\n");
			HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
		}

	}
	else if (VRY >= JOYSTICK_DOWN_THR){
		main_CLK_autoreload++;
		bonus--;
		memset(MSG, '\0', sizeof(MSG));
		sprintf(MSG, "CLK Speed Decreased, bonus = %d\r\n\r\n", bonus);
		HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
	}
}

void user_input_pushbutton(void){
	current_press_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
	if ((prev_press_state == 0) && (current_press_state == 1)){
		if (disable_single_click == 1){
			disable_single_click = 0;
		}
		else{
			if (fail_flag == 1){
				initialise_game();
				pause_game = 1;
				score = 0;
				fail_flag = 0;
				main_CLK_autoreload = 20;
			}
			else if (fail_flag == -1){
				initialise_game();
				pause_game = 1;
				fail_flag = 0;
				main_CLK_autoreload = main_CLK_autoreload - 2;
				memset(MSG, '\0', sizeof(MSG));
				sprintf(MSG, "CLK Speed Increased\r\n\r\n");
				HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
			}
			else{
				pause_game = !pause_game;
			}
			PADDLE_PWM = PADDLE_PWM_FAIL;
			breath_request = 1;
		}
	}

	prev_press_state = current_press_state;
}

int8_t random_initial_speed(void){
	uint8_t randomRaw = RNG % 2;
	if (randomRaw == 1){
		return 1;
	}
	else{
		return -1;
	}
}

int8_t random_initial_pos(void){
	uint8_t randomRaw = RNG % 3;
	if (randomRaw == 0){
		return 1;
	}
	else if(randomRaw == 1){
		return -1;
	}
	else{
		return 0;
	}
}

void calculate_blocks(void){
	blocks_left = 0;
	for (uint8_t i = 1; i <= 3; i++){
		for (uint8_t j = 1; j <= 7; j++){
			if (pointProperties[i][j] == 2){
				blocks_left++;
			}
		}
	}
	// Win
	if (blocks_left == 0){
		pause_game = 1;
		Vx = 0; Vy = 0;
		fail_flag = -1;
		score = score + bonus;
		memset(MSG, '\0', sizeof(MSG));
		sprintf(MSG, "You WIN ! Score: %d\r\nPress Button to CONTINUE\r\n\r\n", score);
		HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
	}
}

void path_change_joystick(void){
	if (VRX <= JOYSTICK_LEFT_THR) {
		Vx = -1;
	}
	else if (VRX >= JOYSTICK_RIGHT_THR){
		Vx = 1;
	}
}

void auto_play(void){
	if (auto_play_enable == 1){
		if(ball_loc[1] <= 2){
			paddle_loc[1] = 2;
			update_paddle();
		}
		else if(ball_loc[1] >= 6){
			paddle_loc[1] = 6;
			update_paddle();
		}
		else{
			paddle_loc[1] = ball_loc[1];
			update_paddle();
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
