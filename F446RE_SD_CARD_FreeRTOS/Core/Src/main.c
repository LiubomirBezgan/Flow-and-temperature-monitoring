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
//#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
//#include "timers.h"
//#include "queue.h"
#include "semphr.h"
//#include "event_groups.h"

// RTOS
#include "File_Handling_RTOS.h"

// Temperature, humidity and pressure measurement
#include "bme280_add.h"
#include "bme280_defs.h"
#include "bme280.h"

// Time
#include "LB_time.h"

// Standard
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Flow monitoring
#define FLOW_RATE_COEFFICIENT 5.5

// FreeRTOS
#define STACK_SIZE_MEDIUM 512
#define STACK_SIZE_SMALL 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
/* USER CODE BEGIN PV */
uint32_t IdleCounter = 0;
// Temperature monitoring
float TC_1;
float TC_2;
float TC_3;
float TC_4;

// Flow monitoring
uint16_t Flow1_pulse_counter = 0;
uint16_t Flow2_pulse_counter = 0;
float Flow_Rate_1;
float Flow_Rate_2;

// Temperature, humidity and pressure measurement
struct bme280_dev bme280_sens_dev;
struct bme280_data bme280_sens_data;

// SD CARD
bool measurement_is_active = false;
Time_t Measurement_Time;
File_counter_t Measurement_Counter =
		{
				0,
				false
		};
const char * file_name = "Meas";
const char * file_extension = ".csv";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
xTaskHandle IDLE_Task_Handler;

xTaskHandle HUMID_Task_Handler;

xSemaphoreHandle HUMID_Sem;

xTaskHandle FLOW1_Task_Handler;
xTaskHandle FLOW2_Task_Handler;

xTaskHandle TEMP1_Task_Handler;
xTaskHandle TEMP2_Task_Handler;
xTaskHandle TEMP3_Task_Handler;
xTaskHandle TEMP4_Task_Handler;

xSemaphoreHandle TEMP_Sem;

xTaskHandle SDCARD_Task_Handler;

void IDLE_Task (void *argument);
void HUMID_Task (void *argument);
void FLOW1_Task (void *argument);
void FLOW2_Task (void *argument);
void TEMP1_Task (void *argument);
void TEMP2_Task (void *argument);
void TEMP3_Task (void *argument);
void TEMP4_Task (void *argument);
void SDCARD_Task (void *argument);

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
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // Initializations of time
  LB_Init_Time(&Measurement_Time);

  // The initialization of humidity sensor
  BME280_init(&bme280_sens_dev);

  // FreeRTOS
  xTaskCreate(IDLE_Task, "IDLE", STACK_SIZE_SMALL, NULL, 1, &IDLE_Task_Handler);

  xTaskCreate(HUMID_Task, "HUMID", STACK_SIZE_MEDIUM, NULL, 2, &HUMID_Task_Handler);
  HUMID_Sem = xSemaphoreCreateBinary();

  xTaskCreate(FLOW1_Task, "FLOW1", STACK_SIZE_SMALL, NULL, 2, &FLOW1_Task_Handler);
  xTaskCreate(FLOW2_Task, "FLOW2", STACK_SIZE_SMALL, NULL, 2, &FLOW2_Task_Handler);

  xTaskCreate(TEMP1_Task, "TEMP1", STACK_SIZE_MEDIUM, NULL, 2, &TEMP1_Task_Handler);
  xTaskCreate(TEMP2_Task, "TEMP2", STACK_SIZE_MEDIUM, NULL, 2, &TEMP2_Task_Handler);
  xTaskCreate(TEMP3_Task, "TEMP3", STACK_SIZE_MEDIUM, NULL, 2, &TEMP3_Task_Handler);
  xTaskCreate(TEMP4_Task, "TEMP4", STACK_SIZE_MEDIUM, NULL, 2, &TEMP4_Task_Handler);
  TEMP_Sem = xSemaphoreCreateBinary();

  xTaskCreate(SDCARD_Task, "SD", STACK_SIZE_MEDIUM, NULL, 3, &SDCARD_Task_Handler);

  // TIM1
  HAL_TIM_Base_Start_IT(&htim1); // periodic delay timer

  // FreeRTOS
  vTaskStartScheduler();
  /* USER CODE END 2 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 60000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_T4_Pin|GREEN_LED_Pin|CS_T2_Pin|CS_T1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_T3_GPIO_Port, CS_T3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLUE_BUTTON_Pin */
  GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_T4_Pin GREEN_LED_Pin CS_T2_Pin CS_T1_Pin */
  GPIO_InitStruct.Pin = CS_T4_Pin|GREEN_LED_Pin|CS_T2_Pin|CS_T1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_T3_Pin */
  GPIO_InitStruct.Pin = CS_T3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_T3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void IDLE_Task (void *argument)
{
	while(1)
	{
		IdleCounter++;
	}
}

void HUMID_Task (void *argument)
{
	while(1)
	{
		xSemaphoreTake(HUMID_Sem, 2500);
		BME280_read_data(&bme280_sens_dev, &bme280_sens_data);

		vTaskDelay(500UL);
	}
}

void SDCARD_Task (void *argument)
{
	FRESULT f_result;

	while(1)
	{
		if (measurement_is_active)
		{
			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);

			if (FR_OK == (f_result = Mount_SD("/")))
			{
				LB_Times_Ticking(&Measurement_Time);
				char * buffer = pvPortMalloc(MAX_DATA_LEN * sizeof(char));
				sprintf( buffer, "%02u:%02u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f\r\n", Measurement_Time.time[1], Measurement_Time.time[0], TC_1, TC_2, TC_3, TC_4, Flow_Rate_1, Flow_Rate_2, ( (float) bme280_sens_data.humidity / 1024.0f));
				Update_File(file_name, &Measurement_Counter, file_extension, buffer);
				vPortFree(buffer);
				Unmount_SD("/");
			}
		}
		else
		{
			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
			LB_Init_Time(&Measurement_Time);
		}

		vTaskDelay(1000UL);
	}
}

void FLOW1_Task (void *argument)
{
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	while(1)
	{
		Flow_Rate_1 = (float) Flow1_pulse_counter / FLOW_RATE_COEFFICIENT;
		Flow1_pulse_counter = 0;
		vTaskDelay(1000UL);
	}
}

void FLOW2_Task (void *argument)
{
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
	while(1)
	{
		Flow_Rate_2 = (float) Flow2_pulse_counter / FLOW_RATE_COEFFICIENT;
		Flow2_pulse_counter = 0;
		vTaskDelay(1000UL);
	}
}

void TEMP1_Task (void *argument)
{
	uint8_t temp[2];
	uint16_t Temperature;
	float Celsius;

	while(1)
	{
		xSemaphoreTake(TEMP_Sem, 2500);

		/* SPI */
		HAL_GPIO_WritePin(CS_T1_GPIO_Port, CS_T1_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_SPI_Receive(&hspi1, temp, 2, 10);
		HAL_GPIO_WritePin(CS_T1_GPIO_Port, CS_T1_Pin, GPIO_PIN_SET);

//		OPTION 1
		xSemaphoreGive(TEMP_Sem);

		/* Temperature Conversion */
		Temperature = temp[1];
		Temperature = Temperature << 8;
		Temperature = Temperature + temp[0];
		Temperature = Temperature >> 3;
		Celsius = (float) Temperature * 0.25f;

		/* IIR filter */
		TC_1 = (1.0f - 0.2f) * TC_1 + 0.2f * Celsius;

//		OPTION 2
//		xSemaphoreGive(TEMP_Sem);

		osDelay(pdMS_TO_TICKS(333UL));
//		vTaskDelay(333UL);
	}
}

void TEMP2_Task (void *argument)
{
	uint8_t temp[2];
	uint16_t Temperature;
	float Celsius;

	while(1)
	{
		xSemaphoreTake(TEMP_Sem, 2500);

		/* SPI */
		HAL_GPIO_WritePin(CS_T2_GPIO_Port, CS_T2_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_SPI_Receive(&hspi1, temp, 2, 10);
		HAL_GPIO_WritePin(CS_T2_GPIO_Port, CS_T2_Pin, GPIO_PIN_SET);

//		OPTION 1
		xSemaphoreGive(TEMP_Sem);

		/* Temperature Conversion */
		Temperature = temp[1];
		Temperature = Temperature << 8;
		Temperature = Temperature + temp[0];
		Temperature = Temperature >> 3;
		Celsius = (float) Temperature * 0.25f;

		/* IIR filter */
		TC_2 = (1.0f - 0.2f) * TC_2 + 0.2f * Celsius;

//		OPTION 2
//		xSemaphoreGive(TEMP_Sem);

		osDelay(pdMS_TO_TICKS(333UL));
//		vTaskDelay(333UL);
	}
}

void TEMP3_Task (void *argument)
{
	uint8_t temp[2];
	uint16_t Temperature;
	float Celsius;

	while(1)
	{
		xSemaphoreTake(TEMP_Sem, 2500);

		/* SPI */
		HAL_GPIO_WritePin(CS_T3_GPIO_Port, CS_T3_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_SPI_Receive(&hspi1, temp, 2, 10);
		HAL_GPIO_WritePin(CS_T3_GPIO_Port, CS_T3_Pin, GPIO_PIN_SET);

//		OPTION 1
		xSemaphoreGive(TEMP_Sem);

		/* Temperature Conversion */
		Temperature = temp[1];
		Temperature = Temperature << 8;
		Temperature = Temperature + temp[0];
		Temperature = Temperature >> 3;
		Celsius = (float) Temperature * 0.25f;

		/* IIR filter */
		TC_3 = (1.0f - 0.2f) * TC_3 + 0.2f * Celsius;

//		OPTION 2
//		xSemaphoreGive(TEMP_Sem);

		osDelay(pdMS_TO_TICKS(333UL));
//		vTaskDelay(333UL);
	}
}

void TEMP4_Task (void *argument)
{
	uint8_t temp[2];
	uint16_t Temperature;
	float Celsius;

	while(1)
	{
		xSemaphoreTake(TEMP_Sem, 2500);

		/* SPI */
		HAL_GPIO_WritePin(CS_T4_GPIO_Port, CS_T4_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_SPI_Receive(&hspi1, temp, 2, 10);
		HAL_GPIO_WritePin(CS_T4_GPIO_Port, CS_T4_Pin, GPIO_PIN_SET);

//		OPTION 1
		xSemaphoreGive(TEMP_Sem);

		/* Temperature Conversion */
		Temperature = temp[1];
		Temperature = Temperature << 8;
		Temperature = Temperature + temp[0];
		Temperature = Temperature >> 3;
		Celsius = (float) Temperature * 0.25f;

		/* IIR filter */
		TC_4 = (1.0f - 0.2f) * TC_4 + 0.2f * Celsius;

//		OPTION 2
//		xSemaphoreGive(TEMP_Sem);

		osDelay(pdMS_TO_TICKS(333UL));
//		vTaskDelay(333UL);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (BLUE_BUTTON_Pin == GPIO_Pin)
	{
		if (false == measurement_is_active)
		{
			measurement_is_active = true;
		}
		else
		{
			measurement_is_active = false;
			Measurement_Counter.isfound = false;
		}
	}
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
	if (htim->Instance == TIM1)
	{
		// release the semaphore here
		 /* The xHigherPriorityTaskWoken parameter must be initialized to pdFALSE as
		 it will get set to pdTRUE inside the interrupt safe API function if a
		 context switch is required. */
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		xSemaphoreGiveFromISR(HUMID_Sem, &xHigherPriorityTaskWoken);  // ISR SAFE VERSION

		/* Pass the xHigherPriorityTaskWoken value into portEND_SWITCHING_ISR(). If
		 xHigherPriorityTaskWoken was set to pdTRUE inside xSemaphoreGiveFromISR()
		 then calling portEND_SWITCHING_ISR() will request a context switch. If
		 xHigherPriorityTaskWoken is still pdFALSE then calling
		 portEND_SWITCHING_ISR() will have no effect */

		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
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
