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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct temperature_monitoring {
	SPI_HandleTypeDef * hspi;
	GPIO_TypeDef* CS_GPIOx;
	uint16_t CS_GPIO_Pin;
	uint16_t Temperature;
	float Celsius;
	float Celsius_filtered;
} Temp_monitoring_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THERMOCOUPLES_NUMBER 4
#define FLOW_RATE_COEFFICIENT 5.5
#define STACK_SIZE_NORMAL 512
#define STACK_SIZE_SHORT 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//CMSIS-RTOS2
uint32_t T1_TaskProfiler, T2_TaskProfiler, T3_TaskProfiler, T4_TaskProfiler;

const osThreadAttr_t T1_attr = {
		.name = "Temp1_thread",
		.stack_size = STACK_SIZE_NORMAL,
		.priority = osPriorityNormal
};
const osThreadAttr_t T2_attr = {
		.name = "Temp2_thread",
		.stack_size = STACK_SIZE_NORMAL,
		.priority = osPriorityNormal
};
const osThreadAttr_t T3_attr = {
		.name = "Temp3_thread",
		.stack_size = STACK_SIZE_NORMAL,
		.priority = osPriorityNormal
};
const osThreadAttr_t T4_attr = {
		.name = "Temp1_thread",
		.stack_size = STACK_SIZE_NORMAL,
		.priority = osPriorityNormal
};

const osSemaphoreAttr_t spi_sem_attr = {
		.name = "SPI Semaphore"
};

osSemaphoreId_t spi_sem_id;

const osThreadAttr_t FlowHot_attr = {
		.name = "HotWaterFlow_thread",
		.stack_size = STACK_SIZE_SHORT,
		.priority = osPriorityBelowNormal
};

const osMessageQueueAttr_t pulses_hot_queue_attr = {
		.name = "Hot water queue"
};
osMessageQueueId_t pulses_hot_water_queue_id;
// Temperature monitoring
uint8_t i = 0;
uint8_t temp[2];
uint16_t Temperature[4];
float Cels[4], Cels_filtered[4];
Temp_monitoring_t Thermocouple[THERMOCOUPLES_NUMBER];

// Flow monitoring
/*volatile*/ uint16_t my_counter = 0;
float flow_rate;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void LB_Init_Temp_Monitoring(Temp_monitoring_t * tcouple_array);
void vTempReading(void * pvParameters);
void vFlowReading(void * pvParameters);
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim10);

  LB_Init_Temp_Monitoring(Thermocouple);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  osThreadNew(vTempReading, Thermocouple + 0, &T1_attr);
  osThreadNew(vTempReading, Thermocouple + 1, &T2_attr);
  osThreadNew(vTempReading, Thermocouple + 2, &T3_attr);
  osThreadNew(vTempReading, Thermocouple + 3, &T4_attr);

  osThreadNew(vFlowReading, NULL, &FlowHot_attr);

  spi_sem_id = osSemaphoreNew(1, 1, &spi_sem_attr);

  pulses_hot_water_queue_id = osMessageQueueNew(1, sizeof(uint16_t), &pulses_hot_queue_attr);


  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  /* Flow rate */
//	  flow_rate = (float) my_counter / FLOW_RATE_COEFFICIENT;
//
//	  /* Time delay */
//	  HAL_Delay(100);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
void LB_Init_Temp_Monitoring(Temp_monitoring_t * tcouple_array)
{
	(tcouple_array + 0)->hspi = &hspi1;
	(tcouple_array + 0)->CS_GPIOx =	CS_T1_GPIO_Port;
	(tcouple_array + 0)->CS_GPIO_Pin = CS_T1_Pin;

	(tcouple_array + 1)->hspi = &hspi1;
	(tcouple_array + 1)->CS_GPIOx =	CS_T2_GPIO_Port;
	(tcouple_array + 1)->CS_GPIO_Pin = CS_T2_Pin;

	(tcouple_array + 2)->hspi = &hspi1;
	(tcouple_array + 2)->CS_GPIOx =	CS_T3_GPIO_Port;
	(tcouple_array + 2)->CS_GPIO_Pin = CS_T3_Pin;

	(tcouple_array + 3)->hspi = &hspi1;
	(tcouple_array + 3)->CS_GPIOx =	CS_T4_GPIO_Port;
	(tcouple_array + 3)->CS_GPIO_Pin = CS_T4_Pin;
}

void vTempReading(void * pvParameters)
{
	Temp_monitoring_t * p_tcouple = (Temp_monitoring_t *) pvParameters;
	while(1)
	{
		osSemaphoreAcquire(spi_sem_id, osWaitForever);

		/* SPI */
		HAL_GPIO_WritePin(p_tcouple->CS_GPIOx, p_tcouple->CS_GPIO_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_SPI_Receive(p_tcouple->hspi, temp, 2, 10);
		HAL_GPIO_WritePin(p_tcouple->CS_GPIOx, p_tcouple->CS_GPIO_Pin, GPIO_PIN_SET);

//		osSemaphoreRelease(spi_sem_id);

		/* Temperature Conversion */
		p_tcouple->Temperature = temp[1];
		p_tcouple->Temperature = p_tcouple->Temperature << 8;
		p_tcouple->Temperature = p_tcouple->Temperature + temp[0];
		p_tcouple->Temperature = p_tcouple->Temperature >> 3;
		p_tcouple->Celsius = (float) p_tcouple->Temperature * 0.25;

		/* IIR filter */
		p_tcouple->Celsius_filtered = (1 - 0.2) * p_tcouple->Celsius_filtered + 0.2 * p_tcouple->Celsius;

		osSemaphoreRelease(spi_sem_id);
		osDelay(250);
	}
}

void vFlowReading(void * pvParameters)
{
	uint16_t pulse_counter;
	while(1)
	{
		osMessageQueueGet(pulses_hot_water_queue_id, &pulse_counter, NULL, osWaitForever);
		flow_rate = (float) pulse_counter / FLOW_RATE_COEFFICIENT;
		osDelay(111);
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (TIM2 == htim->Instance) {
    	my_counter++;
    	osMessageQueuePut(pulses_hot_water_queue_id, &my_counter, 0, 0);
    }
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//
//	if (TIM10 == htim->Instance)
//	{
//		my_counter = 0;
//	}
//}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (TIM1 == htim->Instance) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if (TIM10 == htim->Instance)
	{
		my_counter = 0;
	}
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
