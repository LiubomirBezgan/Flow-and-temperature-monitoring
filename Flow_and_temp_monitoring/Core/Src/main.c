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
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Humidity sensor
#include "bme280_add.h"

// SD CARD
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"

// RTOS
#include "File_Handling_RTOS.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Temperature monitoring
typedef struct temperature_monitoring {
	SPI_HandleTypeDef * hspi;
	GPIO_TypeDef* CS_GPIOx;
	uint16_t CS_GPIO_Pin;
	uint16_t Temperature;
	float Celsius;
	float Celsius_filtered;
} Temp_monitoring_t;

// Flow monitoring
typedef struct flow_monitoring {
	TIM_HandleTypeDef *htim;
	volatile uint16_t pulse_counter;
	float flow_rate;
} Flow_monitoring_t;

// SD CARD
typedef struct sd_card_writing {
	char * file_name;
	uint16_t measurement_number;
	Temp_monitoring_t * temp_array;
	Flow_monitoring_t * flow_array;
	struct bme280_data * humid_array;
} SD_card_writing_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Temperature monitoring
#define THERMOCOUPLES_NUMBER 4

// Flow monitoring
#define LIQUID_FLOW_SENSOR_NUMBER 2			// 0 - hot, 1 - cold
#define FLOW_RATE_COEFFICIENT 5.5

// SD CARD
//#define MAX_LEN 37

// CMSIS-RTOS2
#define STACK_SIZE_NORMAL 512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// CMSIS-RTOS2
// Temperature monitoring
const osThreadAttr_t T1_attr = {
		.name = "T1_thread",
		.stack_size = STACK_SIZE_NORMAL,
		.priority = osPriorityNormal
};
const osThreadAttr_t T2_attr = {
		.name = "T2_thread",
		.stack_size = STACK_SIZE_NORMAL,
		.priority = osPriorityNormal
};
const osThreadAttr_t T3_attr = {
		.name = "T3_thread",
		.stack_size = STACK_SIZE_NORMAL,
		.priority = osPriorityNormal
};
const osThreadAttr_t T4_attr = {
		.name = "T1_thread",
		.stack_size = STACK_SIZE_NORMAL,
		.priority = osPriorityNormal
};

const osSemaphoreAttr_t spi_sem_attr = {
		.name = "SPI_Sem"
};

osSemaphoreId_t spi_sem_id;

// Flow monitoring
const osThreadAttr_t FlowHot_attr = {
		.name = "Flow_H_thread",
		.stack_size = STACK_SIZE_NORMAL,
		.priority = osPriorityBelowNormal1
};

const osThreadAttr_t FlowCold_attr = {
		.name = "Flow_C_thread",
		.stack_size = STACK_SIZE_NORMAL,
		.priority = osPriorityBelowNormal1
};

// Temperature, humidity and pressure measurement
const osThreadAttr_t Humid_attr = {
		.name = "Humid_thread",
		.stack_size = STACK_SIZE_NORMAL,
		.priority = osPriorityNormal
};

const osSemaphoreAttr_t i2c_humid_sem_attr = {
		.name = "I2C_Sem"
};

osSemaphoreId_t i2c_humid_sem_id;

// SD CARD
const osThreadAttr_t SD_card_attr = {
		.name = "SD_thread",
		.stack_size = STACK_SIZE_NORMAL,
		.priority = osPriorityNormal1
};

// Temperature monitoring
Temp_monitoring_t Thermocouple[THERMOCOUPLES_NUMBER];

// Flow monitoring
Flow_monitoring_t Liquid_flow_sensor[LIQUID_FLOW_SENSOR_NUMBER];

// Temperature, humidity and pressure measurement
struct bme280_dev bme280_sens_dev;
struct bme280_data bme280_sens_data;
//struct bme280_data bme280_sens_logging_data;
xSemaphoreHandle DATA_Sem;

// SD CARD
SD_card_writing_t SD_card_data;
const char * txt_file_name = "RTOS2.txt";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
// Temperature monitoring
void LB_Init_Temp_Monitoring(Temp_monitoring_t * tcouple_array);
void vTempReading(void * pvParameters);

// Flow monitoring
void LB_Init_Flow_Monitoring(Flow_monitoring_t * flow_sensor_array);
void vFlowReading(void * pvParameters);

// Temperature, humidity and pressure measurement
void vHumidReading(void * pvParameters);

// SD CARD
void LB_Init_SD_Card(SD_card_writing_t * p_SD_card_data_type, Temp_monitoring_t * p_temp, Flow_monitoring_t * p_flow, struct bme280_data * p_humid);
void vUpdateLogs(void * pvParameters);
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
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // The initialization of temperature monitoring
  LB_Init_Temp_Monitoring(Thermocouple);

  // The initialization of flow monitoring
  LB_Init_Flow_Monitoring(Liquid_flow_sensor);

  // The initialization of humidity sensor
  BME280_init(&bme280_sens_dev);

  // The initialization of SD card data type
  LB_Init_SD_Card(&SD_card_data, Thermocouple, Liquid_flow_sensor, &bme280_sens_data);

  // The initialization of timers
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim10);

  // SD card mounting
  Mount_SD("/");
//  Format_SD();
  Create_File(txt_file_name);
  Unmount_SD("/");

  DATA_Sem = xSemaphoreCreateBinary();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  // SAVE THIS CODE BEGIN
  // Temperature monitoring
  osThreadNew(vTempReading, Thermocouple + 0, &T1_attr);
  osThreadNew(vTempReading, Thermocouple + 1, &T2_attr);
  osThreadNew(vTempReading, Thermocouple + 2, &T3_attr);
  osThreadNew(vTempReading, Thermocouple + 3, &T4_attr);

  spi_sem_id = osSemaphoreNew(1, 1, &spi_sem_attr);

  // Flow monitoring
  osThreadNew(vFlowReading, Liquid_flow_sensor + 0, &FlowHot_attr);
  osThreadNew(vFlowReading, Liquid_flow_sensor + 1, &FlowCold_attr);

  // Temperature, humidity and pressure measurement
  osThreadNew(vHumidReading, NULL, &Humid_attr);

  i2c_humid_sem_id = osSemaphoreNew(1, 1, &i2c_humid_sem_attr);

  osThreadNew(vUpdateLogs, &SD_card_data, &SD_card_attr);

  HAL_TIM_Base_Start_IT(&htim1);
  // SAVE THIS CODE END

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
// Temperature monitoring
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
	uint8_t temp[2];
	Temp_monitoring_t * p_tcouple = (Temp_monitoring_t *) pvParameters;
	while(1)
	{
		osSemaphoreAcquire(spi_sem_id, osWaitForever);

		/* SPI */
		HAL_GPIO_WritePin(p_tcouple->CS_GPIOx, p_tcouple->CS_GPIO_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_SPI_Receive(p_tcouple->hspi, temp, 2, 10);
		HAL_GPIO_WritePin(p_tcouple->CS_GPIOx, p_tcouple->CS_GPIO_Pin, GPIO_PIN_SET);

//		OPTION 1
		osSemaphoreRelease(spi_sem_id);

		/* Temperature Conversion */
		p_tcouple->Temperature = temp[1];
		p_tcouple->Temperature = p_tcouple->Temperature << 8;
		p_tcouple->Temperature = p_tcouple->Temperature + temp[0];
		p_tcouple->Temperature = p_tcouple->Temperature >> 3;
		p_tcouple->Celsius = (float) p_tcouple->Temperature * 0.25;

		/* IIR filter */
		p_tcouple->Celsius_filtered = (1 - 0.2) * p_tcouple->Celsius_filtered + 0.2 * p_tcouple->Celsius;

//		OPTION 2
//		osSemaphoreRelease(spi_sem_id);

		osDelay(pdMS_TO_TICKS(333UL));
	}
}

// Flow monitoring
void LB_Init_Flow_Monitoring(Flow_monitoring_t * flow_sensor_array)
{
	(flow_sensor_array + 0)->htim = &htim2;
	(flow_sensor_array + 0)->pulse_counter = 0;
	(flow_sensor_array + 0)->flow_rate = 0;

	(flow_sensor_array + 1)->htim = &htim3;
	(flow_sensor_array + 1)->pulse_counter = 0;
	(flow_sensor_array + 1)->flow_rate = 0;
}

void vFlowReading(void * pvParameters)
{
	Flow_monitoring_t * p_flow_sensor = (Flow_monitoring_t *) pvParameters;
	while(1)
	{
		p_flow_sensor->flow_rate = (float) p_flow_sensor->pulse_counter / FLOW_RATE_COEFFICIENT;
		osDelay(pdMS_TO_TICKS(111UL));
	}
}

// Temperature, humidity and pressure measurement
void vHumidReading(void * pvParameters)
{
	while(1)
	{
//		osSemaphoreAcquire(spi_sem_id, osWaitForever);
		xSemaphoreTake(DATA_Sem, 2500);
		BME280_read_data(&bme280_sens_dev, &bme280_sens_data);
		osDelay(pdMS_TO_TICKS(500UL));
//		osDelay(pdMS_TO_TICKS(444UL));
	}
}

// SD CARD
void LB_Init_SD_Card(SD_card_writing_t * p_SD_card_data_type, Temp_monitoring_t * p_temp, Flow_monitoring_t * p_flow, struct bme280_data * p_humid)
{
	p_SD_card_data_type->file_name = "Measurement";
	p_SD_card_data_type->measurement_number = 1;
	p_SD_card_data_type->temp_array = p_temp;
	p_SD_card_data_type->flow_array = p_flow;
	p_SD_card_data_type->humid_array = p_humid;
}

void vUpdateLogs(void * pvParameters)
{
//	FATFS fs;
//	FIL fil;
//	FRESULT fresult;
//	uint8_t message[MAX_LEN];
//
//	SD_card_writing_t * p_sd_card_data_type = (SD_card_writing_t *) pvParameters;
//
//	while(1)
//	{
//		// Mount SD Card
//		if ( FR_DISK_ERR == (fresult = f_mount(&fs, "", 1)) )
//		{
//			FATFS_UnLinkDriver(USERPath);
//			MX_FATFS_Init();
//			if ( FR_OK != (fresult = f_mount(&fs, "", 1)) )
//			{
//				fresult = f_mount(NULL, "", 1);
//				osDelay(pdMS_TO_TICKS(1000UL));
////				return fresult;
//			}
//		}
//
//		// Open the file with write access
//		if ( FR_OK != (fresult = f_open(&fil, p_sd_card_data_type->file_name, FA_OPEN_EXISTING | FA_WRITE)) )
//		{
//			if ( FR_OK != (fresult = f_open(&fil, p_sd_card_data_type->file_name, FA_OPEN_ALWAYS | FA_WRITE)) )
//			{
//				// Unmount SD CARD
//				fresult = f_mount(NULL, "", 1);
//				osDelay(pdMS_TO_TICKS(1000UL));
////				return fresult;
//			}
//			fresult = f_puts("Date,Time,Temperature [C],Humidity [%], Pressure [mmHg]\r\n\r\n", &fil);
//		}
//
//		// Move offset to the end of file
//		fresult = f_lseek(&fil, /*fil.fptr*/ f_size(&fil));
//
//		// an action
//		p_sd_card_data_type->measurement_number++;
//
//		/*** Updating an existing file ***/
//		// Write a string to the file
//		sprintf( (char *) message, "%u\r\n",  p_sd_card_data_type->measurement_number);
//		fresult = f_puts((TCHAR *) message, &fil);
//
//		// Close the file
//		f_close(&fil);
//
//		// Unmount SD CARD
//		fresult = f_mount(NULL, "", 1);
//
//		osDelay(pdMS_TO_TICKS(1000UL));
//	}

	int index = 1;

	while(1)
	{
		char * buffer = pvPortMalloc(50 * sizeof(char));
		sprintf(buffer, /*"%03d. - %02f, %02f, %02f, %02f, %02f, %02f %lu.%02lu\r\n"*/"%03d. - %lu.%02lu\r\n", index, /*Thermocouple[1].Celsius_filtered, Thermocouple[2].Celsius_filtered, Thermocouple[3].Celsius_filtered, Thermocouple[0].Celsius_filtered, Liquid_flow_sensor[0].flow_rate, Liquid_flow_sensor[1].flow_rate,*/ (bme280_sens_data.humidity / 1024UL), ((bme280_sens_data.humidity % 1024UL) / 10));
		Mount_SD("/");
		Update_File(txt_file_name, buffer);
		vPortFree(buffer);
		Unmount_SD("/");

		index++;

		vTaskDelay(1000);
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (TIM2 == htim->Instance) {
    	Liquid_flow_sensor[0].pulse_counter++;
    }
    if (TIM3 == htim->Instance) {
    	Liquid_flow_sensor[1].pulse_counter++;
    }
}

//}
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

		xSemaphoreGiveFromISR(DATA_Sem, &xHigherPriorityTaskWoken);  // ISR SAFE VERSION

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
	if (TIM10 == htim->Instance)
	{
    	Liquid_flow_sensor[0].pulse_counter = 0;
    	Liquid_flow_sensor[1].pulse_counter = 0;
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
