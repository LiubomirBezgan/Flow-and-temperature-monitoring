/*
 * 1-wire.c
 *
 *  Created on: Sep 19, 2023
 *      Author: SKG.POTOP
 */

#include "1-wire.h"
#include "main.h"

//extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart6;

//HAL_StatusTypeDef wire_init(void)
//{
//  return HAL_TIM_Base_Start(&htim7);
//}
//
//	/* bit-banging */
//static void delay_us(uint32_t us)
//{
//	__HAL_TIM_SET_COUNTER(&htim7, 0);
//	while (__HAL_TIM_GET_COUNTER(&htim7) < us) {}
//}

static void set_baudrate(uint32_t baudrate)
{
	  huart6.Instance = USART6;
	  huart6.Init.BaudRate = baudrate;
	  huart6.Init.WordLength = UART_WORDLENGTH_8B;
	  huart6.Init.StopBits = UART_STOPBITS_1;
	  huart6.Init.Parity = UART_PARITY_NONE;
	  huart6.Init.Mode = UART_MODE_TX_RX;
	  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	  if (HAL_UART_Init(&huart6) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

HAL_StatusTypeDef wire_reset(void)
{
//	int rc = 1;
//
//	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
//	delay_us(480);
//	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);
//	delay_us(70);
//	rc = HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin);
//	delay_us(410);
//
//	if (0 == rc)
//	{
//		return HAL_OK;
//	}
//	else
//	{
//		return HAL_ERROR;
//	}
	uint8_t data_out = 0xF0;
	uint8_t data_in = 0;

	set_baudrate(9600);
	HAL_UART_Transmit(&huart6, &data_out, 1, HAL_MAX_DELAY);
	HAL_UART_Receive(&huart6, &data_in, 1, HAL_MAX_DELAY);
	set_baudrate(115200);

	if (0xF0 != data_in)
	{
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}
}

static void write_bit(int value)
{
	if (value)
	{
		uint8_t data_out = 0xFF;
		HAL_UART_Transmit(&huart6, &data_out, 1, HAL_MAX_DELAY);
	}
	else
	{
		uint8_t data_out = 0x0;
		HAL_UART_Transmit(&huart6, &data_out, 1, HAL_MAX_DELAY);
	}
}

static int read_bit(void)
{
	uint8_t data_out = 0xFF;
	uint8_t data_in = 0x0;
	HAL_UART_Transmit(&huart6, &data_out, 1, HAL_MAX_DELAY);
	HAL_UART_Receive(&huart6, &data_in, 1, HAL_MAX_DELAY);
	return data_in & 0x01;

}

void wire_write(uint8_t byte)
{
	int i;
	for (i = 0; i < BYTE; i++)
	{
		write_bit(byte & 0x01);
		byte >>= 1;
	}
}

uint8_t wire_read(void)
{
	uint8_t value = 0;
	int i;
	for (i = 0; i < BYTE; i++)
	{
		value >>= 1;
		if (read_bit())
		{
			value |= 0x80;
		}
	}
	return value;
}

// CRC
static uint8_t byte_crc(uint8_t crc, uint8_t byte)
{
	int i;
	for (i = 0; i < BYTE; i++)
	{
		uint8_t b = crc ^ byte;
		crc >>= 1;
		if (b & 0x01)
		{
			crc ^= 0x8c;
		}
		byte >>= 1;
	}
	return crc;
}

uint8_t wire_crc (const uint8_t * data, int len)
{
	int i;
	uint8_t crc = 0;

	for (i = 0; i < len; i++)
	{
		crc = byte_crc(crc, data[i]);
	}
	return crc;
}
