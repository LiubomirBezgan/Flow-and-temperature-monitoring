/*
 * 1-wire.c
 *
 *  Created on: Sep 19, 2023
 *      Author: SKG.POTOP
 */

#include "1-wire.h"
#include "main.h"

extern TIM_HandleTypeDef htim7;

HAL_StatusTypeDef wire_init(void)
{
  return HAL_TIM_Base_Start(&htim7);
}

	/* bit-banging */
static void delay_us(uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim7, 0);
	while (__HAL_TIM_GET_COUNTER(&htim7) < us) {}
}

HAL_StatusTypeDef wire_reset(void)
{
	int rc = 1;

	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
	delay_us(480);
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);
	delay_us(70);
	rc = HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin);
	delay_us(410);

	if (0 == rc)
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
		HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
		delay_us(6);
		HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);
		delay_us(64);
	}
	else
	{
		HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
		delay_us(60);
		HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);
		delay_us(10);
	}
}

static int read_bit(void)
{
	int rc;

	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
	delay_us(6);
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);
	delay_us(9);
	rc = HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin);
	delay_us(55);

	return rc;

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
