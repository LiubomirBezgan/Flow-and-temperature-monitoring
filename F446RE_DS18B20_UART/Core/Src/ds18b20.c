/*
 * ds18b20.c
 *
 *  Created on: Sep 19, 2023
 *      Author: SKG.POTOP
 */

#include <string.h>
#include "ds18b20.h"
#include "1-wire.h"

HAL_StatusTypeDef ds18b20_init(void)
{
  return wire_init();
}

HAL_StatusTypeDef ds18b20_read_address(uint8_t* rom_code)
{
	int i;
	uint8_t crc;

	if ( HAL_OK != wire_reset() )
	{
		return HAL_ERROR;
	}

	wire_write(DS18B20_READ_ROM);

	for (i = 0; i < DS18B20_ROM_CODE_SIZE; i++)
	{
		rom_code[i] = wire_read();
	}

	crc = wire_crc(rom_code, DS18B20_ROM_CODE_SIZE - 1);
	if (rom_code[DS18B20_ROM_CODE_SIZE - 1] == crc)
	{
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}
}

static HAL_StatusTypeDef send_cmd(const uint8_t * rom_code, uint8_t cmd)
{
	int i;

	if ( HAL_OK != wire_reset() )
	{
		return HAL_ERROR;
	}

	if (!rom_code)
	{
		wire_write(DS18B20_SKIP_ROM);
	}
	else
	{
		wire_write(DS18B20_MATCH_ROM);
		for (i = 0; i < DS18B20_ROM_CODE_SIZE; i++)
		{
			wire_write(rom_code[i]);
		}
	}

	wire_write(cmd);
	return HAL_OK;
}

HAL_StatusTypeDef ds18b20_start_measure(const uint8_t * rom_code)
{
	return send_cmd(rom_code, DS18B20_CONVERT_T);
}

static HAL_StatusTypeDef ds18b20_read_scratchpad(const uint8_t * rom_code, uint8_t * scratchpad)
{
	int i;
	uint8_t crc;

	if (HAL_OK != send_cmd(rom_code, DS18B20_READ_SCRATCHPAD))
	{
		return HAL_ERROR;
	}

	for (i = 0; i < DS18B20_SCRATCHPAD_SIZE; i++)
	{
		scratchpad[i] = wire_read();
	}

	crc = wire_crc(scratchpad, DS18B20_SCRATCHPAD_SIZE -1);
	if (scratchpad[DS18B20_SCRATCHPAD_SIZE - 1] == crc)
	{
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}
}

float ds18b20_get_temp(const uint8_t * rom_code)
{
	float result;
	uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE];
	uint16_t temp;

	if ( HAL_OK != ds18b20_read_scratchpad(rom_code, scratchpad) )
	{
		result = 85.0f;
		return result;
	}

	memcpy(&temp, &scratchpad[0], sizeof(temp));
	result = temp / 16.0f;
	return result;
}
