/*
 * ds18b20.h
 *
 *  Created on: Sep 19, 2023
 *      Author: SKG.POTOP
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include "stm32f4xx.h"

#define DS18B20_ROM_CODE_SIZE 8
#define DS18B20_SCRATCHPAD_SIZE    9

#define DS18B20_READ_ROM           0x33
#define DS18B20_MATCH_ROM          0x55
#define DS18B20_SKIP_ROM           0xCC

#define DS18B20_CONVERT_T          0x44
#define DS18B20_READ_SCRATCHPAD    0xBE
#define DS18B20_WRITE_SCRATCHPAD   0x4E

HAL_StatusTypeDef ds18b20_init(void);

HAL_StatusTypeDef ds18b20_read_address(uint8_t* rom_code);

HAL_StatusTypeDef ds18b20_start_measure(const uint8_t* rom_code);

HAL_StatusTypeDef ds18b20_read_scratchpad(const uint8_t * rom_code, uint8_t * scratchpad);	// Previously this function was static.

HAL_StatusTypeDef ds18b20_write_scratchpad(const uint8_t * rom_code, uint8_t resolution);

float ds18b20_get_temp(const uint8_t* rom_code);

#endif /* INC_DS18B20_H_ */
