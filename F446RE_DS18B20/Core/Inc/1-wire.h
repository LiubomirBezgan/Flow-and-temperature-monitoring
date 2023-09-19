/*
 * 1-wire.h
 *
 *  Created on: Sep 19, 2023
 *      Author: SKG.POTOP
 */

#ifndef INC_1_WIRE_H_
#define INC_1_WIRE_H_

#include "stm32f4xx.h"

#define BYTE 8
#define SCRATCHPAD_MEMORY 9

// 1-wire bit-banging
HAL_StatusTypeDef wire_init(void);
HAL_StatusTypeDef wire_reset(void);
uint8_t wire_read(void);
void wire_write(uint8_t byte);
uint8_t wire_crc (const uint8_t * data, int len);

#endif /* INC_1_WIRE_H_ */
