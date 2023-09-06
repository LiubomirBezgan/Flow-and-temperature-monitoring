/*
 * bme280_add.h
 *
 *  Created on: Mar 31, 2023
 *      Author: SKG.POTOP
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_BME280_ADD_H_
#define INC_BME280_ADD_H_

/* Includes ------------------------------------------------------------------*/
#include "bme280_defs.h"

/* General type definitions --------------------------------------------------*/
typedef enum {
	SI,
	Imperial
} UNITS_e;

/* Function prototypes -------------------------------------------------------*/
/**
  * @brief  is used to specify a delay while the sensor completes a measurement
  * @param  period determines the delay value in milliseconds
  * @param  intf_ptr can be used as a variable to store the I2C address of the device
  * @retval None
  */
void delay_us(uint32_t period, void *intf_ptr); // old void delay_ms(uint32_t period, void *intf_ptr);

/**
  * @brief  reads an amount of data in blocking mode from the BME280 sensor.
  * @param  reg_addr determines an internal memory address
  * @param  reg_data points to data buffer
  * @param  len determines an amount of data to be sent
  * @param  intf_ptr can be used as a variable to store the I2C address of the device
  * @retval 0 for Success, non-zero for failure
  */
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/**
  * @brief  writes an amount of data in blocking mode to the BME280 sensor.
  * @param  reg_addr determines an internal memory address
  * @param  reg_data points to data buffer
  * @param  len determines an amount of data to be sent
  * @param  intf_ptr can be used as a variable to store the I2C address of the device
  * @retval 0 for Success, non-zero for failure
  */
int8_t i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/**
  * @brief  initializes a struct bme280_dev data type (bme280 device structure)
  * @param  dev points to the struct bme280_dev data type (bme280 device structure)
  * @retval API success code (BME280_OK) for Success, API error code for failure (see name API error codes in bme280_defs.h)
  */
int8_t BME280_init(struct bme280_dev *dev);

/**
  * @brief  reads the pressure, temperature and humidity data from the sensor, compensates the data and store it in the bme280_data structure instance passed by the user
  * @param  dev points to the struct bme280_dev data type (bme280 device structure)
  * @param  pcomp_data points to the bme280_data structure that contains the pressure, temperature and humidity data from the sensor
  * @retval API success code (BME280_OK) for Success, API error code for failure (see name API error codes in bme280_defs.h)
  */
int8_t BME280_read_data (struct bme280_dev *dev, struct bme280_data * pcomp_data);

#endif /* INC_BME280_ADD_H_ */
