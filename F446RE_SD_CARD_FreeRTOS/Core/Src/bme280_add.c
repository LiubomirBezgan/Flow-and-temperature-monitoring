/*
 * bme280_add.c
 *
 *  Created on: Mar 31, 2023
 *      Author: SKG.POTOP
 */

/* Includes ------------------------------------------------------------------*/
#include "bme280_defs.h"
#include "bme280.h"
#include "main.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"


extern I2C_HandleTypeDef hi2c1;

/* Function definitions ------------------------------------------------------*/

/**
  * @brief  is used to specify a delay while the sensor completes a measurement
  * @param  period determines the delay value in milliseconds
  * @param  intf_ptr can be used as a variable to store the I2C address of the device
  * @retval None
  */
void delay_us(uint32_t period, void *intf_ptr)
{
    /*
     * The parameter intf_ptr can be used as a variable to store the I2C address of the device
     */
	HAL_Delay(period);
}

/**
  * @brief  reads an amount of data in blocking mode from the BME280 sensor.
  * @param  reg_addr determines an internal memory address
  * @param  reg_data points to data buffer
  * @param  len determines an amount of data to be sent
  * @param  intf_ptr can be used as a variable to store the I2C address of the device
  * @retval 0 for Success, non-zero for failure
  */
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter intf_ptr can be used as a variable to store the I2C address of the device
     */
    HAL_I2C_Mem_Read(&hi2c1, (uint16_t) (BME280_I2C_ADDR_PRIM << 1), reg_addr, 1, reg_data, len, 100);

    return rslt;
}

/**
  * @brief  writes an amount of data in blocking mode to the BME280 sensor.
  * @param  reg_addr determines an internal memory address
  * @param  reg_data points to data buffer
  * @param  len determines an amount of data to be sent
  * @param  intf_ptr can be used as a variable to store the I2C address of the device
  * @retval 0 for Success, non-zero for failure
  */
int8_t i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter intf_ptr can be used as a variable to store the I2C address of the device
     */
    HAL_I2C_Mem_Write(&hi2c1, (uint16_t) (BME280_I2C_ADDR_PRIM << 1), reg_addr, 1, reg_data, len, 100);
    return rslt;
}

/**
  * @brief  initializes a struct bme280_dev data type (bme280 device structure)
  * @param  dev points to the struct bme280_dev data type (bme280 device structure)
  * @retval API success code (BME280_OK) for Success, API error code for failure (see name API error codes in bme280_defs.h)
  */
int8_t BME280_init(struct bme280_dev *dev)
{
	int8_t rslt = BME280_OK;
	uint8_t settings_sel;
	uint8_t dev_addr = BME280_I2C_ADDR_PRIM;

	dev->intf_ptr = &dev_addr;
	dev->intf = BME280_I2C_INTF;
	dev->read = i2c_read;
	dev->write = i2c_write;
	dev->delay_us = delay_us;

	rslt = bme280_init(dev);

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

	return rslt;
}

/**
  * @brief  reads the pressure, temperature and humidity data from the sensor, compensates the data and store it in the bme280_data structure instance passed by the user
  * @param  dev points to the struct bme280_dev data type (bme280 device structure)
  * @param  pcomp_data points to the bme280_data structure that contains the pressure, temperature and humidity data from the sensor
  * @retval API success code (BME280_OK) for Success, API error code for failure (see name API error codes in bme280_defs.h)
  */
int8_t BME280_read_data (struct bme280_dev *dev, struct bme280_data * pcomp_data)
{
	int8_t rslt;

	/* Delay while the sensor completes a measurement */
	dev->delay_us(70, dev->intf_ptr);
	rslt = bme280_get_sensor_data(BME280_ALL, pcomp_data, dev);

	return rslt;
}
