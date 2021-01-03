/*
 * bme280_2.h
 *
 *  Created on: Dec 25, 2020
 *      Author: lissi
 */

#ifndef BME280_2
#define BME280_2

#include "sensors.h"
#include "stm32f0xx_hal.h"

#define BME280_ADDR 0x76
#define BME280_CTRL_HUM 0xF2
#define BME280_CTRL_MEAS 0xF4
#define BME280_OS_X1 0x01
#define BME280_MODE_FORCE 0x01
#define BME280_TEMP_MSB 0xFA
#define BME280_HUM_MSB 0xFD
#define BME280_PRESS_MSB 0xF7
#define BME280_CALIB_TEMP 0x88
#define BME280_CALIB_PRESS 0x8E
#define BME280_CALIB_HUM_1 0xA1
#define BME280_CALIB_HUM_2 0xE1

//uint8_t ctrl_hum_set[];
//uint8_t ctrl_meas_set[];
//uint8_t temp_data[3];
//uint8_t hum_data[2];
//uint8_t press_data[3];
//uint8_t temp_calib[6];
//uint8_t press_calib[18];
//uint8_t hum_calib[9];

//i2c_register_t bme280_ctrl_hum = {
//	.address = BME280_CTRL_HUM,
//	.data = ctrl_hum_set,
//	.size = 1
//};
//
//i2c_register_t bme280_ctrl_meas = {
//	.address = BME280_CTRL_MEAS,
//	.data = ctrl_meas_set,
//	.size = 1
//};
//
//i2c_register_t bme280_temp = {
//	.address = BME280_TEMP_MSB,
//	.data = temp_data,
//	.size = 3,
//};
//
//i2c_register_t bme280_hum = {
//	.address = BME280_HUM_MSB,
//	.data = hum_data,
//	.size = 2
//};
//
//i2c_register_t bme280_press = {
//	.address = BME280_PRESS_MSB,
//	.data = press_data,
//	.size = 3
//};
//
//i2c_register_t calibration_data_temp = {
//	.address = BME280_CALIB_TEMP,
//	.size = 6,
//	.data = temp_calib
//};
//
//i2c_register_t calibration_data_press = {
//	.address = BME280_CALIB_PRESS,
//	.size = 18,
//	.data = press_calib
//};
//
//i2c_register_t calibration_data_hum = {
//	.address = BME280_CALIB_HUM,
//	.data = hum_calib,
//	.size = 9
//};

void bme280_init(I2C_HandleTypeDef* hi2c);
void bme280_read(I2C_HandleTypeDef* hi2c);
int32_t temp_calibrate(void);
uint32_t press_calibrate(void);
uint32_t hum_calibrate(void);


#endif /* BME280_2 */
