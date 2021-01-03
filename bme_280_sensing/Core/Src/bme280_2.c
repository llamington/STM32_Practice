/*
 * bme280_2.c
 *
 *  Created on: Dec 25, 2020
 *      Author: lissi
 */

#include "bme280_2.h"


uint8_t ctrl_hum_set[] = {BME280_OS_X1};
uint8_t ctrl_meas_set[] = {BME280_OS_X1 << 5 | BME280_OS_X1 << 2 | BME280_MODE_FORCE};\
uint8_t temp_data[3];
uint8_t hum_data[2];
uint8_t press_data[3];
uint8_t temp_calib[6];
uint8_t press_calib[18] = {0};
uint8_t hum_calib_1[1] = {0};
uint8_t hum_calib_2[7];
uint8_t dig_t3_calib[2] = {0};

int32_t temp_fine;


i2c_register_t bme280_ctrl_hum = {
	.address = BME280_CTRL_HUM,
	.data = ctrl_hum_set,
	.size = 1
};

i2c_register_t bme280_ctrl_meas = {
	.address = BME280_CTRL_MEAS,
	.data = ctrl_meas_set,
	.size = 1
};

i2c_register_t bme280_temp = {
	.address = BME280_TEMP_MSB,
	.data = temp_data,
	.size = 3,
};

i2c_register_t bme280_hum = {
	.address = BME280_HUM_MSB,
	.data = hum_data,
	.size = 2
};

i2c_register_t bme280_press = {
	.address = BME280_PRESS_MSB,
	.data = press_data,
	.size = 3
};

i2c_register_t calibration_data_temp = {
	.address = BME280_CALIB_TEMP,
	.size = 6,
	.data = temp_calib
};

i2c_register_t calibration_data_press = {
	.address = BME280_CALIB_PRESS,
	.size = 18,
	.data = press_calib
};

i2c_register_t calibration_data_hum_1 = {
	.address = BME280_CALIB_HUM_1,
	.data = hum_calib_1,
	.size = 1
};

i2c_register_t calibration_data_hum_2 = {
	.address = BME280_CALIB_HUM_2,
	.data = hum_calib_2,
	.size = 7
};

//i2c_register_t dig_t3_reg = {
//	.address = 0x8C,
//	.data = dig_t3_calib,
//	.size = 2
//};

// initialise BME280 by setting settings and mode to forced
void bme280_init(I2C_HandleTypeDef* hi2c)
{
	// set humidity settings, followed by other settings
	write_i2c_register(hi2c, &bme280_ctrl_hum, BME280_ADDR);
	write_i2c_register(hi2c, &bme280_ctrl_meas, BME280_ADDR);
	read_i2c_register(hi2c, &calibration_data_temp, BME280_ADDR);
	read_i2c_register(hi2c, &calibration_data_press, BME280_ADDR);
	read_i2c_register(hi2c, &calibration_data_hum_1, BME280_ADDR);
	read_i2c_register(hi2c, &calibration_data_hum_2, BME280_ADDR);
}

// read temperature, humidity, pressure from registers
void bme280_read(I2C_HandleTypeDef *hi2c)
{
	// set sensor back to force mode
	write_i2c_register(hi2c, &bme280_ctrl_meas, BME280_ADDR);
	// read ADC registers
	read_i2c_register(hi2c, &bme280_temp, BME280_ADDR);
	read_i2c_register(hi2c, &bme280_hum, BME280_ADDR);
	read_i2c_register(hi2c, &bme280_press, BME280_ADDR);
	// compensate the raw data
	if(bme280_temp.status == HAL_OK) {
		sensor_data.air_temperature = temp_calibrate()/100.0;
	} else {
		sensor_data.air_temperature = 0;
	}
	if(bme280_press.status == HAL_OK) {
		sensor_data.air_pressure = press_calibrate()/256.0;
	} else {
		sensor_data.air_pressure = 0;
	}
	if(bme280_hum.status == HAL_OK) {
		sensor_data.air_humidity = hum_calibrate()/1024.0;
	} else {
		sensor_data.air_humidity = 0;
	}

}


int32_t temp_calibrate(void)
{
	int32_t adc_temp = ((uint32_t)bme280_temp.data[0] << 12) | ((uint32_t)bme280_temp.data[1] << 4) | (bme280_temp.data[2] >> 4);
	int32_t var1, var2, result_t;
	uint16_t dig_t1 = calibration_data_temp.data[0] | ((uint16_t)calibration_data_temp.data[1] << 8);
	int16_t dig_t2 = calibration_data_temp.data[2] | ((int16_t)calibration_data_temp.data[3] << 8);
	int16_t dig_t3 = calibration_data_temp.data[4] | ((int16_t)calibration_data_temp.data[5] << 8);
	// int16_t dig_t3 = dig_t3_reg.data[0];
	var1 = ((((adc_temp>>3) - ((int32_t)dig_t1<<1))) * ((int32_t)dig_t2)) >> 11;
	var2 = (((((adc_temp>>4) - ((int32_t)dig_t1)) * ((adc_temp>>4) - ((int32_t)dig_t1)))>>12) * ((int32_t)dig_t3))>>14;
	temp_fine = var1 + var2;
	result_t = (temp_fine * 5 + 128) >> 8;
	return result_t;
}

uint32_t press_calibrate(void)
{
	int32_t adc_press = ((uint32_t)bme280_press.data[0] << 12) | ((uint32_t)bme280_press.data[1] << 4) | (bme280_press.data[2] >> 4);
	uint16_t dig_p1 = calibration_data_press.data[0] | ((uint16_t)calibration_data_press.data[1] << 8);
	int16_t dig_p2 = calibration_data_press.data[2] | ((uint16_t)calibration_data_press.data[3] << 8);
	int16_t dig_p3 = calibration_data_press.data[4] | ((uint16_t)calibration_data_press.data[5] << 8);
	int16_t dig_p4 = calibration_data_press.data[6] | ((uint16_t)calibration_data_press.data[7] << 8);
	int16_t dig_p5 = calibration_data_press.data[8] | ((uint16_t)calibration_data_press.data[9] << 8);
	int16_t dig_p6 = calibration_data_press.data[10] | ((uint16_t)calibration_data_press.data[11] << 8);
	int16_t dig_p7 = calibration_data_press.data[12] | ((uint16_t)calibration_data_press.data[13] << 8);
	int16_t dig_p8 = calibration_data_press.data[14] | ((uint16_t)calibration_data_press.data[15] << 8);
	int16_t dig_p9 = calibration_data_press.data[16] | ((uint16_t)calibration_data_press.data[17] << 8);
	int64_t var1, var2, result_p;
	var1 = ((int64_t)temp_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_p6;
	var2 = var2 + ((var1*(int64_t)dig_p5)<<17);
	var2 = var2 + (((int64_t)dig_p4)<<35);
	var1 = ((var1*var1*(int64_t)dig_p3)>>8) + ((var1 * (int64_t)dig_p2)<<12);
	var1 = (((((int64_t)1)<<47)+var1)) * ((int64_t)dig_p1)>>33;
	if(var1 == 0) return 0;
	result_p = 1048576 - adc_press;
	result_p = (((result_p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_p9)) * (result_p>>13) * (result_p>>13) >> 25;
	var2 = (((int64_t)dig_p8) * result_p) >> 19;
	result_p = ((result_p + var1 + var2) >> 8) + (((int64_t)dig_p7)<<4);
	return (int64_t)result_p;
}

uint32_t hum_calibrate(void)
{
	int32_t adc_hum = ((uint32_t)bme280_hum.data[0] << 8) | bme280_hum.data[1];
	uint8_t dig_h1 = calibration_data_hum_1.data[0];
	int16_t dig_h2 = calibration_data_hum_2.data[0] | ((uint16_t)calibration_data_hum_2.data[1] << 8);
	uint8_t dig_h3 = calibration_data_hum_2.data[2];
	int16_t dig_h4 = ((uint16_t)calibration_data_hum_2.data[3] << 4) | (calibration_data_hum_2.data[4] & 0b1111);
	int16_t dig_h5 = (calibration_data_hum_2.data[4] >> 4) | ((uint16_t)calibration_data_hum_2.data[5] << 4);
	int8_t dig_h6 = calibration_data_hum_2.data[6];
	int32_t var = (temp_fine - ((int32_t)76800));
	var = (((((adc_hum<<14) - (((int32_t)dig_h4)<<20) - (((int32_t)dig_h5) * var)) +
		((int32_t)16384))>>15) * (((((((var*((int32_t)dig_h6))>>10) *
		(((var*((int32_t)dig_h3))>>11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_h2) + 8192) >> 14));
	var = (var - (((((var >> 15) * (var >> 15)) >> 7) * ((int32_t)dig_h1))>>4));
	var = (var < 0 ? 0 : var);
	var = (var > 419430400 ? 419430400 : var);
	return (uint32_t)(var >> 12);
}

