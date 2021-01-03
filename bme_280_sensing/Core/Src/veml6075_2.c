/*
 * veml6075_2.c
 *
 *  Created on: Dec 27, 2020
 *      Author: lissi
 */

#include "veml6075_2.h"

uint8_t uv_conf_set[2] = {0x06, 0x00};
uint8_t uv_comp1[2];
uint8_t uv_comp2[2];
uint8_t uva_data[2];
uint8_t uvb_data[2];

i2c_register_t veml6075_uv_conf = {
	.address = VEML6075_UV_CONF,
	.data = uv_conf_set,
	.size = 2
};

i2c_register_t veml6075_uvcomp1 = {
	.address = VEML6075_UV_COMP1,
	.data = uv_comp1,
	.size = 2
};

i2c_register_t veml6075_uvcomp2 = {
	.address = VEML6075_UV_COMP2,
	.data = uv_comp2,
	.size = 2
};

i2c_register_t veml6075_uva_data = {
	.address = VEML6075_UVA_DATA,
	.data = uva_data,
	.size = 2
};

i2c_register_t veml6075_uvb_data = {
	.address = VEML6075_UVB_DATA,
	.data = uvb_data,
	.size = 2
};


void veml6075_init(I2C_HandleTypeDef* hi2c)
{
	// set humidity settings, followed by other settings
	write_i2c_register(hi2c, &veml6075_uv_conf, VEML6075_ADDR);
}

void veml6075_read(I2C_HandleTypeDef* hi2c)
{
	write_i2c_register(hi2c, &veml6075_uv_conf, VEML6075_ADDR);
	read_i2c_register(hi2c, &veml6075_uva_data, VEML6075_ADDR);
	read_i2c_register(hi2c, &veml6075_uvb_data, VEML6075_ADDR);
	read_i2c_register(hi2c, &veml6075_uvcomp1, VEML6075_ADDR);
	read_i2c_register(hi2c, &veml6075_uvcomp2, VEML6075_ADDR);
}

void uv_calc(void)
{
	uint16_t uva = (uint16_t)veml6075_uva_data.data[1] << 8 | veml6075_uva_data.data[0];
	uint16_t uvb = (uint16_t)veml6075_uvb_data.data[1] << 8 | veml6075_uvb_data.data[0];
	uint16_t comp1 = (uint16_t)veml6075_uvcomp1.data[1] << 8 | veml6075_uvcomp1.data[0];
	uint16_t comp2 = (uint16_t)veml6075_uvcomp2.data[1] << 8 | veml6075_uvcomp2.data[0];
	if ((veml6075_uvcomp1.status == HAL_OK) && (veml6075_uvcomp2.status == HAL_OK)) {
		if(veml6075_uva_data.status == HAL_OK) {
			sensor_data.uva = uva - (VEML6075_COEFF_A * VEML6075_COEFF_ALPHA * comp1) / VEML6075_COEFF_GAMMA -
						 (VEML6075_COEFF_B * VEML6075_COEFF_ALPHA * comp2) / VEML6075_COEFF_DELTA;
		} else {
			sensor_data.uva = 0;
		}
		if(veml6075_uvb_data.status == HAL_OK) {
			sensor_data.uvb = uvb - (VEML6075_COEFF_C * VEML6075_COEFF_BETA * comp1) / VEML6075_COEFF_GAMMA -
							 (VEML6075_COEFF_D * VEML6075_COEFF_BETA * comp2) / VEML6075_COEFF_DELTA;
		} else {
			sensor_data.uvb = 0;
		}
	} else {
		sensor_data.uva = 0;
		sensor_data.uvb = 0;
	}
};
