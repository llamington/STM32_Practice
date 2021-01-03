/*
 * veml6075_2.h
 *
 *  Created on: Dec 27, 2020
 *      Author: lissi
 */

#ifndef INC_VEML6075_2_H_
#define INC_VEML6075_2_H_

#include <stdint.h>
#include "sensors.h"

#define VEML6075_ADDR 0x10
#define VEML6075_UV_CONF 0x00
#define VEML6075_UV_COMP1 0x0A
#define VEML6075_UV_COMP2 0x0B
#define VEML6075_UVA_DATA 0x07
#define VEML6075_UVB_DATA 0x09

#define VEML6075_COEFF_A 2.22
#define VEML6075_COEFF_B 1.33
#define VEML6075_COEFF_C 2.95
#define VEML6075_COEFF_D 1.75
#define VEML6075_COEFF_ALPHA 1 // set these once golden sample is determined
#define VEML6075_COEFF_BETA 1
#define VEML6075_COEFF_GAMMA 1
#define VEML6075_COEFF_DELTA 1

//uint8_t uv_conf_set[2] = {0x06, 0x00};
//uint8_t uv_comp1[2];
//uint8_t uv_comp2[2];
//uint8_t uva_data[2];
//uint8_t uvb_data[2];


void veml6075_init(I2C_HandleTypeDef* hi2c);
void veml6075_read(I2C_HandleTypeDef* hi2c);

//i2c_register_t veml6075_uv_conf = {
//	.address = VEML6075_UV_CONF,
//	.data = uv_conf_set,
//	.size = 2
//};
//
//i2c_register_t veml6075_uvcomp1 = {
//	.address = VEML6075_UV_COMP1,
//	.data = uv_comp1,
//	.size = 2
//};
//
//i2c_register_t veml6075_uvcomp2 = {
//	.address = VEML6075_UV_COMP2,
//	.data = uv_comp2,
//	.size = 2
//};
//
//i2c_register_t veml6075_uva_data = {
//	.address = VEML6075_UVA_DATA,
//	.data = uva_data,
//	.size = 2
//};
//
//i2c_register_t veml6075_uvb_data = {
//	.address = VEML6075_UVB_DATA,
//	.data = uvb_data,
//	.size = 2
//};


#endif /* INC_VEML6075_2_H_ */
