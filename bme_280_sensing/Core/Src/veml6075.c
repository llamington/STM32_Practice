#include "veml6075.h"
#include "main.h"

int VEML6075_read_word(uint16_t addr, uint8_t reg, uint16_t *val)
{
	int err = 0;
	int retry = 3;
	struct i2c_msg msg[2];
	uint8_t data[2];
	while (retry--) {
		/* Send slave address & register */
		msg[0].addr = addr;
		msg[0].flags = I2C_M_WR;
		msg[0].len = 1;
		msg[0].buf = &reg;
		/* Read word data */
		msg[1].addr = addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = 2;
		msg[1].buf = data;
		err = i2c_transfer(msg, 2);
		if (err == 0) {
			*val = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
			return err;
		}
	}
	return err;
}

int VEML6075_write_word(uint16_t addr, uint8_t reg, uint16_t val)
{
	int err = 0;
	int retry = 3;
	struct i2c_msg msg;
	uint8_t data[3];
	while (retry--) {
		data[0] = reg;
		data[1] = (uint8_t)(val & 0xFF);
		data[2] = (uint8_t)((val & 0xFF00) >> 8);
		/* Send slave address, register and word data */
		msg.addr = addr;
		msg.flags = I2C_M_WR;
		msg.len = 3;
		msg.buf = data;
		err = i2c_transfer(&msg, 1);
		if (err == 0) return 0;
	}
	return err;
}

/* Calculates the UV level for the given UV type*/
uint16_t uv_calc(uint16_t uvx, uint16_t comp1, uint16_t comp2, uint8_t uv_type)
{
	uint16_t calculated;
	if(uv_type == VEML6075_TYPE_UVA) {
		 calculated = uvx - (VEML6075_COEFF_A * VEML6075_COEFF_ALPHA * comp1) / VEML6075_COEFF_GAMMA -
				 (VEML6075_COEFF_B * VEML6075_COEFF_ALPHA * comp2) / VEML6075_COEFF_DELTA;
	} else {
		calculated = uvx - (VEML6075_COEFF_C * VEML6075_COEFF_BETA * comp1) / VEML6075_COEFF_GAMMA -
				 (VEML6075_COEFF_D * VEML6075_COEFF_BETA * comp2) / VEML6075_COEFF_DELTA;
	}
	return calculated;
}
