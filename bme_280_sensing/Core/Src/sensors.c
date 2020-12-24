#include "sensors.h"
#include "main.h"

void read_i2c_registers(I2C_HandleTypeDef* hi2c, struct i2c_sensor* sensor) {
	uint8_t i2c_addr = sensor->address;
	uint8_t num_reg = sensor->num_registers;
	struct i2c_register* cur_reg;
	for(uint8_t i = 0; i < num_reg; i++) {
		cur_reg = &sensor->registers[i];
		cur_reg->status = HAL_I2C_Mem_Read(hi2c, i2c_addr, cur_reg->address, I2C_MEMADD_SIZE_8BIT, cur_reg->data, cur_reg->size, HAL_MAX_DELAY);
	}
}


