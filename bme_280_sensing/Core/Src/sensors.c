#include "sensors.h"
#include "main.h"

// read from an array of registers within i2c sensor
//void read_i2c_registers(I2C_HandleTypeDef* hi2c, i2c_sensor_t* sensor)
//{
//	uint8_t i2c_addr = sensor->address;
//	uint8_t num_reg = sensor->num_read_registers;
//	i2c_register_t* cur_reg;
//	for(uint8_t i = 0; i < num_reg; i++) {
//		cur_reg = &sensor->read_registers[i];
//		read_i2c_register(hi2c, cur_reg, i2c_addr);
//	}
//}

// write i2c_register to sensor defined by sensor_addr
void write_i2c_register(I2C_HandleTypeDef* hi2c, i2c_register_t* i2c_register, uint8_t sensor_addr)
{
	i2c_register->status = HAL_I2C_Mem_Write(
		hi2c,
		sensor_addr << 1,
		i2c_register->address,
		I2C_MEMADD_SIZE_8BIT,
		i2c_register->data,
		i2c_register->size,
		HAL_MAX_DELAY
	);
}

// read from i2c_register from sensor defined by sensor_addr
void read_i2c_register(I2C_HandleTypeDef* hi2c, i2c_register_t* i2c_register, uint8_t sensor_addr)
{
	i2c_register->status = HAL_I2C_Mem_Read(
		hi2c,
		sensor_addr << 1,
		i2c_register->address,
		I2C_MEMADD_SIZE_8BIT,
		i2c_register->data,
		i2c_register->size,
		HAL_MAX_DELAY
	);
}
