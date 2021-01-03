#ifndef SENSORS
#define SENSORS

#include <stdint.h>
#include "stm32f0xx_hal.h"

// 8 bit i2c sensor addresses left shifted once to fit within 7 bit address limit of i2c bus
//#define VEML6075_ADDR 0x10 << 1
//#define BME280_ADDR 0x76 << 1

// typedef void (sensor_init*)(void);
// contains the data from each sensor
typedef struct sensors_data_t {
	float battery_percentage;
	float air_humidity;
	float air_pressure;
	float air_temperature;
	float uva;
	float uvb;
} sensors_data_t;


sensors_data_t sensor_data;

//// contains information about an i2c sensor
//typedef struct i2c_sensor_t {
//	uint8_t address; // address of the i2c sensor (bit shifted for 7 bits)
//	//struct i2c_register_t* read_registers; // pointer to register structs to read from
//	//struct i2c_register_t* write_registers; // pointer to register structs to read to
//	//uint8_t num_read_registers; // number of registers containing read sensor data
//	//uint8_t num_write_registers; // number of registers containing data to write to
//	// (void*)() init; //initialisation function for the sensor
//} i2c_sensor_t;

// contains i2c register information
typedef struct i2c_register_t {
	HAL_StatusTypeDef status; // status of last read from register
	uint8_t address; // address of the register
	uint8_t* data; // data contained within the register
	uint8_t size; // size of the data contained within the register (bytes)
} i2c_register_t;

//void read_i2c_registers(I2C_HandleTypeDef* hi2c, i2c_sensor_t* sensor);
void write_i2c_register(I2C_HandleTypeDef* hi2c, i2c_register_t* i2c_register, uint8_t sensor_addr);
void read_i2c_register(I2C_HandleTypeDef* hi2c, i2c_register_t* i2c_register, uint8_t sensor_addr);

#endif
