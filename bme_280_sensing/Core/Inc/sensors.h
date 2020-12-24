#ifndef SENSORS
#define SENSORS

#include <stdint.h>
#include "stm32f0xx_hal.h"

#define VEML6075_ADDR = 0x10 << 1
#define BME280_ADDR = 0x76 << 1

typedef void (sensor_init*)(void);
// contains the data from each sensor
struct sensors_data {
	float battery_percentage;
	float air_humidity;
	float air_pressure;
	float air_temperature;
};

// contains information about an i2c sensor
struct i2c_sensor {
	uint8_t address; // address of the i2c sensor (bit shifted for 7 bits)
	struct i2c_register* registers; // pointer to register structs
	uint8_t num_registers; // number of registers containing sensor data
	(void*)() init; //initialisation function for the sensor

};

// contains i2c register information
struct i2c_register {
	int8_t status; // status of last read from register
	uint8_t address; // address of the register
	uint8_t data[10]; // data contained within the register
	uint8_t size; // size of the data contained within the register (bytes)
};

void read_i2c_registers(I2C_HandleTypeDef* hi2c, struct i2c_sensor* sensor);

#endif
