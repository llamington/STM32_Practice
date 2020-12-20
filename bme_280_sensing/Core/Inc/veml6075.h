#ifndef VEML6075
#define VEML6075

#include <stdint.h>

/* VEML6075 slave address */
#define VEML6075_ADDR 0x20 // 7-bit: 0x10
/* Registers define */
#define VEML6075_CONF_REG 0x00
#define VEML6075_UVA_DATA_REG 0x07
#define VEML6075_UVB_DATA_REG 0x09
#define VEML6075_UVCOMP1_DATA_REG 0x0A
#define VEML6075_UVCOMP2_DATA_REG 0x0B
#define VEML6075_ID_REG 0x0C
/* Register value define : CONF */
#define VEML6075_CONF_SD 0x01
#define VEML6075_CONF_UV_AF_AUTO 0x00
#define VEML6075_CONF_UV_AF_FORCE 0x02
#define VEML6075_CONF_UV_TRIG_NO 0x00
#define VEML6075_CONF_UV_TRIG_ONCE 0x04
#define VEML6075_CONF_HD 0x08
#define VEML6075_CONF_UV_IT_MASK 0x70
#define VEML6075_CONF_UV_IT_50MS 0x00
#define VEML6075_CONF_UV_IT_100MS 0x10
#define VEML6075_CONF_UV_IT_200MS 0x20
#define VEML6075_CONF_UV_IT_400MS 0x30
#define VEML6075_CONF_UV_IT_800MS 0x40
#define VEML6075_CONF_DEFAULT (VEML6075_CONF_UV_AF_AUTO | VEML6075_CONF_UV_TRIG_NO | VEML6075_CONF_UV_IT_100MS)
/* I2C message, used for I2C transaction */
struct i2c_msg {
	uint16_t addr;
	uint16_t flags;
	#define I2C_M_TEN 0x0010
	#define I2C_M_RD 0x0001
	#define I2C_M_WR 0x0000
	#define I2C_M_NOSTART 0x4000
	#define I2C_M_REV_DIR_ADDR 0x2000
	#define I2C_M_IGNORE_NAK 0x1000
	#define I2C_M_NO_RD_ACK 0x0800
	#define I2C_M_RECV_LEN 0x0400
	uint16_t len;
	uint8_t *buf;
};

int VEML6075_read_word(uint16_t addr, uint8_t reg, uint16_t *val);
int VEML6075_write_word(uint16_t addr, uint8_t reg, uint16_t val);

#endif
