#include <stdint.h>
#include <peripherals/io_expander.h>
#include <hardware/i2c.h>

/**
 * This implementation is for the PCA9536 IO Expander.
 */
#define I2C_PCA9536_ADDR 0b01000001

int i2c_io_exp_init(i2c_inst_t *i2c)
{
	uint8_t rx[1];
	return i2c_read_blocking(i2c, I2C_PCA9536_ADDR, rx, 1, false);
}

int i2c_io_exp_set_reg(i2c_inst_t *i2c, io_exp_reg reg)
{
	return i2c_write_blocking(i2c, I2C_PCA9536_ADDR, &reg, 1, false);
}

int i2c_io_exp_write_reg(i2c_inst_t *i2c, uint8_t dat)
{
	return i2c_write_blocking(i2c, I2C_PCA9536_ADDR, &dat, 1, false);
}

int i2c_io_exp_read_reg(i2c_inst_t *i2c, uint8_t *dat)
{
	return i2c_read_blocking(i2c, I2C_PCA9536_ADDR, dat, 1, false);
}
