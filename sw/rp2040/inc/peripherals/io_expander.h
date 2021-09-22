#pragma once

#include <hardware/i2c.h>
#include <stdint.h>

typedef enum {
	IO_EXP_INPUT_PORT = 0,
	IO_EXP_OUTPUT_PORT,
	IO_EXP_INVERSION,
	IO_EXP_DIRECTION
} io_exp_reg;

#define IO_EXP_DIR_OUTPUT 0u
#define IO_EXP_DIR_INPUT  1u
#define IO_EXP_DIR_UNUSED IO_EXP_DIR_INPUT
#define IO_EXP_GB_RESET_PIN 0u
#define IO_EXP_SWITCH_PIN   1u
#define IO_EXP_STATLED_PIN  2u
#define IO_EXP_UNUSED_PIN   3u

/**
 * Initialises the IO Expander.
 * \param i2c	I2C context of the RP2040.
 * \return 	PICO_OK on success,
 * 		else PICO_ERROR_GENERIC if IO Expander not found.
 */
int i2c_io_exp_init(i2c_inst_t *i2c);

/**
 * Set the register to read/write to on the IO expander.
 * \param i2c 	I2C context of the RP2040.
 * \param reg 	Register to select.
 * \return 	PICO_OK on success.
 */
int i2c_io_exp_set_reg(i2c_inst_t *i2c, io_exp_reg reg);

/**
 * Write to set register on the IO expander.
 * \param i2c 	I2C context of the RP2040.
 * \param dat 	Value to write to set register.
 * \return 	PICO_OK on success.
 */
int i2c_io_exp_write_reg(i2c_inst_t *i2c, uint8_t dat);

/**
 * Read set register of the IO expander.
 * \param i2c 	I2C context of the RP2040.
 * \param dat 	Pointer to write value to.
 * \return 	PICO_OK on success.
 */
int i2c_io_exp_read_reg(i2c_inst_t *i2c, uint8_t *dat);
