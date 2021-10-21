#pragma once

#include <hardware/i2c.h>
#include <stdint.h>

typedef enum {
	RTC_SEC = 0,
	RTC_MIN,
	RTC_HOUR,
	RTC_DAY,
	RTC_DATE,
	RTC_MONTH,
	RTC_YEAR,
	RTC_ALARM1_SEC,
	RTC_ALARM1_MIN,
	RTC_ALARM1_HOUR,
	RTC_ALARM1_DAY,
	RTC_ALARM1_DATE,
	RTC_ALARM2_MIN,
	RTC_ALARM2_HOUR,
	RTC_ALARM2_DAY,
	RTC_ALARM2_DATE,
	RTC_CONTROL,
	RTC_CONTROL_STATUS,
	RTC_CONTROL_AGING,
	RTC_CONTROL_TEMP_MSB,
	RTC_CONTROL_TEMP_LSB
} rtc_reg;

/**
 * Initialises the IO Expander.
 * \param i2c	I2C context of the RP2040.
 * \return 	PICO_OK on success,
 * 		else PICO_ERROR_GENERIC if IO Expander not found.
 */
int i2c_rtc_init(i2c_inst_t *i2c);

#if 0
/**
 * Write to registers on the I2C RTC peripheral.
 * \param i2c 	I2C context of the RP2040.
 * \param reg 	Register to start reading from.
 * \return 	PICO_OK on success.
 */
int i2c_rtc_write(i2c_inst_t *i2c, rtc_reg start_reg, uint8_t *tx, size_t len);

/**
 * Read registers from the I2C RTC peripheral.
 * \param i2c 	I2C context of the RP2040.
 * \param dat 	Value to write to set register.
 * \return 	PICO_OK on success.
 */
int i2c_rtc_read(i2c_inst_t *i2c, rtc_reg start_reg, uint8_t *rx, size_t len);
#endif

/**
 * Read time from real time clock.
 * \param i2c	I2C context of the RP2040.
 * \param dt	Date time structure to fill.
 * \return	PICO_OK on success, else failure.
 */
int i2c_rtc_read_time(i2c_inst_t *i2c, datetime_t *dt);

/**
 * Read temperature from real time clock.
 * \param i2c	I2C context of the RP2040.
 * \param t	Pointer to store temperature.
 * \return	PICO_OK on success, else failure.
 */
int i2c_rtc_read_temperature_i(i2c_inst_t *i2c, int8_t *t);
int i2c_rtc_read_temperature_f(i2c_inst_t *i2c, float *t);
