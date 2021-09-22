#include <stdint.h>
#include <peripherals/rtc.h>
#include <hardware/i2c.h>

/**
 * This implementation is for the DS3231M Real Time Clock.
 */
#define I2C_DS3231M_ADDR 0b01101000

#define CURRENT_MILLENNIUM	21
#define RTC_YEARS_EPOCH		((CURRENT_MILLENNIUM - 1) * 100)

inline uint8_t bcd_to_int(uint8_t x)
{
	return x - 6 * (x >> 4);
}

int i2c_rtc_init(i2c_inst_t *i2c)
{
	uint8_t rx[1];
	return i2c_read_blocking(i2c, I2C_DS3231M_ADDR, rx, 1, false);
}

#if 0
int i2c_rtc_write(i2c_inst_t *i2c, rtc_reg start_reg, uint8_t *tx, size_t len)
{
	return i2c_write_blocking(i2c, I2C_DS3231M_ADDR, tx, len, false);
}

int i2c_rtc_read(i2c_inst_t *i2c, rtc_reg start_reg, uint8_t *rx, size_t len)
{
	return i2c_read_blocking(i2c, I2C_DS3231M_ADDR, rx, len, false);
}
#endif

int i2c_rtc_read_time(i2c_inst_t *i2c, datetime_t *dt)
{
	uint8_t tx = RTC_SEC;
	uint8_t rx[RTC_YEAR + 1];
	int ret;

	/* Select second register. */
	ret = i2c_write_blocking(i2c, I2C_DS3231M_ADDR, &tx, 1, true);
	if(ret != PICO_OK)
		goto out;

	/* Read time values. */
	ret = i2c_read_blocking(i2c, I2C_DS3231M_ADDR, rx, sizeof(rx), false);
	if(ret != PICO_OK)
		goto out;

	dt->sec  = bcd_to_int(rx[RTC_SEC]);
	dt->min  = bcd_to_int(rx[RTC_MIN]);
	dt->hour = bcd_to_int(rx[RTC_HOUR] & 0b00111111);
	dt->dotw = rx[RTC_DAY];
	dt->day  = bcd_to_int(rx[RTC_DATE]);
	dt->month = bcd_to_int(rx[RTC_MONTH] & 0b00011111);
	dt->year = bcd_to_int(rx[RTC_YEAR] & 0b00011111) + RTC_YEARS_EPOCH;
	/* TODO: Year 3000 problem? */

out:
	return ret;
}

int i2c_rtc_read_temperature_i(i2c_inst_t *i2c, int8_t *t)
{
	uint8_t tx = RTC_CONTROL_TEMP_MSB;
	int ret;

	/* Select MSB temperature register. */
	ret = i2c_write_blocking(i2c, I2C_DS3231M_ADDR, &tx, 1, true);
	if(ret != PICO_OK)
		goto out;

	/* Only read the most significant byte of the temperature value. */
	ret = i2c_read_blocking(i2c, I2C_DS3231M_ADDR, t, 1, false);
	if(ret != PICO_OK)
		goto out;

out:
	return ret;
}

int i2c_rtc_read_temperature_f(i2c_inst_t *i2c, float *t)
{
	uint8_t tx = RTC_CONTROL_TEMP_MSB;
	uint8_t rx[2];
	float frac[4] = { 0.0f, 0.25f, 0.5f, 0.75f };
	int ret;

	/* Select first temperature register. */
	ret = i2c_write_blocking(i2c, I2C_DS3231M_ADDR, &tx, 1, true);
	if(ret != PICO_OK)
		goto out;

	/* Read both temperature registers. */
	ret = i2c_read_blocking(i2c, I2C_DS3231M_ADDR, rx, sizeof(rx), false);
	if(ret != PICO_OK)
		goto out;

	/* Calculate temperature in float value. */
	*t = rx[0] & 0b01111111; /* Ignore sign bit for now. */

	/* If sign bit is set, make the temperature negative. */
	if(rx[0] & 0b10000000)
		*t *= -1;

	/* Add fractional value. */
	*t += frac[rx[1]];

out:
	return ret;
}
