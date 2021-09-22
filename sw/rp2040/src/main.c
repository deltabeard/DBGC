#include <hardware/pio.h>
#include <hardware/i2c.h>
#include <hardware/rtc.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>

#include <main.h>
#include <peripherals/io_expander.h>
#include <peripherals/rtc.h>

#include "comms.pio.h"
#include "gb_manager.gb.h"

void start_io_expander(void)
{
	/* Check that the IO expander is connected. */
	if(i2c_io_exp_init(i2c0) != PICO_OK)
	{
		/* Connection to the IO expander is mandatory. If that fails,
		 * then reboot to USB for reprogramming, debugging, etc. */
		reset_usb_boot(0, 0);
		__builtin_unreachable();
	}

	/* Set pin directions on the IO expander. */
	i2c_io_exp_set_reg(i2c0, IO_EXP_DIRECTION);
	i2c_io_exp_write_reg(i2c0, (IO_EXP_DIR_UNUSED << IO_EXP_UNUSED_PIN) |
		(IO_EXP_DIR_OUTPUT << IO_EXP_STATLED_PIN) |
		(IO_EXP_DIR_INPUT << IO_EXP_SWITCH_PIN) |
		(IO_EXP_DIR_OUTPUT << IO_EXP_GB_RESET_PIN));

	/* Hold Game Boy in reset before checking for button press.
	 * Initialise LED to be off. */
	i2c_io_exp_set_reg(i2c0, IO_EXP_OUTPUT_PORT);
	i2c_io_exp_write_reg(i2c0,
		(0 << IO_EXP_STATLED_PIN) | (1 << IO_EXP_GB_RESET_PIN));

	{
		uint8_t in;

		/* Check to see if button is pressed. */
		i2c_io_exp_set_reg(i2c0, IO_EXP_INPUT_PORT);
		i2c_io_exp_read_reg(i2c0, &in);
		/* If button isn't being pressed down, do nothing. */
		if((in & (1 << IO_EXP_SWITCH_PIN)) == 1)
			return;

		/* If button is being held down, reboot to USB Boot mode. */
		/* Turn on the LED to show that we're in USB Boot mode due to
		 * user pushing button. */
		i2c_io_exp_set_reg(i2c0, IO_EXP_OUTPUT_PORT);
		i2c_io_exp_write_reg(i2c0, (1 << IO_EXP_STATLED_PIN));

		reset_usb_boot(0, 0);
		__builtin_unreachable();
	}
}

void start_rtc(void)
{
	datetime_t dt;

	/* Initilise internal RTC. */
	rtc_init();

	/* Initiliase external I2C RTC. */
	if(i2c_rtc_init(i2c0) != PICO_OK)
		return;

	/* Read date and time from external RTC. */
	if(i2c_rtc_read_time(i2c0, &dt) != PICO_OK)
		return;

	/* Set date and time to internal RTC. */
	if(rtc_set_datetime(&dt) != PICO_OK)
		return;
}

int main(void)
{
	/* Initialise and set I2C baudrate to 400kHz. */
	i2c_init(i2c0, 400 * 1000);
	/* Declare I2C pins for picoboot. */
	bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));
	/* Initiliase GPIO pins for I2C0. */
	gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
	/* Disable weak internal pull-ups on I2C pins; there are external
	 * pull-ups connected to these pins for I2C operation. */
	gpio_disable_pulls(I2C_SDA_PIN);
	gpio_disable_pulls(I2C_SCL_PIN);
	/* If the user has held down the button on startup, reset
	 * to USB Boot mode. Otherwise, turn off LED and hold Game Boy in
	 * reset. */
	start_io_expander();

	/* Obtain time and date from RTC IC. */
	start_rtc();

	/* TODO: Initilise FRAM. */


	/* Temporarily testing data output. */
	gpio_set_dir_all_bits(0b1);

#if 0
	/* Declare PIO pins for picoboot. */
	bi_decl(bi_pin_range_with_func(PIO_PHI_3V3, PIO_PHI_DIR, GPIO_FUNC_PIO0));

	PIO pio = pio0;
	uint sm_a15, sm_cs, sm_wr, sm_do;

	gb_bus_program_init(pio, &sm_a15, &sm_cs, &sm_wr, &sm_do,
			PIN_D0, PIN_A15, PIN_CS, PIN_WR, PIN_DATADIR);
	sleep_ms(500);
#endif

	return 0;
}
