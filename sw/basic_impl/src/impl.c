#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <hardware/i2c.h>
#include <hardware/pio.h>
#include <hardware/vreg.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/bootrom.h>

#include "comms.pio.h"
#include <configuration.h>

#define I2C_PCA9536_ADDR 0b01000001
#define I2C_DS3231M_ADDR 0b01101000

typedef enum {
	IO_EXP_INPUT_PORT = 0,
	IO_EXP_OUTPUT_PORT,
	IO_EXP_INVERSION,
	IO_EXP_DIRECTION
} io_exp_reg_e;

typedef enum {
	GB_POWER_ON = 0,
	GB_POWER_OFF = 1
} gb_pwr_e;

#include <gb_manager.gb.h>

void func_gb(gb_pwr_e pwr);

void __no_inline_not_in_flash_func(gpio_gb_bus)(void)
{
	uint8_t *gb;
	unsigned address = 0;
	unsigned data;
	static unsigned last_address = 0;
	unsigned stor[512];
	unsigned s_i = 0;

	gb = malloc(gb_manager_rom_len);
	if(gb == NULL)
	{
		printf("Malloc failed.\n");
		return;
	}

	memcpy(gb, gb_manager_rom, gb_manager_rom_len);

	vreg_set_voltage(VREG_VOLTAGE_1_30);
	sleep_ms(100);
	set_sys_clock_khz(360000, true);
	sleep_ms(100);

	printf("GB Power on\n");
	func_gb(GB_POWER_ON);

	while(1)
	{
		if(gpio_get(PIO_NRD) != 0)
		{
			gpio_put(PIO_DIR, 0);
			continue;
		}

		address = (gpio_get_all() >> 5) & 0xFFFF;
		if(address == last_address)
			continue;

		last_address = address;
		gpio_put(PIO_DIR, 1);

		if(address >= gb_manager_rom_len)
			continue;

		data = (uint32_t)gb[address];
		data <<= 21;
		data |= (1UL << PIO_DIR);

		stor[s_i] = address;
		s_i++;

		gpio_put_all(data);

		if(s_i > sizeof(stor)/sizeof(*stor))
			break;

	}

	func_gb(GB_POWER_OFF);

	for(unsigned i = 0; i < s_i; i++)
	{
		printf("0x%04X\n", address);
	}

#if 0
	while(1)
	{
		uint32_t all;
		uint16_t address;
		uint32_t data;

		while(gpio_get(PIO_A15) == 1)
			tight_loop_contents();

		/* Read address. */
		all = gpio_get_all();
		address = (all >> 5) & 0xFFFF;
		data = (uint32_t)gb[address];
		data <<= 21;
		data |= (1UL << PIO_DIR);

		gpio_put_all(data);
		//printf("0x%04X 0x%02lX\n", address, data);

		while(gpio_get(PIO_A15) == 0)
			tight_loop_contents();

		gpio_put(PIO_DIR, 0);
	}
#endif

	free(gb);

	return;
}

#if 0
void __no_inline_not_in_flash_func(func_pio)(unsigned sm_clk,
	unsigned sm_a15,
	unsigned sm_do)
{
#include <gb_manager.gb.h>

	/* Turn off Game Boy. */
	func_gb(GB_POWER_OFF);

	/* Enable state machines. */
	pio_sm_set_enabled(pio0, sm_clk, true);
	pio_sm_set_enabled(pio0, sm_a15, true);
	pio_sm_set_enabled(pio0, sm_do,  true);

	printf("GB Power on\n");

	/* Power cycle GB. */
	sleep_ms(100);
	func_gb(GB_POWER_ON);

	while (1)
	{
		/**
		 * sm_a15 state machine RX value:
		 * | 16-bit Address | 0x0000 |
		 */
		uint32_t address;
		uint8_t data;

		address = pio_sm_get_blocking(pio0, sm_a15);
		if(address == 0)
			break;

		address >>= 16;
		data = gb_manager_rom[address];
		pio_sm_put(pio0, sm_do, data);
	}

	pio_sm_set_enabled(pio0, sm_clk, false);
	pio_sm_set_enabled(pio0, sm_a15, false);
	pio_sm_set_enabled(pio0, sm_do,  false);

	func_gb(GB_POWER_OFF);

	return;
}
#endif

void func_gb(gb_pwr_e pwr)
{
	uint8_t tx[2];
	tx[0] = IO_EXP_OUTPUT_PORT;
	tx[1] = 0b11111110 | pwr;
	i2c_write_blocking(i2c_default, I2C_PCA9536_ADDR, tx,
			   sizeof(tx), false);

	return;
}

int main(void)
{
	int c;

	/* Reduce power consumption to stop IO Expander Power-On Reset Errata. */
	sleep_ms(10);

	//set_sys_clock_48mhz();
	stdio_usb_init();

	printf("Starting\n");
	while((c = getchar_timeout_us(0)) == PICO_ERROR_TIMEOUT)
	{
		printf("Press a key to start\n");
		sleep_ms(500);
	}

	printf("\n");

	if(c == 'r')
		goto out;

	printf("Running program\n");
	/* Initialise I2C. */
	i2c_init(i2c_default, 100 * 1000);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_disable_pulls(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_disable_pulls(PICO_DEFAULT_I2C_SCL_PIN);
	// Make the I2C pins available to picotool
	bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN,
		PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

	/* Set external IO expander configuration. */
	{
		uint8_t tx[2];
		tx[0] = IO_EXP_DIRECTION;
		tx[1] = 0b11111110;
		i2c_write_blocking(i2c_default, I2C_PCA9536_ADDR, tx,
				   sizeof(tx), false);
	}

	func_gb(GB_POWER_OFF);

	for(unsigned i = PIO_PHI; i <= PIO_A15; i++)
	{
		gpio_init(i);
		//gpio_set_input_enabled(i, true);
		gpio_set_dir(i, GPIO_IN);
	}

	for(unsigned i = PIO_D0; i <= PIO_DIR; i++)
	{
		gpio_init(i);
		//gpio_set_input_enabled(i, false);
		gpio_set_dir(i, GPIO_OUT);
		gpio_set_drive_strength(i, GPIO_DRIVE_STRENGTH_12MA);
	}

#if 0
	for(unsigned i = PIO_PHI; i <= PIO_DIR; i++)
	{
		/* Use fast slew rate for GB Bus. */
		gpio_set_slew_rate(i, GPIO_SLEW_RATE_FAST);
		/* Disable schmitt triggers on GB Bus. The bus transceivers
		 * already have schmitt triggers. */
		gpio_set_input_hysteresis_enabled(i, false);
	}
#endif

	gpio_gb_bus();

#if 0
	{
		int sm_clk, sm_a15, sm_do;

		/* This will panic if sm is not available. */
		sm_clk = pio_claim_unused_sm(pio0, false);
		sm_a15 = pio_claim_unused_sm(pio0, false);
		sm_do = pio_claim_unused_sm(pio0, false);
		if(sm_clk == -1 || sm_a15 == -1 || sm_do == -1)
			goto out;

		gb_bus_program_init(pio0, sm_clk, sm_a15, sm_do);
		func_pio(sm_clk, sm_a15, sm_do);
	}
#endif

out:
	reset_usb_boot(0, 0);

	return 0;
}
