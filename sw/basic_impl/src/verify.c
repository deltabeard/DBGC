#include <sys/timespec.h>
#include <sys/cdefs.h>

#define _GNU_SOURCE

#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>
#include <pico/binary_info.h>
#include <hardware/i2c.h>
#include <string.h>
#include <stdlib.h>
#include <core2.h>

#define ARRAYSIZE(array)	(sizeof(array)/sizeof(array[0]))
#define CLR_SCRN		"\033[2J"

#define I2C_PCA9536_ADDR 0b01000001
#define I2C_DS3231M_ADDR 0b01101000

struct func_map {
	char *long_arg;
	char *help;
	void (*func)(const char *cmd);
};

void func_help(const char *cmd);
void func_gb(const char *cmd);
void func_bus(const char *cmd);
void func_reboot(const char *cmd);

static const struct func_map map[] = {
	{ "HELP",	"Print usage information",		func_help	},
	{ "GB",		"Turn GB on (0) or off (1)\n"
			       "\t'GB 1'",			func_gb		},
	{ "BUS",	"Display PIO/GB bus",			func_bus	},
	{ "REBOOT",	"Reboot to USBBOOT",			func_reboot 	}
};

typedef enum {
	IO_EXP_INPUT_PORT = 0,
	IO_EXP_OUTPUT_PORT,
	IO_EXP_INVERSION,
	IO_EXP_DIRECTION
} io_exp_reg;

void func_bus(const char *cmd)
{
	/* Power cycle GB. */
	func_gb("GB 1");
	sleep_ms(10);
	func_gb("GB 0");

	gpio_set_dir(PIO_PHI, false);
	gpio_set_dir(PIO_NRD, false);
	gpio_set_dir(PIO_NCS, false);
	for(unsigned i = 0; i < 16; i++)
		gpio_set_dir(PIO_A0 + i, false);

	gpio_set_dir(PIO_DIR, true);

	while (getchar_timeout_us(0) == PICO_ERROR_TIMEOUT)
	{
		uint32_t address;

		for(unsigned i = 0; i < 8; i++)
			gpio_set_dir(PIO_D0 + i, false);

		gpio_put(PIO_DIR, false);

		/* Wait for PHI to rise. */
phi_rise:
		while(gpio_get(PIO_PHI) == false);
		if(gpio_get(PIO_A15) == true)
			goto phi_rise;

		address = gpio_get_all();
		address >>= PIO_A0;
		address &= 0xFFFF;

		for(unsigned i = 0; i < 8; i++)
			gpio_set_dir(PIO_D0 + i, true);

		gpio_put(PIO_D0, true);
		gpio_put(PIO_D0+1, false);
		gpio_put(PIO_D0+2, false);
		gpio_put(PIO_D0+3, false);
		gpio_put(PIO_D0+4, true);
		gpio_put(PIO_D0+5, false);
		gpio_put(PIO_D0+6, true);
		gpio_put(PIO_D0+7, false);
		gpio_put(PIO_DIR, true);
		printf("0x%04x\n", (uint16_t)address);
		while(gpio_get(PIO_PHI) == true);
	}

	printf("Exiting PIO printing\n");
	return;
}

void func_gb(const char *cmd)
{
	bool turn_gb_on;

	cmd += strlen("GB ");
	if(*cmd == '1')
		turn_gb_on = true;
	else if(*cmd == '0')
		turn_gb_on = false;
	else
	{
		printf("Syntax error\n");
		return;
	}

	uint8_t tx[2];
	tx[0] = IO_EXP_OUTPUT_PORT;
	tx[1] = 0b11111110 | turn_gb_on;
	i2c_write_blocking(i2c_default, I2C_PCA9536_ADDR, tx,
			   sizeof(tx), false);

	return;
}

void func_reboot(const char *cmd)
{
	(void) cmd;
	reset_usb_boot(0, 0);
}

void func_help(const char *cmd)
{
	(void) cmd;

	puts("Usage:");
	for(unsigned i = 0; i < ARRAYSIZE(map); i++)
	{
		printf("%s: %s\r", map[i].long_arg, map[i].help);
	}
}

int main(void)
{
	char buf[64];

	/* Reduce power consumption to stop IO Expander Power-On Reset Errata. */
	sleep_ms(10);

	//set_sys_clock_48mhz();

	i2c_init(i2c_default, 400 * 1000);
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

	/* If baudrate is set to PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE, then the
	 * RP2040 will reset to BOOTSEL mode. */
	stdio_init_all();

	printf("%s", CLR_SCRN);

new_cmd:
	printf("CMD> ");
	for(unsigned i = 0; i < sizeof(buf); i++)
	{
		buf[i] = getchar();
		putchar(buf[i]);
		if(buf[i] == '\b')
		{
			i--;
			continue;
		}
		else if(buf[i] == '\r')
		{
			buf[i] = '\0';
			break;
		}
	}

	if(buf[0] == '\0')
		strcpy(buf, "<no input>");

	for(unsigned i = 0; i < ARRAYSIZE(map); i++)
	{
		if(strncmp(buf, map[i].long_arg, strlen(map[i].long_arg)) != 0)
			continue;

		map[i].func(buf);
		goto new_cmd;
	}

	printf("Unrecognised command '%s'\n", buf);
	goto new_cmd;

	return 0;
}
