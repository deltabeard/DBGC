/**
 * Verification and testing tool for DBGC.
 * Copyright (c) 2021 Mahyar Koshkouei
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the above copyright notice and
 * this permission notice appear in all copies.
 * THIS SOFTWARE IS PROVIDED 'AS-IS', WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
 * IN NO EVENT WILL THE AUTHORS BE HELD LIABLE FOR ANY DAMAGES ARISING FROM
 * THE USE OF THIS SOFTWARE.
 */

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>
#include <pico/binary_info.h>
#include <hardware/spi.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/sync.h>
#include <hardware/vreg.h>

#include <ds1302.pio.h>
#include <generic.h>

#define CURRENT_MILLENNIUM	21
#define RTC_YEARS_EPOCH		((CURRENT_MILLENNIUM - 1) * 100)

struct func_map {
	char *long_arg;
	char *help;
	void (*func)(const char *cmd);
};

void func_help(const char *cmd);
void func_rtcread(const char *cmd);
void func_rtcwrite(const char *cmd);
void func_gb(const char *cmd);
void func_framdump(const char *cmd);
void func_framnuke(const char *cmd);
void func_led(const char *cmd);
void func_btn(const char *cmd);
void func_reboot(const char *cmd);

static const struct func_map map[] = {
	{ "HELP",	"Print usage information",		func_help	},
	{ "RTC READ",	"Read date and time from RTC and set internal RTC",
								func_rtcread	},
	{ "RTC WRITE",	"Set date and time to RTC \n"
			"\t'RTC WRITE <DOTW>:<DAY>/<MONTH>/<YEAR> <HOUR>:<MIN>:<SEC>'",
			      					func_rtcwrite	},
	{ "FRAM DUMP",	"Dump full contents of 32KiB FRAM",	func_framdump	},
	{ "FRAM NUKE",	"Nuke the contents of 32KiB FRAM",	func_framnuke	},
	{ "LED",	"Toggles LED",				func_led	},
	{ "BTN",	"Get button status",			func_btn	},
	{ "GB",		"Turn GB on (0) or off (1)\n"
			       "\t'GB 1'",			func_gb		},
	{ "REBOOT",	"Reboot to USBBOOT",			func_reboot 	}
};

typedef enum {
	RTC_SEC = 0x80,
	RTC_MIN = 0x82,
	RTC_HOUR = 0x84,
	RTC_DATE = 0x86,
	RTC_MONTH = 0x88,
	RTC_DAY = 0x8A,
	RTC_YEAR = 0x8C,
	RTC_WRITE_PROTECT = 0x8E,
	RTC_TRICKLE_CHARGER = 0x90,

	RTC_CLOCK_BURST = 0xBE,

	RTC_RAM_0 = 0xC0,
	RTC_RAM_31 = 0xFC,

	RTC_RAM_BURST = 0xFE
} rtc_reg_wr;
#define RTC_RED_RD_BIT		0x1

void func_framnuke(const char *cmd)
{
	(void) cmd;
	puts("Not implemented");
}

void func_framdump(const char *cmd)
{
	(void) cmd;
	puts("Not implemented");
}

void func_btn(const char *cmd)
{
	(void) cmd;
	puts("Not implemented");
}

void func_led(const char *cmd)
{
	bool led_state;
	(void) cmd;

	led_state = gpio_get(GPIO_LED_GREEN);
	gpio_put(GPIO_LED_GREEN, !led_state);
}

inline uint8_t bcd_to_int(uint8_t x)
{
	return x - 6 * (x >> 4);
}

uint8_t decToBcd(uint8_t val)
{
	return ((val/10*16) + (val%10));
}

void func_rtcwrite(const char *cmd)
{
	int ret;
	uint8_t tx[8];

	return;

	tx[0] = RTC_SEC;

	//RTC WRITE <DOTW>:<DAY>/<MONTH>/<YEAR> <HOUR>:<MIN>:<SEC>
	ret = sscanf(cmd, "RTC WRITE %hhx:%hhx/%hhx/%hhx %hhx:%hhx:%hhx",
		&tx[RTC_DAY+1], &tx[RTC_DATE+1], &tx[RTC_MONTH+1],
		&tx[RTC_YEAR+1], &tx[RTC_HOUR+1], &tx[RTC_MIN+1],
		&tx[RTC_SEC+1]);
	if(ret != 7)
	{
		printf("sscanf acquired only %d items of %d from string "
		       "'%s'\n", ret, 7, cmd);
		return;
	}

	return;
}

void func_rtcread(const char *cmd)
{
	io_ro_32 *rdrx = (io_ro_32 *) &pio1->rxf[PIO1_SM_RTC_RD];
	io_wo_32 *rdtx = (io_wo_32 *) &pio1->txf[PIO1_SM_RTC_RD];
	io_wo_32 *wrtx = (io_wo_32 *) &pio1->txf[PIO1_SM_RTC_WR];

	(void) cmd;

	gpio_put(PIO_RTC_CE, 1);
	*wrtx = RTC_WRITE_PROTECT;
	*wrtx = 0x00;
	gpio_put(PIO_RTC_CE, 0);
	sleep_ms(1);

	gpio_put(PIO_RTC_CE, 1);
	*wrtx = RTC_RAM_0;
	*wrtx = 0x69;
	gpio_put(PIO_RTC_CE, 0);
	sleep_ms(1);

	for(uint8_t reg_rd = RTC_SEC + RTC_RED_RD_BIT;
		reg_rd <= (RTC_TRICKLE_CHARGER + RTC_RED_RD_BIT);
		reg_rd += 2)
	{
		printf("%02X: ", reg_rd);

		gpio_put(PIO_RTC_CE, 1);
		*wrtx = reg_rd;
		*rdtx = 1;
		while(pio_sm_is_rx_fifo_empty(pio1, PIO1_SM_RTC_RD) == true);
		gpio_put(PIO_RTC_CE, 0);

		printf("%02X\n", *rdrx);
		sleep_ms(1);
	}

	for(uint8_t reg_rd = RTC_RAM_0 + RTC_RED_RD_BIT;
		reg_rd <= (RTC_RAM_31 + RTC_RED_RD_BIT);
		reg_rd += 2)
	{
		printf("%02X: ", reg_rd);

		gpio_put(PIO_RTC_CE, 1);
		*wrtx = reg_rd;
		*rdtx = 1;
		while(pio_sm_is_rx_fifo_empty(pio1, PIO1_SM_RTC_RD) == true);
		gpio_put(PIO_RTC_CE, 0);

		printf("%02X\n", *rdrx);
		sleep_ms(1);
	}

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
		puts("Syntax error");
		return;
	}

	gpio_put(GPIO_GB_RESET, !turn_gb_on);

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
		printf("%s: %s\n", map[i].long_arg, map[i].help);
	}
}

_Noreturn void __no_inline_not_in_flash_func(usb_commander)(void)
{
	char buf[64];

new_cmd:
	/* Reduce power consumption. */
	sleep_ms(1);

	printf("CMD> ");
	for(unsigned i = 0; i < sizeof(buf); i++)
	{
		int new_char = getchar();

		if((new_char < ' ' || new_char > '~') &&
				(new_char != '\n' && new_char != '\r' && new_char != '\b'))
			continue;

		if(new_char == '\b')
		{
			i--;
			putchar(new_char);
			continue;
		}
		else if(new_char == '\r' || new_char == '\n')
		{
			buf[i] = '\0';
			break;
		}
		else if(new_char >= 'a' && new_char <= 'z')
		{
			new_char = toupper(new_char);
		}

		buf[i] = new_char;
		putchar(buf[i]);
	}

	putchar('\n');

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
}

void init_peripherals(void)
{
	/** SIO **/
	/* Initialise GPIO states. */
	gpio_init_mask(1 << GPIO_LED_GREEN |
			1 << GPIO_SWITCH | /* Not required for inputs. */
			1 << GPIO_MOTOR |
			1 << GPIO_GB_RESET |
			1 << SPI_CSn);
	/* Set GPIO pin directions. */
	gpio_set_dir_out_masked(1 << GPIO_LED_GREEN |
			1 << GPIO_MOTOR |
			1 << GPIO_GB_RESET |
			1 << SPI_CSn);
	/* Set initial output state. */
	gpio_set_mask(0 << GPIO_LED_GREEN |
			0 << GPIO_MOTOR |
			1 << GPIO_GB_RESET | /* Hold GB in reset. */
			1 << SPI_CSn);

	/* Set pulls. */
	gpio_disable_pulls(GPIO_LED_GREEN);
	gpio_pull_up(GPIO_SWITCH);
	gpio_disable_pulls(SPI_CSn);
	gpio_disable_pulls(GPIO_MOTOR);
	gpio_disable_pulls(GPIO_GB_RESET);
	/* External pull-ups are on the OE pins. */
	gpio_disable_pulls(PIO_ADDR1_OE);
	gpio_disable_pulls(PIO_ADDR2_OE);
	gpio_disable_pulls(PIO_DATA_OE);
	/* The TXU0104 has a weak pull-down. */
	gpio_disable_pulls(PIO_PHI);
	gpio_disable_pulls(PIO_NWR);
	gpio_disable_pulls(PIO_NRD);
	gpio_disable_pulls(PIO_NCS);

	/** PIO **/
	/* Initialise PIO0 (GB Bus) */
	for(uint_fast8_t pin = PIO_PHI; pin <= PIO_M7; pin++)
	{
		/* Disable schmitt triggers on GB Bus. The bus transceivers
		 * already have schmitt triggers. */
		gpio_set_input_hysteresis_enabled(pin, false);
		/* Use fast slew rate for GB Bus. */
		gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
		/* Initialise PIO0 pins. */
		pio_gpio_init(pio0, pin);
	}

	/* Initialise PIO1 (RTC) */
	pio_gpio_init(pio1, PIO_RTC_SCLK);
	pio_gpio_init(pio1, PIO_RTC_IO);
	pio_gpio_init(pio1, PIO_RTC_CE);
	ds1302_program_init(pio1, PIO1_SM_RTC_WR, PIO1_SM_RTC_RD);
	/* Enable state machines. */
	pio_sm_set_enabled(pio1, PIO1_SM_RTC_WR, true);
	/* PIO_SM_NCS should be enabled when cart RAM access is expected. */
	pio_sm_set_enabled(pio1, PIO1_SM_RTC_RD, true);

	/** SPI **/
	/* Default settings of spi_init are correct for the MB85RS256B. */
	spi_init(spi0, MB85RS256B_BAUDRATE);
	gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
	gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
	gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
}

int main(void)
{
	set_sys_clock_48mhz();
	init_peripherals();

	/* If baudrate is set to PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE, then the
	 * RP2040 will reset to BOOTSEL mode. */
	stdio_usb_init();

	usb_commander();

	return 0;
}
