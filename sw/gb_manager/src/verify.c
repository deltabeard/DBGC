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
//#include <comms.pio.h>
#include <comms_basic.pio.h>
#include <generic.h>
#include <pico/multicore.h>
#include <hardware/structs/bus_ctrl.h>
#include "libbet.gb.h"

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
void func_play(const char *cmd);
void func_led(const char *cmd);
void func_info(const char *cmd);
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
	{ "INFO",	"Print information",			func_info	},
	{ "BTN",	"Get button status",			func_btn	},
	{ "GB",		"Turn GB on (0) or off (1)\n"
			       "\t'GB 1'",			func_gb		},
	{ "PLAY",	"Play a test ROM on the Game Boy",	func_play	},
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

typedef enum {
	GB_POWER_ON = 0,
	GB_POWER_OFF = 1
} gb_power_e;

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

_Noreturn void core1_play_rom(void)
{
	/* NCS state machine is disabled because a ROM only image is being
	 * played here. */
	pio_set_sm_mask_enabled(GB_BUS_PIO,
		1 << PIO_SM_A15 | 0 << PIO_SM_NCS | 1 << PIO_SM_DO |
		1 << PIO_SM_DI, true);
	gpio_put(GPIO_GB_RESET, GB_POWER_ON);

	while(1)
	{
		//io_ro_8 *data_rx = (io_ro_8 *) &GB_BUS_PIO->rxf[PIO_SM_DI]
		// + 3;
		io_wo_8 *data_tx = (io_wo_8 *) &GB_BUS_PIO->rxf[PIO_SM_DO] + 3;
		io_ro_16 *addr_a15 = (io_ro_16 *) &GB_BUS_PIO->rxf[PIO_SM_A15] + 1;
		//io_ro_16 *addr_ncs = (io_ro_16 *)
		//	&GB_BUS_PIO->rxf[PIO_SM_NCS] + 1;
		uint16_t address;

		/* Wait until we receive a new address. */
		while(pio_sm_is_rx_fifo_empty(GB_BUS_PIO, PIO_SM_A15));

		/* Only reads are expected in a non-banked ROM. */
		address = *addr_a15;
		*data_tx = libbet_gb[address];
	}
}

void func_play(const char *cmd)
{
	static bool already_playing = false;

	(void) cmd;

#if 0
	if(already_playing == true)
	{
		puts("Already playing");
		return;
	}

	already_playing = true;

	/* Grant high bus priority to the second core. */
	bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;

	multicore_launch_core1(core1_play_rom);
#elif 1
	if(already_playing == false)
	{
		gb_bus_program_basic_init(GB_BUS_PIO, PIO_SM_A15);
		pio_sm_set_enabled(GB_BUS_PIO, PIO_SM_A15, true);
		puts("SM Init");
	}

	already_playing = true;
	gpio_put(GPIO_GB_RESET, GB_POWER_ON);
	puts("Power on");
	(void)save_and_disable_interrupts();

	while(1)
	{
		io_wo_8 *data_tx =
			(io_wo_8 *) &GB_BUS_PIO->txf[PIO_SM_A15] + 3;
		io_ro_16 *addr_a15 = (io_ro_16 *)
			&GB_BUS_PIO->rxf[PIO_SM_A15] + 1;

		if(pio_sm_is_rx_fifo_empty(GB_BUS_PIO, PIO_SM_A15) == false)
		{
			uint16_t address = *addr_a15;
			uint8_t data = libbet_gb[address];
			*data_tx = data;
			//printf("%04X %02X\n", address, data);
		}
	}
#else
	gpio_put(GPIO_GB_RESET, GB_POWER_ON);
#endif

}

inline uint8_t bcd_to_int(uint8_t x)
{
	return x - 6 * (x >> 4);
}

uint8_t decToBcd(uint8_t val)
{
	return ((val/10*16) + (val%10));
}

// Copyright (c) 2009, Matt Sparks
// All rights reserved.
// https://github.com/msparks/arduino-ds1302/
// Distributed under the 2-clause BSD license.
void ds1302_write(const uint8_t value, bool read_after)
{
	gpio_set_dir(PIO_RTC_IO, 1);
	for (int i = 0; i < 8; ++i) {
		gpio_put(PIO_RTC_IO, (value >> i) & 1);
		sleep_us(4);
		gpio_put(PIO_RTC_SCLK, 1);
		sleep_us(4);

		if (read_after && i == 7) {
			// We're about to read data -- ensure the pin is back in input mode
			// before the clock is lowered.
			gpio_set_dir(PIO_RTC_IO, 0);
		} else {
			gpio_put(PIO_RTC_SCLK, 0);
			sleep_us(4);
		}
	}
}
uint8_t ds1302_read(void)
{
	uint8_t input_value = 0;
	uint8_t bit = 0;
	gpio_set_dir(PIO_RTC_IO, 0);

	// Bits from the DS1302 are output on the falling edge of the clock
	// cycle. This is called after readIn (which will leave the clock low) or
	// writeOut(..., true) (which will leave it high).
	for (int i = 0; i < 8; ++i) {
		gpio_put(PIO_RTC_SCLK, 1);
		sleep_us(4);
		gpio_put(PIO_RTC_SCLK, 0);
		sleep_us(4);

		bit = gpio_get(PIO_RTC_IO);
		input_value |= (bit << i);  // Bits are read LSB first.
	}

	return input_value;
}

uint8_t ds1302_read_register(const rtc_reg_wr reg)
{
	const uint8_t cmd_byte = reg | 0x1;
	uint8_t ret;

	gpio_put(GPIO_RTC_CE, 1);
	sleep_us(4);
	ds1302_write(cmd_byte, true);
	ret = ds1302_read();
	gpio_put(GPIO_RTC_CE, 0);

	return ret;
}

void ds1302_write_register(const rtc_reg_wr reg, const uint8_t value)
{
	const uint8_t cmd_byte = reg;

	gpio_put(GPIO_RTC_CE, 1);
	sleep_us(4);
	ds1302_write(cmd_byte, false);
	ds1302_write(value, false);
	gpio_put(GPIO_RTC_CE, 0);
}

void func_rtcwrite(const char *cmd)
{
	int ret;
	uint8_t tx[7];
	uint8_t *send = &tx[0];

	//RTC WRITE <DAY>:<DATE>/<MONTH>/<YEAR> <HOUR>:<MIN>:<SEC>
	ret = sscanf(cmd, "RTC WRITE %hhx:%hhx/%hhx/%hhx %hhx:%hhx:%hhx",
		&tx[5], &tx[3], &tx[4],
		&tx[6], &tx[2], &tx[1],
		&tx[0]);
	if(ret != 7)
	{
		printf("sscanf acquired only %d items of %d from string "
		       "'%s'\n", ret, 7, cmd);
		return;
	}

	ds1302_write_register(RTC_WRITE_PROTECT, 0x00);
	for(rtc_reg_wr reg = RTC_SEC; reg <= RTC_YEAR; reg += 2)
	{
		ds1302_write_register(reg, *send);
		send++;
	}
	ds1302_write_register(RTC_WRITE_PROTECT, 0xFF);

	return;
}

void func_rtcread(const char *cmd)
{
	// io_wo_32 *rdtx = (io_wo_32 *) &pio1->txf[PIO1_SM_RTC_RD];
	// io_wo_32 *wrtx = (io_wo_32 *) &pio1->txf[PIO1_SM_RTC_WR];

	(void) cmd;

	for(rtc_reg_wr reg = RTC_SEC; reg <= RTC_TRICKLE_CHARGER; reg += 2)
	{
		printf("%02X: %02X\n", reg, ds1302_read_register(reg));
	}

	for(rtc_reg_wr reg = RTC_RAM_0; reg <= RTC_RAM_31; reg += 2)
	{
		printf("%02X ", ds1302_read_register(reg));
		if((reg % 8) == 7)
			putchar('\n');
	}

	putchar('\n');

#if 0
	gpio_put(GPIO_RTC_CE, 1);
	sleep_us(2);
	*wrtx = RTC_RAM_0;
	*wrtx = 0x69;
	gpio_put(GPIO_RTC_CE, 0);
	sleep_us(2);

	for(uint8_t reg_rd = RTC_SEC + RTC_RED_RD_BIT;
		reg_rd <= (RTC_TRICKLE_CHARGER + RTC_RED_RD_BIT);
		reg_rd += 2)
	{
		printf("%02X: ", reg_rd);

		gpio_put(GPIO_RTC_CE, 1);
		sleep_us(2);
		*wrtx = reg_rd;
		*rdtx = 1;
		while(pio_sm_is_rx_fifo_empty(pio1, PIO1_SM_RTC_RD) == true)
			sleep_us(2);

		gpio_put(GPIO_RTC_CE, 0);

		printf("%02X\n", *rdrx);
		sleep_us(2);
	}

	for(uint8_t reg_rd = RTC_RAM_0 + RTC_RED_RD_BIT;
		reg_rd <= (RTC_RAM_31 + RTC_RED_RD_BIT);
		reg_rd += 2)
	{
		printf("%02X: ", reg_rd);

		gpio_put(GPIO_RTC_CE, 1);
		*wrtx = reg_rd;
		*rdtx = 1;
		while(pio_sm_is_rx_fifo_empty(pio1, PIO1_SM_RTC_RD) == true)
			sleep_us(2);

		gpio_put(GPIO_RTC_CE, 0);

		printf("%02X\n", *rdrx);
		sleep_us(2);
	}
#endif
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

void func_info(const char *cmd)
{
	(void) cmd;

	/* Get RP2040 version information. */
	{
		const uint8_t chip = rp2040_chip_version();
		const uint8_t rom = rp2040_rom_version();

		printf("RP2040:\n"
			"  Chip: %d\n"
			"  ROM: %d\n",
			chip, rom);

		printf("  Frequencies:\n");
		for(unsigned i = 0; i < CLK_COUNT; i++)
		{
			static const char *clkstr[] = {
				"GPOUT0", "GPOUT1", "GPOUT2", "GPOUT3",
				"REF", "SYS", "PERI", "USB", "ADC", "RTC"
			};
			printf("    %-6s: %9lu\n", clkstr[i], clock_get_hz(i));
		}
	}

	/* Get information on connected FRAM. */
	{
		const uint8_t src[1] = { 0b10011111 };
		uint8_t dst[4];
		unsigned mem_sz;

		gpio_put(SPI_CSn, 0);
		spi_write_blocking(spi0, src, sizeof(src));
		spi_read_blocking(spi0, 0x00, dst, sizeof(dst));
		gpio_put(SPI_CSn, 1);

		mem_sz = 1 << (dst[2] & 0b11111);

		printf("FRAM:\n"
			"  Manufacturer: %02X\n"
			"  Continuation Code: %02X\n"
			"  Product ID: %02X %02X\n"
			"  Size: %u KiB\n",
			dst[0], dst[1], dst[2], dst[3], mem_sz);
	}
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

static inline void init_peripherals(void)
{
	/** SIO **/
	/* Initialise GPIO states. */
	gpio_init_mask(1 << GPIO_LED_GREEN |
			1 << GPIO_SWITCH | /* Not required for inputs. */
			1 << GPIO_MOTOR |
			1 << GPIO_GB_RESET |
			1 << SPI_CSn |
			1 << PIO_RTC_SCLK |
			1 << PIO_RTC_IO |
			1 << GPIO_RTC_CE);
	/* Set GPIO pin directions. */
	gpio_set_dir_out_masked(1 << GPIO_LED_GREEN |
			1 << GPIO_MOTOR |
			1 << GPIO_GB_RESET |
			1 << SPI_CSn |
			1 << PIO_RTC_SCLK |
			1 << GPIO_RTC_CE);
	/* Set initial output state. */
	gpio_set_mask(0 << GPIO_LED_GREEN |
			0 << GPIO_MOTOR |
			1 << GPIO_GB_RESET | /* Hold GB in reset. */
			1 << SPI_CSn |
			0 << PIO_RTC_SCLK |
			0 << GPIO_RTC_CE);

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

#if 0
	/* Initialise PIO1 (RTC) */
	ds1302_program_init(pio1, PIO1_SM_RTC_WR, PIO1_SM_RTC_RD);
	/* Enable state machines. */
	pio_sm_set_enabled(pio1, PIO1_SM_RTC_WR, true);
	/* PIO_SM_NCS should be enabled when cart RAM access is expected. */
	pio_sm_set_enabled(pio1, PIO1_SM_RTC_RD, true);
#endif

	/** SPI **/
	/* Default settings of spi_init are correct for the MB85RS256B. */
	spi_init(spi0, MB85RS256B_BAUDRATE);
	gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
	gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
	gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
}

int main(void)
{
	clocks_init();
	/* Unused clocks are stopped. */
	//clock_stop(clk_adc);
	//clock_stop(clk_rtc);

	{
		/* The value for VCO set here is meant for least power
		 * consumption. */
		const unsigned vco = 500000000;
		const unsigned div1 = 2, div2 = 1;

		vreg_set_voltage(VREG_VOLTAGE_1_15);
		sleep_ms(4);
		set_sys_clock_pll(vco, div1, div2);
		sleep_ms(4);
	}

	init_peripherals();

	/* If baudrate is set to PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE, then the
	 * RP2040 will reset to BOOTSEL mode. */
	stdio_usb_init();

	usb_commander();

	return 0;
}
