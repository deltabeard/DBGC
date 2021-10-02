#include <sys/timespec.h>
#include <sys/cdefs.h>

#define _GNU_SOURCE

#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>
#include <pico/binary_info.h>
#include <hardware/i2c.h>
#include <hardware/clocks.h>
#include <hardware/rtc.h>
#include <string.h>
#include <stdlib.h>
#include <pico/util/datetime.h>
#include <core2.h>
#include <pico/multicore.h>

#define ARRAYSIZE(array)	(sizeof(array)/sizeof(array[0]))
#define CLR_SCRN		"\033[2J"

#define I2C_PCA9536_ADDR 0b01000001
#define I2C_DS3231M_ADDR 0b01101000

#define CURRENT_MILLENNIUM	21
#define RTC_YEARS_EPOCH		((CURRENT_MILLENNIUM - 1) * 100)

struct func_map {
	char *long_arg;
	char *help;
	void (*func)(const char *cmd);
};

void func_help(const char *cmd);
void func_i2cscan(const char *cmd);
void func_i2csend(const char *cmd);
void func_i2crecv(const char *cmd);
void func_rtctemp(const char *cmd);
void func_rtcread(const char *cmd);
void func_rtcwrite(const char *cmd);
void func_gb(const char *cmd);

void func_pio(const char *cmd);
void func_bus(const char *cmd);
void func_date(const char *cmd);
void func_reboot(const char *cmd);

static const struct func_map map[] = {
	{ "HELP",	"Print usage information",		func_help	},
	{ "I2C SCAN",	"Perform I2C bus scan",			func_i2cscan	},
	{ "I2C SEND",	"Send bytes 0xDD to address 0xAA on I2C bus\n"
			     "\t'I2C SEND 0xAA 0xDD [0xDD ...]'", func_i2csend	},
	{ "I2C RECV",	"Receive a byte from address 0xAA on I2C bus\n"
			     "\t'I2C RECV 0xAA'",		func_i2crecv	},
	{ "RTC TEMP",	"Read temperature from RTC",		func_rtctemp	},
	{ "RTC READ",	"Read date and time from RTC and set internal RTC",
								func_rtcread	},
	{ "RTC WRITE",	"Write date and time to internal RTC and set external RTC \n"
			"\t'RTC WRITE <DOTW>:<DAY>/<MONTH>/<YEAR> <HOUR>:<MIN>:<SEC>'",
			      					func_rtcwrite	},
	{ "GB",		"Turn GB on (0) or off (1)\n"
			       "\t'GB 1'",			func_gb		},
	{ "DATE",	"Read date and time from internal RTC",	func_date	},
	{ "PIO",	"Display state of PIO",			func_pio	},
	{ "BUS",	"Display PIO/GB bus",			func_bus	},
	{ "REBOOT",	"Reboot to USBBOOT",			func_reboot 	}
};

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
	RTC_CONTROL_TEMP_MSB = 0x11,
	RTC_CONTROL_TEMP_LSB = 0x12
} rtc_reg;

typedef enum {
	IO_EXP_INPUT_PORT = 0,
	IO_EXP_OUTPUT_PORT,
	IO_EXP_INVERSION,
	IO_EXP_DIRECTION
} io_exp_reg;

#include <core2.h>
#include <hardware/pio.h>
#include <hardware/irq.h>
#include "comms.pio.h"

void func_pio(const char *cmd)
{
	unsigned times = 1;

	/* Power cycle GB. */
	func_gb("GB 1");
	sleep_ms(10);
	func_gb("GB 0");

	pio_sm_clear_fifos(pio0, PIO_SM_A15);
	pio_sm_clear_fifos(pio0, PIO_SM_DO);

	while (getchar_timeout_us(0) == PICO_ERROR_TIMEOUT)
	{
		/**
		 * PIO_SM_A15 state machine RX value:
		 * | 8-bit Data | 16-bit Address | CS | RD | PHI | 00000 |
		 * | Unused     | Readable       | U  | R  | U   | UUUUU |
		 * Legend: U-Unused, R-Readable
		 */
		if (pio_sm_is_rx_fifo_empty(pio0, PIO_SM_A15) == false)
		{
			unsigned address;

			address = pio_sm_get(pio0, PIO_SM_A15);
			address >>= 16;
			pio_sm_put(pio0, PIO_SM_DO, 0b01010101);
			printf("0x%04X\t", address);

			if(times % 8 == 0)
				puts("");

			times++;
		}
	}

	printf("Exiting PIO printing\n");
	return;
}

void func_bus(const char *cmd)
{
	/* Power cycle GB. */
	func_gb("GB 1");
	sleep_ms(10);
	func_gb("GB 0");

	pio_sm_clear_fifos(pio0, PIO_SM_A15);
	pio_sm_clear_fifos(pio0, PIO_SM_DO);

	puts("ADDR\tPIO\t~RD\t~CS\tA15-PC\tDO-PC");

	while (getchar_timeout_us(0) == PICO_ERROR_TIMEOUT)
	{
		/**
		 * PIO_SM_A15 state machine RX value:
		 * | 8-bit Data | 16-bit Address | CS | RD | PHI | 00000 |
		 * | Unused     | Readable       | U  | R  | U   | UUUUU |
		 * Legend: U-Unused, R-Readable
		 */
		if (pio_sm_is_rx_fifo_empty(pio0, PIO_SM_A15) == false)
		{
			static unsigned one_time = 0;
			unsigned address;

			address = pio_sm_get(pio0, PIO_SM_A15);
			address >>= 16;

			pio_sm_put(pio0, PIO_SM_DO, 0x0F);

			while(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_A15) == true)
			{
				if(one_time == 1)
					break;

				printf("0x%04X\t%hhu\t%hhu\t%hhu\t%hhu\t%hhu\n",
				       address,
				       gpio_get(PIO_PHI), gpio_get(PIO_NRD),
				       gpio_get(PIO_NCS),
				       pio_sm_get_pc(pio0, PIO_SM_DO),
				       pio_sm_get_pc(pio0, PIO_SM_DO));

				one_time = 1;
			}
		}
	}

	printf("Exiting PIO printing\n");
	return;
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
	datetime_t dt;
	uint8_t tx[8];

	tx[0] = RTC_SEC;

	//RTC WRITE <DOTW>:<DAY>/<MONTH>/<YEAR> <HOUR>:<MIN>:<SEC>
	ret = sscanf(cmd, "RTC WRITE %hhd:%hhd/%hhd/%hd %hhd:%hhd:%hhd",
		&dt.dotw, &dt.day, &dt.month, &dt.year,
		&dt.hour, &dt.min, &dt.sec);
	if(ret != 7)
	{
		printf("sscanf acquired only %d items of %d from string "
		       "'%s'\n", ret, 7, cmd);
		return;
	}

	if(rtc_set_datetime(&dt) == false)
	{
		printf("Failed to set internal RTC\n");
		return;
	}

	dt.year -= RTC_YEARS_EPOCH;
	tx[1] = dt.sec;
	tx[2] = dt.min;
	tx[3] = dt.hour;
	tx[4] = dt.dotw + 1;
	tx[5] = dt.day;
	tx[6] = dt.month;
	tx[7] = dt.year;

	for(unsigned i = 1; i < sizeof(tx); i++)
	{
		printf("%hd\t", tx[i]);
		tx[i] = decToBcd(tx[i]);
	}
	printf("\n");

	for(unsigned i = 1; i < sizeof(tx); i++)
	{
		printf("%#04x\t", tx[i]);
	}
	printf("\n");

	/* Set 24-hour bit of hour register. */
	tx[3] |= 0b01000000;

	ret = i2c_write_blocking(i2c_default, I2C_DS3231M_ADDR,
		tx, sizeof(tx), false);
	if(ret == PICO_ERROR_GENERIC)
	{
		printf("Error setting external RTC\n");
		return;
	}

	func_date(NULL);
	return;
}

void func_rtcread(const char *cmd)
{
	uint8_t tx = RTC_SEC;
	uint8_t rx[RTC_YEAR + 1];
	int ret;
	datetime_t dt;

	(void) cmd;

	/* Select second register. */
	ret = i2c_write_blocking(i2c_default, I2C_DS3231M_ADDR, &tx, 1, false);
	if(ret == PICO_ERROR_GENERIC)
	{
		printf("Error writing to RTC: %d\n", ret);
		return;
	}

	/* Read time values. */
	ret = i2c_read_blocking(i2c_default, I2C_DS3231M_ADDR, rx, sizeof(rx), false);
	if(ret == PICO_ERROR_GENERIC)
	{
		printf("Error reading from RTC: %d\n", ret);
		return;
	}

	dt.sec  = bcd_to_int(rx[RTC_SEC]);
	dt.min  = bcd_to_int(rx[RTC_MIN]);
	dt.hour = bcd_to_int(rx[RTC_HOUR] & 0b00111111);
	dt.dotw = rx[RTC_DAY] - 1;
	dt.day  = bcd_to_int(rx[RTC_DATE]);
	dt.month = bcd_to_int(rx[RTC_MONTH] & 0b00011111);
	dt.year = bcd_to_int(rx[RTC_YEAR]) + RTC_YEARS_EPOCH;
	/* TODO: Year 3000 problem? */

	rtc_init();
	if(rtc_set_datetime(&dt) == false)
	{
		printf("Datetime is not valid\n");
		return;
	}

	func_date(NULL);

	return;
}

void func_date(const char *cmd)
{
	char datetime_buf[256];
	char *datetime_str = &datetime_buf[0];
	datetime_t dt;

	(void) cmd;

	if(rtc_get_datetime(&dt) == false)
	{
		printf("RTC is not initialised\n");
		return;
	}
	datetime_to_str(datetime_str, sizeof(datetime_buf), &dt);
	printf("%s\n", datetime_str);
}

void func_rtctemp(const char *cmd)
{
	uint8_t tx = RTC_CONTROL_TEMP_MSB;
	uint8_t rx[2];
	const char *frac[4] = {
		".00", ".25", ".50", ".75"
	};
	int ret;
	int8_t t;

	(void) cmd;

	/* Select first temperature register. */
	ret = i2c_write_blocking(i2c_default, I2C_DS3231M_ADDR, &tx, 1, false);
	if(ret == PICO_ERROR_GENERIC)
	{
		printf("Error writing to RTC: %d\n", ret);
		return;
	}

	/* Read both temperature registers. */
	ret = i2c_read_blocking(i2c_default, I2C_DS3231M_ADDR, rx, sizeof(rx), false);
	if(ret == PICO_ERROR_GENERIC)
	{
		printf("Error reading from RTC: %d\n", ret);
		return;
	}

	t = (int8_t)rx[0];

	rx[1] >>= 6;
	printf("Temperature: %d%s Celsius\n", t, frac[rx[1]]);

	return;
}

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr)
{
	return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void func_i2cscan(const char *cmd)
{
	(void) cmd;

	printf("\nI2C Bus Scan\n");
	printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

	for (int addr = 0; addr < (1 << 7); ++addr)
	{
		int ret;
		uint8_t rxdata;

		if (addr % 16 == 0)
		{
			printf("%02x ", addr);
		}

		// Perform a 1-byte dummy read from the probe address. If a slave
		// acknowledges this address, the function returns the number of bytes
		// transferred. If the address byte is ignored, the function returns
		// -1.

		// Skip over any reserved addresses.
		if (reserved_addr(addr))
			ret = PICO_ERROR_GENERIC;
		else
			ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

		printf(ret < 0 ? "_" : "@");
		printf(addr % 16 == 15 ? "\n" : "  ");
	}
}

void func_i2csend(const char *cmd)
{
	int ret;
	uint8_t addr;
	uint8_t data[8];
	unsigned len = 0;
	char *endptr;

	(void) cmd;

	if(strlen(cmd) < strlen("I2C SEND 0x00 0x00"))
	{
		printf("Syntax error: string length incorrect\n");
		return;
	}

	cmd += strlen("I2C SEND");
	addr = (uint8_t)strtol(cmd, &endptr, 0);
	if(endptr == cmd && addr == 0)
	{
		printf("Syntax error: failed to decode address\n");
		return;
	}

	cmd = endptr;
	data[len] = (uint8_t)strtol(cmd, &endptr, 0);
	if(endptr == cmd && data[0] == 0)
	{
		printf("Syntax error: failed to decode data\n");
		return;
	}
	len++;

	while(len < sizeof(data))
	{
		cmd = endptr;
		data[len] = (uint8_t)strtol(cmd, &endptr, 0);
		if(endptr == cmd && data[len] == 0)
			break;

		len++;
	}

	printf("Sending command %#04x with %d bytes to %#04x\n", data[0], len,
		addr);

	ret = i2c_write_blocking(i2c_default, addr, data, len, false);
	if(ret == PICO_ERROR_GENERIC)
		printf("Error %d\n", ret);
	else
		printf("Sent %d byte(s)\n", ret);
}

void func_i2crecv(const char *cmd)
{
	int ret;
	uint8_t addr;
	char *endptr;
	uint8_t dat[1];

	(void) cmd;

	if(strlen(cmd) != strlen("I2C RECV 0x00"))
	{
		printf("Syntax error: string length incorrect\n");
		return;
	}

	cmd += strlen("I2C RECV");
	addr = (uint8_t)strtol(cmd, &endptr, 0);
	if(endptr == cmd && addr == 0)
	{
		printf("Syntax error: failed to decode address\n");
		return;
	}

	printf("Receiving byte from %#04x\n", addr);

	ret = i2c_read_blocking(i2c_default, addr, dat, sizeof(dat), false);
	if(ret == PICO_ERROR_GENERIC)
		printf("Error %d\n", ret);
	else
	{
		printf("Received byte: %#04x, 0b", dat[0]);
		for(int i = 7; i >= 0; i--)
			printf("%d", (dat[0] >> i) & 1);

		printf("\n");
	}
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

	/* Set external RTC configuration. */
	{
		uint8_t tx[2];
		tx[0] = RTC_CONTROL;
		tx[1] = 0b00111100;
		i2c_write_blocking(i2c_default, I2C_DS3231M_ADDR, tx,
			sizeof(tx), false);
	}

	/* Set external IO expander configuration. */
	{
		uint8_t tx[2];
		tx[0] = IO_EXP_DIRECTION;
		tx[1] = 0b11111110;
		i2c_write_blocking(i2c_default, I2C_PCA9536_ADDR, tx,
				   sizeof(tx), false);
	}

	/* Initialise Game Boy data communication. */
	{
		gb_bus_program_init(pio0, PIO_SM_A15, PIO_SM_DO);

		/* Enable IRQ0 to trigger when data in RX FIFO. */
#if 0
		pio_set_irq0_source_mask_enabled(pio0,
						 PIO_INTR_SM0_RXNEMPTY_LSB |
						 PIO_INTR_SM1_RXNEMPTY_LSB |
						 PIO_INTR_SM2_RXNEMPTY_LSB,
						 true);
		irq_set_exclusive_handler(PIO0_IRQ_0, pio0_irq0);
		irq_set_enabled(PIO0_IRQ_0, true);
#endif

		gpio_set_dir(PIO_DIR, true);
		/* Enable state machines. */
		pio_sm_set_enabled(pio0, PIO_SM_A15, true);
		pio_sm_set_enabled(pio0, PIO_SM_DO,  true);
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
