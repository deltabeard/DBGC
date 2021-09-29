#define _GNU_SOURCE

#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>
#include <pico/binary_info.h>
#include <hardware/i2c.h>
#include <hardware/clocks.h>
#include <string.h>
#include <stdlib.h>

#define arraysize(array)	(sizeof(array)/sizeof(array[0]))
#define CLR_SCRN		"\033[2J"

struct func_map {
	char *long_arg;
	char *help;
	void (*func)(const char *cmd);
};

void func_help(const char *cmd);
void func_i2cscan(const char *cmd);
void func_i2csend(const char *cmd);
void func_i2crecv(const char *cmd);
void func_reboot(const char *cmd);

static const struct func_map map[] = {
	{ "HELP", "Print usage information",	func_help },
	{ "I2C SCAN", "Perform I2C bus scan",	func_i2cscan },
	{ "I2C SEND", "Send bytes 0xDD to address 0xAA on I2C bus 'I2C SEND "
		      "0xAA 0xDD [0xDD ...]'", func_i2csend },
	{ "I2C RECV", "Receive a byte from address 0xAA on I2C bus 'I2C "
		      "RECV 0xAA'", func_i2crecv },
	{ "REBOOT", "Reboot to USBBOOT",	func_reboot },
};

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

	for (int addr = 0; addr < (1 << 7); ++addr) {
		if (addr % 16 == 0) {
			printf("%02x ", addr);
		}

		// Perform a 1-byte dummy read from the probe address. If a slave
		// acknowledges this address, the function returns the number of bytes
		// transferred. If the address byte is ignored, the function returns
		// -1.

		// Skip over any reserved addresses.
		int ret;
		uint8_t rxdata;
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

void func_reboot(const char *cmd)
{
	(void) cmd;
	reset_usb_boot(0, 0);
}

void func_help(const char *cmd)
{
	(void) cmd;

	puts("Usage:");
	for(unsigned i = 0; i < arraysize(map); i++)
	{
		printf("%s: %s\r", map[i].long_arg, map[i].help);
	}
}

int main(void)
{
	char buf[64];

	set_sys_clock_48mhz();

	i2c_init(i2c_default, 400 * 1000);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_disable_pulls(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_disable_pulls(PICO_DEFAULT_I2C_SCL_PIN);
	// Make the I2C pins available to picotool
	bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));


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
		if(buf[i] == '\r')
		{
			buf[i] = '\0';
			break;
		}
	}

	if(buf[0] == '\0')
		strcpy(buf, "<no input>");

	for(unsigned i = 0; i < arraysize(map); i++)
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
