#define _GNU_SOURCE

#include <pico/stdio.h>
#include <pico/bootrom.h>
#include <stdio.h>
#include <string.h>

#define arraysize(array)    (sizeof(array)/sizeof(array[0]))

struct func_map {
	char *long_arg;
	char *help;
	void (*func)(const char *cmd, const char *arg);
};

void func_help(const char *cmd, const char *arg);
void func_reboot(const char *cmd, const char *arg);

static const struct func_map map[] = {
	{ "HELP", "Print usage information", func_help },
	{ "REBOOT", "Reboot to USBBOOT", func_reboot }
};

void func_reboot(const char *cmd, const char *arg)
{
	reset_usb_boot(0, 0);
}

void func_help(const char *cmd, const char *arg)
{
	(void) cmd;
	(void) arg;

	puts("Usage:");
	for(unsigned i = 0; i < arraysize(map); i++)
	{
		fputs(map[i].long_arg, stdout);
		fputs(": \t", stdout);
		puts(map[i].help);
	}
}

int main(void)
{
	char buf[64];
	size_t first_arg_len;

	/* If baudrate is set to PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE, then the
	 * RP2040 will reset to BOOTSEL mode. */
	stdio_init_all();

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
new_cmd:
	fputs("CMD> ", stdout);
	fgets(buf, sizeof(buf), stdin);

	/* Get length of first argument. */
	{
		char *end_of_first_arg;
		end_of_first_arg = strchrnul(buf, ' ');
		first_arg_len = end_of_first_arg - &buf[0];
	}

	for(unsigned i = 0; i < arraysize(map); i++)
	{
		int cmp;
		cmp = strncmp(buf, map[i].long_arg, first_arg_len);
		if(cmp == 0)
		{
			map[i].func(buf, &buf[first_arg_len]);
			goto new_cmd;
		}
	}

	puts("Unrecognised command.");
	goto new_cmd;

#pragma clang diagnostic pop
	__builtin_unreachable();
}
