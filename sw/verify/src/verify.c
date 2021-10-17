#include <sys/cdefs.h>

#define _GNU_SOURCE

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>
#include <pico/binary_info.h>
#include <hardware/i2c.h>
#include <hardware/clocks.h>
#include <hardware/rtc.h>
#include <hardware/sync.h>
#include <pico/util/datetime.h>
#include <hardware/structs/xip_ctrl.h>
#include <verify.h>

#define OPT_LIKELY(expr)	__builtin_expect(!!(expr), 1)
#define OPT_UNLIKELY(expr)	__builtin_expect(!!(expr), 0)
#define OPT_INLINE		inline
#define OPT_FORCE_INLINE	__attribute__((__always_inline__)) OPT_INLINE

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
void func_set_clock(const char *cmd);

void func_play(const char *cmd);
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
	{ "PLAY",	"Play a game",			func_play	},
	{ "CLOCK",	"Set CPU clock speed",		func_set_clock	},
	{ "REBOOT",	"Reboot to USBBOOT",		func_reboot 	}
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

#include <hardware/pio.h>
#include "comms.pio.h"

#define ROM_BANK_SIZE   0x4000
#define CRAM_BANK_SIZE  0x2000
#define CART_RAM_ADDR   0xA000

#include <pokered.gbc.h>
#include <gb240p.gb.h>
#include <libbet.gb.h>
#include <hardware/vreg.h>
#include <pico/multicore.h>
#include <ctype.h>
#include <hardware/dma.h>
#include <hardware/structs/ssi.h>

typedef enum {
	MULTICORE_CMD_SM_ENABLED = 0,

	MULTICORE_CMD_MBC_ACCEPTED,
	MULTICORE_CMD_MBC_FAIL,

	MULTICORE_CMD_ROM_ACCEPTED,
	MULTICORE_CMD_ROM_FAIL,

	MULTICORE_CMD_RAM_ACCEPTED,
	MULTICORE_CMD_RAM_FAIL,

	MULTICORE_CMD_PLAYING
} multicore_cmd_e;

const uint16_t mbc_location = 0x0147;
const uint16_t bank_count_location = 0x0148;
const uint16_t ram_size_location = 0x0149;

const uint8_t cart_mbc_lut[0xFF] =
	{
	/* ROM Only */
	[0x00] = 0,

	/* MBC1 */
	[0x01] = 1, 1, 1,
	[0x04] = -1,

	/* MBC2 */
	[0x05] = 2, 2,
	[0x07] = -1,

	/* ROM with RAM (Unsupported) */
	[0x08] = -1, -1,
	[0x0A] = -1,

	/* MMM01 (Unsupported) */
	[0x0B] = -1, -1, -1,
	[0x0E] = -1,

	/* MBC3 */
	[0x0F] = 3, 3, 3, 3, 3,

	/* Everything else is unsupported for now. */
	[0x14] = -1
	};
const uint8_t cart_ram_lut[] =
	{
		0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
		1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0
	};
const uint16_t num_rom_banks_lut[] =
	{
		2, 4, 8, 16, 32, 64, 128, 256, 512
	};
const uint8_t num_ram_banks_lut[] = {0, 1, 1, 4, 16, 8};

void power_gb(bool turn_gb_on)
{
	uint8_t tx[2];
	tx[0] = IO_EXP_OUTPUT_PORT;
	tx[1] = 0b11111110 | !turn_gb_on;
	i2c_write_blocking(i2c_default, I2C_PCA9536_ADDR, tx,
			   sizeof(tx), false);
}

static inline bool pio_sm_is_rx_fifo_empty_mask(PIO pio, uint sm_mask) {
	check_pio_param(pio);
	return (pio->fstat & (sm_mask << PIO_FSTAT_RXEMPTY_LSB)) != 0;
}

void __no_inline_not_in_flash_func(core1_pio_manager)(void){
	unsigned sm_a15, sm_ncs, sm_do;
	const unsigned char *rom;
	unsigned char *ram = NULL;
	unsigned rom_sz, ram_sz;

	/* Cartridge information:
	 * Memory Bank Controller (MBC) type. */
	uint8_t mbc;
	/* Whether the MBC has internal RAM. */
	uint8_t cart_ram;
	/* Number of ROM banks in cartridge. */
	uint16_t num_rom_banks_mask;
	/* Number of RAM banks in cartridge. */
	uint8_t num_ram_banks;
	uint16_t selected_rom_bank = 1;
	uint8_t cart_ram_bank = 0;
	uint8_t enable_cart_ram = 0;
	/* Cartridge ROM/RAM mode select. */
	uint8_t cart_mode_select = 0;
	union
	{
		struct __attribute__ ((__packed__))
		{
			uint8_t sec;
			uint8_t min;
			uint8_t hour;
			uint8_t yday;
			uint8_t high;
		} rtc_bits;
		uint8_t cart_rtc[5];
	} rtc = { .cart_rtc = 0 };

	/* This will panic if sm is not available. */
	sm_a15 = pio_claim_unused_sm(pio0, true);
	sm_ncs = pio_claim_unused_sm(pio0, true);
	sm_do = pio_claim_unused_sm(pio0, true);
	gb_bus_program_init(pio0, sm_a15, sm_ncs, sm_do);

	/* Initialise Game Boy data communication. */
	power_gb(false);

	switch(multicore_fifo_pop_blocking())
	{
		default:
	case 1:
		rom = libbet_gb + (XIP_NOCACHE_NOALLOC_BASE - XIP_BASE);
		rom_sz = libbet_gb_len;
		break;

	case 2:
		rom = gb240p_gb + (XIP_NOCACHE_NOALLOC_BASE - XIP_BASE);
		rom_sz = gb240p_gb_len;
		break;

#if 1
	case 3:
		rom = pokered_gbc + (XIP_NOCACHE_NOALLOC_BASE - XIP_BASE);
		rom_sz = pokered_gbc_len;
		break;
#endif
	}

	/* Initialise ROM data. */
	/* Check if cartridge type is supported, and set MBC type. */
	{
		const uint8_t mbc_value = rom[mbc_location];
		const uint8_t ram_sz_value = rom[ram_size_location];

		if(mbc_value > sizeof(cart_mbc_lut) - 1 ||
			(mbc = cart_mbc_lut[mbc_value]) == 255u || mbc > 1)
		{
			multicore_fifo_push_blocking(MULTICORE_CMD_MBC_FAIL);
			return;
		}
		multicore_fifo_push_blocking(MULTICORE_CMD_MBC_ACCEPTED);

		num_ram_banks = num_ram_banks_lut[ram_sz_value];
		multicore_fifo_push_blocking(MULTICORE_CMD_RAM_ACCEPTED);

		cart_ram = cart_ram_lut[mbc_value];
		ram_sz = num_ram_banks * CRAM_BANK_SIZE;

		if(ram_sz != 0)
		{
			ram = calloc(ram_sz, sizeof(*ram));
			if(ram == NULL)
			{
				multicore_fifo_push_blocking(
					MULTICORE_CMD_RAM_FAIL);
				return;
			}
		}

		num_rom_banks_mask = num_rom_banks_lut[rom[bank_count_location]] - 1;
		multicore_fifo_push_blocking(MULTICORE_CMD_ROM_ACCEPTED);
	}

	/* Enable state machines. */
	pio_sm_set_enabled(pio0, sm_a15, true);
	pio_sm_set_enabled(pio0, sm_ncs, true);
	pio_sm_set_enabled(pio0, sm_do,  true);
	multicore_fifo_push_blocking(MULTICORE_CMD_SM_ENABLED);

	multicore_fifo_push_blocking(MULTICORE_CMD_PLAYING);

	/* Power cycle GB. */
	sleep_ms(100);
	power_gb(true);

	while(1)
	{
		/* Only read the address, which is stored in the most
		 * significant two bytes of the RX FIFO. */
		io_ro_32 *rx_sm_a15 = &pio0->rxf[sm_a15];
		io_ro_32 *rx_sm_ncs = &pio0->rxf[sm_ncs];
		io_wo_8 *tx_sm_do = (io_wo_8 *) &pio0->txf[sm_do];
		union {
			struct {
				uint16_t address;
				uint8_t data;
				uint8_t is_write;
			};
			uint32_t raw;
		} in;
		uint16_t address;
		uint8_t data;

		while(1)
		{
			if(pio_sm_is_rx_fifo_empty(pio0, sm_a15) == false)
			{
				in.raw = *rx_sm_a15;
				break;
			}
			else if(pio_sm_is_rx_fifo_empty(pio0, sm_ncs) == false)
			{
				in.raw = *rx_sm_ncs;

#if 1
				/* Catch invalid addresses here. */
				if(in.address < 0xA000 ||
						in.address > 0xBFFF)
					continue;
#endif

				break;
			}
		}

		multicore_fifo_push_blocking(in.raw);
		address = in.address;

		if(in.is_write)
		{
			/* If we need to write data to ROM, then we obtain the
			 * data byte from the third byte of the RX FIFO. */
			data = in.data;

			switch(address >> 12)
			{
			case 0x0:
			case 0x1:
				if(mbc > 0 && cart_ram)
					enable_cart_ram = ((data & 0x0F) == 0x0A);

				break;

			case 0x2:
			case 0x3:
				if(mbc == 1)
				{
					selected_rom_bank = (data & 0x1F) | (selected_rom_bank & 0x60);

					if((selected_rom_bank & 0x1F) == 0x00)
						selected_rom_bank++;
				}
				else if(mbc == 3)
				{
					selected_rom_bank = data & 0x7F;

					if(!selected_rom_bank)
						selected_rom_bank++;
				}

				selected_rom_bank = selected_rom_bank & num_rom_banks_mask;

				break;

			case 0x4:
			case 0x5:
				if(mbc == 1)
				{
					cart_ram_bank = (data & 3);
					selected_rom_bank = ((data & 3) << 5) | (selected_rom_bank & 0x1F);
					selected_rom_bank = selected_rom_bank & num_rom_banks_mask;
				}
				else if(mbc == 3)
					cart_ram_bank = data;

				break;

			case 0x6:
			case 0x7:
				cart_mode_select = (data & 1);
				break;

			case 0xA:
			case 0xB:
				if(cart_ram && enable_cart_ram)
				{
					if(mbc == 3 && cart_ram_bank >= 0x08)
						rtc.cart_rtc[cart_ram_bank -
						0x08] = data;
					else if(cart_mode_select &&
						cart_ram_bank < num_ram_banks)
					{
						ram[address - CART_RAM_ADDR + (cart_ram_bank * CRAM_BANK_SIZE)] = data;
					}
					else if(num_ram_banks)
					{
						ram[address -
						    CART_RAM_ADDR] = data;
					}
				}

				break;

			default:
				/* This is an invalid address. */
				break;
			}

			continue;
		}

		switch(address >> 12)
		{
		case 0x0:
		case 0x1:
		case 0x2:
		case 0x3:
			data = rom[address];
			break;

		case 0x4:
		case 0x5:
		case 0x6:
		case 0x7:
			if(mbc == 1 && cart_mode_select)
				data = rom[address + ((selected_rom_bank & 0x1F) - 1) * ROM_BANK_SIZE];
			else
				data = rom[address + (selected_rom_bank - 1) * ROM_BANK_SIZE];

			break;

		case 0xA:
		case 0xB:
			if(cart_ram && enable_cart_ram)
			{
				if(mbc == 3 && cart_ram_bank >= 0x08)
					data = rtc.cart_rtc[cart_ram_bank -
					0x08];
				else if((cart_mode_select || mbc != 1) &&
					cart_ram_bank < num_ram_banks)
				{
					data = ram[address - CART_RAM_ADDR +
						   (cart_ram_bank * CRAM_BANK_SIZE)];
				}
				else
					data = ram[address - CART_RAM_ADDR];
			}
			else
			{
				/* Cart RAM was not enabled. */
				data = 0xFF;
			}

			break;

		default:
			/* Ignore invalid addresses. */
			continue;
		}

		*tx_sm_do = data;
	}
}

_Noreturn void core1_main(void){
	//save_and_disable_interrupts();
	core1_pio_manager();

	while(1)
		__wfi();
}

void __no_inline_not_in_flash_func(func_play)(const char *cmd)
{
	static bool started = false;
	unsigned long game_selection;
	union {
		struct {
			uint16_t address;
			uint8_t data;
			uint8_t is_write;
		};
		uint32_t raw;
	} in_stash[8192];
	unsigned in_i = 0;

	if(started == true)
	{
		printf("Core1 already started\n");
		multicore_reset_core1();
		pio_clear_instruction_memory(pio0);
		pio_sm_set_enabled(pio0, 0, false);
		pio_sm_set_enabled(pio0, 1, false);
	}

	cmd += strlen("PLAY ");
	game_selection = strtoul(cmd, NULL, 10);
	if(game_selection == 0)
	{
		printf("Select a game between 1 and 3:\nPLAY 1\n");
		return;
	}

	printf("Starting Core1\n");
	multicore_launch_core1(core1_main);
	multicore_fifo_push_blocking(game_selection);

	while(1)
	{
		uint32_t out;

		out = multicore_fifo_pop_blocking();
		switch(out)
		{
		case MULTICORE_CMD_SM_ENABLED:
			printf("State machines enabled\n");
			break;

		case MULTICORE_CMD_MBC_ACCEPTED:
			printf("MBC accepted\n");
			break;

		case MULTICORE_CMD_MBC_FAIL:
			printf("MBC check failed\n");
			goto err;

		case MULTICORE_CMD_ROM_ACCEPTED:
			printf("ROM accepted\n");
			break;

		case MULTICORE_CMD_ROM_FAIL:
			printf("ROM rejected\n");
			goto err;

		case MULTICORE_CMD_RAM_ACCEPTED:
			printf("RAM accepted\n");
			break;

		case MULTICORE_CMD_RAM_FAIL:
			printf("RAM rejected\n");
			goto err;

		case MULTICORE_CMD_PLAYING:
			printf("Playing\n");
			started = true;
			while(in_i < ARRAYSIZE(in_stash) &&
			getchar_timeout_us(0) == PICO_ERROR_TIMEOUT)
			{
				in_stash[in_i].raw =
					multicore_fifo_pop_blocking();
				in_i++;
			}
			for(unsigned i = 0; i < in_i; i++)
			{
				printf("%c %04X %02X\t",
					in_stash[i].is_write != 0 ? 'W' : 'R',
					in_stash[i].address,
					in_stash[i].data);
			}
			return;

		default:
			break;
		}
	}

err:
	printf("Core1 did not start properly\n");
	multicore_reset_core1();
	pio_clear_instruction_memory(pio0);

	return;
}

void func_set_clock(const char *cmd)
{
	unsigned long vco;
	cmd += strlen("PLAY ");
	vco = strtoul(cmd, NULL, 10);
	if(vco < 500 || vco > 1500)
	{
		printf("CLOCK VCO\n"
		       "CPU clock will be set to VCO/2\n"
		       "VCO must be between 1500 and 500\n");
		return;
	}

	vco *= 1000 * 1000;
	printf("Setting clock to %lu\n", vco / 2);
	sleep_ms(10);
	vreg_set_voltage(VREG_VOLTAGE_1_20);
	sleep_ms(10);
	set_sys_clock_pll(vco, 2, 1);
	sleep_ms(10);
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
		for(unsigned i = 0; i < sizeof(rx); i++)
			printf("%02x ", rx[i]);

		printf("\n"
		       "%d %d %d %d %d %d %d\n",
			dt.sec, dt.min, dt.hour, dt.dotw, dt.day, dt.month,
			dt.year);
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

	power_gb(turn_gb_on);
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

_Noreturn void __no_inline_not_in_flash_func(usb_commander)(void)
{
	char buf[64];

new_cmd:
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

int main(void)
{
	/* Reduce power consumption to stop IO Expander Power-On Reset Errata. */
	sleep_ms(10);

	/* Set system clock to 276MHz and flash to 138MHz. */
	{
		const unsigned vco = 552000000;
		const unsigned div1 = 2, div2 = 1;

		vreg_set_voltage(VREG_VOLTAGE_1_15);
		sleep_ms(10);
		set_sys_clock_pll(vco, div1, div2);
		sleep_ms(10);
	}

	i2c_init(i2c_default, 100 * 1000);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_disable_pulls(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_disable_pulls(PICO_DEFAULT_I2C_SCL_PIN);

	// Make the I2C pins available to picotool
	bi_decl_if_func_used(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN,
		PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

	for(unsigned i = PIO_PHI; i <= PIO_A15; i++)
	{
		gpio_set_input_enabled(i, true);
		/* Disable schmitt triggers on GB Bus. The bus transceivers
		 * already have schmitt triggers. */
		gpio_set_input_hysteresis_enabled(i, false);
	}

#if 1
	for(unsigned i = PIO_PHI; i <= PIO_DIR; i++)
	{
		/* Use fast slew rate for GB Bus. */
		gpio_set_slew_rate(i, GPIO_SLEW_RATE_FAST);
	}
#endif

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

	//gpio_set_drive_strength(PIO_DIR, GPIO_DRIVE_STRENGTH_12MA);
	//gpio_set_function(PIO_DIR, GPIO_FUNC_PIO0);
	//gpio_set_dir(PIO_DIR, true);
	//gpio_put(PIO_DIR, 1);

	func_gb("GB 1");
	bi_decl_if_func_used(bi_program_feature("PIO0 Game Boy Bus"));

	/* If baudrate is set to PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE, then the
	 * RP2040 will reset to BOOTSEL mode. */
	stdio_init_all();

	//func_play("PLAY 3");
	printf("%s", CLR_SCRN);
	usb_commander();

	return 0;
}
