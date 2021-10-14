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
void dma_test(const char *cmd);

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
	{ "DMA",	"Test DMA Speed",			dma_test	},
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

#include <megaman1.gb.h>
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

const uint8_t cart_mbc_lut[] =
	{
		0, 1, 1, 1, -1, 2, 2, -1, 0, 0, -1, 0, 0, 0, -1, 3,
		3, 3, 3, 3, -1, -1, -1, -1, -1, 5, 5, 5, 5, 5, 5, -1
	};
const uint8_t cart_ram_lut[] =
	{
		0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
		1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0
	};
const uint16_t num_rom_banks_lut[] =
	{
		2, 4, 8, 16, 32, 64, 128, 256, 512, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 72, 80, 96, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	};
const uint8_t num_ram_banks_lut[] = {0, 1, 1, 4, 16, 8};
const uint32_t ram_sizes[] =
	{
		0x00, 0x800, 0x2000, 0x8000, 0x20000
	};

#define DMA_COPY_ROM 0

// This example DMAs 16kB of data from the start of flash to SRAM, and
// measures the transfer speed.
//
// The SSI (flash interface) inside the XIP block has DREQ logic, so we can
// DMA directly from its FIFOs. Unlike the XIP stream hardware (see
// flash_xip_stream.c) this can *not* be done whilst code is running from
// flash, without careful footwork like we do here. The tradeoff is that it's
// ~2.5x as fast in QSPI mode, ~2x as fast in SPI mode.

void __no_inline_not_in_flash_func(flash_bulk_read)(uint32_t *rxbuf, uint32_t flash_offs, size_t len,
						    uint dma_chan) {
	// SSI must be disabled to set transfer size. If software is executing
	// from flash right now then it's about to have a bad time
	ssi_hw->ssienr = 0;
	ssi_hw->ctrlr1 = len - 1; // NDF, number of data frames
	ssi_hw->dmacr = SSI_DMACR_TDMAE_BITS | SSI_DMACR_RDMAE_BITS;
	ssi_hw->ssienr = 1;
	// Other than NDF, the SSI configuration used for XIP is suitable for a bulk read too.

	// Configure and start the DMA. Note we are avoiding the dma_*() functions
	// as we can't guarantee they'll be inlined
	dma_hw->ch[dma_chan].read_addr = (uint32_t) &ssi_hw->dr0;
	dma_hw->ch[dma_chan].write_addr = (uint32_t) rxbuf;
	dma_hw->ch[dma_chan].transfer_count = len;
	// Must enable DMA byteswap because non-XIP 32-bit flash transfers are
	// big-endian on SSI (we added a hardware tweak to make XIP sensible)
	dma_hw->ch[dma_chan].ctrl_trig =
		DMA_CH0_CTRL_TRIG_BSWAP_BITS |
		DREQ_XIP_SSIRX << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB |
		dma_chan << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB |
		DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS |
		DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB |
		DMA_CH0_CTRL_TRIG_EN_BITS;

	// Now DMA is waiting, kick off the SSI transfer (mode continuation bits in LSBs)
	ssi_hw->dr0 = (flash_offs << 8u) | 0xa0u;

	// Wait for DMA finish
	while (dma_hw->ch[dma_chan].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS);

	// Reconfigure SSI before we jump back into flash!
	ssi_hw->ssienr = 0;
	ssi_hw->ctrlr1 = 0; // Single 32-bit data frame per transfer
	ssi_hw->dmacr = 0;
	ssi_hw->ssienr = 1;
}

#define DATA_SIZE_WORDS 4096

uint32_t rxdata[DATA_SIZE_WORDS];
uint32_t *expect = (uint32_t *) XIP_NOCACHE_NOALLOC_BASE;

void dma_test(const char *cmd)
{
	uint32_t interrupts;
	uint32_t words;

	cmd += strlen("DMA ");
	words = strtoul(cmd, NULL, 10);
	if(words == 0 || words > DATA_SIZE_WORDS)
	{
		printf("Invalid word count\n");
		return;
	}

	memset(rxdata, 0, words * sizeof(uint32_t));

	printf("Starting DMA transfer of %lu bytes\n", words * sizeof(uint32_t));
	interrupts = save_and_disable_interrupts();
	uint32_t start_time = time_us_32();
	flash_bulk_read(rxdata, 0, words, 0);
	uint32_t finish_time = time_us_32();
	restore_interrupts(interrupts);
	printf("DMA finished\n");

	float elapsed_time_s = 1e-6f * (float)(finish_time - start_time);
	printf("Transfer speed: %.3f MB/s\n", ((float)sizeof(uint32_t) * words / 1e6f) / elapsed_time_s);

	bool mismatch = false;
	for (int i = 0; i < words; ++i) {
		if (rxdata[i] != expect[i]) {
			printf("Mismatch at %d: expected %08x, got %08x\n", i, expect[i], rxdata[i]);
			mismatch = true;
			break;
		}
	}
	if (!mismatch)
		printf("Data check ok\n");
}

void power_gb(bool turn_gb_on)
{
	uint8_t tx[2];
	tx[0] = IO_EXP_OUTPUT_PORT;
	tx[1] = 0b11111110 | !turn_gb_on;
	i2c_write_blocking(i2c_default, I2C_PCA9536_ADDR, tx,
			   sizeof(tx), false);
}

void __no_inline_not_in_flash_func(core1_pio_manager)(void){
	unsigned sm_a15, sm_do;
	const unsigned char *rom_flash;
	const unsigned char *rom;
	uint32_t rom_len;

	/* Cartridge information:
	 * Memory Bank Controller (MBC) type. */
	uint8_t mbc;
	/* Whether the MBC has internal RAM. */
	uint8_t cart_ram;
	/* Number of ROM banks in cartridge. */
	uint16_t num_rom_banks;

	uint16_t selected_rom_bank = 1;

	/* This will panic if sm is not available. */
	sm_a15 = pio_claim_unused_sm(pio0, true);
	sm_do = pio_claim_unused_sm(pio0, true);
	gb_bus_program_init(pio0, sm_a15, sm_do);

	/* Initialise Game Boy data communication. */
	power_gb(false);

	/* Enable state machines. */
	pio_sm_set_enabled(pio0, sm_a15, true);
	pio_sm_set_enabled(pio0, sm_do,  true);

	switch(multicore_fifo_pop_blocking())
	{
		default:
	case 1:
		rom_flash = libbet_gb;
		rom_len = libbet_gb_len;
		break;

	case 2:
		rom_flash = gb240p_gb + (XIP_NOCACHE_NOALLOC_BASE - XIP_BASE);
		rom_len = gb240p_gb_len;
		break;

	case 3:
		rom_flash = megaman1_gb + (XIP_NOCACHE_NOALLOC_BASE - XIP_BASE);
		rom_len = megaman1_gb_len;
		break;
	}

#if 0
	rom = malloc(ROM_BANK_SIZE * 2);
	if(rom == NULL)
	{
		multicore_fifo_push_blocking(MULTICORE_CMD_ROM_FAIL);
		return;
	}
	memcpy(rom, rom_flash, ROM_BANK_SIZE * 2);
#elif DMA_COPY_ROM
	rom = malloc(ROM_BANK_SIZE * 2);
	if(rom == NULL)
	{
		multicore_fifo_push_blocking(MULTICORE_CMD_ROM_FAIL);
		return;
	}
	save_and_disable_interrupts();
	flash_bulk_read((uint32_t *) rom,
			(uint32_t) (rom_flash - XIP_NOCACHE_NOALLOC_BASE),
			ROM_BANK_SIZE * 2, 0);
#else
	rom = rom_flash;
#endif

	/* Let core0 know that we're running. */
	multicore_fifo_push_blocking(MULTICORE_CMD_SM_ENABLED);

	/* Initialise ROM data. */
	/* Check if cartridge type is supported, and set MBC type. */
	{
		const uint8_t mbc_value = rom[mbc_location];
		const uint8_t ram_sz_value = rom[ram_size_location];
		unsigned ram_sz;

		if(mbc_value > sizeof(cart_mbc_lut) - 1 ||
		   (mbc = cart_mbc_lut[mbc_value]) == 255u ||
		   mbc > 1)
		{
			multicore_fifo_push_blocking(MULTICORE_CMD_MBC_FAIL);
			return;
		}
		multicore_fifo_push_blocking(MULTICORE_CMD_MBC_ACCEPTED);

		cart_ram = cart_ram_lut[mbc_value];
		ram_sz = ram_sizes[ram_sz_value];

#if 0
		if(ram_sz != 0)
		{
			ram = malloc(ram_sz);
			if(ram == NULL)
			{
				multicore_fifo_push_blocking(
					MULTICORE_CMD_RAM_FAIL);
				return;
			}
		}
#else
		if(ram_sz != 0)
		{
			multicore_fifo_push_blocking(MULTICORE_CMD_RAM_FAIL);
			return;
		}
#endif
		multicore_fifo_push_blocking(MULTICORE_CMD_RAM_ACCEPTED);

		num_rom_banks = num_rom_banks_lut[rom[bank_count_location]];
		multicore_fifo_push_blocking(MULTICORE_CMD_ROM_ACCEPTED);
	}

	multicore_fifo_push_blocking(MULTICORE_CMD_PLAYING);

	/* Power cycle GB. */
	sleep_ms(100);
	power_gb(true);

	/* Disable cache. */
	//xip_ctrl_hw->ctrl &= ~XIP_CTRL_EN_BITS;
	/* XIP Cache is required for data fetching. */

	save_and_disable_interrupts();

	while(1)
	{
		/* Only read the address, which is stored in the most
		 * significant two bytes of the RX FIFO. */
		io_ro_32 *rx_sm_a15 = &pio0->rxf[sm_a15];
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

		while(pio_sm_is_rx_fifo_empty(pio0, sm_a15))
			tight_loop_contents();

		in.raw = *rx_sm_a15;
		address = in.address;

#if 1
		if(OPT_UNLIKELY(in.is_write))
		{
			/* If we need to write data to ROM, then we obtain the
			 * data byte from the third byte of the RX FIFO. */
			data = in.data;

			switch(address >> 12)
			{
			case 0x0:
			case 0x1:
				/* TODO: RAM enable is currently ignored. */
				break;

			case 0x2:
			case 0x3:
				if(mbc == 1)
				{
					//selected_rom_bank = data & 0x7;
					selected_rom_bank = (data & 0x1F) |
						(selected_rom_bank & 0x60);

					/* Select bank 1 if bank 0 is selected. */
					if((selected_rom_bank & 0x1F) == 0x00)
						selected_rom_bank++;
					else
					{
						/* Wrap bank number. */
						selected_rom_bank = selected_rom_bank %
							num_rom_banks;
					}
				}


				break;

			case 0x4:
			case 0x5:
				if(mbc == 1)
				{
					//cart_ram_bank = (data & 3);
					selected_rom_bank =
						((data & 3) << 5) |
						(selected_rom_bank & 0x1F);
					selected_rom_bank =
						selected_rom_bank %
						num_rom_banks;
				}

				break;

			case 0x6:
			case 0x7:
				//cart_mode_select = (data & 1);
				break;
			}

			continue;
		}
#endif
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
			data = rom[address + ((selected_rom_bank - 1) *
				   ROM_BANK_SIZE)];

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

	func_play("PLAY 3");
	printf("%s", CLR_SCRN);
	usb_commander();

	return 0;
}
