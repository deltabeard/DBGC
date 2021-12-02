/**
 * DBGC core for the RP2040.
 * Copyright (c) 2021 Mahyar Koshkouei
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the above copyright notice and
 * this permission notice appear in all copies.
 * THIS SOFTWARE IS PROVIDED 'AS-IS', WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
 * IN NO EVENT WILL THE AUTHORS BE HELD LIABLE FOR ANY DAMAGES ARISING FROM
 * THE USE OF THIS SOFTWARE.
 */

/* The management ROM is used for selecting a game on boot, and for saving
 * data from the FRAM to NOR flash memory.
 * Remains disabled as it doesn't work properly. */
#define USE_MGMT_ROM 0

#include <sys/cdefs.h>
#include <stdlib.h>
#include <string.h>
#include <hardware/spi.h>
#include <hardware/pio.h>
#include <hardware/vreg.h>
#include <hardware/pll.h>
#include <hardware/dma.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/bootrom.h>
#include <hardware/structs/bus_ctrl.h>
#include <hardware/structs/xip_ctrl.h>

#include "comms.pio.h"
#include <cart.h>
#include <generic.h>

/* ROMs */
//#include <gb_manager.gb.h>
#include <libbet.gb.h>
//#include <pcss.gbc.h>

typedef enum {
	IO_EXP_INPUT_PORT = 0,
	IO_EXP_OUTPUT_PORT,
	IO_EXP_INVERSION,
	IO_EXP_DIRECTION
} io_exp_reg_e;

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
	GB_POWER_OFF = 1,
	GB_POWER_ON = 0
} gb_pwr_e;

union gb_bus_rx {
	struct {
		uint16_t address;
		uint8_t data;
		uint8_t is_write;
	};
	uint32_t raw;
};

union cart_rtc
{
	struct __attribute__ ((__packed__))
	{
		uint8_t sec;
		uint8_t min;
		uint8_t hour;
		uint8_t yday;
		uint8_t high;
	} rtc_bits;
	uint8_t bytes[5];
} rtc;

#if USE_MGMT_ROM
struct gb_mgmt_ctx {
	uint8_t cmd;
	uint8_t param;
	uint8_t ret;

	union {
		struct {
			const uint8_t *rom;
			uint8_t remaining_length;
		} game_name;
	} ctx;
};

const uint8_t *roms[] = {
	libbet_gb,
	__2048_gb
};
#endif

/* Save file storage. The first two bytes are the address to start writing to
 * in the FRAM. */
static uint8_t i2c_ram[32770] = { 0x00, 0x00 };
/* Pointer to the first byte of where the game will save data. */
static uint8_t *const ram = &i2c_ram[2];
/* Number of writes to cartridge RAM (battery backed RAM) since last sync
 * with FRAM. This should reduce FRAM writes and battery power a bit. */
static int ram_write = 0;

/**
 * Turn the Game Boy on or off.
 */
void gb_power(gb_pwr_e pwr)
{
	uint8_t tx[2];
	tx[0] = IO_EXP_OUTPUT_PORT;
	tx[1] = 0b11111110 | pwr;
	//i2c_write_blocking(i2c_default, I2C_PCA9536_ADDR, tx,
	//		   sizeof(tx), false);

	return;
}

/**
 * Initialise GPIO pins for this application.
 */
void init_gpio_pins(void)
{
	/** SIO **/
	/* Initialise GPIO states. */
	gpio_init_mask(
		1 << GPIO_LED_GREEN |
		1 << GPIO_SWITCH | /* Not required for inputs. */
		1 << GPIO_MOTOR |
		1 << GPIO_GB_RESET |
		1 << SPI_CSn);
	/* Set GPIO pin directions. */
	gpio_set_dir_out_masked(
		1 << GPIO_LED_GREEN |
		1 << GPIO_MOTOR |
		1 << GPIO_GB_RESET |
		1 << SPI_CSn);
	/* Set initial output state. */
	gpio_set_mask(
		0 << GPIO_LED_GREEN |
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

	/** SPI **/
	/* Default settings of spi_init are correct for the MB85RS256B. */
	spi_init(spi0, MB85RS256B_BAUDRATE);
	gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
	gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
	gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
}

uint8_t bcd_to_int(uint8_t x)
{
	return x - 6 * (x >> 4);
}

uint_fast16_t get_day_num(uint8_t year, uint8_t month, uint8_t day)
{
	uint8_t days_in_month[12] = {
		31, 28, 31, 30, 31, 30, 31, 30, 31, 30, 31, 30
	};
	uint_fast16_t days = 0;

	/* Day must be more than 1 and less than 32.
	 * Month must be 1 to 12.
	 * Year must be 0 to 99. */

	for(unsigned i = 0; i < month; i++)
		days += days_in_month[i];

	if(month > 2 && year % 4 == 0)
		days++;

	days += day - 1;

	return days;
}

#if 0
/**
 * Initialise the I2C peripherals. This also reads save data from the FRAM.
 * \return	PICO_ERROR_GENERIC on error or bytes written.
 */
int init_i2c_peripherals(void)
{
	int ret = 0;

	/* Initialise I2C. */
	UNUSED_RET i2c_init(i2c_default, 400 * 1000);

	/* Set external IO expander configuration. */
	{
		uint8_t tx[2];
		tx[0] = IO_EXP_DIRECTION;
		tx[1] = 0b11111010;
		ret = i2c_write_blocking(i2c_default, I2C_PCA9536_ADDR, tx,
			sizeof(tx), false);

		if(ret == PICO_ERROR_GENERIC)
			goto out;

		/* Set to input address for future button polling. */
		tx[0] = IO_EXP_INPUT_PORT;
		UNUSED_RET i2c_write_blocking(i2c_default, I2C_PCA9536_ADDR,
			&tx[0], 1, false);
	}

	/* Read time from RTC. */
	{
		uint8_t tx = RTC_SEC;
		uint8_t rx[RTC_YEAR + 1];
		uint_fast16_t days;

		/* Select second register. */
		ret = i2c_write_blocking(i2c_default, I2C_DS3231M_ADDR, &tx, 1, false);
		if(ret == PICO_ERROR_GENERIC)
			goto out;

		/* Read time values. */
		ret = i2c_read_blocking(i2c_default, I2C_DS3231M_ADDR, rx, sizeof(rx), false);
		if(ret == PICO_ERROR_GENERIC)
			goto out;


		days = get_day_num(bcd_to_int(rx[RTC_YEAR]),
			bcd_to_int(rx[RTC_MONTH]), bcd_to_int(rx[RTC_DAY]));

		rtc.rtc_bits.sec = bcd_to_int(rx[RTC_SEC]);
		rtc.rtc_bits.min = bcd_to_int(rx[RTC_MIN]);
		rtc.rtc_bits.hour = bcd_to_int(rx[RTC_HOUR]);
		rtc.rtc_bits.yday = days & 0xFF;
		if(days > 0xFF)
			rtc.rtc_bits.high |= 0x01;
	}

	/* Read save data from FRAM. */
	{
		uint8_t tx[2];

		/* Set read address to 0x0000. */
		tx[0] = 0x00;
		tx[1] = 0x00;
		ret = i2c_write_blocking(i2c_default, I2C_MB85RC256V_ADDR, tx,
			sizeof(tx), true);

		if(ret == PICO_ERROR_GENERIC)
			goto out;

		UNUSED_RET i2c_read_blocking(i2c_default,
			I2C_MB85RC256V_ADDR, ram, sizeof(i2c_ram)-2, false);
	}

	/* TODO: RTC is ignored here. */

out:
	return ret;
}
#endif

int init_peripherals(void)
{
	/* Initialise RTC. */

	/* Initialise FRAM and obtain previous save data. */

	/* Initialise GB Bus PIO state machines. */
	gb_bus_program_init(pio0, PIO_SM_A15, PIO_SM_NCS, PIO_SM_DO);
	/* Enable state machines. */
	pio_sm_set_enabled(pio0, PIO_SM_A15, true);
	/* PIO_SM_NCS should be enabled when cart RAM access is expected. */
	pio_sm_set_enabled(pio0, PIO_SM_NCS, false);
	pio_sm_set_enabled(pio0, PIO_SM_DO,  true);

	return 0;
}

_Noreturn void __not_in_flash_func(play_rom_only)(const uint8_t *rom)
{
	while(1)
	{
		/* Only read the address, which is stored in the most
		 * significant two bytes of the RX FIFO. */
		io_ro_16 *rx_sm_a15 = (io_ro_16 *) &pio0->rxf[PIO_SM_A15];
		io_wo_8 *tx_sm_do = (io_wo_8 *) &pio0->txf[PIO_SM_DO];
		uint16_t address;

		/* Wait until we receive a new address. */
		while(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_A15));

		/* Only reads are expected in a non-banked ROM. */
		address = *rx_sm_a15;
		*tx_sm_do = rom[address];
	}
}

_Noreturn void __not_in_flash_func(play_mbc1_rom)(
		const uint8_t *const rom, uint8_t *const ram,
		uint16_t num_rom_banks_mask, uint8_t num_ram_banks)
{
	uint16_t selected_rom_bank = 1;
	uint8_t cart_ram_bank = 0;
	uint8_t enable_cart_ram = 0;
	/* Cartridge ROM/RAM mode select. */
	uint8_t cart_mode_select = 0;

	while(1)
	{
		/* Only read the address, which is stored in the most
		 * significant two bytes of the RX FIFO. */
		io_ro_32 *rx_sm_a15 = &pio0->rxf[PIO_SM_A15];
		io_ro_32 *rx_sm_ncs = &pio0->rxf[PIO_SM_NCS];
		io_wo_8 *tx_sm_do = (io_wo_8 *) &pio0->txf[PIO_SM_DO];
		union gb_bus_rx rx;
		uint16_t address;
		uint8_t data;

		while(1)
		{
			if(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_A15) == false)
			{
				rx.raw = *rx_sm_a15;
				break;
			}
			else if(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_NCS) == false)
			{
				rx.raw = *rx_sm_ncs;

				/* Catch invalid addresses here. */
				if(rx.address < 0xA000 || rx.address > 0xBFFF)
					continue;

				if(rx.is_write)
					__atomic_store_n(&ram_write, 1, __ATOMIC_SEQ_CST);

				break;
			}
		}

		address = rx.address;

		if(UNLIKELY(rx.is_write))
		{
			/* If we need to write data to ROM, then we obtain the
			 * data byte from the third byte of the RX FIFO. */
			data = rx.data;

			switch(address >> 12)
			{
			case 0x0:
			case 0x1:
				if(num_ram_banks)
					enable_cart_ram = ((data & 0x0F) == 0x0A);

				break;

			case 0x2:
			case 0x3:
				selected_rom_bank = (data & 0x1F) | (selected_rom_bank & 0x60);

				if((selected_rom_bank & 0x1F) == 0x00)
					selected_rom_bank++;

				selected_rom_bank = selected_rom_bank & num_rom_banks_mask;
				break;

			case 0x4:
			case 0x5:
				cart_ram_bank = (data & 3);
				selected_rom_bank = ((data & 3) << 5) | (selected_rom_bank & 0x1F);
				selected_rom_bank = selected_rom_bank & num_rom_banks_mask;
				break;

			case 0x6:
			case 0x7:
				cart_mode_select = (data & 1);
				break;

			case 0xA:
			case 0xB:
				if(num_ram_banks && enable_cart_ram)
				{
					if(cart_mode_select &&
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
			data = *((uint8_t *)XIP_SRAM_BASE + address);
			break;

		case 0x4:
		case 0x5:
		case 0x6:
		case 0x7:
			if(UNLIKELY(cart_mode_select))
				data = rom[address + ((selected_rom_bank & 0x1F) - 1) * ROM_BANK_SIZE];
			else
				data = rom[address + (selected_rom_bank - 1) * ROM_BANK_SIZE];

			break;

		case 0xA:
		case 0xB:
			if(LIKELY(num_ram_banks && enable_cart_ram))
			{
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

#if 1
_Noreturn void __not_in_flash_func(play_mbc3_rom)(
	const uint8_t *const rom, uint8_t *const ram,
	uint16_t num_rom_banks_mask, uint8_t num_ram_banks)
{
	uint16_t selected_rom_bank = 1;
	uint8_t cart_ram_bank = 0;
	uint8_t enable_cart_ram = 0;
	/* Cartridge ROM/RAM mode select. */
	uint8_t cart_mode_select = 0;

	/* MBC3 allows for the use of external RAM. */
	//pio_sm_set_enabled(pio0, PIO_SM_NCS, true);

	while(1)
	{
		/* Only read the address, which is stored in the most
		 * significant two bytes of the RX FIFO. */
		io_ro_32 *rx_sm_a15 = &pio0->rxf[PIO_SM_A15];
		io_ro_32 *rx_sm_ncs = &pio0->rxf[PIO_SM_NCS];
		io_wo_8 *tx_sm_do = (io_wo_8 *) &pio0->txf[PIO_SM_DO];
		union gb_bus_rx rx;
		uint16_t address;
		uint8_t data;

		while(1)
		{
			if(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_A15) == false)
			{
				rx.raw = *rx_sm_a15;
				break;
			}
			else if(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_NCS) == false)
			{
				rx.raw = *rx_sm_ncs;

				/* Catch invalid addresses here. */
				if(rx.address < 0xA000 || rx.address > 0xBFFF)
					continue;

				if(rx.is_write)
					__atomic_store_n(&ram_write, 1, __ATOMIC_SEQ_CST);

				break;
			}
		}

		address = rx.address;

		if(UNLIKELY(rx.is_write))
		{
			/* If we need to write data to ROM, then we obtain the
			 * data byte from the third byte of the RX FIFO. */
			data = rx.data;

			switch(address >> 12)
			{
			case 0x0:
			case 0x1:
				if(num_ram_banks)
					enable_cart_ram = ((data & 0x0F) == 0x0A);

				break;

			case 0x2:
			case 0x3:
				selected_rom_bank = data;

				/* Selecting ROM Bank 0 actually selects Bank
				 * 1. */
				if(UNLIKELY(selected_rom_bank == 0))
				{
					/* Masking ROM banks isn't required
					 * here because we know ROM bank 1 is
					 * always available. */
					selected_rom_bank++;
				}
				else
				{
					/* Mask ROM banks. */
					selected_rom_bank = selected_rom_bank
						& num_rom_banks_mask;
				}
				break;

			case 0x4:
			case 0x5:
				cart_ram_bank = data;
				break;

			case 0x6:
			case 0x7:
				cart_mode_select = (data & 1);
				break;

			case 0xA:
			case 0xB:
				if(num_ram_banks && enable_cart_ram)
				{
					if(cart_ram_bank >= 0x08)
						rtc.bytes[cart_ram_bank -
							0x08] = data;
					else if(cart_mode_select &&
						cart_ram_bank < num_ram_banks)
					{
						ram[address - CART_RAM_ADDR + (cart_ram_bank * CRAM_BANK_SIZE)] = data;
					}
					else
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
			//data = rom[address];
			data = *((uint8_t *)XIP_SRAM_BASE + address);
			break;

		case 0x4:
		case 0x5:
		case 0x6:
		case 0x7:
			data = rom[address + (selected_rom_bank - 1) * ROM_BANK_SIZE];
			break;

		case 0xA:
		case 0xB:
			if(LIKELY(num_ram_banks && enable_cart_ram))
			{
				if(cart_ram_bank >= 0x08)
				{
					data = rtc.bytes[cart_ram_bank - 0x08];
				}
				else if(cart_mode_select &&
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
#endif

_Noreturn
void __no_inline_not_in_flash_func(check_and_play_rom)(const uint8_t *rom)
{
	const uint16_t mbc_location = 0x0147;
	const uint16_t bank_count_location = 0x0148;
	const uint16_t ram_size_location = 0x0149;
	const uint8_t cart_mbc_lut[0xFF] = {
		/* ROM Only */
		[0x00] = 0,

		/* MBC1 */
		[0x01] = 1, 1, 1,
		[0x04] = -1,

		/* MBC2 */
		[0x05] = -1, -1,//2, 2, TODO: Unsupported
		[0x07] = -1,

		/* ROM with RAM (Unsupported) */
		[0x08] = -1, -1,
		[0x0A] = -1,

		/* MMM01 (Unsupported) */
		[0x0B] = -1, -1, -1,
		[0x0E] = -1,

		/* MBC3 */
		//[0x0F] = -1,-1,-1,-1,-1,
		[0x0F] = 3, 3, 3, 3, 3,

		/* Everything else is unsupported for now. */
		[0x14] = -1
	};
	const uint16_t num_rom_banks_lut[] = {
		2, 4, 8, 16, 32, 64, 128, 256, 512
	};
	const uint8_t num_ram_banks_lut[] = { 0, 0, 1, 4, 16, 8 };
	/* Cartridge information:
	 * Memory Bank Controller (MBC) type. */
	uint8_t mbc;
	/* Number of ROM banks in cartridge. */
	uint16_t num_rom_banks_mask;
	/* Number of RAM banks in cartridge. */
	uint8_t num_ram_banks;

	/* The Game Boy is held in reset again because the selected game may
	 * change the mode that the Game Boy has to boot in. */
	//gb_power(GB_POWER_OFF);

	/* Initialise ROM data. */
	/* Check if cartridge type is supported, and set MBC type. */
	{
		const uint8_t mbc_value = rom[mbc_location];
		const uint8_t ram_sz_value = rom[ram_size_location];

		if(mbc_value > sizeof(cart_mbc_lut) - 1 ||
			(mbc = cart_mbc_lut[mbc_value]) == 255u)
			goto err;

		num_ram_banks = num_ram_banks_lut[ram_sz_value];
		/* Limit RAM size to 32KiB. */
		if(num_ram_banks > 4)
			goto err;

		/* If cart RAM is required, enable SM_NCS state machine. */
		if(num_ram_banks != 0)
			pio_sm_set_enabled(pio0, PIO_SM_NCS, true);

		num_rom_banks_mask = num_rom_banks_lut[rom[bank_count_location]] - 1;
	}

	/* Force the ROM to not use the XIP cache. */
	rom += (XIP_NOCACHE_NOALLOC_BASE - XIP_BASE);

	/* Copy Bank0 to XIP Cache-as-SRAM. */
	memcpy((uint32_t *)XIP_SRAM_BASE, (uint32_t *)rom, ROM_BANK_SIZE);

	multicore_fifo_push_blocking(num_ram_banks);

	/* Force the ROM to not use the XIP cache. */
	rom += (XIP_NOCACHE_NOALLOC_BASE - XIP_BASE);

	switch(mbc)
	{
	case 0:
		play_rom_only(rom);
		break;

	case 1:
		play_mbc1_rom(rom, ram, num_rom_banks_mask, num_ram_banks);
		break;

#if 1
	case 3:
		play_mbc3_rom(rom, ram, num_rom_banks_mask, num_ram_banks);
		break;
#endif

	default:
		goto err;
	}

err:
	reset_usb_boot(0, 0);
}

#if USE_MGMT_ROM
/**
 * The management ROM has no banking functionality, but performs special
 * functions instead.
 */
_Noreturn void __no_inline_not_in_flash_func(play_mgmt_rom)(void)
{
	const uint8_t *rom_flash = gb_manager_gb + (XIP_NOCACHE_NOALLOC_BASE - XIP_BASE);
	uint8_t *rom = (uint8_t *)XIP_SRAM_BASE;

	memset(rom, 0xFF, ROM_BANK_SIZE);
	memcpy(rom, rom_flash, gb_manager_gb_len);

	rom[ADDR_NUMBER_OF_GAMES] = ARRAYSIZE(roms);
	for(unsigned i = 0; i < ARRAYSIZE(roms); i++)
	{
		uint8_t *targ_addr = (uint8_t *)(ADDR_GET_GAME_NAME);
		uint8_t name_len = 14;
		const uint8_t *game = roms[i];

		/* Shift to correct game name offset. */
		targ_addr += (i << 4);
		if(game[ROM_OLD_LICENSE_LOC] == 0x33)
			name_len = 10;

		memcpy(targ_addr, &game[ROM_TITLE_LOC], name_len);
		targ_addr[++name_len] = '\0';
	}

	multicore_fifo_push_blocking(0);

	while(1)
	{
		/* Only read the address, which is stored in the most
		 * significant two bytes of the RX FIFO. */
		io_ro_32 *rx_sm_a15 = &pio0->rxf[PIO_SM_A15];
		io_wo_8 *tx_sm_do = (io_wo_8 *) &pio0->txf[PIO_SM_DO];
		union gb_bus_rx rx;
		uint16_t address;
		uint8_t data;

		/* Wait until we receive a new address. */
		while(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_A15));

		/* Only reads are expected in a non-banked ROM. */
		rx.raw = *rx_sm_a15;
		address = rx.address;

#if 0
		if(UNLIKELY(rx.is_write))
		{
			data = rx.data;
			switch(address)
			{
			/* Play game. */
			case 0x2001:
				/* This shouldn't happen. */
				if(data > ARRAYSIZE(roms))
					continue;

				check_and_play_rom(roms[data]);
				UNREACHABLE();

			/* Firmware upgrade. */
			case 0x2002:
				reset_usb_boot(0, 0);
				UNREACHABLE();

			default:
				continue;
			}
		}
#endif
		*tx_sm_do = rom[address];
	}
}
#endif

void core1_main(void)
{
	/* Disable XIP Cache.
	 * This is done after using I2C, as that is read from flash by the
	 * pico-sdk. */
	xip_ctrl_hw->ctrl &= ~XIP_CTRL_EN_BITS;

#if USE_MGMT_ROM
	play_mgmt_rom();
#else
	/* Set the ROM you want to play here. */
	check_and_play_rom(libbet_gb);
	//check_and_play_rom(gb_manager_gb);
#endif
}

#if 0
void i2cDMA(i2c_inst_t *i2c, unsigned count, uint8_t *buf)
{
	const int dma_write = 1;
	dma_channel_config c_write;

	// configure write DMA
	c_write = dma_channel_get_default_config(dma_write);
	channel_config_set_transfer_data_size(&c_write, DMA_SIZE_8);
	channel_config_set_read_increment(&c_write, false);
	channel_config_set_write_increment(&c_write, false);
	channel_config_set_dreq(&c_write, DREQ_I2C0_TX);

	dma_channel_configure(dma_write, &c_write,
		&i2c->hw->data_cmd,	/* Destination pointer */
		buf,			/* Source pointer */
		count,			/* Number of transfers */
		true);

	//dma_channel_start(dma_write);
}
#endif

static ALWAYS_INLINE void busy_wait_us_31(uint32_t delay_us)
{
	// we only allow 31 bits
	uint32_t start = timer_hw->timerawl;
	while (timer_hw->timerawl - start < delay_us)
		tight_loop_contents();
}

_Noreturn void __no_inline_not_in_flash_func(loop_forever)(uint32_t ram_sz)
{
	/* If this game has no save data (does not use cart RAM) then sleep this
	 * core forever to save power. */

	if(ram_sz == 0)
	{
		/* Sleep forever. */
		__asm volatile ("cpsid i");
		while(1)
			__wfi();

		UNREACHABLE();
	}

	while(1)
	{
#if 0
		uint8_t conf = IO_EXP_INPUT_PORT;
		uint8_t rx;

		//sleep_ms(16);
		i2c_write_blocking(i2c_default, I2C_PCA9536_ADDR, &conf,
			sizeof(conf), false);
		i2c_read_blocking(i2c_default, I2C_PCA9536_ADDR, &rx,
			sizeof(rx), false);

		/* If button is pressed, save and power off. */
		if((rx & 0b0010))
			continue;

		i2c_write_blocking(i2c_default, I2C_MB85RC256V_ADDR, i2c_ram,
			ram_sz + 2, false);

		/* The Game Boy powers off to signify that saving was
		 * successful.
		 * TODO: This should be changed to an LED blinking. */
		gb_power(GB_POWER_OFF);
		while(i2c_default->hw->txflr != 0);
		while(1)
			__wfi();
		//sleep_ms(512);
		//gb_power(GB_POWER_ON);
#elif 1

		/* Save to FRAM every 1ms.
		 * Where each byte has an endurance of 10^12 writes, the FRAM
		 * is expected to last for at least 1902 years. */
		//busy_wait_us_31(1*1024);

		sleep_us(2 * 1024);

		if(__atomic_load_n(&ram_write, __ATOMIC_SEQ_CST) == 0)
			continue;

		__atomic_store_n(&ram_write, 0, __ATOMIC_SEQ_CST);
		//i2c_write_blocking(i2c_default, I2C_MB85RC256V_ADDR, i2c_ram,
		//	ram_sz + 2, false);
#endif
	}
}

void begin_playing(void)
{
	/* Disable interrupts on this core.
	 * There should not be any interrupts running on this core anyway. */
	//__asm volatile ("cpsid i");

	/* Grant high bus priority to the second core. */
	bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;

	multicore_launch_core1(core1_main);
}

void
__attribute__((noreturn))
__printflike(1, 0)
dbgc_panic(__unused const char *fmt, ...)
{
	reset_usb_boot(0, 0);
}

bool __no_inline_not_in_flash_func(rtc_callback)(repeating_timer_t *rt)
{
	/* Skip if RTC is disabled. */
	if((rtc.rtc_bits.high & 0x40) != 0)
		goto out;

	rtc.rtc_bits.sec++;
	if(rtc.rtc_bits.sec != 60)
		goto out;

	/* Seconds overflowed.*/
	rtc.rtc_bits.sec = 0;
	rtc.rtc_bits.min++;
	if(rtc.rtc_bits.min != 60)
		goto out;

	/* Minutes overflowed. */
	rtc.rtc_bits.min = 0;
	rtc.rtc_bits.hour++;
	if(rtc.rtc_bits.hour != 24)
		goto out;

	/* Hours overflowed. */
	rtc.rtc_bits.hour = 0;
	rtc.rtc_bits.yday++;
	if(rtc.rtc_bits.yday != 0)
		goto out;

	/* Set 8th bit of day register if lower bits overflowed. */
	if(rtc.rtc_bits.high & 1)
	{
		/* If 8th bit already set, set the overflow bit. */
		rtc.rtc_bits.high |= 0x80;
	}
	rtc.rtc_bits.high ^= 1;

out:
	return true;
}

int main(void)
{
	/* Previously used 572000000 VCO, which sets system clock to 286MHz and
	 * flash to 143MHz.
	 * A VCO of at least 480000000 is required for single speed games to
	 * work. This sets system clock to 240MHz and flash to 120MHz. */
	{
		/* The value for VCO set here is meant for least power
		 * consumption. */
		const unsigned vco = 512000000; /* 256MHz/128MHz */
		const unsigned div1 = 2, div2 = 1;

		vreg_set_voltage(VREG_VOLTAGE_1_15);
		sleep_ms(4);
		set_sys_clock_pll(vco, div1, div2);
		sleep_ms(4);
	}

	init_gpio_pins();
	init_peripherals();

	/* After initialising the IO expander, ensure that the Game Boy is held
	 * in reset. */
	gb_power(GB_POWER_OFF);

#if 0
	{
		uint8_t conf = IO_EXP_INPUT_PORT;
		uint8_t rx;

		i2c_write_blocking(i2c_default, I2C_PCA9536_ADDR, &conf,
			sizeof(conf), false);
		i2c_read_blocking(i2c_default, I2C_PCA9536_ADDR, &rx,
			sizeof(rx), false);

		/* If button is pressed, go to programming mode. */
		if((rx & 0b0010) == 0)
			reset_usb_boot(0, 0);
	}
#endif

	begin_playing();

	{
		static repeating_timer_t rt;
		add_repeating_timer_us(-1000000, rtc_callback, NULL, &rt);
	}

	/* Wait until core1 is ready to play. */
	{
		uint32_t num_ram_banks;
		uint32_t ram_sz;

		/* Wait for Core1 to initialise. */
		num_ram_banks = multicore_fifo_pop_blocking();
		ram_sz = num_ram_banks * CRAM_BANK_SIZE;
		gb_power(GB_POWER_ON);
		loop_forever(ram_sz);
	}

	UNREACHABLE();
}
