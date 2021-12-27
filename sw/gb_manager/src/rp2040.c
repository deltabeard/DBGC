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

#include <hardware/vreg.h>
#include <pico/time.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>
#include <hardware/spi.h>
#include <pico/multicore.h>
#include <hardware/structs/bus_ctrl.h>
#include <hardware/structs/xip_ctrl.h>
#include <hardware/pio.h>
#include <hardware/irq.h>
#include <string.h>
#include <stdio.h>

#include "comms_basic.pio.h"
#include "generic.h"
#include "cart.h"

__attribute__((section ("gb_rom_section")))
//#include "libbet.gb.h"
#include "bluestar.gbc.h"
//#include "rom_512kb.gb.h"
//#include "ram_64kb.gb.h"

static uint8_t ram[32768];
/* Number of writes to cartridge RAM (battery backed RAM) since last sync
 * with FRAM. This should reduce FRAM writes and battery power a bit. */
static int ram_write = 0;

typedef enum {
	GB_POWER_ON = 0,
	GB_POWER_OFF = 1
} gb_power_e;

_Noreturn static void play_rom_only(const uint8_t *rom)
{
	while(1)
	{
		io_wo_8 *data_tx = (io_wo_8 *) &GB_BUS_PIO->txf[PIO_SM_DO] + 3;
		io_ro_16 *addr_a15 = (io_ro_16 *)
			&GB_BUS_PIO->rxf[PIO_SM_A15] + 1;
		uint16_t address;
		uint8_t data;

		while(pio_sm_is_rx_fifo_empty(GB_BUS_PIO, PIO_SM_A15))
			tight_loop_contents();

		address = *addr_a15;
		address = __builtin_bswap16(address);
		data = rom[address];
		*data_tx = data;
	}
}


_Noreturn static void play_mbc1_rom(const uint8_t *const rom,
	uint8_t *const ram, uint16_t num_rom_banks_mask, uint8_t num_ram_banks)
{
	uint16_t selected_rom_bank = 1;
	uint8_t cart_ram_bank = 0;
	uint8_t enable_cart_ram = 0;
	/* Cartridge ROM/RAM mode select. */
	uint8_t cart_mode_select = 0;

	io_wo_8  *tx_sm_do  = (io_wo_8 *)&GB_BUS_PIO->txf[PIO_SM_DO] + 3;
	io_wo_8  *req_cart_wr = (io_wo_8 *)&GB_BUS_PIO->txf[PIO_SM_DI] + 3;
	io_ro_8  *tx_sm_di  = (io_ro_8 *)&GB_BUS_PIO->rxf[PIO_SM_DI] + 3;
	io_ro_16 *rx_sm_a15 = (io_ro_16 *)&GB_BUS_PIO->rxf[PIO_SM_A15] + 1;
	io_ro_16 *rx_sm_ncs = (io_ro_16 *)&GB_BUS_PIO->rxf[PIO_SM_NCS] + 1;

	while(1)
	{
		uint16_t address;
		uint8_t data;

		while(1)
		{
			if(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_A15) == false)
			{
				address = *rx_sm_a15;
				address = __builtin_bswap16(address);
				break;
			}
			else if(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_NCS) == false)
			{
				address = *rx_sm_ncs;
				address = __builtin_bswap16(address);

				/* Catch invalid addresses here. */
				if(address < 0xA000 || address > 0xBFFF)
					continue;

				break;
			}
		}

#if 1
		if(UNLIKELY(gpio_get(PIO_NRD)))
		{
			*req_cart_wr = 0xFF;

			/* TODO: Could use IRQ to handle writes to cart. */
			while(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_DI))
				tight_loop_contents();

			data = *tx_sm_di;

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
				if(!(num_ram_banks && enable_cart_ram))
					break;

				if(cart_mode_select &&
					cart_ram_bank < num_ram_banks)
				{
					ram[address - CART_RAM_ADDR + (cart_ram_bank * CRAM_BANK_SIZE)] = data;
				}
				else if(num_ram_banks)
				{
					ram[address - CART_RAM_ADDR] = data;
				}

				__atomic_store_n(&ram_write, 1, __ATOMIC_SEQ_CST);

				break;

			default:
				/* This is an invalid address. */
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
			//data = *((uint8_t *)XIP_SRAM_BASE + address);
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


_Noreturn static void play_mbc3_rom(const uint8_t *const rom,
	uint8_t *const ram, uint16_t num_rom_banks_mask, uint8_t num_ram_banks)
{
	uint16_t selected_rom_bank = 1;
	uint8_t cart_ram_bank = 0;
	uint8_t enable_cart_ram = 0;
	/* Cartridge ROM/RAM mode select. */
	uint8_t cart_mode_select = 0;

	io_wo_8  *tx_sm_do  = (io_wo_8 *)&GB_BUS_PIO->txf[PIO_SM_DO] + 3;
	io_wo_8  *req_cart_wr = (io_wo_8 *)&GB_BUS_PIO->txf[PIO_SM_DI] + 3;
	io_ro_8  *tx_sm_di  = (io_ro_8 *)&GB_BUS_PIO->rxf[PIO_SM_DI] + 3;
	io_ro_16 *rx_sm_a15 = (io_ro_16 *)&GB_BUS_PIO->rxf[PIO_SM_A15] + 1;
	io_ro_16 *rx_sm_ncs = (io_ro_16 *)&GB_BUS_PIO->rxf[PIO_SM_NCS] + 1;

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

	while(1)
	{
		uint16_t address;
		uint8_t data;

		while(1)
		{
			if(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_A15) == false)
			{
				address = *rx_sm_a15;
				address = __builtin_bswap16(address);
				break;
			}
			else if(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_NCS) == false)
			{
				address = *rx_sm_ncs;
				address = __builtin_bswap16(address);

				/* Catch invalid addresses here. */
				if(address < 0xA000 || address > 0xBFFF)
					continue;

				break;
			}
		}

#if 1
		if(UNLIKELY(gpio_get(PIO_NRD)))
		{
			*req_cart_wr = 0xFF;
			/* TODO: Could use IRQ to handle writes to cart. */
			while(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_DI))
				tight_loop_contents();

			data = *tx_sm_di;

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
				if(!(num_ram_banks && enable_cart_ram))
					break;

				if(cart_ram_bank >= 0x08)
				{
					rtc.bytes[cart_ram_bank - 0x08] = data;
					break;
				}

				if(cart_mode_select &&
					cart_ram_bank < num_ram_banks)
				{
					ram[address - CART_RAM_ADDR + (cart_ram_bank * CRAM_BANK_SIZE)] = data;
				}
				else
				{
					ram[address - CART_RAM_ADDR] = data;
				}

				__atomic_store_n(&ram_write, 1, __ATOMIC_SEQ_CST);
				break;

			default:
				/* This is an invalid address. */
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
			//data = *((uint8_t *)XIP_SRAM_BASE + address);
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

static uint8_t get_ram_banks(const uint8_t *rom)
{
	const uint16_t ram_size_location = 0x0149;
	const uint8_t num_ram_banks_lut[] = { 0, 0, 1, 4, 16, 8 };
	uint8_t ram_sz_value;
	uint8_t num_ram_banks;

	ram_sz_value = rom[ram_size_location];
	num_ram_banks = num_ram_banks_lut[ram_sz_value];
	return num_ram_banks;
}

_Noreturn static void check_and_play_rom(const uint8_t *rom)
{
	const uint16_t mbc_location = 0x0147;
	const uint16_t bank_count_location = 0x0148;
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
	/* Cartridge information:
	 * Memory Bank Controller (MBC) type. */
	uint8_t mbc;
	/* Number of ROM banks in cartridge. */
	uint16_t num_rom_banks_mask;
	/* Number of RAM banks in cartridge. */
	uint8_t num_ram_banks;

	/* Initialise ROM data. */
	/* Check if cartridge type is supported, and set MBC type. */
	{
		const uint8_t mbc_value = rom[mbc_location];

		if(mbc_value > sizeof(cart_mbc_lut) - 1 ||
				(mbc = cart_mbc_lut[mbc_value]) == 255u)
			goto err;

		num_ram_banks = get_ram_banks(rom);
		/* Limit RAM size to 32KiB. */
		if(num_ram_banks > 4)
			goto err;

		num_rom_banks_mask = num_rom_banks_lut[rom[bank_count_location]] - 1;
	}

	/* Force the ROM to not use the XIP cache. */
	//rom += (XIP_NOCACHE_NOALLOC_BASE - XIP_BASE);

	/* Copy Bank0 to XIP Cache-as-SRAM. */
	//memcpy((uint32_t *)XIP_SRAM_BASE, (uint32_t *)rom, ROM_BANK_SIZE);
	gpio_put(GPIO_GB_RESET, GB_POWER_ON);

	switch(mbc)
	{
	case 0:
		pio_set_sm_mask_enabled(GB_BUS_PIO,
			1 << PIO_SM_A15 | 0 << PIO_SM_NCS | 1 << PIO_SM_DO |
				0 << PIO_SM_DI, true);
		play_rom_only(rom);
		break;

	case 1:
		pio_set_sm_mask_enabled(GB_BUS_PIO,
			1 << PIO_SM_A15 |
			(num_ram_banks != 0) << PIO_SM_NCS |
			1 << PIO_SM_DO |
			1 << PIO_SM_DI, true);
		play_mbc1_rom(rom, ram, num_rom_banks_mask, num_ram_banks);
		break;

	case 3:
		pio_set_sm_mask_enabled(GB_BUS_PIO,
			1 << PIO_SM_A15 |
			(num_ram_banks != 0) << PIO_SM_NCS |
			1 << PIO_SM_DO |
			1 << PIO_SM_DI, true);
		play_mbc3_rom(rom, ram, num_rom_banks_mask, num_ram_banks);
		break;

	default:
		goto err;
	}

err:
	reset_usb_boot(0, 0);
}

_Noreturn void core1_play_rom(void)
{
	gb_bus_program_basic_init(GB_BUS_PIO, PIO_SM_A15, PIO_SM_NCS,
		PIO_SM_DO, PIO_SM_DI);
	check_and_play_rom(bluestar_gbc);
}

static void rst_callback(uint gpio, uint32_t events)
{
	(void) gpio; /* GPIO will always be GPIO_SWITCH. */
	(void) events;

	gpio_put(GPIO_GB_RESET, GB_POWER_OFF);
	reset_usb_boot(0, 0);
}

static inline void begin_playing(void)
{
	/* Disable XIP Cache. */
	xip_ctrl_hw->ctrl &= ~XIP_CTRL_EN_BITS;

	/* Grant high bus priority to the second core. */
	bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;
	multicore_launch_core1(core1_play_rom);
}

static bool sync_fram(repeating_timer_t *rt)
{
	const unsigned *ram_sz = rt->user_data;


	/* If RAM size is 0, then do not write any save data. */
	if(*ram_sz == 0)
		return false;

	/* Do not sync FRAM if no writes were made to the cart RAM. */
	if(ram_write == 0)
		return true;

	/* Reset RAM write indicator. */
	__atomic_store_n(&ram_write, 0, __ATOMIC_SEQ_CST);

	/* Indicate that a save is taking place. */
	gpio_put(GPIO_LED_GREEN, true);

	{
		const uint8_t send[] = {
			0b00000110 /* Write enable */
		};
		gpio_put(SPI_CSn, 0);
		busy_wait_us(1);
		spi_write_blocking(spi0, send, sizeof(send));
		busy_wait_us(1);
		gpio_put(SPI_CSn, 1);
	}

	/* Read save data. */
	{
		const uint8_t cmd[] = {
			0b00000010, /* Write */
			0x00, 0x00, /* Address */
		};
		gpio_put(SPI_CSn, 0);
		busy_wait_us(1);
		spi_write_blocking(spi0, cmd, sizeof(cmd));
		spi_write_blocking(spi0, ram, *ram_sz);
		busy_wait_us(1);
		gpio_put(SPI_CSn, 1);
	}

	gpio_put(GPIO_LED_GREEN, false);

	return true;
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

	/* Handle reset switch button press. */
	gpio_set_irq_enabled_with_callback(GPIO_SWITCH, 0b0100, true,
		rst_callback);

	/** SPI **/
	/* Default settings of spi_init are correct for the MB85RS256B. */
	spi_init(spi0, MB85RS256B_BAUDRATE);
	gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
	gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
	gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
}

int main(void)
{
	{
		/* The value for VCO set here is meant for least power
		 * consumption. */
		const unsigned vco = 532000000; /* 266MHz/133MHz */
		const unsigned div1 = 2, div2 = 1;

		vreg_set_voltage(VREG_VOLTAGE_1_15);
		sleep_ms(2);
		set_sys_clock_pll(vco, div1, div2);
		sleep_ms(2);
	}

	init_peripherals();

	do {
		uint8_t num_ram_banks;
		static unsigned ram_sz;
		static repeating_timer_t fram_timer;

		num_ram_banks = get_ram_banks(bluestar_gbc);
		ram_sz = (unsigned)num_ram_banks * CRAM_BANK_SIZE;

		if(ram_sz == 0)
			break;

		/* Read save data. */
		{
			const uint8_t cmd[4] = {
				0b00001011, /* FSTRD */
				0x00, 0x00, /* Address */
				0x00 /* Dummy */
			};
			gpio_put(SPI_CSn, 0);
			busy_wait_us(1);
			spi_write_blocking(spi0, cmd, sizeof(cmd));
			spi_read_blocking(spi0, 0x00, ram, ram_sz);
			busy_wait_us(1);
			gpio_put(SPI_CSn, 1);
		}

		add_repeating_timer_us(256 * 1024, sync_fram,
			&ram_sz, &fram_timer);
	} while(0);

	begin_playing();

	/* Sleep forever. */
	while(1)
	{
		__wfi();
	}

	UNREACHABLE();
}

void
__attribute__((noreturn))
__printflike(1, 0)
dbgc_panic(__unused const char *fmt, ...)
{
	(void) fmt;
	reset_usb_boot(0, 0);
}
