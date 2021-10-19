#include <stdlib.h>
#include <hardware/i2c.h>
#include <hardware/pio.h>
#include <hardware/vreg.h>
#include <hardware/pll.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/bootrom.h>
#include <hardware/structs/bus_ctrl.h>
#include <hardware/structs/xip_ctrl.h>

#include "comms.pio.h"
#include <cart.h>
#include <generic.h>

/* ROMs */
#include <gb_manager.gb.h>
#include <libbet.gb.h>
#include <pokered.gbc.h>

typedef enum {
	IO_EXP_INPUT_PORT = 0,
	IO_EXP_OUTPUT_PORT,
	IO_EXP_INVERSION,
	IO_EXP_DIRECTION
} io_exp_reg_e;

typedef enum {
	GB_POWER_ON = 0,
	GB_POWER_OFF = 1
} gb_pwr_e;

void gb_power(gb_pwr_e pwr)
{
	uint8_t tx[2];
	tx[0] = IO_EXP_OUTPUT_PORT;
	tx[1] = 0b11111110 | pwr;
	i2c_write_blocking(i2c_default, I2C_PCA9536_ADDR, tx,
			   sizeof(tx), false);

	return;
}

void init_gpio_pins(void)
{
	/* I2C0 */
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	/* I2C pins have an external pull-up resistor connected. */
	gpio_disable_pulls(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_disable_pulls(PICO_DEFAULT_I2C_SCL_PIN);

	/* PIO0 */
	for(uint_fast8_t i = PIO_PHI; i <= PIO_A15; i++)
	{
		gpio_set_input_enabled(i, true);
		/* Disable schmitt triggers on GB Bus. The bus transceivers
		 * already have schmitt triggers. */
		gpio_set_input_hysteresis_enabled(i, false);
	}

	for(uint_fast8_t i = PIO_PHI; i <= PIO_DIR; i++)
	{
		/* Use fast slew rate for GB Bus. */
		gpio_set_slew_rate(i, GPIO_SLEW_RATE_FAST);
	}
}

int init_i2c_peripherals(void)
{
	int ret = 0;

	/* Initialise I2C. */
	i2c_init(i2c_default, 100 * 1000);

	/* Set external IO expander configuration. */
	{
		uint8_t tx[2];
		tx[0] = IO_EXP_DIRECTION;
		tx[1] = 0b11111110;
		ret = i2c_write_blocking(i2c_default, I2C_PCA9536_ADDR, tx,
			sizeof(tx), false);
	}

	/* TODO: RTC and FRAM are ignored here. */

	return ret;
}

union gb_bus_rx {
	struct {
		uint16_t address;
		uint8_t data;
		uint8_t is_write;
	};
	uint32_t raw;
};

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
};


const uint8_t *roms[] = {
	libbet_gb, pokered_gbc
};

_Noreturn void __no_inline_not_in_flash_func(play_mgmt_rom)(void);

_Noreturn void __no_inline_not_in_flash_func(play_rom_only)(const uint8_t *rom)
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

_Noreturn void __no_inline_not_in_flash_func(play_mbc1_rom)(
		const uint8_t *const rom, uint8_t *const ram,
		uint16_t num_rom_banks_mask, uint8_t num_ram_banks)
{
	uint16_t selected_rom_bank = 1;
	uint8_t cart_ram_bank = 0;
	uint8_t enable_cart_ram = 0;
	/* Cartridge ROM/RAM mode select. */
	uint8_t cart_mode_select = 0;

	/* MBC1 allows for the use of external RAM. */
	pio_sm_set_enabled(pio0, PIO_SM_NCS, true);

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
			data = rom[address];
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


_Noreturn void __no_inline_not_in_flash_func(play_mbc3_rom)(
	const uint8_t *const rom, uint8_t *const ram,
	uint16_t num_rom_banks_mask, uint8_t num_ram_banks)
{
	uint16_t selected_rom_bank = 1;
	uint8_t cart_ram_bank = 0;
	uint8_t enable_cart_ram = 0;
	/* Cartridge ROM/RAM mode select. */
	uint8_t cart_mode_select = 0;
	union cart_rtc rtc = { 0 };

	/* MBC3 allows for the use of external RAM. */
	pio_sm_set_enabled(pio0, PIO_SM_NCS, true);

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
				selected_rom_bank = data & 0x7F;

				if(!selected_rom_bank)
					selected_rom_bank++;

				selected_rom_bank = selected_rom_bank & num_rom_banks_mask;
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
			data = rom[address];
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

_Noreturn
void __no_inline_not_in_flash_func(check_and_play_rom)(const uint8_t *rom)
{
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
	const uint16_t num_rom_banks_lut[] =
		{
			2, 4, 8, 16, 32, 64, 128, 256, 512
		};
	const uint8_t num_ram_banks_lut[] = {0, 1, 1, 4, 16, 8};
	/* Cartridge information:
	 * Memory Bank Controller (MBC) type. */
	uint8_t mbc;
	/* Number of ROM banks in cartridge. */
	uint16_t num_rom_banks_mask;
	/* Number of RAM banks in cartridge. */
	uint8_t num_ram_banks;

	unsigned char *ram = NULL;
	unsigned ram_sz;


	gb_power(GB_POWER_OFF);

	/* Initialise ROM data. */
	/* Check if cartridge type is supported, and set MBC type. */
	{
		const uint8_t mbc_value = rom[mbc_location];
		const uint8_t ram_sz_value = rom[ram_size_location];

		if(mbc_value > sizeof(cart_mbc_lut) - 1 ||
			(mbc = cart_mbc_lut[mbc_value]) == 255u)
			goto err;

		num_ram_banks = num_ram_banks_lut[ram_sz_value];
		ram_sz = num_ram_banks * CRAM_BANK_SIZE;

		if(ram_sz != 0)
		{
			ram = calloc(ram_sz, sizeof(*ram));
			if(ram == NULL)
				goto err;
		}

		num_rom_banks_mask = num_rom_banks_lut[rom[bank_count_location]] - 1;
	}

	sleep_ms(8);
	gb_power(GB_POWER_ON);

	switch(mbc)
	{
	case 0:
		play_rom_only(rom);
		break;

	case 1:
		play_mbc1_rom(rom, ram, num_rom_banks_mask, num_ram_banks);
		break;

	case 3:
		play_mbc3_rom(rom, ram, num_rom_banks_mask, num_ram_banks);
		break;

	default:
		goto err;
	}

err:
	play_mgmt_rom();
}

/**
 * This function handles ROM-only games. These games have no banking
 * functionality.
 */
_Noreturn void __no_inline_not_in_flash_func(play_mgmt_rom)(void)
{
	union gb_bus_rx rx = { .raw = 0 };
	struct gb_mgmt_ctx mgmt = { 0 };
	const uint8_t *rom = gb_manager_rom;

	/* ROM only game does not use cart RAM. */
	pio_sm_set_enabled(pio0, PIO_SM_NCS, false);

	gb_power(GB_POWER_OFF);
	sleep_ms(8);
	gb_power(GB_POWER_ON);

	while(1)
	{
		/* Only read the address, which is stored in the most
		 * significant two bytes of the RX FIFO. */
		io_ro_32 *rx_sm_a15 = (io_ro_32 *) &pio0->rxf[PIO_SM_A15];
		io_wo_8 *tx_sm_do = (io_wo_8 *) &pio0->txf[PIO_SM_DO];
		uint16_t address;
		uint8_t data;

		/* Wait until we receive a new address. */
		while(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_A15));

#if 0
		{
			if(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_A15) == false)
			{

				break;
			}
			else if(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_NCS) ==
			false)
			{
				rx.raw = *rx_sm_ncs;

				/* Catch invalid addresses here. */
				if(rx.address < 0xA000 || rx.address > 0xBFFF)
					continue;

				break;
			}
		}
#endif

		/* Only reads are expected in a non-banked ROM. */
		rx.raw = *rx_sm_a15;
		address = rx.address;

		if(UNLIKELY(rx.is_write))
		{
			data = rx.data;
			switch(address)
			{
			case ADDR_NEW_CMD:
				mgmt.cmd = data;
				switch(mgmt.cmd)
				{
				case CART_CMD_NOP:
					mgmt.ret = 0;
					mgmt.param = 0;
					break;

				case CART_CMD_GET_NUMBER_OF_GAMES:
					mgmt.ret = ARRAYSIZE(roms);
					break;

				case CART_CMD_GET_GAME_NAME:
					mgmt.ctx.game_name.rom =
						roms[mgmt.param];
					/* Find length of ROM title. */
					if(mgmt.ctx.game_name.rom[ROM_OLD_LICENSE_LOC] == 0x33)
						mgmt.ctx.game_name
						.remaining_length = 11;
					else
						mgmt.ctx.game_name.remaining_length =
							15;

					mgmt.ctx.game_name.rom =
						&rom[ROM_TITLE_LOC];
					mgmt.ret = *(mgmt.ctx.game_name.rom++);
					break;

				case CART_CMD_PLAY_GAME:
					if(mgmt.param < ARRAYSIZE(roms))
					{
						check_and_play_rom(roms[mgmt.param]);
						UNREACHABLE();
					}
					break;

				case CART_CMD_UPGRADE:
					reset_usb_boot(0, 0);
					UNREACHABLE();
				}
				break;

			case ADDR_CMD_PARAM:
				mgmt.param = data;
				break;

			default:
			case ADDR_CMD_RET:
				/* Not a valid write. */
				break;
			}

			continue;
		}

		if(UNLIKELY(address == ADDR_NEW_CMD))
		{
			*tx_sm_do = mgmt.cmd;
		}
		else if(UNLIKELY(address == ADDR_CMD_RET))
		{
			*tx_sm_do = mgmt.ret;
			if(mgmt.cmd == CART_CMD_GET_GAME_NAME)
			{
				if(mgmt.ctx.game_name.remaining_length)
				{
					mgmt.ctx.game_name.remaining_length--;
					mgmt.ret = *(mgmt.ctx.game_name.rom++);
				}
				else
					mgmt.ret = '\0';
			}
		}
		else
		{
			*tx_sm_do = rom[address];
		}
	}
}

void core1_main(void)
{
	/* Disable XIP Cache. */
	xip_ctrl_hw->ctrl &= ~XIP_CTRL_EN_BITS;

	play_mgmt_rom();
}

void init_pio(void)
{
	/* Initilise PIO program. */
	gb_bus_program_init(pio0, PIO_SM_A15, PIO_SM_NCS, PIO_SM_DO);
	/* Enable state machines. */
	pio_sm_set_enabled(pio0, PIO_SM_A15, true);
	/* PIO_SM_NCS should be enabled when cart RAM access is expected. */
	pio_sm_set_enabled(pio0, PIO_SM_NCS, false);
	pio_sm_set_enabled(pio0, PIO_SM_DO,  true);

	return;
}

void __no_inline_not_in_flash_func(loop_forever)(void)
{
	/* Sleep forever. */
	while(1)
		__wfi();
}

void begin_playing(void)
{
	/* Disable interrupts on this core. */
	__asm volatile ("cpsid i");

	/* Grant high bus priority to the second core. */
	bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;

	multicore_launch_core1(core1_main);
}

bool is_gameboy_connected(void)
{
	bool is_connected = false;

	gb_power(GB_POWER_ON);

	/* Wait for Game Boy to start reading from ROM. */
	sleep_ms(64);

	/* If the Game Boy has attempted to read from ROM, then we know that
	 * it's connected. */
	is_connected = pio_sm_is_rx_fifo_full(pio0, PIO_SM_A15);
	gb_power(GB_POWER_OFF);

	/* Clear the A15 state machine RX fifo. the NCS state machine is
	 * expected to be empty, since the Game Boy bootrom only reads from
	 * Bank 0. */
	while(pio_sm_get_rx_fifo_level(pio0, PIO_SM_A15))
		UNUSED(pio_sm_get(pio0, PIO_SM_A15));

	return is_connected;
}

int main(void)
{
	/* Reduce power consumption to stop IO Expander Power-On Reset Errata. */
	sleep_ms(10);

	/* Set system clock to 286MHz and flash to 143MHz. */
	{
		/* The value for VCO set here is meant for least power
		 * consumption. */
		const unsigned vco = 572000000;
		const unsigned div1 = 2, div2 = 1;

		vreg_set_voltage(VREG_VOLTAGE_1_15);
		sleep_ms(4);
		set_sys_clock_pll(vco, div1, div2);
		sleep_ms(4);
	}

	init_gpio_pins();

	if(init_i2c_peripherals() < 0)
		goto err;

	gb_power(GB_POWER_OFF);
	init_pio();

	if(is_gameboy_connected() == false)
		goto err;

	begin_playing();
	loop_forever();

err:
	reset_usb_boot(0, 0);

	return 0;
}
