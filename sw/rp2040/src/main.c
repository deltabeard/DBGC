#include <sys/cdefs.h>
#include <hardware/pio.h>
#include <hardware/i2c.h>
#include <hardware/rtc.h>
#include <hardware/sync.h>
#include <hardware/irq.h>
#include <pico/binary_info.h>
#include <pico/bootrom.h>
#include <pico/multicore.h>
#include <pico/unique_id.h>
#include <usb.h>
#include <tusb.h>
#include <bsp/board.h>

#include <main.h>
#include <peripherals/io_expander.h>
#include <peripherals/rtc.h>

#include "comms.pio.h"

static inline void multicore_fifo_push_inline(uint32_t data)
{
	if(multicore_fifo_wready() == false)
		return;

	sio_hw->fifo_wr = data;
	__sev();
}

_Noreturn void core1_entry(void)
{
#if 1
	tusb_init();

	{
		uint8_t buffer[5+PICO_UNIQUE_BOARD_ID_SIZE_BYTES];
		buffer[0] = 'R';
		buffer[1] = rp2040_rom_version();
		buffer[2] = 'C';
		buffer[3] = rp2040_chip_version();
		buffer[4] = 'U';
		pico_get_unique_board_id((pico_unique_board_id_t *) &buffer[5]);
		tud_hid_report(0, buffer, sizeof(buffer));
		tud_task();
	}

	while (1)
	{
		uint32_t buffer[8];
		unsigned len = 0;

		buffer[len++] = multicore_fifo_pop_blocking();
		while(multicore_fifo_rvalid())
		{
			buffer[len++] = sio_hw->fifo_rd;
			if(len == 8)
				break;
		}

		if(tud_hid_ready())
		{
			uint8_t sz = len * sizeof(uint32_t);
			tud_hid_report(0, buffer, sz);
		}

		tud_task();
	}

#else
	while(1)
		__wfi();
#endif
}

void pio0_irq0(void)
{
#include "gb_manager.gb.h"
	static uint_fast16_t address = 0;

	irq_clear(PIO0_IRQ_0);

	if(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_A15) == false)
	{
		uint32_t val = pio_sm_get(pio0, PIO_SM_A15);
		/**
		 * PIO_SM_A15 state machine RX value:
		 * | 8-bit Data | 16-bit Address | CS | RD | PHI | 00000 |
		 * | Unused     | Readable       | U  | R  | U   | UUUUU |
		 * Legend: U-Unused, R-Readable
		 */
		bool rd;

		multicore_fifo_push_inline(val);
		address = (val & 0x00FFFF00) >> 8;
		rd = (val & 0b01000000) != 0;

		/* If RD is HIGH, then the Game Boy wants to write data to the
		 * given address, however, the data is not ready when this read
		 * takes place. The address is saved in the static variable to
		 * allow the PIO_SM_DI to write to that address. */
		if(rd == true)
			return;

		/* TODO: CS and PHI are not useful. */

		/* Check that address is within range. */
		/* TODO: Maybe change this to assert? */
		if(address >= gb_manager_rom_len)
			address = 0;

		pio_sm_put(pio0, PIO_SM_DO, gb_manager_rom[address]);
	}
	else if(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_CS) == false)
	{
		uint32_t val = pio_sm_get(pio0, PIO_SM_CS);
		bool rd;

		multicore_fifo_push_inline(val);
		address = (val & 0x00FFFF00) >> 8;
		rd = (val & 0b01000000) != 0;
		if(rd == true)
			return;

		/* Do not perform any action on internal addresses. */
		if(address >= 0xC000)
			return;

		/* TODO: Implement RAM read. */
		pio_sm_put(pio0, PIO_SM_DO, 0x00);
	}
	else if(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_DI) == false)
	{
		uint32_t val = pio_sm_get(pio0, PIO_SM_DI);
		//uint8_t data = val >> 24;

		/* This state machine requires the previously acquired address
		 * value. */

		multicore_fifo_push_inline(val);
		/* TODO: Implement banking. */
		/* TODO: Implement RAM writing. */
	}

	return;
}

void start_io_expander(void)
{
	/* Check that the IO expander is connected. */
	if(i2c_io_exp_init(i2c0) != PICO_OK)
	{
		/* Connection to the IO expander is mandatory. If that fails,
		 * then reboot to USB for reprogramming, debugging, etc. */
		reset_usb_boot(0, 0);
		__builtin_unreachable();
	}

	/* Set pin directions on the IO expander. */
	i2c_io_exp_set_reg(i2c0, IO_EXP_DIRECTION);
	i2c_io_exp_write_reg(i2c0, (IO_EXP_DIR_UNUSED << IO_EXP_UNUSED_PIN) |
		(IO_EXP_DIR_OUTPUT << IO_EXP_STATLED_PIN) |
		(IO_EXP_DIR_INPUT << IO_EXP_SWITCH_PIN) |
		(IO_EXP_DIR_OUTPUT << IO_EXP_GB_RESET_PIN));

	/* Hold Game Boy in reset before checking for button press.
	 * Initialise LED to be off. */
	i2c_io_exp_set_reg(i2c0, IO_EXP_OUTPUT_PORT);
	i2c_io_exp_write_reg(i2c0,
		(0 << IO_EXP_STATLED_PIN) | (1 << IO_EXP_GB_RESET_PIN));

	{
		uint8_t in;

		/* Check to see if button is pressed. */
		i2c_io_exp_set_reg(i2c0, IO_EXP_INPUT_PORT);
		i2c_io_exp_read_reg(i2c0, &in);
		/* If button isn't being pressed down, do nothing. */
		if(in & (1 << IO_EXP_SWITCH_PIN))
			return;

		/* If button is being held down, reboot to USB Boot mode. */
		/* Turn on the LED to show that we're in USB Boot mode due to
		 * user pushing button. */
		i2c_io_exp_set_reg(i2c0, IO_EXP_OUTPUT_PORT);
		i2c_io_exp_write_reg(i2c0, (1 << IO_EXP_STATLED_PIN));

		reset_usb_boot(0, 0);
		__builtin_unreachable();
	}
}

void start_rtc(void)
{
	datetime_t dt;

	/* Initilise internal RTC. */
	rtc_init();

	/* Initiliase external I2C RTC. */
	if(i2c_rtc_init(i2c0) != PICO_OK)
		return;

	/* Read date and time from external RTC. */
	if(i2c_rtc_read_time(i2c0, &dt) != PICO_OK)
		return;

	/* Set date and time to internal RTC. */
	if(rtc_set_datetime(&dt) != PICO_OK)
		return;
}

int main(void)
{
	/* On init, the RP2040 is running at 125 MHz. */

	/* Initialise and set I2C baudrate to 400kHz. */
	i2c_init(i2c0, 400 * 1000);
	/* Declare I2C pins for picoboot. */
	bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));
	/* Initiliase GPIO pins for I2C0. */
	gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
	/* Disable weak internal pull-ups on I2C pins; there are external
	 * pull-ups connected to these pins for I2C operation. */
	gpio_disable_pulls(I2C_SDA_PIN);
	gpio_disable_pulls(I2C_SCL_PIN);
	/* If the user has held down the button on startup, reset
	 * to USB Boot mode. Otherwise, turn off LED and hold Game Boy in
	 * reset. */
	start_io_expander();

	/* Obtain time and date from RTC IC. */
	start_rtc();

	/* TODO: Initilise FRAM for RAM reading and writing. */

	multicore_launch_core1(core1_entry);
#if 0
	/* Declare PIO pins for picoboot. */
	bi_decl(bi_pin_range_with_func(PIO_PHI_3V3, PIO_PHI_DIR, GPIO_FUNC_PIO0));

	PIO pio = pio0;
	uint sm_a15, sm_cs, sm_wr, sm_do;

	gb_bus_program_init(pio, &sm_a15, &sm_cs, &sm_wr, &sm_do,
			PIN_D0, PIN_A15, PIN_CS, PIN_WR, PIN_DATADIR);
	sleep_ms(500);
#endif

	/* Enable PIO interrupt. */
#if 0
	pio_set_irq0_source_mask_enabled(pio0,
		pis_sm0_rx_fifo_not_empty | pis_sm1_rx_fifo_not_empty |
		pis_sm2_rx_fifo_not_empty, true);
	irq_set_exclusive_handler(PIO0_IRQ_0, pio0_irq);
	irq_set_enabled(PIO0_IRQ_0, true);
#endif
	{
		gb_bus_program_init(pio0, PIO_SM_A15, PIO_SM_CS, PIO_SM_DI, PIO_SM_DO);

		/* Enable IRQ0 to trigger when data in RX FIFO. */
		pio_set_irq0_source_mask_enabled(pio0,
			PIO_INTR_SM0_RXNEMPTY_LSB |
			PIO_INTR_SM1_RXNEMPTY_LSB |
			PIO_INTR_SM2_RXNEMPTY_LSB,
			true);
		irq_set_exclusive_handler(PIO0_IRQ_0, pio0_irq0);
		irq_set_enabled(PIO0_IRQ_0, true);

		/* Enable state machines. */
		pio_sm_set_enabled(pio0, PIO_SM_A15, true);
		pio_sm_set_enabled(pio0, PIO_SM_CS,  true);
		pio_sm_set_enabled(pio0, PIO_SM_DI,  true);
		pio_sm_set_enabled(pio0, PIO_SM_DO,  true);
	}

	/* Switch on Game Boy. */
	i2c_io_exp_set_reg(i2c0, IO_EXP_OUTPUT_PORT);
	i2c_io_exp_write_reg(i2c0,
		(0 << IO_EXP_STATLED_PIN) | (0 << IO_EXP_GB_RESET_PIN));

	/* Sleep until we receive read or write instructions from the Game Boy.
	 */
	while(1)
	{
		__wfi();
		tight_loop_contents();
	}

	__builtin_unreachable();
	return 0;
}
