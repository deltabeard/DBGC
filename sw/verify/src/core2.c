#include <core2.h>
#include <hardware/pio.h>
#include <hardware/irq.h>
#include "comms.pio.h"

void pio0_irq0(void)
{
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
		pio_sm_put(pio0, PIO_SM_DO, 0b01010101);
	}
	else if(pio_sm_is_rx_fifo_empty(pio0, PIO_SM_CS) == false)
	{
		uint32_t val = pio_sm_get(pio0, PIO_SM_CS);
		bool rd;

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

		/* TODO: Implement banking. */
		/* TODO: Implement RAM writing. */
	}

	return;
}


void init_gb_comms_pins(void)
{
	gb_bus_program_init(pio0, PIO_SM_A15, PIO_SM_CS, PIO_SM_DI, PIO_SM_DO);
}

_Noreturn void core1_entry(void)
{
	init_gb_comms_pins();

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

	while(1);
}
