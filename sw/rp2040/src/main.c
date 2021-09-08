#include <pico/stdlib.h>
#include <hardware/pio.h>

#include "comms.pio.h"

#define PIN_D0	0
#define PIN_A15	23
#define PIN_DATADIR 25
#define PIN_WR	27
#define PIN_CS	29

int main(void)
{
	PIO pio = pio0;
	uint sm_a15, sm_cs, sm_wr, sm_do;

	gb_bus_program_init(pio, &sm_a15, &sm_cs, &sm_wr, &sm_do,
			PIN_D0, PIN_A15, PIN_CS, PIN_WR, PIN_DATADIR);
	sleep_ms(500);

	return 0;
}
