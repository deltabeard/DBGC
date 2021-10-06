#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/bootrom.h>
#include <hardware/vreg.h>

int main(void)
{
	int c;
	bool clock_success;

	/* Reduce power consumption to stop IO Expander Power-On Reset Errata. */
	sleep_ms(10);

	//set_sys_clock_48mhz();

	stdio_usb_init();

	while((c = getchar_timeout_us(0)) == PICO_ERROR_TIMEOUT)
	{
		printf("Press a key to start\n");
		sleep_ms(500);
	}

	if(c == 'r')
		goto out;

	set_sys_clock_khz(20000, false);
	vreg_set_voltage(VREG_VOLTAGE_1_20);
	sleep_ms(100);

	for(unsigned i = 20000; i < 420000; i += 1000)
	{
		clock_success = set_sys_clock_khz(i, false);
		if(clock_success == false)
			continue;

		sleep_ms(100);
		printf("Clock %u, \tFlash: %u\n",
			i, i/PICO_FLASH_SPI_CLKDIV);
		sleep_ms(100);
	}


	sleep_ms(1000);

out:
	reset_usb_boot(0, 0);

	return 0;
}
