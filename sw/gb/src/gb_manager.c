/**
 * GB 2040 CART MANAGER
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <gb/gb.h>
#include <gb/font.h>
#include <gb/console.h>
#include <gb/drawing.h>

#define DBGC_API_VER                0
#define THROWAWAY_READ(addr)

#define object_distance(a, b) ((void *)&(b) - (void *)&(a))

uint8_t enable_dbgc_api(void)
{
	const char enable_str[] = {'D', 'B', 'G', 'C', DBGC_API_VER};
	uint8_t ver;

	for(unsigned i = 0; i < sizeof(enable_str); i++)
		memcpy(&ver, (const void *)enable_str[i], 1);

	return ver;
}

_Noreturn void loop_forever(void)
{
	while(1)
		__asm__("halt");
}
void loop_forever_end() {}

unsigned char __at _loop_forever_hiram hiram_buffer[];
extern void loop_forever_hiram();

void main(void)
{
	font_t font;
	uint8_t selection = 0;
	uint8_t selection_prev = 1;
	uint8_t key_prev = 0;

	/* First, init the font system */
	font_init();
	font = font_load(font_ibm);

	/* Attempt to enable DBGC API. */
	if(enable_dbgc_api() != DBGC_API_VER)
	{
		hiramcpy((uint8_t)&hiram_buffer, (void *)&loop_forever,
			(uint8_t)object_distance(loop_forever, loop_forever_end));

		/* If DBGC API could not be enabled, print an error and halt. */
		puts("FATAL: DBGC API could not be enabled.");
		loop_forever_hiram();
	}

#if 0
	/* Print main menu. */
	for(uint8_t i = 0; i < games_n; i++)
	{
		putchar(' ');
		puts(games_str[i]);
	}

	/* Print some text! */
	while(1)
	{
		uint8_t key;
		uint8_t key_changes;

		wait_vbl_done();
		key = joypad();

		/* Only change cursor on key down. */
		key_changes = (key ^ key_prev) & key;

		if(key_changes & J_UP && selection != 0)
			selection--;
		else if(key_changes & J_DOWN && selection < games_n - 1)
			selection++;
		else if(key_changes & J_LEFT)
			selection = 0;
		else if(key_changes & J_RIGHT)
			selection = games_n - 1;

		key_prev = key;

		if(selection_prev == selection)
			continue;

		/* Clear previous cursor. */
		gotoxy(0, selection_prev);
		setchar(' ');
		/* Show cursor at new selection. */
		gotoxy(0, selection);
		setchar('>');

		selection_prev = selection;
	}
#endif
}
