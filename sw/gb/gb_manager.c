/**
 * GB 2040 CART MANAGER
 */

#include <stdio.h>
#include <gb/gb.h>
#include <gb/font.h>
#include <gb/console.h>
#include <gb/drawing.h>

void main(void) {
    font_t font;
    const char *games_str[] = {
            "2040-gb", "Snake", "uCity", "Galaxia", "Libbet and the Ma-",
            "Carazu"
    };
    const uint8_t games_n = sizeof(games_str) / sizeof(*games_str);
    uint8_t selection = 0;
    uint8_t selection_prev = 1;

    /* First, init the font system */
    font_init();
    font = font_load(font_ibm);

    /* Turn scrolling off (why not?) */
    mode(get_mode() | M_NO_SCROLL);
    /* Print game list. */
    for (uint8_t i = 0; i < games_n; i++) {
        putchar(' ');
        puts(games_str[i]);
    }

    /* Print some text! */
    while (1) {
        uint8_t key;

        wait_vbl_done();
        key = joypad();
        if (key & J_UP && selection != 0)
            selection--;
        else if (key & J_DOWN && selection < games_n - 1)
            selection++;
        else if (key & J_LEFT)
            selection = 0;
        else if (key & J_RIGHT)
            selection = games_n - 1;

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
}
