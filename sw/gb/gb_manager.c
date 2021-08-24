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
    uint8_t key_prev = 0;

    /* First, init the font system */
    font_init();
    font = font_load(font_ibm);

    /* Print game list. */
    for (uint8_t i = 0; i < games_n; i++) {
        putchar(' ');
        puts(games_str[i]);
    }

    /* Print some text! */
    while (1) {
        uint8_t key;
        uint8_t key_changes;

        wait_vbl_done();
        key = joypad();

        /* Only change cursor on key down. */
        key_changes = (key ^ key_prev) & key;

        if (key_changes & J_UP && selection != 0)
            selection--;
        else if (key_changes & J_DOWN && selection < games_n - 1)
            selection++;
        else if (key_changes & J_LEFT)
            selection = 0;
        else if (key_changes & J_RIGHT)
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
}
