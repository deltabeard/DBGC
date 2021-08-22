/**
 * GB 2040 CART MANAGER
 */

#include <stdio.h>
#include <gb/font.h>
#include <gb/console.h>
#include <gb/drawing.h>

void main(void)
{
    font_t font;

    /* First, init the font system */
    font_init();
    font = font_load(font_spect);

    /* Turn scrolling off (why not?) */
    mode(get_mode() | M_NO_SCROLL);

    /* Print some text! */
    font_set(font);
    printf("Select Game:\n"
	">2040-gb\n"
	" Snake\n"
	" uCity\n"
	" Glaxia\n"
	" Libbet and the Mag-"
	" Carazu");
}

    
    
