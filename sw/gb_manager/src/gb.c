/**
 * DBGC Cart Manager for the Game Boy.
 * Copyright (c) 2021 Mahyar Koshkouei
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the above copyright notice and
 * this permission notice appear in all copies.
 * THIS SOFTWARE IS PROVIDED 'AS-IS', WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
 * IN NO EVENT WILL THE AUTHORS BE HELD LIABLE FOR ANY DAMAGES ARISING FROM
 * THE USE OF THIS SOFTWARE.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <gb/gb.h>
#include <gb/font.h>
#include <gb/console.h>
#include <gb/drawing.h>

#include <cart.h>

/* Macros. */
#define object_distance(a, b)	((void *)&(b) - (void *)&(a))
#define arraysize(array)	(sizeof(array)/sizeof(array[0]))

#define MENU_SELECT_ITEM(menu, sel)				\
	do{							\
		if((sel - 1) < menu->items_nmemb)		\
			menu->item_selected = sel;		\
	}while(0)

struct menu_ctx
{
	struct menu_ctx *parent;
	const char *title;
	uint8_t items_nmemb;
	const struct menu_item *items;
};

typedef enum
{
	/* Opens a sub menu. */
	MENU_SUB_MENU,

	/* Executes a function. */
	MENU_EXEC_FUNC
} menu_op_e;

struct menu_item
{
	char name[32];
	menu_op_e op;

	union param_u
	{
		/* Pointer to sub menu. */
		struct menu_ctx *sub_menu;

		/* Pointer to function to execute if item selected. */
		struct {
			uint8_t func_param;
			void (*func)(uint8_t param);
		};
	} param;
};

#if 0
inline void menu_init(struct menu_ctx *menu, struct menu_ctx *parent,
	const char *title, uint8_t items_nmemb, struct menu_item *items)
{
	menu->parent = parent;
	menu->title = title;
	menu->items_nmemb = items_nmemb;
	menu->items = items;
}

inline void menu_set_items(struct menu_ctx *menu, uint8_t nmemb,
	struct menu_item *items)
{
	menu->items_nmemb = nmemb;
	menu->items = items;
}

void play_game(uint8_t param)
{
	disable_interrupts();

	*(uint8_t *)ADDR_PLAY_GAME = param;
	loop_forever_ram();
}

void do_nothing(uint8_t param)
{
	(void) param;
	return;
}

_Noreturn void reboot_to_usbboot(uint8_t param)
{
	(void) param;

	cls();
	gotoxy(0, 0);
	puts("USB Firmware Upgrade");

	gotoxy(0, 2);
	puts("Game Boy is halted.");
	puts("Do not remove the\n"
	     "USB cable during\n"
	     "upgrade.");
	wait_vbl_done();
	disable_interrupts();
	
	*(uint8_t *)ADDR_UPGRADE = 0;
	loop_forever_ram();
}

void draw_menu(struct menu_ctx *current, uint8_t item_selected)
{
	cls();
	gotoxy(0, 0);
	for(uint8_t i = 0; i < current->items_nmemb; i++)
	{
		if(i == item_selected)
			putchar('>');
		else
			putchar(' ');

		puts(current->items[i].name);
	}
}

uint8_t fill_play_items(struct menu_item *play_items, uint8_t max_items)
{
	uint8_t number_of_games;

	number_of_games = *(uint8_t *)(ADDR_NUMBER_OF_GAMES);

	if(number_of_games > max_items)
		number_of_games = max_items;

	for(uint8_t i = 0; i < number_of_games; i++)
	{
		char *name = &play_items[i].name[0];
		char *name_src;
		uint16_t game_n = i;
		play_items[i].op = MENU_EXEC_FUNC;
		play_items[i].param.func_param = i;
		play_items[i].param.func = &play_game;

		name_src = (char *)(ADDR_GET_GAME_NAME + (game_n << 4));

		while(1)
		{
			char c = *name_src;
			*name = *name_src;
			name++;
			name_src++;

			if(c == '\0')
				break;
		};
	}

	return number_of_games;
}

_Noreturn void menu(void)
{
	struct menu_ctx play, menu;
	struct menu_ctx *current = &menu;
	struct menu_item play_items[32];
	struct menu_item root_items[] = {
		{
			.name = "Play",
			.op = MENU_SUB_MENU,
			.param.sub_menu = &play
		},
		{
			.name = "Reboot to USBBOOT",
			.op = MENU_EXEC_FUNC,
			.param.func_param = 0,
			.param.func = &reboot_to_usbboot
		}
	};
	uint8_t key_prev = 0;
	uint8_t item_selected = 0;

	{
		uint8_t play_items_n;
		play_items_n = fill_play_items(play_items,
			arraysize(play_items));
		menu_init(&menu, NULL, NULL, arraysize(root_items), root_items);
		menu_init(&play, &menu, NULL, play_items_n, play_items);
	}

	draw_menu(current, item_selected);

	while(1)
	{
		uint8_t key;
		uint8_t key_changes;

		wait_vbl_done();
		key = joypad();

		/* Only change cursor on key down. */
		key_changes = (key ^ key_prev) & key;

		if(key_changes & J_UP)
		{
			if(item_selected > 0)
				item_selected--;
			else
				goto next;
		}
		else if(key_changes & J_DOWN)
		{
			if(item_selected < (current->items_nmemb - 1))
				item_selected++;
			else
				goto next;
		}
		else if(key_changes & J_A)
		{
			const struct menu_item *item =
				current->items + item_selected;

			if(item->op == MENU_SUB_MENU)
			{
				current = item->param.sub_menu;
				item_selected = 0;
				draw_menu(current, item_selected);
			}
			else if(item->op == MENU_EXEC_FUNC)
			{
				item->param.func(item->param.func_param);
			}
		}
		else if(key_changes & J_B)
		{
			if(current->parent != NULL)
			{
				current = current->parent;
				item_selected = 0;
				draw_menu(current, item_selected);
			}
			else
				goto next;
		}
		else
			goto next;

		for(uint8_t i = 0; i < current->items_nmemb; i++)
		{
			gotoxy(0, i);
			if(i == item_selected)
				setchar('>');
			else
				setchar(' ');
		}

next:
		key_prev = key;
	}
}
#endif

void main(void)
{
	font_t font;

	/* First, init the font system */
	font_init();
	font = font_load(font_ibm);

#if 0
	{
		uint8_t number_of_games = *(uint8_t * )(ADDR_NUMBER_OF_GAMES);

		for(uint8_t i = 0; i < number_of_games; i++)
		{
			const char *name;
			uint16_t game_n = i;
			name = (char *) (ADDR_GET_GAME_NAME + (game_n << 4));
			puts(name);
		}
	}
#endif

	puts("Test");

	while(1) wait_vbl_done();
	//loop_forever_ram();

	//menu();
}
