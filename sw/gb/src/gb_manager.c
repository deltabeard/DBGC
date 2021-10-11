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

#define DBGC_API_VER		0
#define object_distance(a, b)	((void *)&(b) - (void *)&(a))
#define arraysize(array)	(sizeof(array)/sizeof(array[0]))

uint8_t enable_dbgc_api(void)
{
	const char enable_str[] = {'D', 'B', 'G', 'C', DBGC_API_VER};
	uint8_t ver;

	for(unsigned i = 0; i < sizeof(enable_str); i++)
		ver = *(uint8_t*)enable_str[i];

	return ver;
}

_Noreturn void loop_forever(void)
{
	while(1)
		__asm__("halt");
}
void loop_forever_end() {}
unsigned char __at _loop_forever_hiram hiram_buffer[];
extern void loop_forever_hiram(void);


#define MENU_SELECT_ITEM(menu, sel)				\
	do{							\
		if((sel - 1) < menu->items_nmemb)		\
			menu->item_selected = sel;		\
	}while(0)

struct menu_ctx_s
{
	struct menu_ctx_s *parent;
	const char *title;
	const char *help;
	unsigned long item_selected;
	unsigned long items_nmemb;
	const struct menu_item_s *items;
};
typedef struct menu_ctx_s menu_ctx;

typedef enum {
	/* Go back to the previous item.
	 * Could be used when user presses UP. */
	MENU_INSTR_PREV_ITEM,

	/* Go to next item in menu.
	 * Could be used when user presses DOWN. */
	MENU_INSTR_NEXT_ITEM,

	/* Go to parent menu if one exists.
	 * Could be used when user presses B. */
	MENU_INSTR_PARENT_MENU,

	/* Execute item operation.
	 * Could be used when user presses A. */
	MENU_INSTR_EXEC_ITEM
} menu_instruction_e;

typedef enum
{
	/* Opens a sub menu. */
	MENU_SUB_MENU,

	/* Executes a function. */
	MENU_EXEC_FUNC
} menu_op_e;

struct menu_item_s
{
	const char *name;
	const char *help;
	menu_op_e op;

	union param_u
	{
		/* Pointer to sub menu. */
		menu_ctx *sub_menu;

		/* Pointer to function to execute if item selected. */
		void (*func)(void);
	} param;
};
typedef struct menu_item_s menu_item;

inline void menu_init(menu_ctx *menu, menu_ctx *parent, const char *title,
	const char *help, uint8_t items_nmemb,
	struct menu_item_s *items)
{
	menu->parent = parent;
	menu->title = title;
	menu->help = help;
	menu->item_selected = 0;
	menu->items_nmemb = items_nmemb;
	menu->items = items;
}

inline void menu_set_items(menu_ctx *menu, uint8_t nmemb,
	struct menu_item_s *items)
{
	menu->items_nmemb = nmemb;
	menu->items = items;
}

inline menu_ctx *menu_instruct(menu_ctx *ctx, menu_instruction_e instr)
{
	menu_ctx *ret = ctx;
	switch(instr)
	{
	case MENU_INSTR_PREV_ITEM:
		if(ctx->item_selected > 0)
			ctx->item_selected--;

		break;

	case MENU_INSTR_NEXT_ITEM:
		if(ctx->item_selected < (ctx->items_nmemb - 1))
			ctx->item_selected++;

		break;

	case MENU_INSTR_PARENT_MENU:
		if(ctx->parent != NULL)
			ret = ctx->parent;

		break;

	case MENU_INSTR_EXEC_ITEM:
	{
		const struct menu_item_s *item =
			ctx->items + ctx->item_selected;

		switch(item->op)
		{
		case MENU_SUB_MENU:
			ret = item->param.sub_menu;
			break;

		case MENU_EXEC_FUNC:
			item->param.func();
			break;
		}
		break;
	}
	}

	return ret;
}

_Noreturn void firmware_upgrade_dbgc(void)
{
	loop_forever_hiram();
}

void do_nothing(void)
{
	return;
}

_Noreturn void menu(void)
{
	menu_ctx root, play;
	menu_ctx *current = &root;
	uint8_t key_prev = 0;

	menu_item root_items[] = {
		{
			"Play",
			"Select and play a game.",
			MENU_SUB_MENU,
			.param.sub_menu = &play
		},
		{
			"Update",
			"Reboot DBGC to firmware update mode.",
			MENU_EXEC_FUNC,
			.param.func = &firmware_upgrade_dbgc
		}
	};
	menu_item play_items[] = {
		{
			"Not implemented",
			NULL,
			MENU_EXEC_FUNC,
			.param.func = &do_nothing
		}
	};

	menu_init(&root, NULL, "Main Menu", NULL, arraysize(root_items),
		root_items);
	menu_init(&play, &root, "Select a game", NULL, arraysize(play_items),
		play_items);

	/* Print some text! */
	while(1)
	{
		uint8_t key;
		uint8_t key_changes;

		wait_vbl_done();
		key = joypad();

		/* Only change cursor on key down. */
		key_changes = (key ^ key_prev) & key;

		if(key_changes & J_UP)
			current = menu_instruct(current, MENU_INSTR_PREV_ITEM);
		else if(key_changes & J_DOWN)
			current = menu_instruct(current, MENU_INSTR_NEXT_ITEM);
		else if(key_changes & J_A)
			current = menu_instruct(current, MENU_INSTR_EXEC_ITEM);
		else if(key_changes & J_B)
			current = menu_instruct(current, MENU_INSTR_PARENT_MENU);
		else
			continue;

		key_prev = key;

		gotoxy(0, 0);
		for(uint8_t i = 0; i < current->items_nmemb; i++)
		{
			if(i == current->item_selected)
				putchar('>');
			else
				putchar(' ');

			puts(current->items[i].name);
		}
	}
}
void menu_end() {}
unsigned char __at _menu_wram wram_buffer[];
extern void menu_wram(void);

void main(void)
{
	font_t font;
	uint8_t selection = 0;
	uint8_t selection_prev = 1;
	uint8_t key_prev = 0;

	/* First, init the font system */
	font_init();
	font = font_load(font_ibm);

	hiramcpy((uint8_t)&hiram_buffer, (void *)&loop_forever,
		(uint8_t)object_distance(loop_forever, loop_forever_end));

	/* Attempt to enable DBGC API. */
	if(enable_dbgc_api() != DBGC_API_VER)
	{
		/* If DBGC API could not be enabled, print an error and halt. */
		puts("FATAL: DBGC API could not be enabled.");
		loop_forever_hiram();
	}

	memcpy(&wram_buffer, (void *)&menu,
		(uint16_t)object_distance(menu, menu_end));
	menu_wram();
}
