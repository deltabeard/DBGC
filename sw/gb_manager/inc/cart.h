#pragma once


/* Cart API. */
/* Writing to this address executes a new command.
 * Reading from this address gives the currently executing command, or 0x00
 * for no command taking place. */
#define ADDR_NEW_CMD		0x0000
/* Cart commands that can be written to ADDR_NEW_CMD. */
typedef enum {
	/**
	 * No operation. Does nothing.
	 * void nop(void)
	 */
	CART_CMD_NOP = 0,

	/**
	 * Get the number of games available to be played.
	 * uint8_t get_num_games(void)
	 */
	CART_CMD_GET_NUMBER_OF_GAMES = 1,

	/**
	 * Get the name of a game when given a game number.
	 * const char *get_game_name(uint8_t game_number)
	 * The returned string is null terminated and the length of the string
	 * is limited to 32 characters (including the null character).
	 */
	CART_CMD_GET_GAME_NAME = 2,

	/**
	 * Set a game to play, and reset the game boy. This exists the cart API.
	 * void play_game(uint8_t game_number)
	 */
	CART_CMD_PLAY_GAME = 3,

	/**
	 * Reboot the DBGC cart into firmware upgrade mode.
	 */
	CART_CMD_UPGRADE = 4
} cart_cmd_e;

/* Writing to this address sets the parameter for the *next* command.
 * Reading from this address is undefined. */
#define ADDR_CMD_PARAM		0x0001

/* Writing to this address has no effect.
 * Reading from this address gives the return value of the command.
 * ADDR_CMD_STATUS will reset to CART_STATUS_READY once all the return values
 * from the command are read from this address. */
#define ADDR_CMD_RET		0x0002

#define ROM_TITLE_LOC		0x0134
#define ROM_TITLE_END_LOC	0x0143
#define ROM_TITLE_NEW_END_LOC	0x013E
#define ROM_OLD_LICENSE_LOC	0x014B

#define ROM_BANK_SIZE   0x4000
#define CRAM_BANK_SIZE  0x2000
#define CART_RAM_ADDR   0xA000
