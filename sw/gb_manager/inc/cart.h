#pragma once

#define ROM_NINTENDO_LOGO_LOC	0x0104
#define ROM_TITLE_LOC		0x0134
#define ROM_TITLE_END_LOC	0x0143
#define ROM_TITLE_NEW_END_LOC	0x013E
#define ROM_OLD_LICENSE_LOC	0x014B
#define ROM_CART_RAM_LOC	0xA000

#define ROM_BANK_SIZE		0x4000
#define CRAM_BANK_SIZE		0x2000
#define CART_RAM_ADDR		0xA000

/**
 * Number of games in program.
 * Unsigned 8-bit integer, Read Only.
 */
#define ADDR_NUMBER_OF_GAMES	0x2000
/**
 * Set to begin playing game.
 * Unsigned 8-bit integer, Write only.
 */
#define ADDR_PLAY_GAME		0x2001
/**
 * Write any value to reboot to upgrade mode (USBBOOT).
 * Unsigned 8-bit integer, Write only.
 */
#define ADDR_UPGRADE		0x2002
/**
 * Read name of game XXh from 0x3XX0. Zero indexed.
 * Null terminated string, Read Only.
 */
#define ADDR_GET_GAME_NAME	0x3000
#define ADDR_GET_GAME_NAME_0	0x3000
#define ADDR_GET_GAME_NAME_1	0x3010
#define ADDR_GET_GAME_NAME_2	0x3020
