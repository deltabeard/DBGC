/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------

#ifndef _BOARDS_DELTABEARD_DBGC_H
#define _BOARDS_DELTABEARD_DBGC_H

// For board detection
#define DELTABEARD_DBGC

// On some samples, the xosc can take longer to stabilize than is usual
#ifndef PICO_XOSC_STARTUP_DELAY_MULTIPLIER
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64
#endif

// --- GPIO ---
#define GPIO_LED_GREEN		1
#define GPIO_SWITCH		2
#define GPIO_MOTOR		12
#define GPIO_GB_RESET		29

// --- SPI0 ---
/* The READ command is not compatible with 33MHz operation. Use FSTRD
 * instead. */
#define MB85RS256B_BAUDRATE	(33 * 1000 * 1000)
#define SPI_MOSI		3
#define SPI_MISO		4
#define SPI_CSn			5
#define SPI_SCK			6

// --- PIO0 ---
#define PIO_PHI			13
#define PIO_NWR			14
#define PIO_NRD			15
#define PIO_NCS			16
#define PIO_ADDR1_OE		17
#define PIO_ADDR2_OE		18
#define PIO_DATA_OE		19
#define PIO_DATA_DIR		20
#define PIO_M0			21
#define PIO_M7			28
#define NUM_MULTIPLEX_PINS	(PIO_M7 - PIO_M0) + 1
// PIO0: Static state machine configuration.
#define PIO_SM_A15		0
#define PIO_SM_NCS		1
#define PIO_SM_DO		2

// --- PIO1 ---
#define PIO_RTC_SCLK		9
#define PIO_RTC_IO		10
#define PIO_RTC_CE		11
// PIO1: Static state machine configuration
#define PIO1_SM_RTC_WR		0
#define PIO1_SM_RTC_RD		1

// --- FLASH ---
#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif

// All boards have B1 RP2040
#ifndef PICO_FLOAT_SUPPORT_ROM_V1
#define PICO_FLOAT_SUPPORT_ROM_V1 0
#endif

#ifndef PICO_DOUBLE_SUPPORT_ROM_V1
#define PICO_DOUBLE_SUPPORT_ROM_V1 0
#endif

#endif
