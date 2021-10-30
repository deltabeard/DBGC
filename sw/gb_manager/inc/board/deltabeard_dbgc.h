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

// --- I2C ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 0
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 1
#endif

#define I2C_PCA9536_ADDR	0b01000001
#define I2C_DS3231M_ADDR	0b01101000
#define I2C_MB85RC256V_ADDR	0b01010000

// --- PIO ---
#define PIO_PHI			2
#define PIO_NRD			3
#define PIO_NCS			4
#define PIO_A0			5
#define PIO_A15			20
#define PIO_D0			21
#define PIO_D7			28
#define PIO_DIR			29
#define NUM_ADDRESS_PINS	16
#define NUM_DATA_PINS		8
#define NUM_MISC_PINS		3
#define NUM_TOTAL_PINS (NUM_ADDRESS_PINS + NUM_DATA_PINS + NUM_MISC_PINS)

// PIO: Static state machine configuration.
#define PIO_SM_A15	0
#define PIO_SM_NCS	1
#define PIO_SM_DO	2


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
