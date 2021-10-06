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

// --- FLASH ---
//#define PICO_BOOT_STAGE2_CHOOSE_GENERIC_03H 1

// Voltage		Max Freq (MHz)		CLKDIV
// VREG_VOLTAGE_1_25	416			4
// VREG_VOLTAGE_1_20	381			4
// VREG_VOLTAGE_1_05	273			4
// VREG_VOLTAGE_1_05	136			2
// VREG_VOLTAGE_1_00	131			2
// VREG_VOLTAGE_0_95	125			2
// VREG_VOLTAGE_0_90	118			2
// VREG_VOLTAGE_0_85	FAIL

// Flash is limited to ~70MHz. At the default frequency of 125MHz, CLKDIV of 2
// will produce a clock of 62.5MHz.
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
