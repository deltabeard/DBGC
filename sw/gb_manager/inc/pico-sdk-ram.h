#pragma once

#include "hardware/i2c.h"
#include "hardware/resets.h"
#include "pico/timeout_helper.h"

/* Pico-sdk functions that must be executed from RAM. */
int __not_in_flash_func(i2c_write_blocking_ram)(i2c_inst_t *i2c, uint8_t addr,
	const uint8_t *src, size_t len, bool nostop);
