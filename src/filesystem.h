#pragma once

#include <ch.h>
#include <hal.h>

int filesystem_write(uint32_t sector, const uint8_t *data, uint32_t bytes);
int filesystem_read(uint32_t sector, uint8_t *data, uint32_t bytes);

void filesystem_init();

