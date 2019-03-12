#pragma once

#include <stdbool.h>

void tlv493_init();

bool tlv493_read(int *x, int *y, int *z);
