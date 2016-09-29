#pragma once

#include <time.h>
#include <stdint.h>

#define _countof(arr) sizeof(arr) / sizeof(arr[0])
#define INT_TO_FLOAT(i) 1.0f * ((float) (i >> 4) + ((i & 15) / 16.0f))

uint64_t gettime_us();

uint32_t gettime_ms();