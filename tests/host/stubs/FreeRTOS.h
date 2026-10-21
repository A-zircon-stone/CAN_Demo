#pragma once
#include <stdint.h>
using TickType_t = uint32_t;
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define pdMS_TO_TICKS(ms) (ms)
