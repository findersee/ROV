#ifndef _DSHOT_H_
#define _DSHOT_H_

#include "pico.h"
#include "hardware/i2c.h"
#include "commands.h"

uint32_t dshot_parse_throttle(uint16_t *value,bool telemetry);

uint32_t dshot_parse_cmd(uint8_t cmd,bool telemetry);

#endif