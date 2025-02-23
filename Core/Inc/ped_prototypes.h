#ifndef PED_PROTOTYPES_H
#define PED_PROTOTYPES_H

#include <stdint.h>

#define to_mV(raw_value) (float)raw_value * (3.3f / 4095.0f)

#define PRINTLN(b, s)      \
    tud_cdc_write_str(b);  \
    tud_cdc_write_flush(); \
    b[0] = '\r';           \
    b[1] = '\n';           \
    b[2] = 0;              \
    tud_cdc_write_str(b);  \
    tud_cdc_write_flush(); \
    memset(b, 0, s);

uint32_t get_current_time_ms();

#endif  // PED_PROTOTYPES_H
