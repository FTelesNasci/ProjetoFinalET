#ifndef AHT10_RTOS_H
#define AHT10_RTOS_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

typedef struct {
    i2c_inst_t *i2c;
    uint8_t address;
    float temperature;
    float humidity;
} AHT10_t;

// API pública
bool AHT10_Init(AHT10_t *dev);
bool AHT10_Read(AHT10_t *dev);

#endif
