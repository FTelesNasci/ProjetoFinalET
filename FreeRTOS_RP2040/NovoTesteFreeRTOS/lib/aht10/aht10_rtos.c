#include "aht10_rtos.h"
#include "FreeRTOS.h"
#include "task.h"

#define AHT10_CMD_INIT      0xE1
#define AHT10_CMD_TRIGGER   0xAC
#define AHT10_CMD_SOFTRESET 0xBA

#define AHT10_DELAY(ms)     vTaskDelay(pdMS_TO_TICKS(ms))

// Escrita básica
static bool aht10_write(AHT10_t *dev, const uint8_t *data, uint16_t len)
{
    int ret = i2c_write_blocking(dev->i2c, dev->address, data, len, false);
    return (ret == len);
}

// Leitura básica
static bool aht10_read_bytes(AHT10_t *dev, uint8_t *data, uint16_t len)
{
    int ret = i2c_read_blocking(dev->i2c, dev->address, data, len, false);
    return (ret == len);
}


// ---------------------------------------------------------------
// Inicialização baseada no seu programa original
// ---------------------------------------------------------------
bool AHT10_Init(AHT10_t *dev)
{
    uint8_t reset_cmd = AHT10_CMD_SOFTRESET;
    if (!aht10_write(dev, &reset_cmd, 1))
        return false;

    AHT10_DELAY(50);       // suficiente e comprovado no seu driver

    uint8_t init_cmd[3] = { AHT10_CMD_INIT, 0x08, 0x00 };
    if (!aht10_write(dev, init_cmd, 3))
        return false;

    AHT10_DELAY(50);

    return true;
}

// ---------------------------------------------------------------
// Leitura
// ---------------------------------------------------------------
bool AHT10_Read(AHT10_t *dev)
{
    uint8_t cmd[3] = {AHT10_CMD_TRIGGER, 0x33, 0x00};
    uint8_t data[6];

    if (!aht10_write(dev, cmd, 3))
        return false;

    AHT10_DELAY(80);

    if (!aht10_read_bytes(dev, data, 6))
        return false;

    // Conversão baseada no seu driver
    uint32_t raw_h =
        ((uint32_t)data[1] << 12) |
        ((uint32_t)data[2] << 4) |
        ((data[3] >> 4) & 0x0F);

    uint32_t raw_t =
        (((uint32_t)data[3] & 0x0F) << 16) |
        ((uint32_t)data[4] << 8) |
        ((uint32_t)data[5]);

    dev->humidity = ((float)raw_h / 1048576.0f) * 100.0f;
    dev->temperature = (((float)raw_t / 1048576.0f) * 200.0f) - 50.0f;

    return true;
}
