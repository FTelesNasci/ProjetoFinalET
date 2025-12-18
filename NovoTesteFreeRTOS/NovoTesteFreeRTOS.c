#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Drivers
#include "aht10.h"
#include "bh1750.h"
#include "ssd1306.h"

// lwIP
#include "lwip/ip4_addr.h"

// =======================================================
// CONFIGURAÇÃO DE PINOS E CONSTANTES
// =======================================================
#define I2C_PORT_AHT   i2c0
#define SDA_AHT        0
#define SCL_AHT        1

#define I2C_PORT_OLED  i2c1
#define SDA_OLED       14
#define SCL_OLED       15

#define WIFI_SSID      "Lu e Deza"
#define WIFI_PASSWORD  "liukin1208"

// =======================================================
// ESTRUTURA DE DADOS DOS SENSORES
// =======================================================
typedef struct {
    float temperature;
    float humidity;
    float lux;
} SensorData_t;

// =======================================================
QueueHandle_t xSensorQueue;

// =======================================================
// PROTÓTIPOS
// =======================================================
void TaskSensor(void *pv);
void TaskDisplay(void *pv);
void TaskSerial(void *pv);
void TaskWiFi(void *pv);

int  i2c_write_wrapper(uint8_t addr, const uint8_t *data, uint16_t len);
int  i2c_read_wrapper(uint8_t addr, uint8_t *data, uint16_t len);
void delay_ms_wrapper(uint32_t ms);

// =======================================================
// MAIN
// =======================================================
int main(void)
{
    stdio_init_all();

    // ---------- I2C SENSOR ----------
    i2c_init(I2C_PORT_AHT, 100000);
    gpio_set_function(SDA_AHT, GPIO_FUNC_I2C);
    gpio_set_function(SCL_AHT, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_AHT);
    gpio_pull_up(SCL_AHT);

    // ---------- I2C OLED ----------
    i2c_init(I2C_PORT_OLED, 400000);
    gpio_set_function(SDA_OLED, GPIO_FUNC_I2C);
    gpio_set_function(SCL_OLED, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_OLED);
    gpio_pull_up(SCL_OLED);

    ssd1306_init(I2C_PORT_OLED);

    // ---------- QUEUE ----------
    xSensorQueue = xQueueCreate(1, sizeof(SensorData_t));

    // ---------- TASKS ----------
    xTaskCreate(TaskSensor,  "Sensor",  1024, NULL, 3, NULL);
    xTaskCreate(TaskDisplay, "Display", 1024, NULL, 1, NULL);
    xTaskCreate(TaskSerial,  "Serial",  1024, NULL, 1, NULL);
    xTaskCreate(TaskWiFi,    "WiFi",    3072, NULL, 4, NULL);

    vTaskStartScheduler();

    while (true) {}
}

// =======================================================
// WRAPPERS AHT10
// =======================================================
int i2c_write_wrapper(uint8_t addr, const uint8_t *data, uint16_t len)
{
    return (i2c_write_blocking(I2C_PORT_AHT, addr, data, len, false) < 0) ? -1 : 0;
}

int i2c_read_wrapper(uint8_t addr, uint8_t *data, uint16_t len)
{
    return (i2c_read_blocking(I2C_PORT_AHT, addr, data, len, false) < 0) ? -1 : 0;
}

void delay_ms_wrapper(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// =======================================================
// TASK SENSOR (AHT10 + BH1750)
// =======================================================
void TaskSensor(void *pv)
{
    AHT10_Handle aht10 = {
        .iface = {
            .i2c_write = i2c_write_wrapper,
            .i2c_read  = i2c_read_wrapper,
            .delay_ms  = delay_ms_wrapper
        }
    };

    printf("Inicializando AHT10...\n");
    if (!AHT10_Init(&aht10)) {
        printf("ERRO AHT10\n");
        vTaskDelete(NULL);
    }

    printf("Inicializando BH1750...\n");
    bh1750_init(I2C_PORT_AHT);

    while (1)
    {
        SensorData_t data = {0};

        if (AHT10_ReadTemperatureHumidity(&aht10,
                                          &data.temperature,
                                          &data.humidity))
        {
            float lux = bh1750_read_lux(I2C_PORT_AHT);
            data.lux = (lux >= 0.0f) ? lux : -1.0f;

            xQueueOverwrite(xSensorQueue, &data);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// =======================================================
// TASK DISPLAY OLED
// =======================================================
void TaskDisplay(void *pv)
{
    SensorData_t data;
    char buf[16];

    while (1)
    {
        if (xQueuePeek(xSensorQueue, &data, portMAX_DELAY))
        {
            ssd1306_clear();
            ssd1306_draw_string(28, 0, "Embarcatech");

            snprintf(buf, sizeof(buf), "%.2f C", data.temperature);
            ssd1306_draw_string(0, 16, "Temp:");
            ssd1306_draw_string(80, 16, buf);

            snprintf(buf, sizeof(buf), "%.2f %%", data.humidity);
            ssd1306_draw_string(0, 32, "Umid:");
            ssd1306_draw_string(80, 32, buf);

            snprintf(buf, sizeof(buf), "%.1f lx", data.lux);
            ssd1306_draw_string(0, 48, "Luz:");
            ssd1306_draw_string(80, 48, buf);

            ssd1306_show();
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// =======================================================
// TASK SERIAL
// =======================================================
void TaskSerial(void *pv)
{
    SensorData_t data;

    while (1)
    {
        if (xQueuePeek(xSensorQueue, &data, portMAX_DELAY))
        {
            printf("T: %.2f C | H: %.2f %% | L: %.1f lx\n",
                   data.temperature,
                   data.humidity,
                   data.lux);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// =======================================================
// TASK WIFI
// =======================================================
void TaskWiFi(void *pv)
{
    if (cyw43_arch_init()) {
        printf("Falha CYW43\n");
        vTaskDelete(NULL);
    }

    cyw43_arch_enable_sta_mode();

    while (1)
    {
        if (cyw43_arch_wifi_connect_timeout_ms(
                WIFI_SSID,
                WIFI_PASSWORD,
                CYW43_AUTH_WPA2_AES_PSK,
                10000) == 0)
        {
            const ip4_addr_t *ip =
                &cyw43_state.netif[CYW43_ITF_STA].ip_addr;

            printf("Wi-Fi OK | IP: %s\n", ip4addr_ntoa(ip));
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    while (1)
        vTaskDelay(pdMS_TO_TICKS(10000));
}
