// main.c - Datalogger AHT10 -> SD card com FreeRTOS + Queue

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/rtc.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Drivers do projeto */
#include "aht10.h"
#include "ssd1306.h"
#include "ff.h"
#include "sd_card.h"
#include "hw_config.h"

#define I2C_PORT0 i2c0
#define I2C_SDA0 0
#define I2C_SCL0 1

#define I2C_PORT1 i2c1
#define I2C_SDA1 14
#define I2C_SCL1 15

#define LOG_FILENAME "LOG_ENV.TXT"

/* ---------- Estrutura enviada via Queue ---------- */
typedef struct {
    float temp;
    float hum;
} SensorData_t;

/* Queue de comunicação */
static QueueHandle_t xQueueSensor;

/* ---------- Protótipos ---------- */
static void delay_ms(uint32_t ms);
int  i2c_write_adapter(uint8_t addr, const uint8_t *data, uint16_t len);
int  i2c_read_adapter(uint8_t addr, uint8_t *data, uint16_t len);
static bool get_timestamp_string(char *buf, size_t len);
static void log_data(float temp, float hum);
static void display_values(float temp, float hum, bool recording);
static void set_initial_time_if_needed(void);

/* Tasks */
static void SensorTask(void *p);
static void LoggerTask(void *p);

/* Handle global do sensor */
static AHT10_Handle aht;

/* ===========================================================
 *  main()
 * =========================================================== */
int main(void)
{
    stdio_init_all();
    sleep_ms(100);

    /* --- I2C AHT10 --- */
    i2c_init(I2C_PORT0, 100 * 1000);
    gpio_set_function(I2C_SDA0, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL0, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA0);
    gpio_pull_up(I2C_SCL0);

    /* --- I2C OLED --- */
    i2c_init(I2C_PORT1, 400 * 1000);
    gpio_set_function(I2C_SDA1, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL1, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA1);
    gpio_pull_up(I2C_SCL1);

    /* RTC inicial (opcional) */
    set_initial_time_if_needed();

    /* OLED */
    ssd1306_init(I2C_PORT1);
    ssd1306_clear();
    ssd1306_draw_string(8, 14, "Inicializando...");
    ssd1306_show();
    sleep_ms(300);

    /* Configura adaptadores AHT10 */
    aht.iface.i2c_write = i2c_write_adapter;
    aht.iface.i2c_read  = i2c_read_adapter;
    aht.iface.delay_ms  = delay_ms;
    aht.initialized     = false;

    if (!AHT10_Init(&aht)) {
        ssd1306_clear();
        ssd1306_draw_string(8, 18, "Erro AHT10");
        ssd1306_show();
        while (1) sleep_ms(500);
    }

    /* SD: cria arquivo se não existir */
    sd_card_t *pSD = sd_get_by_num(0);
    if (pSD) {
        if (f_mount(&pSD->fatfs, pSD->pcName, 1) == FR_OK) {
            FIL f;
            if (f_open(&f, LOG_FILENAME, FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
                if (f_size(&f) == 0)
                    f_printf(&f, "DataHora; Temperatura(C); Umidade(%%)\n");
                f_close(&f);
            }
            f_unmount(pSD->pcName);
        }
    }

    ssd1306_clear();
    ssd1306_draw_string(6, 6, "Datalogger pronto");
    ssd1306_show();
    sleep_ms(600);

    /* ---------- Cria queue ---------- */
    xQueueSensor = xQueueCreate(5, sizeof(SensorData_t));
    if (!xQueueSensor) {
        printf("ERRO ao criar queue\n");
        while (1);
    }

    /* ---------- Cria Tasks ---------- */
    xTaskCreate(SensorTask, "SensorTask", 2048, NULL, 2, NULL);
    xTaskCreate(LoggerTask, "LoggerTask", 2048, NULL, 1, NULL);

    /* Inicia o FreeRTOS */
    vTaskStartScheduler();

    while (1);
    return 0;
}

/* ===========================================================
 *  SensorTask — Le AHT10 e envia para queue a cada 2s
 * =========================================================== */
static void SensorTask(void *p)
{
    SensorData_t data;

    while (1) {

        if (AHT10_ReadTemperatureHumidity(&aht, &data.temp, &data.hum)) {
            xQueueSend(xQueueSensor, &data, portMAX_DELAY);
        } else {
            printf("Falha leitura AHT10\n");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ===========================================================
 *  LoggerTask — Recebe dados, atualiza OLED e grava no SD
 * =========================================================== */
static void LoggerTask(void *p)
{
    SensorData_t rx;
    uint64_t last_log = to_ms_since_boot(get_absolute_time());

    while (1) {

        /* Aguarda leitura do sensor */
        if (xQueueReceive(xQueueSensor, &rx, portMAX_DELAY) == pdTRUE) {

            display_values(rx.temp, rx.hum, false);

            /* Log a cada 60s */
            uint64_t now = to_ms_since_boot(get_absolute_time());
            if ((now - last_log) >= 60000) {
                display_values(rx.temp, rx.hum, true);
                log_data(rx.temp, rx.hum);
                vTaskDelay(pdMS_TO_TICKS(700));
                display_values(rx.temp, rx.hum, false);
                last_log = now;
            }
        }
    }
}

/* ===========================================================
 *  Funções auxiliares
 * =========================================================== */
static void delay_ms(uint32_t ms) {
    sleep_ms(ms);
}

int i2c_write_adapter(uint8_t addr, const uint8_t *data, uint16_t len) {
    int r = i2c_write_blocking(I2C_PORT0, addr, data, len, false);
    return r < 0 ? -1 : 0;
}
int i2c_read_adapter(uint8_t addr, uint8_t *data, uint16_t len) {
    int r = i2c_read_blocking(I2C_PORT0, addr, data, len, false);
    return r < 0 ? -1 : 0;
}

static bool get_timestamp_string(char *buf, size_t len) {
    time_t t = time(NULL);
    if (t != (time_t)-1) {
        struct tm *tm = localtime(&t);
        if (!tm) return false;
        return strftime(buf, len, "%Y-%m-%d %H:%M:%S", tm) > 0;
    } else {
        uint64_t ms = to_ms_since_boot(get_absolute_time());
        snprintf(buf, len, "BOOT+%llums", (unsigned long long)ms);
        return true;
    }
}

static void log_data(float temp, float hum) {
    sd_card_t *pSD = sd_get_by_num(0);
    if (!pSD) return;

    if (f_mount(&pSD->fatfs, pSD->pcName, 1) != FR_OK) return;

    FIL fil;
    if (f_open(&fil, LOG_FILENAME, FA_OPEN_APPEND | FA_WRITE) == FR_OK) {

        char ts[32];
        if (!get_timestamp_string(ts, sizeof(ts)))
            strcpy(ts, "UNKNOWN");

        f_printf(&fil, "%s;%.2f;%.2f\n", ts, temp, hum);

        f_close(&fil);
    }

    f_unmount(pSD->pcName);
}

static void display_values(float temp, float hum, bool recording) {
    char buf[32];
    ssd1306_clear();

    ssd1306_draw_string(0, 0, "Datalogger");

    snprintf(buf, sizeof(buf), "Temperatura: %.2f C", temp);
    ssd1306_draw_string(0, 20, buf);

    snprintf(buf, sizeof(buf), "Umidade:     %.2f %%", hum);
    ssd1306_draw_string(0, 34, buf);

    if (recording)
        ssd1306_draw_string(70, 50, "Gravando");

    time_t t = time(NULL);
    struct tm *tm = localtime(&t);
    if (tm) {
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
                 tm->tm_hour, tm->tm_min, tm->tm_sec);
        ssd1306_draw_string(80, 0, buf);
    }

    ssd1306_show();
}

static void set_initial_time_if_needed(void) {
    datetime_t dt = {
        .year = 2025, .month = 12, .day = 10,
        .dotw = 4, .hour = 8, .min = 20, .sec = 0
    };
    rtc_init();
    rtc_set_datetime(&dt);
}
