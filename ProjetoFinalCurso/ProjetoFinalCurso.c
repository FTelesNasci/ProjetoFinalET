#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "aht10.h"
#include "bh1750.h"
#include "ssd1306.h"

#define WIFI_SSID "ITSelf"
#define WIFI_SENHA "code2020"

// =======================================================
// CONFIGURAÇÃO DE PINOS E CONSTANTES
// =======================================================
#define I2C_PORT_AHT  i2c0
#define SDA_AHT       0
#define SCL_AHT       1

#define I2C_PORT_OLED i2c1
#define SDA_OLED      14
#define SCL_OLED      15

// =======================================================
// ESTRUTURA DE DADOS PARA A FILA
// =======================================================
typedef struct {
    float temperature;
    float humidity;
    float lux;
} SensorData_t;

// =======================================================
// QUEUE DE COMUNICAÇÃO ENTRE AS TASKS
// =======================================================
QueueHandle_t xSensorQueue;

// =======================================================
// PROTÓTIPOS DAS TASKS E FUNÇÕES
// =======================================================
void TaskSensor(void *pv);
void TaskDisplay(void *pv);
void TaskSerial(void *pv);
int i2c_write_wrapper(uint8_t addr, const uint8_t *data, uint16_t len);
int i2c_read_wrapper(uint8_t addr, uint8_t *data, uint16_t len);
void delay_ms_wrapper(uint32_t ms);
void task_hello(void *pvParameters);

int main()
{
    stdio_init_all();

    // -------------------
    // I2C DO SENSOR
    // -------------------
    i2c_init(I2C_PORT_AHT, 100000);
    gpio_set_function(SDA_AHT, GPIO_FUNC_I2C);
    gpio_set_function(SCL_AHT, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_AHT);
    gpio_pull_up(SCL_AHT);

    // -------------------
    // I2C DO OLED
    // -------------------
    i2c_init(I2C_PORT_OLED, 400000);
    gpio_set_function(SDA_OLED, GPIO_FUNC_I2C);
    gpio_set_function(SCL_OLED, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_OLED);
    gpio_pull_up(SCL_OLED);

    // Inicializa OLED
    ssd1306_init(I2C_PORT_OLED);
    ssd1306_clear();
    ssd1306_draw_string(28, 0, "Embarcatech");

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // Enable wifi station
    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_SENHA, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
        // Read the ip address in a human readable way
        uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    }

    // -------------------
    // CRIA A QUEUE
    // -------------------
    xSensorQueue = xQueueCreate(1, sizeof(SensorData_t));
    xQueueReset(xSensorQueue);

    // -------------------
    // CRIA AS TASKS
    // -------------------
    xTaskCreate(TaskSensor,  "Sensor",  1024, NULL, 3, NULL);
    xTaskCreate(TaskDisplay, "Display", 1024, NULL, 1, NULL);
    xTaskCreate(TaskSerial,  "Serial",  1024, NULL, 1, NULL);
    xTaskCreate(task_hello, "Hello", 1024, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true) {    
    }
}

// =======================================================
// WRAPPERS PARA O DRIVER DO AHT10
// =======================================================
int i2c_write_wrapper(uint8_t addr, const uint8_t *data, uint16_t len)
{
    int r = i2c_write_blocking(I2C_PORT_AHT, addr, data, len, false);
    return (r < 0) ? -1 : 0;
}

int i2c_read_wrapper(uint8_t addr, uint8_t *data, uint16_t len)
{
    int r = i2c_read_blocking(I2C_PORT_AHT, addr, data, len, false);
    return (r < 0) ? -1 : 0;
}

void delay_ms_wrapper(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}


void task_hello(void *pvParameters)
{
    while (1)
    {
        printf("Alô, Mundo! ou a task FreeRTOS\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// =======================================================
// TASK — SENSOR AHT10
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
// TASK — DISPLAY OLED
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
// TASK — SERIAL (UART DEBUG)
// =======================================================
void TaskSerial(void *pv)
{
    SensorData_t data;

    while (1)
    {
        // Lê SEM CONSUMIR a fila
        if (xQueuePeek(xSensorQueue, &data, portMAX_DELAY))
        {
            printf("Temperatura: %.2f C | Umidade: %.2f %%\n",
                   data.temperature,
                   data.humidity);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}