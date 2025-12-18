#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "aht10.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "aht10.h"


// =========================================
// DEFINIÇÕES
// =========================================
#define BUTTON_PIN 5        // Botão conectado ao GPIO 5
#define LED_PIN    11       // LED conectado ao GPIO 11
#define I2C_PORT0 i2c0      // AHT10
#define I2C_SDA0 0          
#define I2C_SCL0 1
#define I2C_PORT1 i2c1      // Display OLED SSD1306
#define I2C_SDA1 14
#define I2C_SCL1 15

// =========================================
// Queue para comunicação
// =========================================
QueueHandle_t queueButton;      // Fila para mensagens do botão


// =========================================
// PROTÓTIPOS
// =========================================
void task_button(void *pvParameters);
void task_led(void *pvParameters);
void task_hello(void *pvParameters);

// =========================================
// FUNÇÃO PRINCIPAL
// =========================================
int main()
{
    stdio_init_all();

    // Inicializa I2C sensor
    i2c_init(I2C_PORT0, 100 * 1000); // 100 kHz
    gpio_set_function(I2C_SDA0, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL0, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA0);
    gpio_pull_up(I2C_SCL0);

    // Inicializa I2C OLED
    i2c_init(I2C_PORT1, 400000);
    gpio_set_function(I2C_SDA1, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL1, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA1);
    gpio_pull_up(I2C_SCL1);

    ssd1306_init(I2C_PORT1);
    ssd1306_clear();
    ssd1306_draw_string(32, 0, "Embarcatech");
    ssd1306_draw_string(20, 10, "Inicializando...");
    ssd1306_show();

    if (cyw43_arch_init()) {
        printf("Falha na inicialização do Wi-Fi\n");
        return -1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Conectando ao Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms("Lu e Deza", "liukin1208",
        CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("Falha ao conectar.\n");
        return 1;
    }
    else
    {
        printf("Conectado.\n");
        uint8_t *ip = (uint8_t*)&cyw43_state.netif[0].ip_addr.addr;
        printf("Endereço IP %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
    }

    // ======================================
    // Criar a fila (queue) para comunicação
    // ======================================
    queueButton = xQueueCreate(5, sizeof(uint8_t));

    if (queueButton == NULL)
    {
        printf("Erro ao criar a queue!\n");
        return -1;
    }

    // ======================================
    // Criar as tasks
    // ======================================
    xTaskCreate(task_button, "Button", 2048, NULL, 2, NULL);
    xTaskCreate(task_led,    "LED",    2048, NULL, 2, NULL);
    xTaskCreate(task_hello,  "Hello",  2048, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true) {}
}

// =========================================
// TASK 1: LER BOTÃO (GPIO 5)
// =========================================
void task_button(void *pvParameters)
{
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);   // botão entre GND e GPIO

    uint8_t msg = 1;
    uint8_t lastState = 1;

    while (1)
    {
        uint8_t state = gpio_get(BUTTON_PIN);

        // Detecta borda de descida (pressionou)
        if (state == 0 && lastState == 1)
        {
            // Envia mensagem para a task do LED
            xQueueSend(queueButton, &msg, 0);
            printf("Botão pressionado -> enviando mensagem\n");

            // tempo para evitar bouncing simples
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        lastState = state;
        vTaskDelay(pdMS_TO_TICKS(20)); // pequeno polling
    }
}

// =========================================
// TASK 2: CONTROLAR LED (GPIO 11)
// =========================================
void task_led(void *pvParameters)
{
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    uint8_t ledState = 0;
    uint8_t rx;

    while (1)
    {
        // Espera mensagem do botão
        if (xQueueReceive(queueButton, &rx, portMAX_DELAY))
        {
            ledState = !ledState;   // alterna estado
            gpio_put(LED_PIN, ledState);

            printf("LED agora = %s\n", ledState ? "LIGADO" : "DESLIGADO");
        }
    }
}

// =========================================
// TASK 3: HELLO WORLD
// =========================================
void task_hello(void *pvParameters)
{
    while (1)
    {
        printf("Hello, world! Sou a task FreeRTOS\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
