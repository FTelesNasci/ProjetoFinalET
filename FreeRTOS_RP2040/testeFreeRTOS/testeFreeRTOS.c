#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


#define SSID "Lu e Deza"
#define PASSWORD "liukin1208"
// =========================================
// DEFINIÇÕES
// =========================================
#define BUTTON_PIN 5
#define LED_PIN    11

// Queue para comunicação
QueueHandle_t queueButton;

// =========================================
// PROTÓTIPOS
// =========================================
void task_button(void *pvParameters);
void task_led(void *pvParameters);
void task_hello(void *pvParameters);


int main()
{
    stdio_init_all();

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // Enable wifi station
    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(SSID, PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
        // Read the ip address in a human readable way
        uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
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
