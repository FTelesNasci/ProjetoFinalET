#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "aht10.h"
#include "bh1750.h"

#define WIFI_SSID "ITSelf"
#define WIFI_SENHA "code2020"

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
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_SENHA, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
        // Read the ip address in a human readable way
        uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    }

    xTaskCreate(task_hello, "Hello", 1024, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true) {    
    }
}

void task_hello(void *pvParameters)
{
    while (1)
    {
        printf("Hello, world! ou a task FreeRTOS\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void task_humidade_temperatura (void *pvParameters)
{
 // Começar a partir daqui Adriano 
}
