<<<<<<< HEAD
=======
// configuração inicial do firmware para Raspberry Pi Pico W
// Inclusão das bibliotecas;
// Conexões com os sensores

>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
#include <stdio.h>
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "aht10.h"
#include "bh1750.h"
#include "ssd1306.h"

// Wi-fi
<<<<<<< HEAD
//#define WIFI_SSID "Lu e Deza"
//#define WIFI_SENHA "liukin1208"
=======
<<<<<<<< HEAD:ProjetoFinalCurso/ProjetoFinalCurso.c
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
// #define WIFI_SSID "ITSelf"
// #define WIFI_SENHA "code2020"
#define WIFI_SSID "Teles"
#define WIFI_SENHA "Sophia2013%"
<<<<<<< HEAD


// Configurações MQTT
#define MQTT_BROKER "broker.hivemq.com"
#define MQTT_BROKER_PORT 1883
#define MQTT_TOPIC "embarca/tem_lux"

// Variáveis Globais
=======
========
#define WIFI_SSID "Lu e Deza"
#define WIFI_SENHA "liukin1208"
//#define WIFI_SSID "ITSelf"
//#define WIFI_SENHA "code2020"
>>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647:AlteracaoProjetoFinalCurso/ProjetoFinalCurso.c

// Configurações MQTT
#define MQTT_BROKER "broker.hivemq.com" // Localização do IP da conexão com o MQTT
#define MQTT_BROKER_PORT 1883 // Porta de comunicação
#define MQTT_TOPIC "embarca/tem_lux" //status do sensor de luminosidade

// Variáveis Globais de comunicação para dizer o funcionamento do sistema MQTT
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
static mqtt_client_t *mqtt_client;
static ip_addr_t broker_ip;
static bool mqtt_connected = false;
volatile bool buzzer_alarm = false;


// =======================================================
// CONFIGURAÇÃO DE PINOS E CONSTANTES
// =======================================================
<<<<<<< HEAD
=======
// Sensores 
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
#define I2C_PORT_AHT  i2c0
#define SDA_AHT       0
#define SCL_AHT       1

<<<<<<< HEAD
=======
// e display
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
#define I2C_PORT_OLED i2c1
#define SDA_OLED      14
#define SCL_OLED      15

// =======================================================
// BUZZER PASSIVO (PWM)
// =======================================================
#define BUZZER_PIN       21
#define BUZZER_FREQ_HZ   2500
#define LED_ALARM_ON   13
#define LED_ALARM_OFF  11

//=========================================================
// Limiares de temperatura para controle do alarme (histerese)
//=========================================================
<<<<<<< HEAD
#define TEMP_LIMIT_ON    27.0f          // Ligar alarme acima de 27°C
#define TEMP_LIMIT_OFF   26.0f          // Desligar alarme abaixo de 26°C (histerese)
=======
#define TEMP_LIMIT_ON    32.0f          // Ligar alarme acima de 32°C
#define TEMP_LIMIT_OFF   31.0f          // Desligar alarme abaixo de 31°C (histerese)
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647

// =======================================================
// ESTRUTURA DE DADOS PARA A FILA
// =======================================================
<<<<<<< HEAD
=======
// Armazenamento, medições, fila (FreeRTOS) e compartilhamento
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
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
<<<<<<< HEAD
=======
// Aquisição dos sensores;
// Exibição;
// Comunicação com o MQTT;
// Temporização;
// Integração
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
void TaskSensor(void *pv);
void TaskBuzzer(void *pv);
void TaskDisplay(void *pv);
void TaskSerial(void *pv);
int i2c_write_wrapper(uint8_t addr, const uint8_t *data, uint16_t len);
int i2c_read_wrapper(uint8_t addr, uint8_t *data, uint16_t len);
void delay_ms_wrapper(uint32_t ms);
void task_hello(void *pvParameters);
void TaskMQTT(void *pv);

// Protótipo das Funções
static void mqtt_connection_callback(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
void publish_msg(bool button_pressed, float temp_c);
<<<<<<< HEAD
//float read_temperature();
void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg);

=======
// Verificar essa variável abaixo;
// Float read_temperature();
void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg);

// Configuração dos periféricos;
// Inicialização dos barramentos I2C (sensores e display);
// Exibição de mensagens à Raspberry;
// Conexão com IoT.
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
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
<<<<<<< HEAD
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // Enable wifi station
    cyw43_arch_enable_sta_mode();

=======
    if (cyw43_arch_init()) 
    {
        printf("Wi-Fi init failed\n");
        return -1;
    }
    // Conexão com a rede Wi-Fi
    // Comunicação com IoT
    // Enable wifi station
    cyw43_arch_enable_sta_mode();
    // Comunicação com o Wi-Fi
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
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

<<<<<<< HEAD
    // Inicializa cliente MQTT
    mqtt_client = mqtt_client_new();

    // Resolve DNS do broker MQTT
    err_t err = dns_gethostbyname(MQTT_BROKER, &broker_ip, dns_check_callback, NULL);
    if (err == ERR_OK) {
        dns_check_callback(MQTT_BROKER, &broker_ip, NULL);
    } else if (err == ERR_INPROGRESS) {
        printf("[DNS] Resolvendo...\n");
    } else {
=======
    // Inicializa a conexão com o MQTT
    mqtt_client = mqtt_client_new();

    // Resolve DNS do broker MQTT
    // Comunicação e avisa se houve falha
    err_t err = dns_gethostbyname(MQTT_BROKER, &broker_ip, dns_check_callback, NULL);
    if (err == ERR_OK) 
    {
        dns_check_callback(MQTT_BROKER, &broker_ip, NULL);
    } 
    else if (err == ERR_INPROGRESS) 
    {
        printf("[DNS] Resolvendo...\n");
    }
     else 
    {
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
        printf("[DNS] Erro ao resolver DNS: %d\n", err);
        return -1;
    }

    // Configura BUZZER como PWM
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint channel   = pwm_gpio_to_channel(BUZZER_PIN);

    uint32_t clock = 125000000; // clock padrão RP2040
    uint32_t divider = 4;
    uint32_t wrap = (clock / (divider * BUZZER_FREQ_HZ)) - 1;

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, divider);
    pwm_config_set_wrap(&config, wrap);

    pwm_init(slice_num, &config, true);
    pwm_set_chan_level(slice_num, channel, 0); // inicia desligado

    gpio_init(LED_ALARM_ON);
    gpio_set_dir(LED_ALARM_ON, GPIO_OUT);
    gpio_put(LED_ALARM_ON, 0);

    gpio_init(LED_ALARM_OFF);
    gpio_set_dir(LED_ALARM_OFF, GPIO_OUT);
    gpio_put(LED_ALARM_OFF, 1); // começa indicando sistema normal


    // -------------------
    // CRIA A QUEUE
    // -------------------
<<<<<<< HEAD
=======
    // Criação da fila;
    // Comunicação com a FreeRTOS;
    // Inicialização das tarefa;
    // Armazenamento e compartilhamento;
    // Sensores, display, MQTT, Execução e Prioridades
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
    xSensorQueue = xQueueCreate(1, sizeof(SensorData_t));
    xQueueReset(xSensorQueue);

    // -------------------
    // CRIA AS TASKS
    // -------------------
    xTaskCreate(TaskSensor,  "Sensor",  1024, NULL, 3, NULL);
    xTaskCreate(TaskDisplay, "Display", 1024, NULL, 1, NULL);
    xTaskCreate(TaskSerial,  "Serial",  1024, NULL, 1, NULL);
    xTaskCreate(task_hello, "Hello", 1024, NULL, 1, NULL);
<<<<<<< HEAD
    xTaskCreate(TaskMQTT, "MQTT", 2048, NULL, 2, NULL);
    xTaskCreate(TaskBuzzer, "Buzzer", 512, NULL, 2, NULL);
=======
<<<<<<<< HEAD:ProjetoFinalCurso/ProjetoFinalCurso.c
    xTaskCreate(TaskMQTT, "MQTT", 4096, NULL, 2, NULL);

========
    xTaskCreate(TaskMQTT, "MQTT", 2048, NULL, 2, NULL);
    xTaskCreate(TaskBuzzer, "Buzzer", 512, NULL, 2, NULL);
>>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647:AlteracaoProjetoFinalCurso/ProjetoFinalCurso.c
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647

    vTaskStartScheduler();

    while (true) {    
    }
}

// =======================================================
// WRAPPERS PARA O DRIVER DO AHT10
// =======================================================
<<<<<<< HEAD
=======
// Temporização;
// Integração com os sensores ao FreeRTOS;
// Gerenciamento da conexão com o MQQT;
// Compartibilidade e confiabilidade com a comunicação.
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
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

<<<<<<< HEAD
// Callback de conexão MQTT
=======

// Callback de conexão MQTT
// Tem a função de transmitir a informação, varredura, executa a função;
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
static void mqtt_connection_callback(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Conectado ao broker!\n");
        mqtt_connected = true;
<<<<<<< HEAD
    } else {
=======
    } 
    else 
    {
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
        printf("[MQTT] Falha na conexão MQTT. Código: %d\n", status);
        mqtt_connected = false;
    }
}
<<<<<<< HEAD

// Callback de DNS
void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
    if (ipaddr != NULL) {
        broker_ip = *ipaddr;
        printf("[DNS] Resolvido: %s -> %s\n", name, ipaddr_ntoa(ipaddr));

        struct mqtt_connect_client_info_t client_info = {
=======
// Inicia a comunicação com o MQTT;
// Serviço remoto;
// Emissão periódica das mensagens
// Callback de DNS
void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg) 
{
    if (ipaddr != NULL) 
    {
        broker_ip = *ipaddr;
        printf("[DNS] Resolvido: %s -> %s\n", name, ipaddr_ntoa(ipaddr));
        struct mqtt_connect_client_info_t client_info = 
        {
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
            .client_id = "pico-client",
            .keep_alive = 60,
            .client_user = NULL,
            .client_pass = NULL,
            .will_topic = NULL,
            .will_msg = NULL,
            .will_qos = 0,
            .will_retain = 0
        };

        printf("[MQTT] Conectando ao broker...\n");
        mqtt_client_connect(mqtt_client, &broker_ip, MQTT_BROKER_PORT, mqtt_connection_callback, NULL, &client_info);
<<<<<<< HEAD
    } else {
=======
    } 
    else 
    {
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
        printf("[DNS] Falha ao resolver DNS para %s\n", name);
    }
}

<<<<<<< HEAD
void task_hello(void *pvParameters)
{
    while (1)
    {
        printf("Alô, Mundo! ou a task FreeRTOS\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
=======
 void task_hello(void *pvParameters)
 {
     while (1)
     {
         printf("Alô, Mundo! ou a task FreeRTOS\n");
         vTaskDelay(pdMS_TO_TICKS(1000));
     }
 }
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647


// =======================================================
// TASK — SENSOR AHT10
// =======================================================
<<<<<<< HEAD
=======
// Comunicação entre os sensores;
// Leitura dos sensores;
// Armazenamento de dados;
// Aquisição periódica.
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
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

            // --------- Controle de Alarme com Histerese ---------
            if (data.temperature > TEMP_LIMIT_ON)
            {
                buzzer_alarm = true;
            }
            else if (data.temperature < TEMP_LIMIT_OFF)
            {
                buzzer_alarm = false;
            }

            xQueueOverwrite(xSensorQueue, &data);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// =======================================================
// TASK — DISPLAY OLED
// =======================================================
<<<<<<< HEAD
=======
// Interface do sistema;
// Atualização cíclica;
// Atraso e estabilidade.
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
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
<<<<<<< HEAD
void TaskSerial(void *pv)
{
=======
// Leitura;
// Exibição periódica dos dados;
// MQTT
 void TaskSerial(void *pv)
 {
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
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

<<<<<<< HEAD
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void publish_sensor_data(const SensorData_t *data)
{
    if (!mqtt_connected) {
=======
       vTaskDelay(pdMS_TO_TICKS(500));
    }
 }


// Adicionei agora
// void TaskSerial(void *pv)
// {
//    SensorData_t data;

//    while (1)
//    {
//        if (xQueuePeek(xSensorQueue, &data, portMAX_DELAY))
//        {
//            printf("Temp: %.2f C | Umid: %.2f %% | Luz: %.1f lx\n",
//                   data.temperature,
//                   data.humidity,
//                   data.lux);
//        }

//        vTaskDelay(pdMS_TO_TICKS(5000));
//    }
//}


void publish_sensor_data(const SensorData_t *data)
{
    if (!mqtt_connected)
    {
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
        printf("[MQTT] Não conectado\n");
        return;
    }

    char payload[160];

    snprintf(payload, sizeof(payload),
        "{"
        "\"temperatura\": %.2f,"
        "\"umidade\": %.2f,"
        "\"luminosidade\": %.1f"
        "}",
        data->temperature,
        data->humidity,
        data->lux
    );
<<<<<<< HEAD

=======
// Publicação dos dados;
// Registro de mensagem (erro ou acerto);
// Leitura, fila, execução e transmissão;
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
    err_t err = mqtt_publish(
        mqtt_client,
        MQTT_TOPIC,
        payload,
        strlen(payload),
        0,
        0,
        NULL,
        NULL
    );

    if (err == ERR_OK)
        printf("[MQTT] Enviado: %s\n", payload);
    else
        printf("[MQTT] Erro publish: %d\n", err);
}

void TaskMQTT(void *pv)
{
    SensorData_t data;

    while (1)
    {
        if (xQueuePeek(xSensorQueue, &data, portMAX_DELAY))
        {
            publish_sensor_data(&data);
        }

<<<<<<< HEAD
        vTaskDelay(pdMS_TO_TICKS(2000)); // intervalo MQTT
=======
        vTaskDelay(pdMS_TO_TICKS(5000)); // intervalo MQTT
>>>>>>> 6e91d33ca8b438889b584f4301b35d1c307ed647
    }
}

void TaskBuzzer(void *pv)
{
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint channel   = pwm_gpio_to_channel(BUZZER_PIN);

    uint32_t clock = 125000000;
    uint32_t divider = 4;
    uint32_t wrap = (clock / (divider * BUZZER_FREQ_HZ)) - 1;

    uint32_t duty = wrap / 2; // 50%

    while (1)
    {
        if (buzzer_alarm)
        {
            // LED estado ALARME
            gpio_put(LED_ALARM_ON, 1);
            gpio_put(LED_ALARM_OFF, 0);

            // Som intermitente
            pwm_set_chan_level(slice_num, channel, duty);
            vTaskDelay(pdMS_TO_TICKS(200));

            pwm_set_chan_level(slice_num, channel, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        else
        {
            // LED estado NORMAL
            gpio_put(LED_ALARM_ON, 0);
            gpio_put(LED_ALARM_OFF, 1);

            pwm_set_chan_level(slice_num, channel, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

