/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "led_strip.h"
#include "esp_wifi.h"
#include "sdkconfig.h"
#include "driver/temperature_sensor.h"
#include "nvs_flash.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "u8g2_esp32_hal.h"

// SDA - GPIO21
#define PIN_SDA 5

// SCL - GPIO22
#define PIN_SCL 6
// MAM - PROJECT

#if !CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#error "Console must be USB Serial/JTAG. Set: Component config -> ESP System Settings -> Channel for console output = USB Serial/JTAG."
#endif
#if !CONFIG_LOG_DEFAULT_LEVEL_NONE
#error "default level of log must be NONE. Set: Component config -> Log output -> Default log verbosity = NONE."
#endif
#if !CONFIG_LOG_MAXIMUM_LEVEL_VERBOSE
#error "MAX_level of log must be VERBOSE. Set: Component config -> Log output -> Default log verbosity = VERBOSE."
#endif

#define BLINK_GPIO 8

#define EXAMPLE_ESP_WIFI_SSID "SimecWifi"
#define EXAMPLE_ESP_WIFI_PASS "heslo1vsemalym"

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static bool wifiConected = false;
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "wifi-station";

static int s_retry_num = 0;

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
static uint8_t s_led_state = 0;

typedef struct
{
    float temp_c;
    float temp_f;
} temperature_t;

static QueueHandle_t temp_q; // queue for temperature values;
static QueueHandle_t period_q;

static void
configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifiConected = false;
        if (s_retry_num < 5)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGW(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        wifiConected = true;
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifiInit()
{
    s_wifi_event_group = xEventGroupCreate();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            // /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
            //  * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
            //  * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
            //  * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
            //  */
            // .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            // .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            // .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s",
                 EXAMPLE_ESP_WIFI_SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s",
                 EXAMPLE_ESP_WIFI_SSID);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void temp_sensor(void *params)
{
    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

    float tsens_value;
    while (true)
    {
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));
        temperature_t s = {
            .temp_c = tsens_value,
            .temp_f = (tsens_value * (9.0 / 5.0)) + 32.0};
        xQueueSend(temp_q, &s, 0);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

static void uart_receive(void *arg)
{
    // pro ESP32C3 je potreba zapnout USB Serial JTAG v menuconfig pro console output
    char str[64];
    while (true)
    {
        int result = scanf("%63s", str); // read one whitespace-separated word, max 63 chars + '\0'
        if (result == 1)
        {
            printf("You entered: \"%s\"\r\n", str);

            if (strcmp(str, "TEMPERATURE") == 0)
            {
                temperature_t s;
                xQueueReceive(temp_q, &s, pdMS_TO_TICKS(1000));

                printf("Temperature value: %.02f °C, %.02f °F \r\n", s.temp_c, s.temp_f);
            }
            else if (strcmp(str, "UNIXTIME") == 0)
            {
                time_t now;
                char strftime_buf[64];
                struct tm timeinfo;
                time(&now);
                localtime_r(&now, &timeinfo);
                strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
                ESP_LOGI(TAG, "UNIXTIME: %s", strftime_buf);
                ESP_LOGI(TAG, "UNIXTIME: %lld", now);
            }
            else if (strncmp(str, "PER:", 4) == 0)
            {
                long param = 0;
                const char *rest = str + 4;
                if (rest[0] != '\0')
                {
                    char *endptr = NULL;
                    errno = 0;
                    param = strtol(rest, &endptr, 10);
                }
                ESP_LOGI(TAG, "Set period to %ld ms", (param * 100));
                TickType_t period = param * 100 / portTICK_PERIOD_MS;
                xQueueOverwrite(period_q, &period);
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void blink_led(void *pvParameters)
{
    /* Configure the peripheral according to the LED type */
    configure_led();
    TickType_t period = 1000 / portTICK_PERIOD_MS;
    while (true)
    {
        TickType_t new_period;
        if (xQueueReceive(period_q, &new_period, 0) == pdPASS)
        {
            if (new_period != period)
            {
                period = new_period;
            }
        }
        gpio_set_level(BLINK_GPIO, s_led_state);
        s_led_state = !s_led_state;
        vTaskDelay(period);
    }
}

void Wifi_station(void *arg)
{
    wifiInit();
    while (true)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    // vTaskDelete(NULL);
}

void NTP_time(void *args)
{

    time_t now;
    time(&now);
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset();
    static bool wifiConectedPrev = false;
    while (true)
    {
        if (wifiConected && !wifiConectedPrev)
        {
            esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
            esp_netif_sntp_init(&config);

            if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK)
            {
                ESP_LOGW(TAG, "NTP connection failed");
            }
            else
            {
                ESP_LOGI(TAG, "NTP connected");
            }
            wifiConectedPrev = wifiConected;
        }
        else if (!wifiConected && wifiConectedPrev)
        {
            wifiConectedPrev = wifiConected;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void task_test_SSD1306i2c(void *ignore)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.bus.i2c.sda = PIN_SDA;
    u8g2_esp32_hal.bus.i2c.scl = PIN_SCL;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_t u8g2; // a structure which will contain all the data for one display
    // u8g2_Setup_ssd1306_i2c_128x32_univision_f(
    u8g2_Setup_ssd1306_i2c_72x40_er_1(
        &u8g2, U8G2_R0,
        // u8x8_byte_sw_i2c,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure
    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

    ESP_LOGI(TAG, "InitDisplay");
    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in
                             // sleep mode after this,

    u8g2_SetPowerSave(&u8g2, 0); // wake up display
    u8g2_ClearBuffer(&u8g2);

    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);

    u8g2_DrawStr(&u8g2, 2, 8, "Hi Simec!");
    u8g2_DrawStr(&u8g2, 2, 16, "Hi Simec!");
    u8g2_DrawStr(&u8g2, 20, 8, "Hi Simec!");
    u8g2_SendBuffer(&u8g2);

    vTaskDelete(NULL);
}

void logLevelSet(void *a)
{
    esp_log_level_set("*", ESP_LOG_NONE);
    esp_log_level_set(TAG, ESP_LOG_INFO);
}

void app_main(void)
{

    temp_q = xQueueCreate(2, sizeof(temperature_t));
    period_q = xQueueCreate(1, sizeof(temperature_t));

    logLevelSet(NULL);
    xTaskCreate(blink_led, "LED", 2048, NULL, 1, NULL);      // LED blinking task
    xTaskCreate(uart_receive, "UART", 4096, NULL, 10, NULL); // task for receiving the string from PC using USB
    xTaskCreate(Wifi_station, "WIFI", 4096, NULL, 9, NULL);

    xTaskCreate(temp_sensor, "TEMP", 4096, NULL, 1, NULL);

    xTaskCreate(NTP_time, "TIME", 4096, NULL, 3, NULL);
    xTaskCreate(task_test_SSD1306i2c, "OLED", 4096, NULL, 3, NULL);

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}