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

// MAM - PROJECT

#if !CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#error "Console must be USB Serial/JTAG. Set: Component config -> ESP System Settings -> Channel for console output = USB Serial/JTAG."
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

// void configure_led(void)
// {
//     ESP_LOGI(TAG, "Example configured to blink addressable LED!");
//     /* LED strip initialization with the GPIO and pixels number*/
//     led_strip_config_t strip_config = {
//         .strip_gpio_num = BLINK_GPIO,
//         .max_leds = 1, // at least one LED on board
//     };
//     led_strip_rmt_config_t rmt_config = {
//         .resolution_hz = 10 * 1000 * 1000, // 10MHz
//     };
//     ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
//     /* Set all LED off to clear all pixels */
//     led_strip_clear(led_strip);
// }

static void configure_led(void)
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
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        wifiConected = true;
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
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

    ESP_LOGI(TAG, "wifi_init_sta finished.");

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
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

double temp_sensor(void *params)
{
    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

    float tsens_value;

    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));

    return tsens_value;
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
                printf("Temperature value: %.02f ℃\r\n", temp_sensor(NULL));
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void blink_led(void *pvParameters)
{
    /* Configure the peripheral according to the LED type */
    configure_led();
    while (true)
    {
        gpio_set_level(BLINK_GPIO, s_led_state);
        s_led_state = !s_led_state;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void NTP_time(void *args)
{

    static bool wifiConectedPrev = false;
    while (true)
    {
        if (wifiConected && !wifiConectedPrev)
        {
            ESP_LOGI("NTP", "NTP connecting");
            esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
            esp_netif_sntp_init(&config);

            if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK)
            {
                ESP_LOGE("NTP", "NTP connection failed");
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

void app_main(void)
{
    xTaskCreate(blink_led, "LED", 2048, NULL, 1, NULL);      // LED blinking task
    xTaskCreate(uart_receive, "UART", 4096, NULL, 10, NULL); // task for receiving the string from PC using USB
    wifiInit();
    xTaskCreate(NTP_time, "TIME", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Temperature value %.02f ℃", temp_sensor(NULL));

    while (1)
    {

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}