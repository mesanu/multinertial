#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "imu_controller_api.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "bmi270.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "sdkconfig.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define PIN_NUM_MISO     7
#define PIN_NUM_MOSI     8
#define PIN_NUM_CLK      6
#define PIN_NUM_CS       9
#define PIN_INT          10

#define ESP_INTR_FLAG_DEFAULT 0

static char *TAG = "main";
static uint8_t wifiRetryNum = 0;
static EventGroupHandle_t wifiConnectEventGroup;

static wifi_config_t wifiConfig = {
    .sta = {
        .ssid = CONFIG_NETWORK_CONTROLLER_WIFI_SSID,
        .password = CONFIG_NETWORK_CONTROLLER_WIFI_PASSWORD,
        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        .sae_h2e_identifier = "",
    },
};

static spi_bus_config_t busConfig = {
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};

static void wifiConnectEventHandler(void* arg, esp_event_base_t eventBase,
                                int32_t eventId, void* eventData) {
    if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifiRetryNum < CONFIG_NETWORK_CONTROLLER_MAXIMUM_RETRY) {
            esp_wifi_connect();
            wifiRetryNum++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(wifiConnectEventGroup, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) eventData;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        wifiRetryNum = 0;
        xEventGroupSetBits(wifiConnectEventGroup, WIFI_CONNECTED_BIT);
    }
}

static void wifiInit(void) {
    wifiConnectEventGroup = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instanceAnyId;
    esp_event_handler_instance_t instanceGotIp;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifiConnectEventHandler,
                                                        NULL,
                                                        &instanceAnyId));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifiConnectEventHandler,
                                                        NULL,
                                                        &instanceGotIp));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifiConfig) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    EventBits_t bits = xEventGroupWaitBits(wifiConnectEventGroup,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 CONFIG_NETWORK_CONTROLLER_WIFI_SSID, CONFIG_NETWORK_CONTROLLER_WIFI_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 CONFIG_NETWORK_CONTROLLER_WIFI_SSID, CONFIG_NETWORK_CONTROLLER_WIFI_PASSWORD);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void app_main(void)
{
    uint8_t imuIndex;
    IMUFIFOData_t *data;

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifiInit();

    spi_bus_initialize(SPI2_HOST, &busConfig, SPI_DMA_CH_AUTO);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    IMUControllerConfigSetSPI(0, SPI2_HOST, PIN_NUM_CS, PIN_INT);
    IMUControllerInit();
    IMUControllerSetConfigAccelRange(0, BMI2_ACC_RANGE_2G);
    IMUControllerSetConfigAccelODR(0, BMI2_ACC_ODR_50HZ);
    IMUControllerSetConfigAccelFilterBWP(0, BMI2_ACC_NORMAL_AVG4);
    IMUControllerSetConfigAccelFilterPerf(0, BMI2_PERF_OPT_MODE);
    IMUControllerSetConfigGyroRange(0, BMI2_GYR_RANGE_500);
    IMUControllerSetConfigGyroODR(0, BMI2_GYR_ODR_50HZ);
    IMUControllerSetConfigGyroFilterBWP(0, BMI2_GYR_NORMAL_MODE);
    IMUControllerSetConfigGyroFilterPerf(0, BMI2_PERF_OPT_MODE);
    IMUControllerUpdateIMUSettings(0);
    xTaskCreate(IMUControllerContinuousSamplingTask, "IMU FIFO task", 8192, NULL, 10, NULL);
    IMUControllerStartContinuousSampling();
    while(1) {
        IMUControllerWaitOnData(&imuIndex);
        data = IMUControllerGetFIFODataPtr(imuIndex);
        ESP_LOGI(TAG, "headTs: %lld, tailTs: %lld", data->headUsTimestamp, data->tailUsTimestamp);
        ESP_LOGI(TAG, "First frame Accel x: %d, y:%d, z:%d", data->accelX[0], data->accelY[0], data->accelZ[0]);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }



}