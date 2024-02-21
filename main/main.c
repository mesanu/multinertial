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

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "led_strip.h"

#include "sdkconfig.h"

#define MAIN_RX_BUFFER_LEN 6

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define PIN_NUM_MISO     5
#define PIN_NUM_MOSI     6
#define PIN_NUM_CLK      4

#define PIN_NUM_CS_0     7
#define PIN_NUM_CS_1     9
#define PIN_NUM_CS_2     0

#define PIN_INT_0        10
#define PIN_INT_1        3
#define PIN_INT_2        1

#define PIN_CAL_BUTTON 2

#define PORT                        3333
#define KEEPALIVE_IDLE              2
#define KEEPALIVE_INTERVAL          1
#define KEEPALIVE_COUNT             3

#define ESP_INTR_FLAG_DEFAULT 0

typedef enum {
    MAIN_COMMAND_NONE,
    MAIN_COMMAND_START,
    MAIN_COMMAND_STOP,
    MAIN_COMMAND_ACK,
    MAIN_COMMAND_CAL_TOGGLE,
    MAIN_COMMAND_INVALID
} MainCommand_t;

static const char *TAG = "main";
static const char *START_COMMAND_STR =    "start";
static const char *STOP_COMMAND_STR =     "stop";
static const char *ACK_COMMAND_STR =      "ack";
static const char *CAL_MODE_COMMAND_STR = "cal";

static uint8_t wifiRetryNum = 0;
static EventGroupHandle_t wifiConnectEventGroup;

static char sockRxCommandBuffer[MAIN_RX_BUFFER_LEN] = {0};


static wifi_config_t wifiConfig = {
    .sta = {
        .ssid = CONFIG_NETWORK_CONTROLLER_WIFI_SSID,
        .password = CONFIG_NETWORK_CONTROLLER_WIFI_PASSWORD,
        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        .sae_h2e_identifier = "",
    },
};

static const spi_bus_config_t busConfig = {
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};

static const gpio_config_t calButtonPinConf = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = (1ULL << PIN_CAL_BUTTON),
    .pull_down_en = 0,
    .pull_up_en = 1,
};

static const led_strip_config_t stripConfig = {
    .strip_gpio_num = 8,
    .max_leds = 1, // at least one LED on board
};

static const led_strip_rmt_config_t rmtConfig = {
    .resolution_hz = 10000000, // 10MHz
};

static led_strip_handle_t ledStrip;

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

static BaseType_t wifiInit(void) {
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
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
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
        return pdTRUE;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 CONFIG_NETWORK_CONTROLLER_WIFI_SSID, CONFIG_NETWORK_CONTROLLER_WIFI_PASSWORD);
        return pdFALSE;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        return pdFALSE;
    }
}

static void socketCreate(int *sock) {
    int opt;

    int sockErr = 0;
    struct sockaddr_storage destAddr;

    struct sockaddr_in *destAddrIp4 = (struct sockaddr_in *)&destAddr;
    destAddrIp4->sin_addr.s_addr = htonl(INADDR_ANY);
    destAddrIp4->sin_family = AF_INET;
    destAddrIp4->sin_port = htons(PORT);

    int listenSock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listenSock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    opt = 1;
    setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    ESP_LOGI(TAG, "Socket created");

    sockErr = bind(listenSock, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (sockErr != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    sockErr = listen(listenSock, 1);
    if (sockErr != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
    }

    struct sockaddr_storage sourceAddr; // Large enough for both IPv4 or IPv6
    socklen_t addrLen = sizeof(sourceAddr);
    *sock = accept(listenSock, (struct sockaddr *)&sourceAddr, &addrLen);
    if (*sock < 0) {
        ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
    } else {
        ESP_LOGI(TAG, "Accepted connection");
    }

    opt = 1;
    setsockopt(*sock, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(int));
    opt = KEEPALIVE_IDLE;
    setsockopt(*sock, IPPROTO_TCP, TCP_KEEPIDLE, &opt, sizeof(int));
    opt = KEEPALIVE_INTERVAL;
    setsockopt(*sock, IPPROTO_TCP, TCP_KEEPINTVL, &opt, sizeof(int));
    opt = KEEPALIVE_COUNT;
    setsockopt(*sock, IPPROTO_TCP, TCP_KEEPCNT, &opt, sizeof(int));
    opt = 1;
    setsockopt(*sock, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(int));
}

static MainCommand_t getCommand(int sock) {
    MainCommand_t command = MAIN_COMMAND_NONE;
    if(sock > -1){
        recv(sock, sockRxCommandBuffer, MAIN_RX_BUFFER_LEN - 1, 0);
        if(sockRxCommandBuffer[0] == 0) {
            command = MAIN_COMMAND_NONE;
        } else if(strcmp(sockRxCommandBuffer, START_COMMAND_STR) == 0) {
            command = MAIN_COMMAND_START;
        } else if(strcmp(sockRxCommandBuffer, STOP_COMMAND_STR) == 0) {
            command = MAIN_COMMAND_STOP;
        } else if(strcmp(sockRxCommandBuffer, ACK_COMMAND_STR) == 0) {
            command = MAIN_COMMAND_ACK;
        } else if(strcmp(sockRxCommandBuffer, CAL_MODE_COMMAND_STR) == 0) {
            command = MAIN_COMMAND_CAL_TOGGLE;
        } else {
            ESP_LOGE(TAG, "Recieved invalid command %s", sockRxCommandBuffer);
            command = MAIN_COMMAND_INVALID;
        }
        sockRxCommandBuffer[0] = 0;
        return command;
    } else {
        ESP_LOGE(TAG, "Invalid socket");
        return MAIN_COMMAND_INVALID;
    }
}

void app_main(void)
{
    IMUFIFOData_t *data;
    int sock = -1;
    BaseType_t sampling = pdFALSE;
    BaseType_t calMode = pdFALSE;
    MainCommand_t command = MAIN_COMMAND_NONE;

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //Set the cal button pin config
    gpio_config(&calButtonPinConf);

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&stripConfig, &rmtConfig, &ledStrip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(ledStrip);

    BaseType_t wifiConnected = wifiInit();

    spi_bus_initialize(SPI2_HOST, &busConfig, SPI_DMA_CH_AUTO);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    IMUControllerConfigSetSPI(0, SPI2_HOST, PIN_NUM_CS_0, PIN_INT_0);
    IMUControllerConfigSetSPI(1, SPI2_HOST, PIN_NUM_CS_2, PIN_INT_2);
    IMUControllerInit();
    IMUControllerSetConfigAccelRange(BMI2_ACC_RANGE_2G);
    IMUControllerSetConfigAccelFilterBWP(BMI2_ACC_NORMAL_AVG4);
    IMUControllerSetConfigAccelFilterPerf(BMI2_PERF_OPT_MODE);
    IMUControllerSetConfigAccelODR(BMI2_ACC_ODR_400HZ);
    IMUControllerSetConfigGyroRange(BMI2_GYR_RANGE_500);
    IMUControllerSetConfigGyroFilterBWP(BMI2_GYR_NORMAL_MODE);
    IMUControllerSetConfigGyroFilterPerf(BMI2_PERF_OPT_MODE);
    IMUControllerSetConfigGyroNoisePerf(BMI2_PERF_OPT_MODE);
    IMUControllerSetConfigGyroODR(BMI2_GYR_ODR_400HZ);
    IMUControllerUpdateIMUSettings(0);
    IMUControllerUpdateIMUSettings(1);
    IMUControllerConfigContinuousSampling();
    xTaskCreate(IMUControllerContinuousSamplingTask, "IMU FIFO task", 8192, NULL, 10, NULL);

    if(wifiConnected){
        socketCreate(&sock);
    }

    while(1) {
        command = getCommand(sock);
        if(sampling == pdFALSE) {
            if(command == MAIN_COMMAND_START) {
                if(calMode == pdTRUE) {
                    while(gpio_get_level(PIN_CAL_BUTTON)) {
                        vTaskDelay(1);
                    }
                    //Set LED to blue
                    led_strip_set_pixel(ledStrip, 0, 0, 0, 8);
                    led_strip_refresh(ledStrip);

                    vTaskDelay(2000/portTICK_PERIOD_MS);

                    //Set LED to green
                    led_strip_set_pixel(ledStrip, 0, 0, 8, 0);
                    led_strip_refresh(ledStrip);
                }
                IMUControllerStartContinuousSampling();
                sampling = pdTRUE;
            } else if (command == MAIN_COMMAND_CAL_TOGGLE) {
                if(calMode == pdTRUE) {
                    calMode = pdFALSE;
                    led_strip_clear(ledStrip);
                } else {
                    calMode = pdTRUE;
                    //Set LED to red
                    led_strip_set_pixel(ledStrip, 0, 8, 0, 0);
                    led_strip_refresh(ledStrip);
                }
            } else {
                vTaskDelay(1);
            }
        }
        if (sampling == pdTRUE) {
            if(command == MAIN_COMMAND_STOP) {
                IMUControllerStopContinuousSampling();
                sampling = pdFALSE;
                if(calMode == pdTRUE) {
                    //Set LED to red
                    led_strip_set_pixel(ledStrip, 0, 8, 0, 0);
                    led_strip_refresh(ledStrip);
                }
            } else {
                if(IMUControllerGetFIFODataPtr(&data)){
                    ESP_LOGI(TAG, "Sent %d", data->devIndex);
                    send(sock, data, sizeof(IMUFIFOData_t), 0);
                    vPortFree(data);
                }
            }
        }
    }
}