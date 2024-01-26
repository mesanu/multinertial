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

#include "sdkconfig.h"

#define MAIN_RX_BUFFER_LEN 6

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define PIN_NUM_MISO     7
#define PIN_NUM_MOSI     8
#define PIN_NUM_CLK      6
#define PIN_NUM_CS       9
#define PIN_INT          10

#define PORT                        3333
#define KEEPALIVE_IDLE              5
#define KEEPALIVE_INTERVAL          5
#define KEEPALIVE_COUNT             3

#define ESP_INTR_FLAG_DEFAULT 0

typedef enum {
    MAIN_COMMAND_NONE,
    MAIN_COMMAND_START,
    MAIN_COMMAND_STOP,
    MAIN_COMMAND_ACK,
    MAIN_COMMAND_INVALID
} MainCommand_t;

static const char *TAG = "main";
static const char *START_COMMAND_STR = "start";
static const char *STOP_COMMAND_STR =  "stop-";
static const char *ACK_COMMAND_STR =   "ack--";

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
    int addrFamily = AF_INET;
    int ipProtocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    int sockErr = 0;
    struct sockaddr_storage destAddr;

    struct sockaddr_in *destAddrIp4 = (struct sockaddr_in *)&destAddr;
    destAddrIp4->sin_addr.s_addr = htonl(INADDR_ANY);
    destAddrIp4->sin_family = AF_INET;
    destAddrIp4->sin_port = htons(PORT);
    ipProtocol = IPPROTO_IP;

    int listenSock = socket(addrFamily, SOCK_STREAM, ipProtocol);
    if (listenSock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    ESP_LOGI(TAG, "Socket created");

    sockErr = bind(listenSock, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (sockErr != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addrFamily);
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

    setsockopt(*sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
    setsockopt(*sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
    setsockopt(*sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
    setsockopt(*sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
}

static MainCommand_t getCommand(int sock) {
    MainCommand_t command = MAIN_COMMAND_NONE;
    if(sock > -1){
        recv(sock, sockRxCommandBuffer, MAIN_RX_BUFFER_LEN - 1, 0);
        if(strcmp(sockRxCommandBuffer, START_COMMAND_STR) == 0) {
            command = MAIN_COMMAND_START;
        } else if(strcmp(sockRxCommandBuffer, STOP_COMMAND_STR) == 0) {
            command = MAIN_COMMAND_STOP;
        } else if(strcmp(sockRxCommandBuffer, ACK_COMMAND_STR) == 0) {
            command = MAIN_COMMAND_ACK;
        } else {
            ESP_LOGE(TAG, "Recieved invalid command %s", sockRxCommandBuffer);
            command = MAIN_COMMAND_INVALID;
        }
    } else {
        ESP_LOGE(TAG, "Invalid socket");
    }
    sockRxCommandBuffer[0] = 0;
    return command;
}

void app_main(void)
{
    uint8_t imuIndex;
    IMUFIFOData_t *data;
    int sock = -1;
    BaseType_t sampling = pdFALSE;
    MainCommand_t command = MAIN_COMMAND_NONE;

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    BaseType_t wifiConnected = wifiInit();

    spi_bus_initialize(SPI2_HOST, &busConfig, SPI_DMA_CH_AUTO);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    IMUControllerConfigSetSPI(0, SPI2_HOST, PIN_NUM_CS, PIN_INT);
    IMUControllerInit();
    IMUControllerSetConfigAccelRange(0, BMI2_ACC_RANGE_2G);
    IMUControllerSetConfigAccelODR(0, BMI2_ACC_ODR_400HZ);
    IMUControllerSetConfigAccelFilterBWP(0, BMI2_ACC_NORMAL_AVG4);
    IMUControllerSetConfigAccelFilterPerf(0, BMI2_PERF_OPT_MODE);
    IMUControllerSetConfigGyroRange(0, BMI2_GYR_RANGE_500);
    IMUControllerSetConfigGyroODR(0, BMI2_GYR_ODR_400HZ);
    IMUControllerSetConfigGyroFilterBWP(0, BMI2_GYR_NORMAL_MODE);
    IMUControllerSetConfigGyroFilterPerf(0, BMI2_PERF_OPT_MODE);
    IMUControllerSetConfigGyroNoisePerf(0, BMI2_PERF_OPT_MODE);
    IMUControllerUpdateIMUSettings(0);
    xTaskCreate(IMUControllerContinuousSamplingTask, "IMU FIFO task", 8192, NULL, 10, NULL);

    if(wifiConnected){
        socketCreate(&sock);
    }

    while(1) {
        command = getCommand(sock);
        if(sampling == pdFALSE) {
            if(command == MAIN_COMMAND_START) {
                IMUControllerStartContinuousSampling();
                sampling = pdTRUE;
                IMUControllerWaitOnData(&imuIndex);
                data = IMUControllerGetFIFODataPtr(imuIndex);
                send(sock, data, sizeof(IMUFIFOData_t), 0);
            } else {
                vTaskDelay(1);
            }
        } else if (sampling == pdTRUE) {
            if(command == MAIN_COMMAND_STOP) {
                IMUControllerStopContinuousSampling();
                sampling = pdFALSE;
            } else {
                IMUControllerWaitOnData(&imuIndex);
                data = IMUControllerGetFIFODataPtr(imuIndex);
                send(sock, data, sizeof(IMUFIFOData_t), 0);
            }
        }
    }
}