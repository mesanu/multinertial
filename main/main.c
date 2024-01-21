#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "imu_controller_api.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "bmi270.h"

#include "esp_log.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define PIN_NUM_MISO     7
#define PIN_NUM_MOSI     8
#define PIN_NUM_CLK      6
#define PIN_NUM_CS       9
#define PIN_INT          10

#define ESP_INTR_FLAG_DEFAULT 0

static char *TAG = "main";

static spi_bus_config_t busConfig = {
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};

void app_main(void)
{
    uint8_t imuIndex;
    IMUFIFOData_t *data;
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