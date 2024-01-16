#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_controller_api.h"
#include "network_controller_api.h"
#include "driver/spi_master.h"
#include "bmi270.h"
#include "esp_log.h"

#define PIN_NUM_MISO     7
#define PIN_NUM_MOSI     8
#define PIN_NUM_CLK      6
#define PIN_NUM_CS       9

static spi_bus_config_t busConfig = {
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 8,
};

static spi_device_interface_config_t SPIInterfaceConfig = {
    .address_bits = 8,
    .clock_speed_hz = CONFIG_IMU_CONTROLLER_SPI_BUS_FREQ,
    .mode = 0,
    .spics_io_num = PIN_NUM_CS,
    .queue_size = 1,
};

void app_main(void)
{
    IMUData_t data;
    spi_bus_initialize(SPI2_HOST, &busConfig, SPI_DMA_CH_AUTO);
    IMUControllerConfigSetSPI(0, SPI2_HOST, PIN_NUM_CS);
    IMUControllerInit();
    IMUSetConfigAccelRange(0, BMI2_ACC_RANGE_2G);
    IMUSetConfigAccelODR(0, BMI2_ACC_ODR_200HZ);
    IMUSetConfigAccelFilterBWP(0, BMI2_ACC_NORMAL_AVG4);
    IMUSetConfigAccelFilterPerf(0, BMI2_PERF_OPT_MODE);
    IMUSetConfigGyroRange(0, BMI2_GYR_RANGE_500);
    IMUSetConfigGyroODR(0, BMI2_GYR_ODR_200HZ);
    IMUSetConfigGyroFilterBWP(0, BMI2_GYR_NORMAL_MODE);
    IMUSetConfigGyroFilterPerf(0, BMI2_PERF_OPT_MODE);
    IMUUpdateIMUSettings(0);
    IMUSetConfigIntPin(0, BMI2_DRDY_INT, BMI2_INT1);
    IMUEnableIMU(0);
    while(1) {
        IMUGetData(0, &data);
        ESP_LOGI("MAIN", "Accel: %f %f %f Gyro: %f %f %f", data.accelX, data.accelY, data.accelZ, data.gyroX, data.gyroY, data.gyroZ);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }



}