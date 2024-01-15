#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_controller_api.h"
#include "network_controller_api.h"
#include "driver/spi_master.h"
#include "bmi270.h"

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
    .queue_size = 6,
};

static IMUConfig_t IMUConfig = {
    .interfaceConfig.spiHost = SPI2_HOST,
    .interfaceConfig.spiInterfaceConfig = &SPIInterfaceConfig,
    .interfaceConfig.spiHandle = NULL,
};

void app_main(void)
{
    spi_bus_initialize(SPI2_HOST, &busConfig, SPI_DMA_CH_AUTO);
    IMUControllerAddDevice(&IMUConfig);
    IMUControllerInit();
}