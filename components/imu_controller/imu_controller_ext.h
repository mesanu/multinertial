#ifndef IMU_CONTROLLER_EXT_H
#define IMU_CONTROLLER_EXT_H

#include "imu_controller_api.h"
#include "freertos/FreeRTOS.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

typedef struct {
    spi_host_device_t spiHost;
    spi_device_interface_config_t *spiInterfaceConfig;
    spi_device_handle_t spiHandle;
    gpio_num_t interruptPin;
} IMUExtConfig_t;

BaseType_t IMUControllerInitIMUExt(IMUConfig_t *imu);
/* Type for rx_buffer is uint32_t (despite the fact that only 8 bytes are being transferred)
 * because according to the "SPI Master Driver" document for the ESP32 the data is always placed
 * in the rx buffer in 4 byte chunks if DMA is used. Assume similar constraints for other MCUs*/
BaseType_t IMUControllerReadRegisterExt(IMUConfig_t *imu, uint8_t regAddr, uint8_t *regVal);
BaseType_t IMUControllerWriteRegisterExt(IMUConfig_t *imu, uint8_t regAddr, uint8_t regVal);


#endif /* IMU_CONTROLLER_EXT_H */