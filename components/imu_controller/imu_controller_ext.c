#include "imu_controller_ext.h"
#include "imu_controller_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define IMU_READ_CMD 1
#define IMU_WRITE_CMD 0

BaseType_t IMUControllerInitIMUExt(IMUConfig_t *imu) {
    esp_err_t err = ESP_OK;
    IMUExtConfig_t *extConfig = (IMUExtConfig_t *)imu->ext;
    err = spi_bus_add_device(extConfig->spiHost, extConfig->spiInterfaceConfig, &extConfig->spiHandle);
    if(err != ESP_OK) {
        return pdFALSE;
    }
    return pdTRUE;
};
/* TODO: Figure out what should be done with tx data fields (null TXBuf ptr, USE_TX_DATA flag, etc)*/
BaseType_t IMUControllerReadRegisterExt(IMUConfig_t *imu, uint8_t regAddr, uint8_t *regVal) {
    IMUExtConfig_t *extConfig = (IMUExtConfig_t *)imu->ext;
    spi_transaction_t transaction = {
        .cmd = IMU_READ_CMD,
        .addr = regAddr,
        .length = 8,
        .rxlength = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    esp_err_t err = spi_device_polling_transmit(extConfig->spiHandle, &transaction);
    //Delay appears to be necessary for write to work properly. Figure out why
    vTaskDelay(configTICK_RATE_HZ / 100);
    if(err != ESP_OK) {
        return pdFALSE;
    }
    *regVal = transaction.rx_data[0];
    return pdTRUE;
};
/* TODO: Figure out what should be done with rx data fields (null RXBuf ptr, USE_RX_DATA flag, etc)*/
BaseType_t IMUControllerWriteRegisterExt(IMUConfig_t *imu, uint8_t regAddr, uint8_t regVal) {
    IMUExtConfig_t *extConfig = (IMUExtConfig_t *)imu->ext;
    spi_transaction_ext_t transaction = {
        .base.cmd = IMU_WRITE_CMD,
        .base.addr = regAddr,
        .base.length = 8,
        .base.tx_data = {regVal},
        .base.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_VARIABLE_DUMMY,
        .dummy_bits = 0,
    };
    esp_err_t err = spi_device_polling_transmit(extConfig->spiHandle, &transaction);
    //Delay appears to be necessary for write to work properly. Figure out why
    vTaskDelay(configTICK_RATE_HZ / 100);
    if(err != ESP_OK) {
        return pdFALSE;
    }
    return pdTRUE;
};