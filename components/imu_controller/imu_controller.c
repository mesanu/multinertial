#include <string.h>
#include "imu_controller_api.h"
#include "imu_controller_ext.h"
#include "sdkconfig.h"
#include "esp_log.h"

static IMUConfig_t internalConfigs[CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS];

static uint32_t registeredIMUs = 0;
static const char *TAG = "IMU";

BaseType_t IMUControllerInit(void) {
    int i;
    uint8_t regVal;
    for(i = 0; i < registeredIMUs; i++) {
        BaseType_t err;
        IMUControllerInitIMUExt(&internalConfigs[i]);
        /* First dummy read to kick BMI270 in to SPI mode */
        err = IMUControllerReadRegisterExt(&internalConfigs[i], 0x00, &regVal);
        ESP_LOGI(TAG, );
        /* Chip ID read */
        err = IMUControllerReadRegisterExt(&internalConfigs[i], 0x00, &regVal);
        ESP_LOGI(TAG, "Chip ID read of 0x%x, err", regVal);
        err = IMUControllerWriteRegisterExt(&internalConfigs[i], 0x41, 0x01);
        err = IMUControllerReadRegisterExt(&internalConfigs[i], 0x41, &regVal);
        ESP_LOGI(TAG, "ACC range status register 0x%x", regVal);
        IMUControllerWriteRegisterExt(&internalConfigs[i], 0x41, 0x03);
        err = IMUControllerReadRegisterExt(&internalConfigs[i], 0x41, &regVal);
        ESP_LOGI(TAG, "ACC range status register 0x%x", regVal);
    }
    return pdTRUE;
};

BaseType_t IMUControllerAddDevice(IMUConfig_t *config) {
    if(!(registeredIMUs < CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS)) {
        return pdFALSE;
    }
    memcpy(&internalConfigs[registeredIMUs], config, sizeof(IMUConfig_t));
    registeredIMUs++;
    return pdTRUE;
}
