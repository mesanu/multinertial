#include <string.h>
#include "imu_controller_api.h"
#include "bmi270.h"
#include "sdkconfig.h"
#include "esp_log.h"

static IMUConfig_t internalConfigs[CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS];

static uint32_t registeredIMUs = 0;
static const char *TAG = "IMU";

BaseType_t IMUControllerInit(void) {
    int i;
    int8_t rslt;
    for(i = 0; i < registeredIMUs; i++) {
        bmi2_interface_init(&(internalConfigs[i].dev), BMI2_SPI_INTF);
        rslt = bmi270_init(&internalConfigs[i].dev);
        bmi2_error_codes_print_result(rslt);
        if(rslt != BMI2_OK){
            ESP_LOGE(TAG, "Failed to initialize IMU %d", i);
            return pdFALSE;
        }
    }
    ESP_LOGI(TAG, "All IMUs initialized successfully");
    return pdTRUE;
};

BaseType_t IMUControllerAddDevice(IMUConfig_t *config) {
    if(!(registeredIMUs < CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS)) {
        return pdFALSE;
    }
    memcpy(&internalConfigs[registeredIMUs], config, sizeof(IMUConfig_t));
    internalConfigs[registeredIMUs].dev.intf_ptr = &(internalConfigs[registeredIMUs].interfaceConfig);
    
    registeredIMUs++;

    return pdTRUE;
}
