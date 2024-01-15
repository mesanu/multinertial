#include <string.h>
#include "imu_controller_api.h"
#include "bmi270.h"
#include "sdkconfig.h"
#include "esp_log.h"

static IMUConfig_t internalConfigs[CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS];

static uint32_t registeredIMUs = 0;
static const char *TAG = "IMU";

static float lsb_to_g(int16_t val, float g_range, uint8_t bit_width);

BaseType_t IMUControllerInit(void) {
    int i;
    int8_t rslt;
    for (i = 0; i < registeredIMUs; i++) {
        if (internalConfigs[i].registered == pdTRUE) {
            bmi2_interface_init(&internalConfigs[i].dev, BMI2_SPI_INTF);
            rslt = bmi270_init(&internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                ESP_LOGE(TAG, "Failed to initialize IMU %d", i);
                return pdFALSE;
            }
            rslt = bmi2_get_sensor_config(&internalConfigs[i].accelConfig, 1, &internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                ESP_LOGE(TAG, "Failed to initialize IMU %d", i);
                return pdFALSE;
            }
        }
    }
    ESP_LOGI(TAG, "All IMUs initialized successfully");
    return pdTRUE;
}

BaseType_t IMUControllerAddDevice(IMUConfig_t *config, uint8_t index) {
    if (!(registeredIMUs < CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS)) {
        return pdFALSE;
    }
    memcpy(&internalConfigs[index], config, sizeof(IMUConfig_t));
    internalConfigs[index].registered = pdTRUE;
    internalConfigs[index].accelConfig.type = BMI2_ACCEL;
    internalConfigs[index].dev.intf_ptr = &(internalConfigs[registeredIMUs].interfaceConfig);

    registeredIMUs++;

    return pdTRUE;
}

BaseType_t IMUSetConfigAccelRange(uint8_t index, uint8_t range) {
    if (internalConfigs[index].registered != pdFALSE) {
        internalConfigs[index].accelConfig.cfg.acc.range = range;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUSetConfigAccelODR(uint8_t index, uint8_t odr) {
    if (internalConfigs[index].registered != pdFALSE) {
        internalConfigs[index].accelConfig.cfg.acc.odr = odr;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUSetConfigFilterBWP(uint8_t index, uint8_t bwp) {
    if (internalConfigs[index].registered != pdFALSE) {
        internalConfigs[index].accelConfig.cfg.acc.bwp = bwp;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUSetConfigFilterPerf(uint8_t index, uint8_t filterPerf) {
    if (internalConfigs[index].registered != pdFALSE) {
        internalConfigs[index].accelConfig.cfg.acc.bwp = filterPerf;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUSetConfigIntPin(uint8_t index, uint8_t pinFunc, uint8_t pin) {
    int8_t rslt = BMI2_OK;
    if (internalConfigs[index].registered != pdFALSE) {
        rslt = bmi2_map_data_int(pinFunc, pin, &internalConfigs[index].dev);
        if (rslt != BMI2_OK) {
            bmi2_error_codes_print_result(rslt);
            ESP_LOGE(TAG, "Failed to map pin function %d", index);
            return pdFALSE;
        }
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUUpdateAccelSettings(uint8_t index) {
    int8_t rslt = BMI2_OK;
    if (internalConfigs[index].registered != pdFALSE) {
        rslt = bmi2_set_sensor_config(&internalConfigs[index].accelConfig, 1, &internalConfigs[index].dev);
        if (rslt != BMI2_OK) {
            bmi2_error_codes_print_result(rslt);
            ESP_LOGE(TAG, "Failed to update accel settings %d", index);
            return pdFALSE;
        }
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUEnableAccel(uint8_t index) {
    uint8_t sensor_list = BMI2_ACCEL;
    int8_t rslt = BMI2_OK;
    if (internalConfigs[index].registered != pdFALSE) {
        rslt = bmi2_sensor_enable(&sensor_list, 1, &internalConfigs[index].dev);
        if (rslt != BMI2_OK) {
            bmi2_error_codes_print_result(rslt);
            ESP_LOGE(TAG, "Failed to enable accel %d", index);
            return pdFALSE;
        }
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUGetAccelData(uint8_t index, IMUAccelData_t *accelData) {
    int8_t rslt;
    struct bmi2_sens_data sens_data = { { 0 } };
    if (internalConfigs[index].registered == pdTRUE) {
        rslt = bmi2_get_sensor_data(&sens_data, &internalConfigs[index].dev);
        bmi2_error_codes_print_result(rslt);
        if ((rslt == BMI2_OK) && (sens_data.status & BMI2_DRDY_ACC)) {
            /* Converting lsb to Gs for 16 bit accelerometer at 2G range. */
            accelData->x = lsb_to_g(sens_data.acc.x, (float)2, internalConfigs[index].dev.resolution);
            accelData->y = lsb_to_g(sens_data.acc.y, (float)2, internalConfigs[index].dev.resolution);
            accelData->z = lsb_to_g(sens_data.acc.z, (float)2, internalConfigs[index].dev.resolution);
        }
        else {
            ESP_LOGE(TAG, "Couldn't get accel data %d", index);
        }
    }
    return pdTRUE;

}

static float lsb_to_g(int16_t val, float g_range, uint8_t bit_width) {
    return (val * g_range) / (float)(1 << (bit_width-1));
}