#include <string.h>
#include "imu_controller_api.h"
#include "bmi270.h"
#include "sdkconfig.h"
#include "esp_log.h"

static IMUConfig_t internalConfigs[CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS];

static uint32_t configuredIMUs = 0;
static const char *TAG = "IMU";

static float lsb_to_meas(int16_t val, float g_range, uint8_t bit_width);

BaseType_t IMUControllerInit(void) {
    int i;
    int8_t rslt;
    for (i = 0; i < CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS; i++) {
        if (internalConfigs[i].registered  != pdFALSE) {
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
                ESP_LOGE(TAG, "Failed to get accel configs IMU %d", i);
                return pdFALSE;
            }
            rslt = bmi2_get_sensor_config(&internalConfigs[i].gyroConfig, 1, &internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                ESP_LOGE(TAG, "Failed to get gyro configs IMU %d", i);
                return pdFALSE;
            }
        }
    }
    ESP_LOGI(TAG, "All IMUs initialized successfully");
    return pdTRUE;
}

BaseType_t IMUControllerConfigSetSPI(uint8_t index, spi_host_device_t spiHost, int csPin) {
    if (internalConfigs[index].registered  != pdFALSE) {
        return pdFALSE;
    }
    internalConfigs[index].interfaceConfig.spiHost = spiHost;

    internalConfigs[index].interfaceConfig.spiInterfaceConfig.address_bits = 8;
    internalConfigs[index].interfaceConfig.spiInterfaceConfig.clock_speed_hz = CONFIG_IMU_CONTROLLER_SPI_BUS_FREQ;
    internalConfigs[index].interfaceConfig.spiInterfaceConfig.mode = 0;
    internalConfigs[index].interfaceConfig.spiInterfaceConfig.spics_io_num = csPin;
    internalConfigs[index].interfaceConfig.spiInterfaceConfig.queue_size = 1;

    internalConfigs[index].accelConfig.type = BMI2_ACCEL;
    internalConfigs[index].gyroConfig.type = BMI2_GYRO;
    internalConfigs[index].dev.intf_ptr = &internalConfigs[index].interfaceConfig;

    internalConfigs[index].registered = pdTRUE;

    configuredIMUs++;

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

BaseType_t IMUSetConfigAccelFilterBWP(uint8_t index, uint8_t bwp) {
    if (internalConfigs[index].registered != pdFALSE) {
        internalConfigs[index].accelConfig.cfg.acc.bwp = bwp;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUSetConfigAccelFilterPerf(uint8_t index, uint8_t filterPerf) {
    if (internalConfigs[index].registered != pdFALSE) {
        internalConfigs[index].accelConfig.cfg.acc.bwp = filterPerf;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUSetConfigGyroRange(uint8_t index, uint8_t range) {
    if (internalConfigs[index].registered != pdFALSE) {
        internalConfigs[index].gyroConfig.cfg.gyr.range = range;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUSetConfigGyroODR(uint8_t index, uint8_t odr) {
    if (internalConfigs[index].registered != pdFALSE) {
        internalConfigs[index].gyroConfig.cfg.gyr.odr = odr;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUSetConfigGyroFilterBWP(uint8_t index, uint8_t bwp) {
    if (internalConfigs[index].registered != pdFALSE) {
        internalConfigs[index].gyroConfig.cfg.gyr.bwp = bwp;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUSetConfigGyroFilterPerf(uint8_t index, uint8_t filterPerf) {
    if (internalConfigs[index].registered != pdFALSE) {
        internalConfigs[index].gyroConfig.cfg.gyr.bwp = filterPerf;
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

BaseType_t IMUUpdateIMUSettings(uint8_t index) {
    int8_t rslt = BMI2_OK;
    if (internalConfigs[index].registered != pdFALSE) {
        rslt = bmi2_set_sensor_config(&internalConfigs[index].accelConfig, 1, &internalConfigs[index].dev);
        if (rslt != BMI2_OK) {
            bmi2_error_codes_print_result(rslt);
            ESP_LOGE(TAG, "Failed to update accel settings %d", index);
            return pdFALSE;
        }
        rslt = bmi2_set_sensor_config(&internalConfigs[index].gyroConfig, 1, &internalConfigs[index].dev);
        if (rslt != BMI2_OK) {
            bmi2_error_codes_print_result(rslt);
            ESP_LOGE(TAG, "Failed to update gyro settings %d", index);
            return pdFALSE;
        }
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUEnableIMU(uint8_t index) {
    uint8_t sensor_list[2] = {BMI2_ACCEL, BMI2_GYRO};
    int8_t rslt = BMI2_OK;
    if (internalConfigs[index].registered != pdFALSE) {
        rslt = bmi2_sensor_enable(&sensor_list, 2, &internalConfigs[index].dev);
        if (rslt != BMI2_OK) {
            bmi2_error_codes_print_result(rslt);
            ESP_LOGE(TAG, "Failed to enable IMU %d", index);
            return pdFALSE;
        }
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUGetData(uint8_t index, IMUData_t *data) {
    int8_t rslt;
    struct bmi2_sens_data sens_data = { { 0 } };

    /* This works because all the accel ranges are powers of 2 */
    float accelGRange = (2 << internalConfigs[index].accelConfig.cfg.acc.range);

    /* This works because all the gyro ranges can be calculated by deviding 2000 by powers of 2 */
    float gyroDPSRange = (2000/(1 << internalConfigs[index].gyroConfig.cfg.gyr.range));

    if (internalConfigs[index].registered == pdTRUE) {
        rslt = bmi2_get_sensor_data(&sens_data, &internalConfigs[index].dev);
        bmi2_error_codes_print_result(rslt);

        if ((rslt == BMI2_OK) && (sens_data.status & BMI2_DRDY_ACC) && (sens_data.status & BMI2_DRDY_ACC)) {
            /* Converting lsb to Gs for 16 bit accelerometer at 2G range. */
            data->accelX = lsb_to_meas(sens_data.acc.x, accelGRange, internalConfigs[index].dev.resolution);
            data->accelY = lsb_to_meas(sens_data.acc.y, accelGRange, internalConfigs[index].dev.resolution);
            data->accelZ = lsb_to_meas(sens_data.acc.z, accelGRange, internalConfigs[index].dev.resolution);
            data->gyroX = lsb_to_meas(sens_data.gyr.x, gyroDPSRange, internalConfigs[index].dev.resolution);
            data->gyroY = lsb_to_meas(sens_data.gyr.y, gyroDPSRange, internalConfigs[index].dev.resolution);
            data->gyroZ = lsb_to_meas(sens_data.gyr.z, gyroDPSRange, internalConfigs[index].dev.resolution);
        }
        else {
            ESP_LOGE(TAG, "Couldn't get accel data %d", index);
        }
    }
    return pdTRUE;

}

static float lsb_to_meas(int16_t val, float range, uint8_t bit_width) {
    return (val * range) / (float)(1 << (bit_width-1));
}