#ifndef IMU_CONTROLLER_API_H
#define IMU_CONTROLLER_API_H

#include "bmi270.h"
#include "driver/spi_master.h"

typedef struct {
    float x;
    float y;
    float z;
} IMUAccelData_t;

typedef struct {
    BaseType_t registered;
    struct bmi2_dev dev;
    struct bmi2_sens_config accelConfig;
    bmi2_intf_config_t interfaceConfig;
} IMUConfig_t;

BaseType_t IMUControllerInit(void);

BaseType_t IMUControllerAddDevice(IMUConfig_t *config, uint8_t index);

BaseType_t IMUSetConfigAccelRange(uint8_t index, uint8_t range);

BaseType_t IMUSetConfigAccelODR(uint8_t index, uint8_t odr);

BaseType_t IMUSetConfigFilterBWP(uint8_t index, uint8_t bwp);

BaseType_t IMUSetConfigFilterPerf(uint8_t index, uint8_t filterPerf);

BaseType_t IMUSetConfigIntPin(uint8_t index, uint8_t pinFunc, uint8_t pin);

BaseType_t IMUUpdateAccelSettings(uint8_t index);

BaseType_t IMUEnableAccel(uint8_t index);

BaseType_t IMUGetAccelData(uint8_t index, IMUAccelData_t *accelData);

#endif /* IMU_CONTROLLER_API_H */