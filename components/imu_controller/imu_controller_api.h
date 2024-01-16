#ifndef IMU_CONTROLLER_API_H
#define IMU_CONTROLLER_API_H

#include "bmi270.h"
#include "driver/spi_master.h"

typedef struct {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
} IMUData_t;

typedef struct {
    BaseType_t registered;
    struct bmi2_dev dev;
    struct bmi2_sens_config accelConfig;
    struct bmi2_sens_config gyroConfig;
    bmi2_intf_config_t interfaceConfig;
} IMUConfig_t;

BaseType_t IMUControllerInit(void);

BaseType_t IMUControllerConfigSetSPI(uint8_t index, spi_host_device_t spiHost, int csPin);

BaseType_t IMUSetConfigAccelRange(uint8_t index, uint8_t range);

BaseType_t IMUSetConfigAccelODR(uint8_t index, uint8_t odr);

BaseType_t IMUSetConfigAccelFilterBWP(uint8_t index, uint8_t bwp);

BaseType_t IMUSetConfigAccelFilterPerf(uint8_t index, uint8_t filterPerf);

BaseType_t IMUSetConfigGyroRange(uint8_t index, uint8_t range);

BaseType_t IMUSetConfigGyroODR(uint8_t index, uint8_t odr);

BaseType_t IMUSetConfigGyroFilterBWP(uint8_t index, uint8_t bwp);

BaseType_t IMUSetConfigGyroFilterPerf(uint8_t index, uint8_t filterPerf);

BaseType_t IMUSetConfigIntPin(uint8_t index, uint8_t pinFunc, uint8_t pin);

BaseType_t IMUUpdateIMUSettings(uint8_t index);

BaseType_t IMUEnableIMU(uint8_t index);

BaseType_t IMUGetData(uint8_t index, IMUData_t *data);

#endif /* IMU_CONTROLLER_API_H */