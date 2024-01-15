#ifndef IMU_CONTROLLER_API_H
#define IMU_CONTROLLER_API_H

#include "bmi270.h"
#include "driver/spi_master.h"


typedef struct bmi2_dev bmi2_dev_t;

typedef struct {
    bmi2_dev_t dev;
    bmi2_intf_config_t interfaceConfig;
} IMUConfig_t;

BaseType_t IMUControllerInit(void);

BaseType_t IMUControllerAddDevice(IMUConfig_t *config);

#endif /* IMU_CONTROLLER_API_H */