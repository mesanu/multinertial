#ifndef IMU_CONTROLLER_EXT_H
#define IMU_CONTROLLER_EXT_H

#include "freertos/FreeRTOS.h"

BaseType_t IMUControllerInitExt(void);
BaseType_t IMUControllerReadRegisterExt(uint8_t regAddr, uint16_t *regVal);
BaseType_t IMUControllerWriteRegisterExt(uint8_t regAddr, uint16_t regVal);


#endif /* IMU_CONTROLLER_EXT_H */