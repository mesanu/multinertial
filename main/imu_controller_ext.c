#include "imu_controller_ext.h"
#include "imu_controller_api.h"

#include "componentExt.h"

BaseType_t IMUControllerInitExt(void) {
    return pdFALSE;
};

BaseType_t IMUControllerReadRegisterExt(uint8_t regAddr, uint16_t *regVal) {
    return pdFALSE;
};

BaseType_t IMUControllerWriteRegisterExt(uint8_t regAddr, uint16_t regVal) {
    return pdFALSE;
};