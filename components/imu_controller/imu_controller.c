#include "imu_controller_api.h"
#include "imu_controller_ext.h"
#include "sdkconfig.h"

static IMUConfig_t configs[CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS];

static uint32_t registeredIMUs = 0;

BaseType_t IMUControllerInit(void) {
    (void)(configs); /* Remove once variable is used */
    return pdFALSE;
};

BaseType_t IMUControllerAddDevice(IMUConfig_t *config) {
    (void)(registeredIMUs); /* Remove once variable is used */
    return pdFALSE;
}
