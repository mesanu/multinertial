#ifndef IMU_CONTROLLER_API_H
#define IMU_CONTROLLER_API_H

#include "freertos/FreeRTOS.h"

typedef enum {
    IMU_OUTPUT_ACCEL_X,
    IMU_OUTPUT_ACCEL_Y,
    IMU_OUTPUT_ACCEL_Z,
    IMU_OUTPUT_GYRO_X,
    IMU_OUTPUT_GYRO_Y,
    IMU_OUTPUT_GYRO_Z,
    IMU_OUTPUT_COUNT
} IMUOutputMap_t;

typedef enum {
    IMU_ACCEL_OUTPUT_DATA_RATE_0P78,
    IMU_ACCEL_OUTPUT_DATA_RATE_1P5,
    IMU_ACCEL_OUTPUT_DATA_RATE_3P1,
    IMU_ACCEL_OUTPUT_DATA_RATE_6P25,
    IMU_ACCEL_OUTPUT_DATA_RATE_12P5,
    IMU_ACCEL_OUTPUT_DATA_RATE_25,
    IMU_ACCEL_OUTPUT_DATA_RATE_50,
    IMU_ACCEL_OUTPUT_DATA_RATE_100,
    IMU_ACCEL_OUTPUT_DATA_RATE_200,
    IMU_ACCEL_OUTPUT_DATA_RATE_400,
    IMU_ACCEL_OUTPUT_DATA_RATE_800,
    IMU_ACCEL_OUTPUT_DATA_RATE_1600,
    MU_ACCEL_OUTPUT_DATA_RATE_COUNT
} IMUAccelOutputDataRate_t;

typedef enum {
    IMU_GYRO_OUTPUT_DATA_RATE_25,
    IMU_GYRO_OUTPUT_DATA_RATE_50,
    IMU_GYRO_OUTPUT_DATA_RATE_100,
    IMU_GYRO_OUTPUT_DATA_RATE_200,
    IMU_GYRO_OUTPUT_DATA_RATE_400,
    IMU_GYRO_OUTPUT_DATA_RATE_800,
    IMU_GYRO_OUTPUT_DATA_RATE_1600,
    IMU_GYRO_OUTPUT_DATA_RATE_COUNT
} IMUGyroOutputDataRate_t;

typedef enum {
    IMU_ACCEL_RANGE_2G,
    IMU_ACCEL_RANGE_4G,
    IMU_ACCEL_RANGE_8G,
    IMU_ACCEL_RANGE_16G,
    IMU_ACCEL_RANGE_COUNT
} IMUAccelRange_t;

typedef enum {
    IMU_GYRO_RANGE_2000DPS,
    IMU_GYRO_RANGE_1000DPS,
    IMU_GYRO_RANGE_500DPS,
    IMU_GYRO_RANGE_250DPS,
    IMU_GYRO_RANGE_125DPS,
    IMU_GYRO_RANGE_COUNT
} IMUGyroRange_t;

typedef struct {
    BaseType_t initialized;
    IMUAccelRange_t accelRange;
    IMUAccelOutputDataRate_t accelOutputDataRate;
    IMUGyroRange_t gyroRange;
    IMUGyroOutputDataRate_t gyroOutputDataRate;
    void *ext;
} IMUConfig_t;

BaseType_t IMUControllerInit(void);

BaseType_t IMUControllerAddDevice(IMUConfig_t *config, void *ext);

#endif /* IMU_CONTROLLER_API_H */