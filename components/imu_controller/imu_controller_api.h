#ifndef IMU_CONTROLLER_API_H
#define IMU_CONTROLLER_API_H

#include "bmi270.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"

/* Running in header accel and gyro mode
 * gives 1b header + 6b gyro + 6b accel */
#define IMU_FIFO_BYTES_PER_FRAME 13

/* Overhead in case we get some extra control
 * frames or data frames. Happens at init when
 * getting the first FIFO. Worst case scenario
 * seems to be an extra 50 bytes worth of frames */
#define IMU_FIFO_OVERHEAD_BYTES 50

/* Set the FIFO watermark level*/
#define IMU_FIFO_WATERMARK_LEVEL CONFIG_IMU_BUFFER_NUM_FRAMES * IMU_FIFO_BYTES_PER_FRAME

/* Allocation size for FIFO array + a bit extra */
#define IMU_FIFO_ALLOC_SIZE IMU_FIFO_WATERMARK_LEVEL + IMU_FIFO_OVERHEAD_BYTES


typedef enum {
    /* Empty struct */
    IMU_STATE_NONE,
    /* SPI device settings configured */
    IMU_STATE_SPI_CONFIGURED,
    /* Chip ID read, registered on SPI bus */
    IMU_STATE_INITIALIZED,
    /* Started sampling with FIFO, havent gotten the first FIFO */
    IMU_STATE_SAMPLING_FIFO_INIT,
    /* Started sampling with FIFO, tossed first FIFO */
    IMU_STATE_SAMPLING_FIFO_RUNNING,
    /* Sampling with no FIFO */
    IMU_STATE_SAMPLING_SINGLE,
    IMU_STATE_COUNT
} IMUState_t;

typedef struct {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
} IMUOneShotData_t;

typedef struct {
    int64_t headUsTimestamp;
    int64_t tailUsTimestamp;
    int16_t accelX[CONFIG_IMU_BUFFER_NUM_FRAMES];
    int16_t accelY[CONFIG_IMU_BUFFER_NUM_FRAMES];
    int16_t accelZ[CONFIG_IMU_BUFFER_NUM_FRAMES];
    int16_t gyroX[CONFIG_IMU_BUFFER_NUM_FRAMES];
    int16_t gyroY[CONFIG_IMU_BUFFER_NUM_FRAMES];
    int16_t gyroZ[CONFIG_IMU_BUFFER_NUM_FRAMES];
} IMUFIFOData_t;

typedef struct {
    uint8_t devIndex;
    IMUState_t devState;
    IMUFIFOData_t FIFOData;
    struct bmi2_dev dev;
    struct bmi2_sens_config accelConfig;
    struct bmi2_sens_config gyroConfig;
    bmi2_intf_config_t interfaceConfig;
    int interruptGPIO;
} IMUDevice_t;

BaseType_t IMUControllerInit(void);

BaseType_t IMUControllerConfigSetSPI(uint8_t index, spi_host_device_t spiHost, int csPin, int interruptGPIO);

BaseType_t IMUControllerSetConfigAccelRange(uint8_t index, uint8_t range);

BaseType_t IMUControllerSetConfigAccelODR(uint8_t index, uint8_t odr);

BaseType_t IMUControllerSetConfigAccelFilterBWP(uint8_t index, uint8_t bwp);

BaseType_t IMUControllerSetConfigAccelFilterPerf(uint8_t index, uint8_t filterPerf);

BaseType_t IMUControllerSetConfigGyroRange(uint8_t index, uint8_t range);

BaseType_t IMUControllerSetConfigGyroODR(uint8_t index, uint8_t odr);

BaseType_t IMUControllerSetConfigGyroFilterBWP(uint8_t index, uint8_t bwp);

BaseType_t IMUControllerSetConfigGyroFilterPerf(uint8_t index, uint8_t filterPerf);

BaseType_t IMUControllerUpdateIMUSettings(uint8_t index);

BaseType_t IMUControllerEnableIMU(uint8_t index);

BaseType_t IMUControllerStartContinuousSampling(void);

BaseType_t IMUControllerGetOneShotData(uint8_t index, IMUOneShotData_t *data);

void IMUControllerContinuousSamplingTask(void *arg);

BaseType_t IMUControllerWaitOnData(uint8_t *index);

IMUFIFOData_t *IMUControllerGetFIFODataPtr(uint8_t index);

#endif /* IMU_CONTROLLER_API_H */