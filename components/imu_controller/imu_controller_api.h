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
 * seems to be an extra 100 bytes worth of frames */
#define IMU_FIFO_OVERHEAD_BYTES 100

/* Set the FIFO watermark level. Add 1 for the dummy byte, that apperently ends up in the FIFO*/
#define IMU_FIFO_WATERMARK_LEVEL CONFIG_IMU_BUFFER_NUM_FRAMES * IMU_FIFO_BYTES_PER_FRAME + 1

/* Allocation size for FIFO array + a bit extra */
#define IMU_FIFO_ALLOC_SIZE IMU_FIFO_WATERMARK_LEVEL + IMU_FIFO_OVERHEAD_BYTES


typedef enum {
    /* Empty struct */
    IMU_STATE_NONE,
    /* SPI device settings configured */
    IMU_STATE_SPI_CONFIGURED,
    /* Chip ID read, registered on SPI bus */
    IMU_STATE_INITIALIZED,
    /* IMU sampling is paused*/
    IMU_STATE_SAMPLING_PAUSED,
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

typedef struct __attribute__ ((packed)) {
    uint8_t devIndex;
    int64_t headUsTimestamp;
    int64_t tailUsTimestamp;
    int16_t numFrames;
    int16_t gyroX[CONFIG_IMU_BUFFER_NUM_FRAMES];
    int16_t gyroY[CONFIG_IMU_BUFFER_NUM_FRAMES];
    int16_t gyroZ[CONFIG_IMU_BUFFER_NUM_FRAMES];
    int16_t accelX[CONFIG_IMU_BUFFER_NUM_FRAMES];
    int16_t accelY[CONFIG_IMU_BUFFER_NUM_FRAMES];
    int16_t accelZ[CONFIG_IMU_BUFFER_NUM_FRAMES];
} IMUFIFOData_t;

typedef struct {
    uint8_t devIndex;
    IMUState_t devState;
    int64_t headUsTimestamp;
    int64_t tailUsTimestamp;
    struct bmi2_dev dev;
    struct bmi2_sens_config accelConfig;
    struct bmi2_sens_config gyroConfig;
    bmi2_intf_config_t interfaceConfig;
    int interruptGPIO;
} IMUDevice_t;

/* --------------------
 * IMUControllerInit
 *
 * Initializes the BMI270 interface, reads chip IDs and part configs,
 * sets up (but does not enable) ESP32 interrupt pin
 * 
 * returns: pdFALSE if initialization fails, pdTRUE otherwise
 * -------------------- */
BaseType_t IMUControllerInit(void);

/* --------------------
 * IMUControllerConfigSetSPI
 *
 * Sets up internal struct for SPI comms for individual parts
 * 
 * index: Assigned index of the IMU. Cannot exceed
 *        CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS
 * spiHost: ESP32 SPI host to use for communications. Only
 *          SPI2_HOST should be used on ESP32C3
 * csPin: GPIO pin to use for the chip select line
 * interruptGPIO: GPIO pin to use for the FIFO ready interrupt
 * 
 * returns: pdFALSE if an IMU with a given index is already
 *          configured
 * -------------------- */
BaseType_t IMUControllerConfigSetSPI(uint8_t index, spi_host_device_t spiHost, int csPin, int interruptGPIO);

/* --------------------
 * IMUControllerSetConfigAccelRange
 *
 * Sets the dynamic range of the accelerometer for all SPI configured
 * parts.
 * 
 * NOTE: Settings will not be updated until
 * IMUControllerUpdateIMUSettings is called
 * 
 * range: Accelerometer G range to use, as defined in bmi2_defs.h
 * -------------------- */
void IMUControllerSetConfigAccelRange(uint8_t range);

/* --------------------
 * IMUControllerSetConfigAccelRange
 *
 * Sets the output data rate for the accelerometer for all SPI
 * configured parts.
 * 
 * NOTE: Settings will not be updated until
 * IMUControllerUpdateIMUSettings is called
 * 
 * odr: Accelerometer ODR to use, as defined in bmi2_defs.h
 * -------------------- */
void IMUControllerSetConfigAccelODR(uint8_t odr);

/* --------------------
 * IMUControllerSetConfigAccelFilterBWP
 *
 * Sets the accelerometer filter mode for all SPI configured
 * parts.
 * 
 * NOTE: Settings will not be updated until
 * IMUControllerUpdateIMUSettings is called
 * 
 * bwp: Accelerometer ODR to use, as defined in bmi2_defs.h
 * -------------------- */
void IMUControllerSetConfigAccelFilterBWP(uint8_t bwp);

/* --------------------
 * IMUControllerSetConfigAccelFilterPerf
 *
 * Sets the accelerometer filter performance mode for all
 * SPI configured parts
 * 
 * NOTE: Settings will not be updated until
 * IMUControllerUpdateIMUSettings is called
 * 
 * filterPerf: Accelerometer filter performance mode to use,
 * as defined in bmi2_defs.h
 * -------------------- */
void IMUControllerSetConfigAccelFilterPerf(uint8_t filterPerf);

/* --------------------
 * IMUControllerSetConfigGyroRange
 *
 * Sets the dynamic range of the gyroscope for all SPI configured
 * parts.
 * 
 * NOTE: Settings will not be updated until
 * IMUControllerUpdateIMUSettings is called
 * 
 * range: Gyroscope DPS range to use, as defined in bmi2_defs.h
 * -------------------- */
void IMUControllerSetConfigGyroRange(uint8_t range);

/* --------------------
 * IMUControllerSetConfigGyroODR
 *
 * Sets the output data rate for the gyroscope for all SPI
 * configured parts.
 * 
 * NOTE: Settings will not be updated until
 * IMUControllerUpdateIMUSettings is called
 * 
 * odr: Gyroscope ODR to use, as defined in bmi2_defs.h
 * -------------------- */
void IMUControllerSetConfigGyroODR(uint8_t odr);

/* --------------------
 * IMUControllerSetConfigGyroFilterBWP
 *
 * Sets the gyroscope filter mode for all SPI configured
 * parts.
 * 
 * NOTE: Settings will not be updated until
 * IMUControllerUpdateIMUSettings is called
 * 
 * bwp: Gyroscope ODR to use, as defined in bmi2_defs.h
 * -------------------- */
void IMUControllerSetConfigGyroFilterBWP(uint8_t bwp);

/* --------------------
 * IMUControllerSetConfigGyroFilterPerf
 *
 * Sets the gyroscope filter performance mode for all
 * SPI configured parts
 * 
 * NOTE: Settings will not be updated until
 * IMUControllerUpdateIMUSettings is called
 * 
 * filterPerf: Gyroscope filter performance mode to use,
 * as defined in bmi2_defs.h
 * -------------------- */
void IMUControllerSetConfigGyroFilterPerf(uint8_t filterPerf);

/* --------------------
 * IMUControllerSetConfigGyroNoisePerf
 *
 * Sets the gyroscope noise performance mode for all
 * SPI configured parts
 * 
 * NOTE: Settings will not be updated until
 * IMUControllerUpdateIMUSettings is called
 * 
 * filterPerf: Gyroscope noise performance mode to use,
 * as defined in bmi2_defs.h
 * -------------------- */
void IMUControllerSetConfigGyroNoisePerf(uint8_t noisePerf);

/* --------------------
 * IMUControllerUpdateIMUSettings
 *
 * Updates the IMUs with the appropriate settings and sets
 * appropriate registers via SPI
 * 
 * index: Index identifier of the IMU to update
 * 
 * returns: pdFALSE if update fails, pdTRUE otherwise
 * -------------------- */
BaseType_t IMUControllerUpdateIMUSettings(uint8_t index);

/* --------------------
 * IMUControllerEnableIMU
 *
 * Enables the sensors IMU for single sample mode
 * 
 * index: Index identifier of the IMU to update
 * 
 * returns: pdFALSE if update fails, pdTRUE otherwise
 * -------------------- */
BaseType_t IMUControllerEnableIMU(uint8_t index);

/* --------------------
 * IMUControllerConfigContinuousSampling
 *
 * Configures the FIFO and IMU pin interrupt for each
 * SPI configured IMU, and enables the ESP32 pin interrupt
 * 
 * returns: pdFALSE if config fails, pdTRUE otherwise
 * -------------------- */
BaseType_t IMUControllerConfigContinuousSampling(void);

/* --------------------
 * IMUControllerStartContinuousSampling
 *
 * Starts sampling on all configured and initialized IMUs
 * with FIFO interrupts
 * 
 * returns: pdFALSE if enabling fails, pdTRUE otherwise
 * -------------------- */
BaseType_t IMUControllerStartContinuousSampling(void);

/* --------------------
 * IMUControllerStopContinuousSampling
 *
 * Stops sampling for all configured IMUs and dumps, then
 * tosses any samples remaining in the FIFO at the time sampling
 * is stipped
 * 
 * returns: pdFALSE if stopping fails, pdTRUE otherwise
 * -------------------- */
BaseType_t IMUControllerStopContinuousSampling(void);

/* --------------------
 * IMUControllerStopContinuousSampling
 *
 * Stops sampling for all configured IMUs and dumps, then
 * tosses any samples remaining in the FIFO at the time sampling
 * is stopped
 * 
 * returns: pdFALSE if stopping fails, pdTRUE otherwise
 * -------------------- */
BaseType_t IMUControllerGetOneShotData(uint8_t index, IMUOneShotData_t *data);

/* --------------------
 * IMUControllerContinuousSamplingTask
 *
 * IMU controller task that responds to FIFO watermark interrupts,
 * extracts the data from the part, parses the data into an
 * IMUFIFOData_t structure and places a pointer to the struct
 * in an output queue
 * 
 * arg: dummy argument, not used
 * -------------------- */
void IMUControllerContinuousSamplingTask(void *arg);

/* --------------------
 * IMUControllerGetFIFODataPtr
 *
 * Extracts a pointer from the internal data output queue
 * that points to a FIFO of data from an IMU. Blocking function
 * that will wait for portMAX_DELAY for data
 * 
 * data: A pointer to a pointer of FIFO data that will point to
 * an extracted FIFO
 * 
 * returns: pdFALSE if there is a timeout waitinf for data,
 *          pdTRUE otherwise
 * -------------------- */
BaseType_t IMUControllerGetFIFODataPtr(IMUFIFOData_t **data);

#endif /* IMU_CONTROLLER_API_H */