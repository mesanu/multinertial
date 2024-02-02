#include <string.h>
#include "imu_controller_api.h"
#include "bmi270.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_timer.h"

static IMUDevice_t internalConfigs[CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS];
static uint8_t FIFOBuffer[IMU_FIFO_ALLOC_SIZE] = {0};
static uint32_t configuredIMUs = 0;
static const char *TAG = "IMU";

static struct bmi2_int_pin_config pinConfig;
static struct bmi2_fifo_frame FIFOFrame;
static const uint8_t sensor_list[2] = {BMI2_ACCEL, BMI2_GYRO};


static QueueHandle_t fifoReadyQueue;
static EventGroupHandle_t fifoRecievedEventGroup;
static EventBits_t eventGroupMask = 0;

static float lsbToMeas(int16_t val, float range, uint8_t bitWidth) {
    return (val * range) / (float)(1 << (bitWidth-1));
}

static void parseFIFOBuffer(IMUFIFOData_t *FIFOData, uint8_t *inputBuffer, uint16_t length) {
    /* First byte in the FIFO is going to be a dummy byte */
    uint16_t bufferIndex = 1;
    uint16_t frameIndex = 0;

    /* First non dummy byte assumed to be a frame header*/
    /* TODO, add proper error handlin from frames that shouldn't be recieved
     * and/or frames that have a variable length */
    while(bufferIndex < length) {
        switch(inputBuffer[bufferIndex++]) {
            case BMI2_FIFO_HEADER_ACC_FRM:
                bufferIndex += BMI2_FIFO_ACC_LENGTH;
                break;
            case BMI2_FIFO_HEADER_GYR_FRM:
                bufferIndex += BMI2_FIFO_GYR_LENGTH;
                break;
            case BMI2_FIFO_HEADER_GYR_ACC_FRM:
                if(((bufferIndex + BMI2_FIFO_ACC_GYR_LENGTH) < length) && (frameIndex < CONFIG_IMU_BUFFER_NUM_FRAMES)){
                    FIFOData->gyroX[frameIndex] = (inputBuffer[++bufferIndex]) << 8;
                    FIFOData->gyroX[frameIndex] |= inputBuffer[++bufferIndex];
                    FIFOData->gyroY[frameIndex] = (inputBuffer[++bufferIndex]) << 8;
                    FIFOData->gyroY[frameIndex] |= inputBuffer[++bufferIndex];
                    FIFOData->gyroZ[frameIndex] = (inputBuffer[++bufferIndex]) << 8;
                    FIFOData->gyroZ[frameIndex] |= inputBuffer[++bufferIndex];
                    FIFOData->accelX[frameIndex] = (inputBuffer[++bufferIndex]) << 8;
                    FIFOData->accelX[frameIndex] |= inputBuffer[++bufferIndex];
                    FIFOData->accelY[frameIndex] = (inputBuffer[++bufferIndex]) << 8;
                    FIFOData->accelY[frameIndex] |= inputBuffer[++bufferIndex];
                    FIFOData->accelZ[frameIndex] = (inputBuffer[++bufferIndex]) << 8;
                    FIFOData->accelZ[frameIndex] |= inputBuffer[++bufferIndex];
                    frameIndex++;
                } else {
                    break;
                }
                
                break;
            case BMI2_FIFO_HEADER_SENS_TIME_FRM:
                bufferIndex += BMI2_SENSOR_TIME_LENGTH;
                break;
            case BMI2_FIFO_HEADER_SKIP_FRM:
                bufferIndex += BMI2_FIFO_SKIP_FRM_LENGTH;
                break;
            case BMI2_FIFO_HEADER_INPUT_CFG_FRM:
                bufferIndex += BMI2_FIFO_INPUT_CFG_LENGTH;
                break;
            default:
                break;
        }
    }
    FIFOData->numFrames = frameIndex;
}

static void fifoReadyISRCallback(void *arg) {
    IMUDevice_t *imu = (IMUDevice_t *)arg;
    imu->FIFOData.tailUsTimestamp = imu->FIFOData.headUsTimestamp;
    imu->FIFOData.headUsTimestamp = esp_timer_get_time();
    gpio_intr_disable(imu->interruptGPIO);
    xQueueSendFromISR(fifoReadyQueue, &arg, NULL);
}

BaseType_t IMUControllerInit(void) {
    int i;
    int8_t rslt;
    gpio_config_t interruptPinConf;

    fifoReadyQueue = xQueueCreate(CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS, sizeof(BaseType_t));

    for (i = 0; i < CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS; i++) {
        if (internalConfigs[i].devState == IMU_STATE_SPI_CONFIGURED) {
            bmi2_interface_init(&internalConfigs[i].dev, BMI2_SPI_INTF);
            rslt = bmi270_init(&internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                ESP_LOGE(TAG, "Failed to initialize unit %d", i);
                return pdFALSE;
            }
            rslt = bmi2_get_sensor_config(&internalConfigs[i].accelConfig, 1, &internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                ESP_LOGE(TAG, "Failed to get accel configs unit %d", i);
                return pdFALSE;
            }
            rslt = bmi2_get_sensor_config(&internalConfigs[i].gyroConfig, 1, &internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                ESP_LOGE(TAG, "Failed to get gyro configs unit %d", i);
                return pdFALSE;
            }
            interruptPinConf.intr_type = GPIO_INTR_DISABLE;
            interruptPinConf.mode = GPIO_MODE_INPUT;
            interruptPinConf.pin_bit_mask = (1ULL << internalConfigs[i].interruptGPIO);
            interruptPinConf.pull_down_en = 0;
            interruptPinConf.pull_up_en = 1;

            gpio_config(&interruptPinConf);
            gpio_set_intr_type(internalConfigs[i].interruptGPIO, GPIO_INTR_LOW_LEVEL);
            gpio_isr_handler_add(internalConfigs[i].interruptGPIO, fifoReadyISRCallback, &internalConfigs[i]);
        
            eventGroupMask |= (1 << i);
            internalConfigs[i].devIndex = i;
            internalConfigs[i].FIFOData.devIndex = i;
            internalConfigs[i].devState = IMU_STATE_INITIALIZED;
        }
    }
    fifoRecievedEventGroup = xEventGroupCreate();
    ESP_LOGI(TAG, "All IMUs initialized successfully");
    return pdTRUE;
}

BaseType_t IMUControllerConfigSetSPI(uint8_t index, spi_host_device_t spiHost, int csPin, int interruptGPIO) {
    if (internalConfigs[index].devState != IMU_STATE_NONE) {
        return pdFALSE;
    }
    internalConfigs[index].interfaceConfig.spiHost = spiHost;

    internalConfigs[index].interfaceConfig.spiInterfaceConfig.address_bits = 8;
    internalConfigs[index].interfaceConfig.spiInterfaceConfig.clock_speed_hz = CONFIG_IMU_CONTROLLER_SPI_BUS_FREQ;
    internalConfigs[index].interfaceConfig.spiInterfaceConfig.mode = 0;
    internalConfigs[index].interfaceConfig.spiInterfaceConfig.spics_io_num = csPin;
    internalConfigs[index].interfaceConfig.spiInterfaceConfig.queue_size = 1;
    internalConfigs[index].interfaceConfig.spiSemaphore = xSemaphoreCreateMutex();

    internalConfigs[index].accelConfig.type = BMI2_ACCEL;
    internalConfigs[index].gyroConfig.type = BMI2_GYRO;
    internalConfigs[index].dev.intf_ptr = &internalConfigs[index].interfaceConfig;

    internalConfigs[index].interruptGPIO = interruptGPIO;

    internalConfigs[index].devState = IMU_STATE_SPI_CONFIGURED;

    configuredIMUs++;

    return pdTRUE;
}

BaseType_t IMUControllerSetConfigAccelRange(uint8_t index, uint8_t range) {
    if (internalConfigs[index].devState == IMU_STATE_INITIALIZED) {
        internalConfigs[index].accelConfig.cfg.acc.range = range;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUControllerSetConfigAccelODR(uint8_t index, uint8_t odr) {
    if (internalConfigs[index].devState == IMU_STATE_INITIALIZED) {
        internalConfigs[index].accelConfig.cfg.acc.odr = odr;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUControllerSetConfigAccelFilterBWP(uint8_t index, uint8_t bwp) {
    if (internalConfigs[index].devState == IMU_STATE_INITIALIZED) {
        internalConfigs[index].accelConfig.cfg.acc.bwp = bwp;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUControllerSetConfigAccelFilterPerf(uint8_t index, uint8_t filterPerf) {
    if (internalConfigs[index].devState == IMU_STATE_INITIALIZED) {
        internalConfigs[index].accelConfig.cfg.acc.bwp = filterPerf;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUControllerSetConfigGyroRange(uint8_t index, uint8_t range) {
    if (internalConfigs[index].devState == IMU_STATE_INITIALIZED) {
        internalConfigs[index].gyroConfig.cfg.gyr.range = range;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUControllerSetConfigGyroODR(uint8_t index, uint8_t odr) {
    if (internalConfigs[index].devState == IMU_STATE_INITIALIZED) {
        internalConfigs[index].gyroConfig.cfg.gyr.odr = odr;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUControllerSetConfigGyroFilterBWP(uint8_t index, uint8_t bwp) {
    if (internalConfigs[index].devState == IMU_STATE_INITIALIZED) {
        internalConfigs[index].gyroConfig.cfg.gyr.bwp = bwp;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUControllerSetConfigGyroFilterPerf(uint8_t index, uint8_t filterPerf) {
    if (internalConfigs[index].devState == IMU_STATE_INITIALIZED) {
        internalConfigs[index].gyroConfig.cfg.gyr.filter_perf = filterPerf;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUControllerSetConfigGyroNoisePerf(uint8_t index, uint8_t noisePerf) {
    if (internalConfigs[index].devState == IMU_STATE_INITIALIZED) {
        internalConfigs[index].gyroConfig.cfg.gyr.noise_perf = noisePerf;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUControllerUpdateIMUSettings(uint8_t index) {
    int8_t rslt = BMI2_OK;
    if (internalConfigs[index].devState == IMU_STATE_INITIALIZED) {
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

BaseType_t IMUControllerEnableIMU(uint8_t index) {
    int8_t rslt = BMI2_OK;
    if (internalConfigs[index].devState == IMU_STATE_INITIALIZED) {
        rslt = bmi2_sensor_enable(&sensor_list, 2, &internalConfigs[index].dev);
        if (rslt != BMI2_OK) {
            bmi2_error_codes_print_result(rslt);
            ESP_LOGE(TAG, "Failed to enable IMU %d", index);
            return pdFALSE;
        }
        internalConfigs[index].devState = IMU_STATE_SAMPLING_SINGLE;
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t IMUControllerStartContinuousSampling(void) {
    int8_t rslt = BMI2_OK;

    for (int i = 0; i < CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS; i++) {
        if ((internalConfigs[i].devState == IMU_STATE_INITIALIZED) || (internalConfigs[i].devState == IMU_STATE_SAMPLING_PAUSED)) {
            ESP_LOGI(TAG, "Starting sampling on IMU %d", i);
            /* Get default configuration for hardware Interrupt */
            rslt = bmi2_get_int_pin_config(&pinConfig, &internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                ESP_LOGE(TAG, "Failed to get interrupt pin config IMU %d", i);
                return pdFALSE;
            }
            rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                ESP_LOGE(TAG, "Couldn't disable adv power save IMU %d", i);
                return pdFALSE;
            }
            rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                ESP_LOGE(TAG, "Couldn't set FIFO config IMU %d", i);
                return pdFALSE;
            }
            FIFOFrame.data_int_map = BMI2_FWM_INT;
            rslt = bmi2_map_data_int(FIFOFrame.data_int_map, BMI2_INT1, &internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                ESP_LOGE(TAG, "Couldn't set interrupt pin config IMU %d", i);
                return pdFALSE;
            }
            pinConfig.pin_type = BMI2_INT1;
            pinConfig.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
            pinConfig.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;
            pinConfig.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
            pinConfig.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
            pinConfig.int_latch = BMI2_INT_NON_LATCH;
            rslt = bmi2_set_int_pin_config(&pinConfig, &internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                ESP_LOGE(TAG, "Couldn't set interrupt pin config IMU %d", i);
                return pdFALSE;
            }
            FIFOFrame.data = FIFOBuffer;
            FIFOFrame.wm_lvl = IMU_FIFO_WATERMARK_LEVEL;
            rslt = bmi2_set_fifo_wm(FIFOFrame.wm_lvl, &internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                return pdFALSE;
            }
            rslt = bmi2_set_fifo_config(BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN, BMI2_ENABLE, &internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                ESP_LOGE(TAG, "Couldn't enable FIFO IMU %d", i);
                return pdFALSE;
            }

            rslt = bmi2_sensor_enable(&sensor_list, 2, &internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                ESP_LOGE(TAG, "Failed to enable IMU %d", i);
                return pdFALSE;
            }
            internalConfigs[i].devState = IMU_STATE_SAMPLING_FIFO_INIT;
            gpio_intr_enable(internalConfigs[i].interruptGPIO);
        }
    }
    return pdFALSE;
}

BaseType_t IMUControllerStopContinuousSampling(void) {
    int8_t rslt = BMI2_OK;
    uint16_t fifoLength = 0;
    int i;

    for (i = 0; i < CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS; i++) {
        ESP_LOGI(TAG, "Stopping sampling on IMU %d", i);
        if ((internalConfigs[i].devState == IMU_STATE_SAMPLING_FIFO_INIT) || (internalConfigs[i].devState == IMU_STATE_SAMPLING_FIFO_RUNNING)) {
            internalConfigs[i].devState = IMU_STATE_SAMPLING_PAUSED;
            gpio_intr_disable(internalConfigs[i].interruptGPIO);
        }
    }

    for (i = 0; i < CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS; i++) {
        if (internalConfigs[i].devState == IMU_STATE_SAMPLING_PAUSED) {
            if(xSemaphoreTake(internalConfigs[i].interfaceConfig.spiSemaphore, 1000/portTICK_PERIOD_MS) == pdFALSE) {
                ESP_LOGE(TAG, "Failed to reserve spi semaphore in time IMU %d", i);
                return pdFALSE;
            }
            xSemaphoreGive(internalConfigs[i].interfaceConfig.spiSemaphore);
            rslt = bmi2_sensor_disable(&sensor_list, 2, &internalConfigs[i].dev);
            if (rslt != BMI2_OK) {
                bmi2_error_codes_print_result(rslt);
                ESP_LOGE(TAG, "Failed to disable IMU %d", i);
                return pdFALSE;
            }
            rslt = bmi2_get_fifo_length(&fifoLength, &internalConfigs[i].dev);
            bmi2_error_codes_print_result(rslt);

            if(FIFOFrame.length < IMU_FIFO_ALLOC_SIZE) {
                rslt = bmi2_read_fifo_data(&FIFOFrame, &internalConfigs[i].dev);
                bmi2_error_codes_print_result(rslt);
            } else {
                ESP_LOGE(TAG, "FIFO too larget to clear IMU %d", internalConfigs[i].devIndex);
                return pdFALSE;
            }

            rslt = bmi2_get_fifo_length(&fifoLength, &internalConfigs[i].dev);
            bmi2_error_codes_print_result(rslt);
            if(fifoLength != 0) {
                ESP_LOGE(TAG, "Couldn't clear FIFO leftovers IMU %d", internalConfigs[i].devIndex);
                return pdFALSE;
            }
        }
    }
    return pdTRUE;
}
void IMUControllerContinuousSamplingTask(void *arg) {
    int8_t rslt = BMI2_OK;
    uint16_t fifoLength = 0;

    while(1) {
        IMUDevice_t *imu;
        if (xQueueReceive(fifoReadyQueue, &imu, portMAX_DELAY)) {
            rslt = bmi2_get_fifo_length(&fifoLength, &imu->dev);
            bmi2_error_codes_print_result(rslt);

            FIFOFrame.length = fifoLength + imu->dev.dummy_byte;

            if(FIFOFrame.length < IMU_FIFO_ALLOC_SIZE) {
                rslt = bmi2_read_fifo_data(&FIFOFrame, &imu->dev);
                bmi2_error_codes_print_result(rslt);
            } else {
                ESP_LOGE(TAG, "FIFO recieved larger than buffer IMU %d", imu->devIndex);
            }

            /* Toss the first FIFO, since it has a few control
             * frames, accel only frames, etc, and is the wrong
             * size even with the watermark set */
            if(imu->devState == IMU_STATE_SAMPLING_FIFO_INIT) {
                imu->devState = IMU_STATE_SAMPLING_FIFO_RUNNING;
                gpio_intr_enable(imu->interruptGPIO);
                imu->FIFOData.headUsTimestamp = esp_timer_get_time();
                continue;
            }
            parseFIFOBuffer(&imu->FIFOData, FIFOBuffer, FIFOFrame.length);

            if((imu->devState != IMU_STATE_SAMPLING_PAUSED)) {
                xEventGroupSetBits(fifoRecievedEventGroup, (1 << imu->devIndex));
                gpio_intr_enable(imu->interruptGPIO);
            }
        } else {
            vTaskDelay(1);
        }
    }
    
}

BaseType_t IMUControllerGetOneShotData(uint8_t index, IMUOneShotData_t*data) {
    int8_t rslt;
    struct bmi2_sens_data sens_data = { { 0 } };

    /* This works because all the accel ranges are powers of 2 */
    float accelGRange = (2 << internalConfigs[index].accelConfig.cfg.acc.range);

    /* This works because all the gyro ranges can be calculated by deviding 2000 by powers of 2 */
    float gyroDPSRange = (2000/(1 << internalConfigs[index].gyroConfig.cfg.gyr.range));
    if (1) {
    //if (internalConfigs[index].devState == IMU_STATE_SAMPLING_SINGLE) {
        rslt = bmi2_get_sensor_data(&sens_data, &internalConfigs[index].dev);
        bmi2_error_codes_print_result(rslt);

        if ((rslt == BMI2_OK) && (sens_data.status & BMI2_DRDY_ACC) && (sens_data.status & BMI2_DRDY_ACC)) {
            /* Converting lsb to Gs for 16 bit accelerometer at 2G range. */
            data->accelX = lsbToMeas(sens_data.acc.x, accelGRange, internalConfigs[index].dev.resolution);
            data->accelY = lsbToMeas(sens_data.acc.y, accelGRange, internalConfigs[index].dev.resolution);
            data->accelZ = lsbToMeas(sens_data.acc.z, accelGRange, internalConfigs[index].dev.resolution);
            data->gyroX = lsbToMeas(sens_data.gyr.x, gyroDPSRange, internalConfigs[index].dev.resolution);
            data->gyroY = lsbToMeas(sens_data.gyr.y, gyroDPSRange, internalConfigs[index].dev.resolution);
            data->gyroZ = lsbToMeas(sens_data.gyr.z, gyroDPSRange, internalConfigs[index].dev.resolution);
        }
        else {
            ESP_LOGE(TAG, "Couldn't get accel data %d", index);
        }
    }
    return pdTRUE;
}

BaseType_t IMUControllerWaitOnData(uint8_t *index) {
    uint8_t i;
    EventBits_t eventBits = xEventGroupWaitBits(fifoRecievedEventGroup, eventGroupMask, pdFALSE, pdFALSE, portMAX_DELAY);
    if(eventGroupMask != 0){
        for (i=0; i<CONFIG_IMU_CONTROLLER_MAX_SUPPORTED_UNITS; i++){
            if((1 << i) & eventBits) {
                *index = i;
                xEventGroupClearBits(fifoRecievedEventGroup, (1 << i));
            }
        }
    }
    return pdTRUE;
}

IMUFIFOData_t *IMUControllerGetFIFODataPtr(uint8_t index) {
    return &internalConfigs[index].FIFOData;
}