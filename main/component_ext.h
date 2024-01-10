#ifndef COMPONENT_EXT
#define COMPONENT_EXT

#include "driver/spi_master.h"
#include "driver/gpio.h"

typedef struct {
    spi_host_device_t spiHost;
    spi_device_interface_config_t *spiInterface;
    spi_device_handle_t spiHandle;
    gpio_num_t csPin;
    gpio_num_t interruptPin;
} IMUExtConfig_t;

#endif /* COMPONENT_EXT */