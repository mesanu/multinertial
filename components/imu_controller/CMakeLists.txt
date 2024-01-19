idf_component_register(SRCS "imu_controller.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer ESP-IDF-BMI270-Sensor-API)