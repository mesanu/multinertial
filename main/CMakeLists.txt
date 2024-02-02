idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")
set(EXTRA_COMPONENT_DIRS "components/imu_controller"
                         "components/ESP-IDF-BMI270-Sensor-API"
                         "components/led_strip")