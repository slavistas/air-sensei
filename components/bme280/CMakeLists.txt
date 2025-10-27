idf_component_register(SRCS "BME280_SensorAPI/bme280.c" "./esp32_common.c"
                       INCLUDE_DIRS "." "BME280_SensorAPI" "BME280_SensorAPI/examples/common"
                       PRIV_REQUIRES "driver" "esp_timer" "esp_common")