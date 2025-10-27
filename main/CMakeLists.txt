idf_component_register(SRCS "main.c"
                        PRIV_REQUIRES "bme280" "driver" "esp_netif" "mqtt" "nvs_flash" "wifi_station")
