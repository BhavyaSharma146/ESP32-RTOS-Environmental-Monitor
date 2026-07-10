idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES mbedtls
                    PRIV_REQUIRES esp_wifi esp_event esp_driver_gpio esp_driver_i2c chill-sam__ssd1306 nvs_flash esp_http_client)