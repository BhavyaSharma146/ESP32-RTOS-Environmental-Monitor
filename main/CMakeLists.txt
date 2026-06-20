idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES chill-sam__ssd1306 esp_driver_gpio esp_driver_i2c driver)