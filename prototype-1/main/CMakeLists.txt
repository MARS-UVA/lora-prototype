# register the component and set "RadioLib", "esp_timer" and "driver" as required
idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES radiolib esp_timer esp_driver_gpio esp_driver_spi)
