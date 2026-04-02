idf_component_register(
    SRCS "custom_mqtt.c"
    INCLUDE_DIRS "."
    REQUIRES mqtt esp_event esp_netif
)