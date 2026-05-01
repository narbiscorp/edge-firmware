idf_component_register(
    SRCS "narbis_ble_central.c"
    INCLUDE_DIRS "include"
    REQUIRES
        bt
        narbis_protocol
    PRIV_REQUIRES
        nvs_flash
        esp_timer
)
