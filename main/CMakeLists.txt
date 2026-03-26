idf_component_register(
    SRCS "modbus.c"
    INCLUDE_DIRS "."
    REQUIRES esp-modbus driver json
)