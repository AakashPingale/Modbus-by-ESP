idf_component_register(
    SRCS "tinyexpr.c" "modbus.c"
    INCLUDE_DIRS "."
    REQUIRES esp-modbus driver json UART_OUTPUT custom_mqtt wifi
)
set_source_files_properties(tinyexpr.c PROPERTIES COMPILE_FLAGS "-Wno-char-subscripts")