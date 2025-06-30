idf_component_register(SRCS "lux.c"
                    REQUIRES "driver"
                    REQUIRES esp_adc
                    INCLUDE_DIRS "include")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)