idf_component_register(
    SRCS
        "udp_crc.c"
        "udp.c"

    INCLUDE_DIRS
        "."
        "${PROJECT_DIR}/src/middleware/mav"
        "${PROJECT_DIR}/src/libs/libcanard"
        "${CMAKE_BINARY_DIR}/cyphal/utility"
        "${CMAKE_BINARY_DIR}/cyphal/mavlink"
        "${CMAKE_BINARY_DIR}/cyphal/reg"
        "${CMAKE_BINARY_DIR}/cyphal/uavcan"
        "${PROJECT_DIR}/build/mavlink"
        "${PROJECT_DIR}/src/app"

    PRIV_REQUIRES  
        esp_timer
        libcanard
        cyphal
        driver
)