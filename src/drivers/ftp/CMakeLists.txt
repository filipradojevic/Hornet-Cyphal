cmake_minimum_required(VERSION 3.20)

project(ftp)

set(SOURCES
    ./ftp_helper.c
    ./ftp_handler.c
    ./ftp_helper.h
)

add_library(${PROJECT_NAME}
    STATIC
    ${SOURCES}
)

set(DRIVERS
    ${LSM_SOURCE_DIR}/src/drivers/ftp
)

set(DEVICE_LPC1768
    ${LSM_SOURCE_DIR}/devices/lpc1768/HAL
    ${LSM_SOURCE_DIR}/devices/lpc1768/core
)

set(MIDDLEWARE
    ${LSM_SOURCE_DIR}/src/middleware/cyphal
    ${LSM_SOURCE_DIR}/src/middleware/lfs/littlefs
    ${LSM_SOURCE_DIR}/src/middleware/mav
    ${LSM_SOURCE_DIR}/src/middleware/tl
    ${LSM_SOURCE_DIR}/src/middleware/FreeRTOS/include
    ${LSM_SOURCE_DIR}/src/middleware/FreeRTOS/portable/GCC/ARM_CM3
)

set(OTHER
    ${LSM_BINARY_DIR}/mavlink
    ${LSM_BINARY_DIR}/cyphal
    ${LSM_SOURCE_DIR}/src/app
    ${LSM_SOURCE_DIR}/src/libs/libcanard
    ${CMAKE_BINARY_DIR}/cyphal/uavcan
    ${CMAKE_BINARY_DIR}/cyphal/reg
    ${CMAKE_BINARY_DIR}/cyphal/mavlink
    ${NUNAVUT_SUPPORT_DIR}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${DRIVERS}
    ${DEVICE_LPC1768}
    ${MIDDLEWARE}
    ${OTHER}
    ${CMAKE_BINARY_DIR}/cyphal/utility
)
