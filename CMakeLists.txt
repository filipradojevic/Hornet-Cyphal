cmake_minimum_required(VERSION 3.16)

# --------------------------------------------------------------------------

# Extra components
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_SOURCE_DIR}/src/app" 
    "${CMAKE_SOURCE_DIR}/src/drivers" 
    "${CMAKE_SOURCE_DIR}/src/middleware"
    "${CMAKE_SOURCE_DIR}/src/middleware/tl"
    "${CMAKE_SOURCE_DIR}/src/middleware/tl/udp_tl"
    "${CMAKE_SOURCE_DIR}/src/libs/libcanard"
    "${CMAKE_SOURCE_DIR}/src/middleware/cyphal"
)

add_compile_options(
    -Wno-dev
    -Wno-unused-function
    -Wno-unused-variable
    -Wno-type-limits 
    -Wno-address-of-packed-member
    -Wno-cast-align
    -Wno-shadow
    -Wno-stack-usage
    -Wno-conversion
    -Wno-aggressive-loop-optimizations
)

# Include ESP-IDF project.cmake
include($ENV{IDF_PATH}/tools/cmake/project.cmake)


# --- Build metadata for main.h (ESP32-S3 friendly) -------------------------
string(TIMESTAMP BUILD_UNIX_TIMESTAMP "%s" UTC)

find_program(GIT_EXECUTABLE NAMES git HINTS ENV PATH)
if (GIT_EXECUTABLE)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-parse --short=8 HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    execute_process(
        COMMAND "${GIT_EXECUTABLE}" describe --tags --abbrev=0
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_TAG
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
endif()

if (NOT GIT_HASH)
    set(GIT_HASH "unknown")
endif()
if (NOT GIT_TAG)
    set(GIT_TAG "v0.0.0")
endif()

# FreeRTOS version
set(OS_VERSION "v0.0.0")
if (EXISTS "${CMAKE_SOURCE_DIR}/src/middleware/FreeRTOS/manifest.yml")
    file(READ "${CMAKE_SOURCE_DIR}/src/middleware/FreeRTOS/manifest.yml" FREERTOS_MANIFEST)
    string(REGEX MATCH "version:[ \t]*\"([^\"]+)\"" _match "${FREERTOS_MANIFEST}")
    if (CMAKE_MATCH_1)
        set(OS_VERSION "${CMAKE_MATCH_1}")
    endif()
elseif(DEFINED ENV{IDF_VER})
    set(OS_VERSION "$ENV{IDF_VER}")
endif()

# LittleFS version
set(LFS_VERSION_BUILD "0")
if (EXISTS "${CMAKE_SOURCE_DIR}/src/middleware/lfs/littlefs/lfs.h")
    file(READ "${CMAKE_SOURCE_DIR}/src/middleware/lfs/littlefs/lfs.h" LFS_HEADER)
    string(REGEX MATCH "#define[ \t]+LFS_VERSION[ \t]+0x([0-9A-Fa-f]+)" _match "${LFS_HEADER}")
    if (CMAKE_MATCH_1)
        string(TOUPPER "${CMAKE_MATCH_1}" LFS_VERSION_BUILD)
    endif()
endif()

message(STATUS "Git Tag           : ${GIT_TAG}")
message(STATUS "Git Commit Hash   : ${GIT_HASH}")
message(STATUS "Build Timestamp   : ${BUILD_UNIX_TIMESTAMP}")
message(STATUS "FreeRTOS OS_VERSION : ${OS_VERSION}")
message(STATUS "LittleFS Version  : ${LFS_VERSION_BUILD}")

# Configure header with build metadata
set(GEN_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/app")

set(FW_IMAGE_SIZE 0)
set(FW_IMAGE_SIZE_WITH_HEADER 0)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/config/main.h.in"
    "${GEN_INCLUDE_DIR}/main.h"
    @ONLY
)

# Include generated headers globally
include_directories("${GEN_INCLUDE_DIR}")


project(BlackBoxESP)

# ----------------------------------------------------------------------------
# Cyphal/MAVLink DSDL generation and Nunavut C codegen (root-level)
# ----------------------------------------------------------------------------

# Ensure we have a Python interpreter for generators
if (NOT DEFINED PYTHON_EXECUTABLE)
    if (Python3_Interpreter_FOUND)
        set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
    else()
        find_program(PYTHON_EXECUTABLE NAMES python3 python REQUIRED)
    endif()
endif()

# Inform the cyphal component CMake to skip its internal generation to avoid duplication
set(CYPHAL_GEN_AT_ROOT TRUE CACHE BOOL "Cyphal code generation performed at root CMakeLists.txt")

# MAVLink -> DSDL conversion (into build tree)
set(MAVLINK_EXISTING_DSDL_ROOT "${CMAKE_BINARY_DIR}/mavlink_dsdl/mavlink")
if(EXISTS "${MAVLINK_EXISTING_DSDL_ROOT}/common" OR EXISTS "${MAVLINK_EXISTING_DSDL_ROOT}/lisum")
    set(MAVLINK_DSDL_DIR "${MAVLINK_EXISTING_DSDL_ROOT}")
    set(USE_EXISTING_MAVLINK_DSDL TRUE)
else()
    set(MAVLINK_DSDL_DIR "${CMAKE_BINARY_DIR}/cyphal/cyphal/messages/mavlink")
    set(USE_EXISTING_MAVLINK_DSDL FALSE)
endif()

file(GLOB MAVLINK_MESSAGE_HEADERS
    "${CMAKE_BINARY_DIR}/mavlink/common/mavlink_msg_*.h"
    "${CMAKE_BINARY_DIR}/mavlink/lisum/mavlink_msg_*.h"
)

add_custom_command(
    OUTPUT "${MAVLINK_DSDL_DIR}/.mav2dsdl.stamp"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${MAVLINK_DSDL_DIR}"
    COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_SOURCE_DIR}/src/libs/libcanard/mavlink_to_dsdl.py"
            --mavlink-dirs "${CMAKE_BINARY_DIR}/mavlink/common" "${CMAKE_BINARY_DIR}/mavlink/lisum"
            --out-dir "${MAVLINK_DSDL_DIR}"
    COMMAND ${CMAKE_COMMAND} -E touch "${MAVLINK_DSDL_DIR}/.mav2dsdl.stamp"
    DEPENDS ${MAVLINK_MESSAGE_HEADERS} "${CMAKE_SOURCE_DIR}/src/libs/libcanard/mavlink_to_dsdl.py"
    COMMENT "Converting MAVLink headers (common + lisum) to DSDL under ${MAVLINK_DSDL_DIR}"
    VERBATIM
)
add_custom_target(mavlink_dsdl_gen DEPENDS "${MAVLINK_DSDL_DIR}/.mav2dsdl.stamp")

# Nunavut output root
set(NUNAVUT_OUT_DIR "${CMAKE_BINARY_DIR}/cyphal")

# Track DSDL inputs to retrigger codegen when changed
file(GLOB_RECURSE CYPHAL_DSDL_FILES_LOCAL
    "${CMAKE_SOURCE_DIR}/messages/*.dsdl"
    "${CMAKE_SOURCE_DIR}/messages/**/*.dsdl"
)
file(GLOB_RECURSE CYPHAL_DSDL_FILES_MAVLINK
    "${CMAKE_BINARY_DIR}/cyphal/cyphal/messages/mavlink/messages_cyphal_uavcan/**/*.dsdl"
)
file(GLOB_RECURSE CYPHAL_DSDL_FILES_REG
    "${CMAKE_SOURCE_DIR}/messages/cyphal/uavcan/**/*.dsdl"
    "${CMAKE_SOURCE_DIR}/messages/cyphal/reg/**/*.dsdl"
)
set(ALL_DSDL_INPUTS ${CYPHAL_DSDL_FILES_REG} ${CYPHAL_DSDL_FILES_LOCAL} ${CYPHAL_DSDL_FILES_MAVLINK})

# Generate regulated UAVCAN headers
add_custom_command(
    OUTPUT "${NUNAVUT_OUT_DIR}/uavcan_stamp"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${NUNAVUT_OUT_DIR}/uavcan"
    COMMAND ${PYTHON_EXECUTABLE} -m nunavut --target-language c --outdir "${NUNAVUT_OUT_DIR}/uavcan" --allow-unregulated-fixed-port-id "${CMAKE_SOURCE_DIR}/messages/cyphal/uavcan"
    COMMAND ${CMAKE_COMMAND} -E touch "${NUNAVUT_OUT_DIR}/uavcan_stamp"
    DEPENDS ${CYPHAL_DSDL_FILES_REG}
    COMMENT "Generating regulated UAVCAN headers into ${NUNAVUT_OUT_DIR}/uavcan"
    VERBATIM
)

# Generate regulated reg headers
add_custom_command(
    OUTPUT "${NUNAVUT_OUT_DIR}/reg_stamp"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${NUNAVUT_OUT_DIR}/reg"
    COMMAND ${PYTHON_EXECUTABLE} -m nunavut --target-language c --outdir "${NUNAVUT_OUT_DIR}/reg" --lookup-dir "${CMAKE_SOURCE_DIR}/messages/cyphal/uavcan" --allow-unregulated-fixed-port-id "${CMAKE_SOURCE_DIR}/messages/cyphal/reg"
    COMMAND ${CMAKE_COMMAND} -E touch "${NUNAVUT_OUT_DIR}/reg_stamp"
    DEPENDS ${CYPHAL_DSDL_FILES_REG}
    COMMENT "Generating regulated reg headers into ${NUNAVUT_OUT_DIR}/reg"
    VERBATIM
)

# Generate MAVLink-derived Cyphal headers via Nunavut (after DSDL conversion)
add_custom_command(
    OUTPUT "${NUNAVUT_OUT_DIR}/mavlink_stamp"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${NUNAVUT_OUT_DIR}/mavlink"
    COMMAND ${PYTHON_EXECUTABLE} -m nunavut --target-language c --outdir "${NUNAVUT_OUT_DIR}/mavlink" --lookup-dir "${CMAKE_SOURCE_DIR}/messages/cyphal/uavcan" --lookup-dir "${CMAKE_SOURCE_DIR}/messages/cyphal/reg" --allow-unregulated-fixed-port-id "${CMAKE_BINARY_DIR}/cyphal/cyphal/messages/mavlink/messages_cyphal_uavcan"
    COMMAND ${CMAKE_COMMAND} -E touch "${NUNAVUT_OUT_DIR}/mavlink_stamp"
    DEPENDS mavlink_dsdl_gen ${ALL_DSDL_INPUTS}
    COMMENT "Generating MAVLink-derived Cyphal headers into ${NUNAVUT_OUT_DIR}/mavlink"
    VERBATIM
)

add_custom_target(cyphal_dsdl ALL DEPENDS
    "${NUNAVUT_OUT_DIR}/uavcan_stamp"
    "${NUNAVUT_OUT_DIR}/reg_stamp"
    "${NUNAVUT_OUT_DIR}/mavlink_stamp"
)
add_dependencies(cyphal_dsdl mavlink_dsdl_gen)

# Auto-generate raw-payload Cyphal publish helpers
set(CYPHAL_MAV_PUB_DIR "${CMAKE_BINARY_DIR}/cyphal/utility")
set(CYPHAL_MAV_PUB_SRC "${CYPHAL_MAV_PUB_DIR}/cyphal_mavlink_publishers.c")
set(CYPHAL_MAV_PUB_HDR "${CYPHAL_MAV_PUB_DIR}/cyphal_mavlink_publishers.h")

add_custom_command(
    OUTPUT "${CYPHAL_MAV_PUB_SRC}" "${CYPHAL_MAV_PUB_HDR}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CYPHAL_MAV_PUB_DIR}"
    COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_SOURCE_DIR}/src/middleware/cyphal/gen_publishers.py" --root "${CMAKE_BINARY_DIR}/cyphal/cyphal/messages/mavlink/messages_cyphal_uavcan" --out "${CYPHAL_MAV_PUB_SRC}"
    DEPENDS cyphal_dsdl "${CMAKE_SOURCE_DIR}/src/middleware/cyphal/gen_publishers.py"
    COMMENT "Generating Cyphal MAVLink publisher helper wrappers"
    VERBATIM
)
add_custom_target(cyphal_mavlink_publishers DEPENDS "${CYPHAL_MAV_PUB_SRC}" "${CYPHAL_MAV_PUB_HDR}")

# UAVCAN and reg publishers
set(CYPHAL_UAV_PUB_SRC "${CMAKE_BINARY_DIR}/cyphal/utility/cyphal_uavcan_publishers.c")
set(CYPHAL_UAV_PUB_HDR "${CMAKE_BINARY_DIR}/cyphal/utility/cyphal_uavcan_publishers.h")
add_custom_command(
    OUTPUT "${CYPHAL_UAV_PUB_SRC}" "${CYPHAL_UAV_PUB_HDR}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CYPHAL_MAV_PUB_DIR}"
    COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_SOURCE_DIR}/src/middleware/cyphal/gen_publishers.py" --root "${CMAKE_SOURCE_DIR}/messages/cyphal/uavcan" --out "${CYPHAL_UAV_PUB_SRC}"
    DEPENDS ${CYPHAL_DSDL_FILES_REG} "${CMAKE_SOURCE_DIR}/src/middleware/cyphal/gen_publishers.py"
    COMMENT "Generating Cyphal UAVCAN publisher helper wrappers"
    VERBATIM
)
add_custom_target(cyphal_uavcan_publishers DEPENDS "${CYPHAL_UAV_PUB_SRC}" "${CYPHAL_UAV_PUB_HDR}")

set(CYPHAL_REG_PUB_SRC "${CMAKE_BINARY_DIR}/cyphal/utility/cyphal_reg_publishers.c")
set(CYPHAL_REG_PUB_HDR "${CMAKE_BINARY_DIR}/cyphal/utility/cyphal_reg_publishers.h")
add_custom_command(
    OUTPUT "${CYPHAL_REG_PUB_SRC}" "${CYPHAL_REG_PUB_HDR}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CYPHAL_MAV_PUB_DIR}"
    COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_SOURCE_DIR}/src/middleware/cyphal/gen_publishers.py" --root "${CMAKE_SOURCE_DIR}/messages/cyphal/reg" --out "${CYPHAL_REG_PUB_SRC}"
    DEPENDS ${CYPHAL_DSDL_FILES_REG} "${CMAKE_SOURCE_DIR}/src/middleware/cyphal/gen_publishers.py"
    COMMENT "Generating Cyphal reg.udral publisher helper wrappers"
    VERBATIM
)
add_custom_target(cyphal_reg_publishers DEPENDS "${CYPHAL_REG_PUB_SRC}" "${CYPHAL_REG_PUB_HDR}")

# Attach generated sources and include dirs to the application target
idf_build_get_property(APP_TARGET EXECUTABLE)
add_dependencies(${APP_TARGET} cyphal_dsdl cyphal_mavlink_publishers cyphal_uavcan_publishers cyphal_reg_publishers)
target_sources(${APP_TARGET} PRIVATE "${CYPHAL_MAV_PUB_SRC}" "${CYPHAL_UAV_PUB_SRC}" "${CYPHAL_REG_PUB_SRC}")
target_include_directories(${APP_TARGET} PRIVATE
    "${CMAKE_BINARY_DIR}/cyphal/utility"
    "${CMAKE_BINARY_DIR}/cyphal/uavcan"
    "${CMAKE_BINARY_DIR}/cyphal/reg"
    "${CMAKE_BINARY_DIR}/cyphal/mavlink"
    "${CMAKE_BINARY_DIR}"
)

# Ensure Python interpreter path for generator scripts is logged (optional)
find_package(Python3 COMPONENTS Interpreter QUIET)
if (Python3_Interpreter_FOUND)
    message(STATUS "Python3 executable: ${Python3_EXECUTABLE}")
endif()

if(CONFIG_COMPILER_OPTIMIZATION_DEBUG)
    # ukloni sve prethodne optimizacije
    set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
endif()

# --- Post-build: prepend firmware header for bootloader OTA ---
set(ROOT_DIR_NAME ${CMAKE_PROJECT_NAME})

add_custom_command(TARGET ${APP_TARGET}
    POST_BUILD
    COMMAND ${PYTHON_EXECUTABLE} -m esptool --chip esp32s3 elf2image
        --flash_mode dio --flash_freq 80m --flash_size 16MB
        -o ${CMAKE_BINARY_DIR}/${ROOT_DIR_NAME}.bin
        $<TARGET_FILE:${APP_TARGET}>
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/prepend_fw_header.py
        ${CMAKE_BINARY_DIR}/${ROOT_DIR_NAME}.bin
        ${CMAKE_BINARY_DIR}/${ROOT_DIR_NAME}-firmware_for_bootloader.bin
        --version ${GIT_TAG}
        --git-hash ${GIT_HASH}
        --git-tag ${GIT_TAG}
        --project-name ${ROOT_DIR_NAME}
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/update_fw_size.py
        ${CMAKE_BINARY_DIR}/${ROOT_DIR_NAME}-firmware_for_bootloader.bin
        ${GEN_INCLUDE_DIR}/main.h
    COMMENT "Generating .bin, prepending firmware header, updating fw sizes in main.h"
)