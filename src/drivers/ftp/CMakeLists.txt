idf_component_register(
    SRCS 
        "ftp_handler.c"
        "ftp_helper.c"
         
    PRIV_REQUIRES 
        driver
        lfs 
        mav 
        libcanard
        cyphal
        esp_timer
        

    INCLUDE_DIRS
        "."
        "${CMAKE_BINARY_DIR}/cyphal/uavcan"
        "${CMAKE_BINARY_DIR}/cyphal/reg"
        "${CMAKE_BINARY_DIR}/cyphal/mavlink"
        "${CMAKE_BINARY_DIR}/cyphal/cyphal/messages/mavlink"
        "${CMAKE_BINARY_DIR}/cyphal/utility"
)

