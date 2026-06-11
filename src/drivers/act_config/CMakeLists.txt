cmake_minimum_required(VERSION 3.20)

project (act_config)

set(SOURCES
	act_config.c
)

add_library(${PROJECT_NAME}
STATIC
	${SOURCES}
)

target_include_directories(${PROJECT_NAME}
PUBLIC
	./
	${CMAKE_SOURCE_DIR}/devices/lpc1768/HAL
	${CMAKE_SOURCE_DIR}/devices/lpc1768/core
)