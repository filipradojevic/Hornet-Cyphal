cmake_minimum_required(VERSION 3.20)

project (bl)

set(SOURCES
	bl.c
)

add_library(${PROJECT_NAME}
STATIC
	${SOURCES}
)

target_include_directories(${PROJECT_NAME}
PUBLIC
	./
	${CMAKE_SOURCE_DIR}/devices/lpc1768/HAL
	${CMAKE_SOURCE_DIR}/src/middleware/mav
	${CMAKE_BINARY_DIR}/mavlink
	${CMAKE_SOURCE_DIR}/src/middleware/tl
    ${CMAKE_SOURCE_DIR}/devices/lpc1768/core
)