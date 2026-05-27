cmake_minimum_required(VERSION 3.20)

project (servo)

set(SOURCES
	servo.c
)

add_library(${PROJECT_NAME}
STATIC
	${SOURCES}
)

target_include_directories(${PROJECT_NAME}
PUBLIC
	./
	${CMAKE_SOURCE_DIR}/devices/lpc1768/HAL
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
	${DEVICE}
)