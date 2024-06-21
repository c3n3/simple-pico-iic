project("simple_i2c"
)

cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(${PROJECT_NAME} STATIC)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_i2c
)

target_sources(${PROJECT_NAME}
    PRIVATE
        src/I2cMaster.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        # where top-level project will look for the library's public headers
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
