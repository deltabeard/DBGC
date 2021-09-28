cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
include(pico_sdk_import.cmake)

project(dbgc_verify
        VERSION 0.1.0.0
        DESCRIPTION "DBGC Verification Tool"
        HOMEPAGE_URL "https://github.com/deltabeard/DBGC"
        LANGUAGES C CXX ASM)

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected; defaulting to Debug")
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(ENV{PICO_BOARD_HEADER_DIRS} ${CMAKE_SOURCE_DIR}/inc/board)
set(PICO_BOARD deltabeard_dbgc)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

set(CMAKE_C_FLAGS_DEBUG "-g3 -Og -Wall -Wextra")
set(CMAKE_C_FLAGS_MINSIZEREL "-g2 -Os -Wall -Wextra -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-g2 -Ofast -Wall -Wextra")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELWITHDEBINFO} -DNDEBUG")

add_executable(dbgc_verify src/verify.c)
target_include_directories(dbgc_verify PRIVATE inc)
pico_enable_stdio_usb(dbgc_verify ENABLED)
pico_enable_stdio_uart(dbgc_verify DISABLED)
target_link_libraries(dbgc_verify
        pico_stdlib pico_binary_info pico_bootrom
        hardware_i2c hardware_rtc)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(dbgc_verify)
