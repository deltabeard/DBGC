cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
include(pico_sdk_import.cmake)

project(gb_rp2040_cart
        VERSION 0.1.0.0
        DESCRIPTION "Game Boy Cart using the RP2040 microcontroller"
        HOMEPAGE_URL "https://github.com/deltabeard/gb_rp2040_cart"
        LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected; defaulting to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Creating target for the Game Boy ROM Manager.
if(NOT DEFINED ENV{GBDK_SDK_DIR})
    message(FATAL_ERROR "GBDK_SDK_DIR was not defined")
endif()

set(GBDK_C_COMPILER "$ENV{GBDK_SDK_DIR}/bin/lcc")
add_custom_command(OUTPUT gb_manager.gb
        COMMAND ${GBDK_C_COMPILER}
        ARGS -Wa-l -Wl-m -Wl-j -Wm-p -Wm-ynGBMGR
        -o ${gb_rp2040_cart_BINARY_DIR}/gb_manager.gb sw/gb/src/gb_manager.c
        WORKING_DIRECTORY ${gb_rp2040_cart_SOURCE_DIR}
        COMMENT "Compiling GB ROM")
add_custom_target(gb_manager ALL
        DEPENDS ${gb_rp2040_cart_BINARY_DIR}/gb_manager.gb
        SOURCES sw/gb/src/gb_manager.c)
add_custom_command(TARGET gb_manager
        POST_BUILD
        COMMAND ${CMAKE_COMMAND}
            -DSOURCE_FILE="${gb_rp2040_cart_BINARY_DIR}/gb_manager.gb"
            -DOUTPUT_FILE="${gb_rp2040_cart_BINARY_DIR}/gb_manager.gb.h"
            -DVARIABLE_NAME="gb_manager_rom"
            -P ${CMAKE_SOURCE_DIR}/gb2header.cmake
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Converting GB ROM to header file")

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_C_FLAGS_DEBUG "-g3 -Og -fanalyzer -Wall -Wextra")
set(CMAKE_C_FLAGS_MINSIZEREL "-g2 -Os -Wall -Wextra -Werror")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-g2 -Ofast -Wall -Wextra -Werror")
set(CMAKE_C_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELWITHDEBINFO})

add_executable(gb_rp2040_cart
        sw/rp2040/src/main.c sw/rp2040/src/io_expander.c sw/rp2040/src/rtc.c)
add_dependencies(gb_rp2040_cart gb_manager)
target_include_directories(gb_rp2040_cart PRIVATE sw/rp2040/inc)
pico_generate_pio_header(gb_rp2040_cart
        ${CMAKE_CURRENT_LIST_DIR}/sw/rp2040/src/comms.pio)
target_link_libraries(gb_rp2040_cart
        pico_stdlib pico_binary_info pico_bootrom
        hardware_pio hardware_i2c hardware_rtc hardware_irq)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(gb_rp2040_cart)
