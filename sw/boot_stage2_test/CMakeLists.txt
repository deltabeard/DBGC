cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
include(pico_sdk_import.cmake)

project(dbgc_stage
        VERSION 0.1.0.0
        DESCRIPTION "DBGC PIO Implementation"
        HOMEPAGE_URL "https://github.com/deltabeard/DBGC"
        LANGUAGES C CXX ASM)

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected; defaulting to Debug")
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(ENV{PICO_BOARD_HEADER_DIRS} ${CMAKE_SOURCE_DIR}/inc/board)
set(ENV{PICO_DEFAULT_BOOT_STAGE2_FILE}
        ${CMAKE_SOURCE_DIR}/inc/board/boot2_w25q128jv.S)
set(PICO_BOARD deltabeard_dbgc)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

set(CMAKE_C_FLAGS_DEBUG "-g3 -Og -Wall -Wextra")
set(CMAKE_C_FLAGS_MINSIZEREL "-g2 -Os -Wall -Wextra -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-g2 -Ofast -Wall -Wextra")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELWITHDEBINFO} -DNDEBUG")

add_executable(dbgc_stage src/test.c)
target_include_directories(dbgc_stage PRIVATE inc)
target_compile_definitions(dbgc_stage PRIVATE
        PARAM_ASSERTIONS_DISABLE_ALL=1
        CFG_TUSB_DEBUG=0
        PICO_FLASH_SPI_CLKDIV=4)
target_link_libraries(dbgc_stage
        pico_stdlib pico_stdio pico_bootrom
        hardware_clocks hardware_vreg
        pico_binary_info)

#pico_set_binary_type(dbgc_stage no_flash)
pico_enable_stdio_usb(dbgc_stage 1)
pico_enable_stdio_uart(dbgc_stage 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(dbgc_stage)
