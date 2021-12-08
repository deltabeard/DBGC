cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
include(pico_sdk_import.cmake)

project(DBGC
        VERSION 0.3.0.0
        DESCRIPTION "Game Boy Cart using the RP2040 microcontroller"
        HOMEPAGE_URL "https://github.com/deltabeard/DBGC"
        LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected; defaulting to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Creating target for the Game Boy ROM Manager.
if(NOT DEFINED ENV{GBDK_SDK_DIR})
    message(FATAL_ERROR "GBDK_SDK_DIR was not defined")
endif()

if(NOT DEFINED ENV{GBDK_C_COMPILER})
    set(ENV{GBDK_C_COMPILER} "$ENV{GBDK_SDK_DIR}/bin/lcc")
endif()

set(GBDK_LINKER_OPTIONS -Wa-l -Wl-m -Wl-j -Wm-p -Wm-ynGBMGR -Wl-yt0)
add_custom_target(gb_manager ALL
        DEPENDS ${DBGC_BINARY_DIR}/gb_manager.gb
                ${DBGC_BINARY_DIR}/gb_manager.asm
                ${DBGC_BINARY_DIR}/gb_manager.gb.h
        SOURCES sw/gb_manager/src/gb.c)
add_custom_command(OUTPUT gb_manager.gb
        COMMAND $ENV{GBDK_C_COMPILER}
        ARGS ${GBDK_LINKER_OPTIONS} -I$ENV{GBDK_SDK_DIR}/include
        -Isw/gb_manager/inc
        -o ${DBGC_BINARY_DIR}/gb_manager.gb sw/gb_manager/src/gb.c
        DEPENDS sw/gb_manager/src/gb.c
        WORKING_DIRECTORY ${DBGC_SOURCE_DIR}
        COMMENT "Compiling GB Manager to ROM")
add_custom_command(OUTPUT gb_manager.asm
        COMMAND $ENV{GBDK_C_COMPILER}
        ARGS -S ${GBDK_LINKER_OPTIONS} -I$ENV{GBDK_SDK_DIR}/include
        -Isw/gb_manager/inc
        -o ${DBGC_BINARY_DIR}/gb_manager.asm sw/gb_manager/src/gb.c
        DEPENDS sw/gb_manager/src/gb.c
        WORKING_DIRECTORY ${DBGC_SOURCE_DIR}
        COMMENT "Compiling GB Manager to assembly")
add_custom_command(OUTPUT gb_manager.gb.h
        PRE_BUILD
        COMMAND ${CMAKE_COMMAND}
            -DSOURCE_FILE="${DBGC_BINARY_DIR}/gb_manager.gb"
            -DOUTPUT_FILE="${DBGC_BINARY_DIR}/gb_manager.gb.h"
            -DVARIABLE_NAME="gb_manager_rom"
            -P ${CMAKE_SOURCE_DIR}/gb2header.cmake
        DEPENDS ${DBGC_BINARY_DIR}/gb_manager.gb
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Converting GB ROM to header file")

set(ENV{PICO_BOARD_HEADER_DIRS} ${CMAKE_SOURCE_DIR}/sw/gb_manager/inc/board)
set(PICO_BOARD deltabeard_dbgc)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_C_FLAGS_DEBUG "-g3 -Og -fanalyzer -Wall -Wextra")
set(CMAKE_C_FLAGS_MINSIZEREL "-g2 -Os -ffast-math -Wall -Wextra -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-g2 -Ofast -Wall -Wextra")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELWITHDEBINFO} -DNDEBUG")

function(pico_add_verbose_dis_output TARGET)
    add_custom_command(TARGET ${TARGET} POST_BUILD
            COMMAND ${CMAKE_OBJDUMP} -h $<TARGET_FILE:${TARGET}> >$<IF:$<BOOL:$<TARGET_PROPERTY:${TARGET},OUTPUT_NAME>>,$<TARGET_PROPERTY:${TARGET},OUTPUT_NAME>,$<TARGET_PROPERTY:${TARGET},NAME>>.dis
            COMMAND ${CMAKE_OBJDUMP} -drwCSl $<TARGET_FILE:${TARGET}> >>$<IF:$<BOOL:$<TARGET_PROPERTY:${TARGET},OUTPUT_NAME>>,$<TARGET_PROPERTY:${TARGET},OUTPUT_NAME>,$<TARGET_PROPERTY:${TARGET},NAME>>.dis
            )
endfunction()

add_executable(manager sw/gb_manager/src/rp2040.c)
add_dependencies(manager gb_manager)
target_include_directories(manager PRIVATE sw/gb_manager/inc)
pico_generate_pio_header(manager
        ${CMAKE_CURRENT_LIST_DIR}/sw/gb_manager/src/comms.pio)
target_link_libraries(manager
        pico_stdlib pico_bootrom pico_multicore
        pico_bootsel_via_double_reset
        hardware_clocks hardware_pll hardware_vreg hardware_pio
        hardware_dma hardware_spi)
target_compile_definitions(manager PRIVATE
        PARAM_ASSERTIONS_DISABLE_ALL=1
        PICO_ENTER_USB_BOOT_ON_EXIT=1
        PICO_PANIC_FUNCTION=dbgc_panic)
#pico_set_binary_type(manager copy_to_ram)
pico_enable_stdio_usb(manager 0)
pico_enable_stdio_uart(manager 0)
pico_add_verbose_dis_output(manager)
pico_add_bin_output(manager)
pico_add_uf2_output(manager)

add_executable(verify sw/gb_manager/src/verify.c)
add_dependencies(verify gb_manager)
target_include_directories(verify PRIVATE sw/gb_manager/inc)
#pico_generate_pio_header(verify
#        ${CMAKE_CURRENT_LIST_DIR}/sw/gb_manager/src/comms.pio)
pico_generate_pio_header(verify
        ${CMAKE_CURRENT_LIST_DIR}/sw/gb_manager/src/ds1302.pio)
target_compile_definitions(verify PRIVATE
        PICO_STDIO_ENABLE_CRLF_SUPPORT=0
        PICO_STDIO_DEFAULT_CRLF=0
        PARAM_ASSERTIONS_DISABLE_ALL=1
        PICO_PRINTF_SUPPORT_FLOAT=0
        PICO_PRINTF_SUPPORT_EXPONENTIAL=0
        PICO_PRINTF_SUPPORT_LONG_LONG=0
        PICO_PRINTF_SUPPORT_PTRDIFF_T=0)
target_link_libraries(verify
        pico_stdlib pico_stdio pico_bootrom pico_multicore
        hardware_clocks hardware_pio hardware_vreg hardware_pio
        hardware_sync hardware_pll hardware_spi
        pico_binary_info)
pico_set_binary_type(verify no_flash)
pico_enable_stdio_usb(verify 1)
pico_enable_stdio_uart(verify 0)
pico_add_verbose_dis_output(verify)
pico_add_bin_output(verify)
pico_add_uf2_output(verify)
