cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
include(pico_sdk_import.cmake)

project(dbgc_impl
        VERSION 0.1.0.0
        DESCRIPTION "DBGC PIO Implementation"
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

add_executable(dbgc_impl src/impl.c)
target_include_directories(dbgc_impl PRIVATE inc)
pico_generate_pio_header(dbgc_impl ${CMAKE_SOURCE_DIR}/src/comms.pio)
target_compile_definitions(dbgc_impl PRIVATE
        PARAM_ASSERTIONS_DISABLE_ALL=1
        PICO_ENTER_USB_BOOT_ON_EXIT=1
        PICO_PANIC_FUNCTION=dbgc_panic)
target_link_libraries(dbgc_impl
        pico_stdlib pico_bootrom pico_multicore
        pico_bootsel_via_double_reset
        hardware_clocks hardware_pll hardware_vreg hardware_i2c hardware_pio)

pico_enable_stdio_usb(dbgc_impl 0)
pico_enable_stdio_uart(dbgc_impl 0)

pico_add_bin_output(dbgc_impl)
pico_add_uf2_output(dbgc_impl)
function(pico_add_verbose_dis_output TARGET)
    add_custom_command(TARGET ${TARGET} POST_BUILD
            COMMAND ${CMAKE_OBJDUMP} -h $<TARGET_FILE:${TARGET}> >$<IF:$<BOOL:$<TARGET_PROPERTY:${TARGET},OUTPUT_NAME>>,$<TARGET_PROPERTY:${TARGET},OUTPUT_NAME>,$<TARGET_PROPERTY:${TARGET},NAME>>.dis
            COMMAND ${CMAKE_OBJDUMP} -drwCSl $<TARGET_FILE:${TARGET}> >>$<IF:$<BOOL:$<TARGET_PROPERTY:${TARGET},OUTPUT_NAME>>,$<TARGET_PROPERTY:${TARGET},OUTPUT_NAME>,$<TARGET_PROPERTY:${TARGET},NAME>>.dis
            )
endfunction()
pico_add_verbose_dis_output(dbgc_impl)
