cmake_minimum_required(VERSION 3.13)

project(gb_manager NONE)

if(NOT DEFINED ENV{GBDK_SDK_DIR})
	message(FATAL_ERROR "GBDK_SDK_DIR was not defined")
endif()

set(GBDK_C_COMPILER "$ENV{GBDK_SDK_DIR}/bin/lcc")
add_custom_command(OUTPUT gb_manager.gb
        COMMAND ${GBDK_C_COMPILER}
        ARGS -Wa-l -Wl-m -Wl-j -Wm-p -Wm-ynGBMGR
            -o gb_manager.gb src/gb_manager.c
        WORKING_DIRECTORY ${gb_manager_SOURCE_DIR}
        COMMENT "Compiling GB ROM")
add_custom_target(gb_manager ALL
        DEPENDS ${gb_manager_BINARY_DIR}/gb_manager.gb
        SOURCES src/gb_manager.c)
