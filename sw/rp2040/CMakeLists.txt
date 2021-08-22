cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
include(pico_sdk_import.cmake)

project(gb_rp2040_cart)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(gb_rp2040_cart src/main.c)

target_link_libraries(gb_rp2040_cart pico_stdlib)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(gb_rp2040_cart)
