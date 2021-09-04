cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
include(pico_sdk_import.cmake)
project(gb_rp2040_cart)

add_subdirectory(sw/rp2040 out/rp2040)
add_subdirectory(sw/gb out/gb)
