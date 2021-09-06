cmake_minimum_required(VERSION 3.13)

project(rp2040_manager
        VERSION 0.1.0.0
        DESCRIPTION "Cart tool for use on the RP2040"
        HOMEPAGE_URL "https://github.com/deltabeard/gb_rp2040_cart"
        LANGUAGES C CXX)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(rp2040_manager src/main.c src/comms.pio)

target_link_libraries(rp2040_manager pico_stdlib)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(rp2040_manager)
