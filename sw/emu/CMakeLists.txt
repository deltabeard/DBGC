CMAKE_MINIMUM_REQUIRED(VERSION 3.19)

PROJECT(DBGC-EMU LANGUAGES C)

IF(NOT CMAKE_BUILD_TYPE)
    MESSAGE(STATUS "No build type selected; defaulting to Release")
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF()

SET(LIBRARY_DISCOVER_METHOD "PKG_CONFIG" CACHE STRING
        "Tool to use for discovering dependencies; options are: PKG_CONFIG, CPM")

IF(MSVC)
    # MSVC typically does not have pkg-config, but MSYS2/MinGW might.
    SET(LIBRARY_DISCOVER_METHOD "CPM")
ENDIF()

ADD_EXECUTABLE(dbgc_emu ${EXE_TARGET_TYPE} src/peanut_sdl.c)

IF(MSVC)
    # Disable DPI scaling.
    SET_PROPERTY(TARGET dbgc_emu PROPERTY VS_DPI_AWARE "PerMonitor")
ENDIF()

IF(${LIBRARY_DISCOVER_METHOD} STREQUAL "PKG_CONFIG")
    INCLUDE(FindPkgConfig)
    PKG_SEARCH_MODULE(SDL2 REQUIRED sdl2)
ELSEIF(${LIBRARY_DISCOVER_METHOD} STREQUAL "CPM")
    INCLUDE(CPM.cmake)
    CPMADDPACKAGE(GITHUB_REPOSITORY libsdl-org/SDL
            NAME SDL2
            GIT_TAG release-2.0.16
            OPTIONS
            "SDL_SHARED_ENABLED_BY_DEFAULT OFF"
            "SDL_STATIC_ENABLED_BY_DEFAULT ON"
            "SDL_STATIC ON"
            "OPT_DEF_LIBC TRUE")
    IF(SDL2_ADDED)
        TARGET_INCLUDE_DIRECTORIES(dbgc_emu PUBLIC $<BUILD_INTERFACE:${SDL2_SOURCE_DIR}/include>)
    ENDIF()
ELSE()
    MESSAGE(SEND_ERROR "LIBRARY_DISCOVER_METHOD '${LIBRARY_DISCOVER_METHOD}' is not valid")
ENDIF()

# Add required dependencies
IF(MSVC)
    TARGET_LINK_LIBRARIES(dbgc_emu PRIVATE SDL2-static SDL2main)
ELSE()
    TARGET_LINK_LIBRARIES(dbgc_emu PRIVATE ${SDL2_LIBRARIES})
ENDIF()

# Some FindSDL2 modules use slightly different variables, so we just use both.
TARGET_INCLUDE_DIRECTORIES(dbgc_emu PUBLIC ${SDL2_INCLUDE_DIRS} ${SDL2_INCLUDE_DIR})
