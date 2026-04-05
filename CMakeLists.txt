cmake_minimum_required(VERSION 3.13)

set(EGC_MAJOR 0)
set(EGC_MINOR 1)
set(EGC_PATCH 0)
execute_process(COMMAND
    git describe --dirty --always --exclude "*"
    OUTPUT_VARIABLE EGC_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

project(
    embedded-game-controller
    VERSION ${EGC_MAJOR}.${EGC_MINOR}.${EGC_PATCH}
    LANGUAGES ASM C
)

option(WITH_BLUETOOTH "Support bluetooth controllers" ON)
option(BUILD_EXAMPLE "Build example program" OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# This is a hack just to provide a more readable name to our target platform.
# We could try to add a cmake toolchain file for Starlet in devkitPro.
if(CMAKE_SYSTEM_NAME MATCHES "Generic-dkP")
    set(CMAKE_SYSTEM_NAME "Starlet")
endif()

if(CMAKE_SYSTEM_NAME MATCHES "Starlet")
    set(ARCH -mcpu=arm926ej-s -mthumb -mthumb-interwork -mbig-endian)
    set(EXTRA_CFLAGS -fno-builtin)
    set(EXTRA_LDFLAGS -nostdlib)
    set(EXTRA_LIBS cios-lib)
endif()

set(CMAKE_C_STANDARD 11)

set(CMAKE_C_FLAGS_DEBUG "-g3")
set(CMAKE_C_FLAGS_RELEASE "-O2")

add_compile_options(
    ${ARCH}
    -ffreestanding
    -fomit-frame-pointer
    -ffunction-sections
    -Wall
    ${EXTRA_CFLAGS}
)

if(WITH_BLUETOOTH)
    set(BUILD_EXAMPLE OFF CACHE BOOL "Build example program" FORCE)
    add_subdirectory(bt-embedded)
    link_libraries(bt-embedded)
    include_directories(${CMAKE_SOURCE_DIR}/bt-embedded)
    add_definitions(-DWITH_BLUETOOTH=1)
endif()
add_subdirectory(embedded-game-controller)

if(CMAKE_SYSTEM_NAME MATCHES "Starlet")
    add_subdirectory(cios-lib)
endif()

if(BUILD_EXAMPLE)
    add_subdirectory(example)
endif()
