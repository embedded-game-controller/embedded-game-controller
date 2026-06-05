find_package(PkgConfig)
pkg_check_modules(LIBUSB IMPORTED_TARGET libusb-1.0)

if(CMAKE_SYSTEM_NAME MATCHES "Starlet")
    set(PLATFORM_SOURCES
        backends/ogc.c
        backends/ogc_arm.c
    )
    list(APPEND EXTRA_LIBS gcc)
elseif(CMAKE_SYSTEM_NAME MATCHES "NintendoWii")
    set(PLATFORM_SOURCES
        backends/wii.c
        hid_drivers/gamecube.c
    )
elseif(CMAKE_SYSTEM_NAME MATCHES "NintendoGameCube")
    set(PLATFORM_SOURCES
        backends/gamecube.c
        hid_drivers/gamecube.c
    )
elseif(LIBUSB_FOUND)
    # Use the libusb backend
    set(PLATFORM_SOURCES
        backends/libusb.c
    )
    list(APPEND EXTRA_LIBS PkgConfig::LIBUSB)
    set(PC_REQUIRES "${PC_REQUIRES} libusb-1.0")
else()
    message(FATAL_ERROR "No available platform backend")
endif()

add_library(embedded-game-controller STATIC
    ${PLATFORM_SOURCES}
    bluetooth.c
    egc.c
    hid_drivers/generic.c
    hid_drivers/generic_mappings.c
    hid_drivers/nintendo_switch.c
    hid_drivers/sony_ds3.c
    hid_drivers/sony_ds4.c
    hid_drivers/wiimote.c
)

target_include_directories(embedded-game-controller PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_options(embedded-game-controller PRIVATE
    ${EXTRA_LDFLAGS}
)

target_link_libraries(embedded-game-controller PRIVATE
    ${EXTRA_LIBS}
)

if(WITH_BLUETOOTH)
    pkg_check_modules(BTEMBEDDED REQUIRED IMPORTED_TARGET bt-embedded)
    set(PC_REQUIRES "${PC_REQUIRES} bt-embedded")
    target_link_libraries(embedded-game-controller PUBLIC PkgConfig::BTEMBEDDED)
    target_compile_definitions(embedded-game-controller PRIVATE WITH_BLUETOOTH=1)
endif()

if(WITH_DEBUG)
    target_compile_definitions(embedded-game-controller PRIVATE EGC_WITH_DEBUG)
endif()

set(NEEDS_LIBM OFF)

if(WITH_DRIVER_DS3)
    add_compile_definitions(WITH_DRIVER_DS3)
endif()
if(WITH_DRIVER_DS4)
    add_compile_definitions(WITH_DRIVER_DS4)
endif()
if(WITH_DRIVER_NINTENDO_SWITCH)
    add_compile_definitions(WITH_DRIVER_NINTENDO_SWITCH)
endif()
if(WITH_DRIVER_WIIMOTE)
    set(NEEDS_LIBM ON)
    add_compile_definitions(WITH_DRIVER_WIIMOTE)
endif()
if(WITH_DRIVER_GENERIC)
    add_compile_definitions(WITH_DRIVER_GENERIC)
endif()
if(WITH_GENERIC_MAPPINGS)
    add_compile_definitions(WITH_GENERIC_MAPPINGS)
endif()

if(NEEDS_LIBM)
    target_link_libraries(embedded-game-controller PRIVATE m)
    set(PC_EXTRA_LIBS "${PC_EXTRA_LIBS} -lm")
endif()

set(PUBLIC_HEADERS
    egc.h
    egc_types.h
)

set_target_properties(embedded-game-controller PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
)

install(TARGETS embedded-game-controller
    LIBRARY
    ARCHIVE
    RUNTIME
    PUBLIC_HEADER DESTINATION "include/embedded-game-controller"
)

install(FILES
    hid_drivers/wiimote.h
    DESTINATION "include/embedded-game-controller/drivers"
)

configure_file(../embedded-game-controller.pc.in embedded-game-controller.pc @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/embedded-game-controller.pc
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
