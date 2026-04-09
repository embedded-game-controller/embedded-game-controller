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

if(WITH_BLUETOOTH)
    set(PC_REQUIRES "${PC_REQUIRES} bt-embedded")
endif()

add_library(embedded-game-controller STATIC
    ${PLATFORM_SOURCES}
    bluetooth.c
    egc.c
    usb_drivers/dragonrise.c
    usb_drivers/nintendo_switch.c
    usb_drivers/sony_ds3.c
    usb_drivers/sony_ds4.c
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

configure_file(../embedded-game-controller.pc.in embedded-game-controller.pc @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/embedded-game-controller.pc
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
