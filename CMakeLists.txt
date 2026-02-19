# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(device_beacon C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(device_beacon
        interval.c
        beacon.c
        )

# optional custom board defs
target_compile_definitions(device_beacon PRIVATE
        #CYW43_DEFAULT_PIN_WL_REG_ON=0
        #CYW43_DEFAULT_PIN_WL_DATA_OUT=1
        #CYW43_DEFAULT_PIN_WL_DATA_IN=1
        #CYW43_DEFAULT_PIN_WL_HOST_WAKE=1
        #CYW43_DEFAULT_PIN_WL_CLOCK=2
        #CYW43_DEFAULT_PIN_WL_CS=3
        )
target_include_directories(device_beacon PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(device_beacon
        pico_cyw43_arch_lwip_poll
        pico_stdlib
        )

pico_enable_stdio_usb(device_beacon 1)
pico_add_extra_outputs(device_beacon)

# ====================================================================================

project(device_access C CXX ASM)

add_executable(device_access
        interval.c
        access.c
        dhcpserver.c
        )

# optional custom board defs
target_compile_definitions(device_access PRIVATE
        #CYW43_DEFAULT_PIN_WL_REG_ON=0
        #CYW43_DEFAULT_PIN_WL_DATA_OUT=1
        #CYW43_DEFAULT_PIN_WL_DATA_IN=1
        #CYW43_DEFAULT_PIN_WL_HOST_WAKE=1
        #CYW43_DEFAULT_PIN_WL_CLOCK=2
        #CYW43_DEFAULT_PIN_WL_CS=3
        )

target_include_directories(device_access PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(device_access
        pico_cyw43_arch_lwip_poll
        pico_stdlib
        )

pico_enable_stdio_usb(device_access 1)
pico_add_extra_outputs(device_access)