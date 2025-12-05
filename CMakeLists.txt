# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(STARTRACKER C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(STARTRACKER)

# must match with pio filename and executable name from above
pico_generate_pio_header(STARTRACKER ${CMAKE_CURRENT_LIST_DIR}/pio/hsync.pio)
pico_generate_pio_header(STARTRACKER ${CMAKE_CURRENT_LIST_DIR}/pio/vsync.pio)
pico_generate_pio_header(STARTRACKER ${CMAKE_CURRENT_LIST_DIR}/pio/rgb.pio)

# must match with executable name and source file names
target_sources(STARTRACKER PRIVATE 
    tracker.c magnetometer.c orientation.c
    ${CMAKE_CURRENT_LIST_DIR}/libs/mpu6050.c
    ${CMAKE_CURRENT_LIST_DIR}/libs/vga16_graphics_v2.c 
    ${CMAKE_CURRENT_LIST_DIR}/libs/vga_graphics_v3.c)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(STARTRACKER pico_stdlib pico_bootsel_via_double_reset pico_multicore hardware_pwm hardware_dma hardware_irq hardware_adc hardware_pio hardware_i2c hardware_clocks m)

# create map/bin/hex file etc.
pico_add_extra_outputs(STARTRACKER)