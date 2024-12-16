# What CMake to start at
cmake_minimum_required(VERSION 3.12)

# Include the subsidiary .cmake file to get the SDK
include(pico_sdk_import.cmake)

# Set the name and version of the project
project(roe-pico VERSION 1.0.0)

# Link the Project to a source file (step 4.6)
add_executable(roe-pico roe-pico.c gclk.c)

# Link the Project to an extra library (pico_stdlib)
target_link_libraries(roe-pico pico_stdlib pico_multicore hardware_pio)

# Initalise the SDK
pico_sdk_init()

# Enable USB, UART output
pico_enable_stdio_usb(roe-pico 1)
pico_enable_stdio_uart(roe-pico 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(roe-pico)
