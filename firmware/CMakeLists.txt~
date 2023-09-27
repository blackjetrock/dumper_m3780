# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/tree/downloaded_tools/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
project(hd36106_replacement C CXX ASM)
set(PICO_COPY_TO_RAM 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# SD FAT LIB
#add_subdirectory(/tree/downloaded_tools/pico/ajm-no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI build)

# Add executable. Default name is the project name, version 0.1

add_executable(hd36106_replacement
hd36106_replacement.c
fx201p_execution.c
#../common/link_common.c
)

#pico_generate_pio_header(hd36106_replacement ${CMAKE_CURRENT_LIST_DIR}/picoputer.pio)

pico_set_program_name(hd36106_replacement "hd36106_replacement")
pico_set_program_version(hd36106_replacement "0.1")

#pico_enable_stdio_uart(hd36106_replacement 0)
pico_enable_stdio_usb(hd36106_replacement 1)

# Add the standard library to the build
target_link_libraries(hd36106_replacement pico_stdlib)

IF (NOT DEFINED N_SD_CARDS)
    SET(N_SD_CARDS 1)
ENDIF()
target_compile_definitions(hd36106_replacement PUBLIC DEBUG N_SD_CARDS=${N_SD_CARDS})

# Add any user requested libraries
target_link_libraries(hd36106_replacement
        #hardware_i2c
        hardware_pio
        hardware_clocks
	hardware_adc
	hardware_dma
#        pico_sd_card
        pico_multicore
#	FatFs_SPI
        )

pico_add_extra_outputs(hd36106_replacement)

