cmake_minimum_required(VERSION 3.13)

# Set PICO_SDK_FETCH_FROM_GIT=1 (default here) to have CMake clone the SDK, or
# point PICO_SDK_PATH at an existing checkout.
if(NOT DEFINED ENV{PICO_SDK_PATH} AND NOT DEFINED PICO_SDK_PATH)
  set(PICO_SDK_FETCH_FROM_GIT ON CACHE BOOL "Fetch Pico SDK from git")
  set(PICO_SDK_FETCH_FROM_GIT_TAG "2.1.1" CACHE STRING "Pico SDK tag")
endif()
set(PICO_BOARD pico CACHE STRING "Target board")

include(pico_sdk_import.cmake)

project(tactile_umi_fw C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(tactile_umi_fw
  src/main.cpp
  src/as5048a.cpp
  src/strain_frontend.cpp
)

target_include_directories(tactile_umi_fw PRIVATE src)
target_compile_options(tactile_umi_fw PRIVATE -Wall -Wextra)

target_link_libraries(tactile_umi_fw
  pico_stdlib
  hardware_spi
  hardware_adc
  hardware_gpio
  hardware_timer
)

# Binary-safe USB CDC: no CRLF translation, no UART stdio.
pico_enable_stdio_usb(tactile_umi_fw 1)
pico_enable_stdio_uart(tactile_umi_fw 0)
target_compile_definitions(tactile_umi_fw PRIVATE
  PICO_STDIO_ENABLE_CRLF_SUPPORT=0
  PICO_STDIO_DEFAULT_CRLF=0
  PICO_STDIO_USB_STDOUT_TIMEOUT_US=2000
)

pico_add_extra_outputs(tactile_umi_fw)  # .uf2, .bin, .hex, .map
