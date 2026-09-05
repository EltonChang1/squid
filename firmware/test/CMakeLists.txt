# Host-side tests (no Pico SDK). Build & run:
#   cmake -S firmware/test -B firmware/build-host && cmake --build firmware/build-host && ctest --test-dir firmware/build-host
cmake_minimum_required(VERSION 3.13)
project(tactile_umi_fw_host_tests CXX)
set(CMAKE_CXX_STANDARD 17)
enable_testing()

add_executable(test_protocol test_protocol.cpp)
target_include_directories(test_protocol PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_compile_options(test_protocol PRIVATE -Wall -Wextra)
add_test(NAME protocol COMMAND test_protocol)
