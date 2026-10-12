# Standalone build of the learning core + tests (no Geode SDK required):
#   cmake -S tests -B build-tests -G Ninja && cmake --build build-tests
#   build-tests/aigd_tests            (add --bench for the learning benchmark)
cmake_minimum_required(VERSION 3.21)
project(aigd_tests CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core ${CMAKE_CURRENT_BINARY_DIR}/core)

add_executable(aigd_tests tests.cpp)
target_link_libraries(aigd_tests PRIVATE aigd_core)

enable_testing()
add_test(NAME aigd_tests COMMAND aigd_tests)
