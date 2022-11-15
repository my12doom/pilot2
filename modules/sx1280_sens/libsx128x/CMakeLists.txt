# Sx1280 Library CMakeLists
cmake_minimum_required(VERSION 3.7.0)
include(ExternalProject)

project(sx128x CXX C)

# The version number.
set(${CMAKE_PROJECT_NAME}_VERSION_MAJOR 0)
set(${CMAKE_PROJECT_NAME}_VERSION_MINOR 1)

# Set test binary name
set(BIN sx128x-util)

# Packaging information.

set(VERBOSE 1)

if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE Debug)
endif()

# Sets compiler flags.
add_definitions(-Wall -Wpedantic -Wno-unused-function -Wno-c11-extensions -g ${FLAGS})

set(CMAKE_C_FLAGS_DEBUG "-std=gnu11 -O0 -g -Wno-int-conversion")
set(CMAKE_CXX_FLAGS_DEBUG "-std=gnu++11 -O0 -g -Wno-int-conversion")

set(CMAKE_C_FLAGS_RELEASE "-std=gnu11 -Os -flto")
set(CMAKE_CXX_FLAGS_RELEASE "-std=gnu++11 -Os -flto")

include(lib/sx128x.cmake)

set(SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
)

# Fetch commit hash
execute_process(COMMAND git rev-parse --short HEAD OUTPUT_VARIABLE ALGO_COMMIT_HASH)

# Add test outputs
add_executable(${BIN} ${SOURCES})
target_link_libraries(${BIN} ${OPTIONAL_LIBS} pthread stdc++)

