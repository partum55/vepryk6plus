cmake_minimum_required(VERSION 3.10)
project(VeprykBridge CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compiler flags
add_compile_options(-Wall -Wextra -O3)

# Include MAVLink headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../headers)

# Main executable
add_executable(vepryk_bridge
    main.cpp
)

# Installation
install(TARGETS vepryk_bridge
    RUNTIME DESTINATION bin
)

message(STATUS "Building Vepryk MAVLink Bridge for Raspberry Pi")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  MAVLink headers: ${CMAKE_CURRENT_SOURCE_DIR}/../../headers")
message(STATUS "  Serial port: /dev/ttyS0")
message(STATUS "  TCP port: 14550")
