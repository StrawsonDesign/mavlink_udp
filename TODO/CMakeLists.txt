cmake_minimum_required(VERSION 3.6)   # CMake version check
project(vicon_mavlink)               # Create project "simple_example"
set(CMAKE_CXX_STANDARD 11)            # Enable c++11 standard

set(SOURCE_FILES vicon_mavlink.cpp mavlink_udp/mavlink_udp.c)            # Add main.cpp file of project root directory as source file
add_executable(vicon_mavlink ${SOURCE_FILES})       # Add executable target with source files listed in SOURCE_FILES variable