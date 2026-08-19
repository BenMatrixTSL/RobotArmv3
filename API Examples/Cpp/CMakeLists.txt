cmake_minimum_required(VERSION 3.14)
project(RobotArmCpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# nlohmann/json — place json.hpp in nlohmann/ next to this file
# Or install via vcpkg/conan and remove this include_directories line.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Core library (networking + API)
add_library(robot_arm STATIC
    ws_client.cpp
    robot_arm.cpp
)

if(WIN32)
    target_link_libraries(robot_arm PUBLIC ws2_32)
else()
    find_package(Threads REQUIRED)
    target_link_libraries(robot_arm PUBLIC Threads::Threads)
endif()

# Basic example executable
add_executable(example_basic example_basic.cpp)
target_link_libraries(example_basic PRIVATE robot_arm)
