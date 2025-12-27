cmake_minimum_required(VERSION 3.10)

project(CopilotTest VERSION 1.0)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add executable
add_executable(copilot_test main.cpp)
