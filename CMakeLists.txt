cmake_minimum_required(VERSION 3.10)

project(CopilotTest VERSION 1.0)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Eigen3 library
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add executables
add_executable(copilot_test main.cpp)
add_executable(mesh_projector AdvancedMeshProjector.cpp)

# Link Eigen3 to mesh_projector
target_link_libraries(mesh_projector Eigen3::Eigen)
