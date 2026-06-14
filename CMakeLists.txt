cmake_minimum_required(VERSION 3.0)
project(planning_pset1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_VERBOSE_MAKEFILE ON)

include_directories(include)

# Add the main executable.
add_executable(run_test
        src/Planner.cpp
        src/runtest.cpp)
# Have the executable depend on the header files.
target_include_directories(run_test PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Define the absolute path to relevant directories.
target_compile_definitions(run_test PRIVATE MAPS_DIR="${CMAKE_SOURCE_DIR}/maps")
target_compile_definitions(run_test PRIVATE OUTPUT_DIR="${CMAKE_SOURCE_DIR}/output")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
