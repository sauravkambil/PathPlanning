cmake_minimum_required(VERSION 3.20)
project(planning_pset1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(PLANNER_ENABLE_DEBUG_LOGGING "Enable detailed planner debug logging" OFF)
option(PLANNER_ENABLE_LOGGING "Enable info logging" ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

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

if(PLANNER_ENABLE_DEBUG_LOGGING)
    target_compile_definitions(run_test PRIVATE PLANNER_ENABLE_DEBUG_LOGGING)
endif()

if(PLANNER_ENABLE_LOGGING)
    target_compile_definitions(run_test PRIVATE PLANNER_ENABLE_LOGGING)
endif()
