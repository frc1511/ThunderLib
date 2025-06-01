###
### This CMake project is not intended to be a replacement for the Gradle build system.
### Its purpose is to provide an easier way to build the C++ library usage by desktop applications (ThunderAuto).
###

cmake_minimum_required(VERSION 3.22)

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "In-source builds are not allowed.")
endif()

###
### Settings
###

option(BUILD_TESTS "Build tests" ON)
option(BUILD_SHARED "Build shared libraries" OFF)

###
### Configure project settings
###

# Language standards
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Default to debug build
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(BUILD_SHARED ON)
endif()

# Enable compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
set(CMAKE_PROJECT_NAME ThunderLib)
project(${CMAKE_PROJECT_NAME})
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# Languages
enable_language(C)
enable_language(CXX)

###
### Build library
###

if(BUILD_SHARED)
    add_library(${CMAKE_PROJECT_NAME} SHARED)
else()
    add_library(${CMAKE_PROJECT_NAME} STATIC)
endif()

# Sources
file(GLOB_RECURSE LIB_SOURCES src/main/native/cpp/*.cpp)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${LIB_SOURCES})

# Includes
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC src/main/native/include)

# Enable all warnings
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Werror -Wall -Wextra -Wpedantic)
endif()

###
### WPILib libraries
###

include(FetchContent)
FetchContent_Declare(
    allwpilib
    GIT_REPOSITORY https://github.com/wpilibsuite/allwpilib.git
    GIT_TAG v2025.3.2
)

set(BUILD_SHARED_LIBS ${BUILD_SHARED} CACHE BOOL "" FORCE)
set(WITH_JAVA OFF CACHE BOOL "" FORCE)
set(WITH_DOCS OFF CACHE BOOL "" FORCE)
set(WITH_CSCORE OFF CACHE BOOL "" FORCE)
set(WITH_NTCORE ON CACHE BOOL "" FORCE)
set(WITH_WPICAL OFF CACHE BOOL "" FORCE)
set(WITH_WPIMATH ON CACHE BOOL "" FORCE)
set(WITH_WPILIB OFF CACHE BOOL "" FORCE)
set(WITH_EXAMPLES OFF CACHE BOOL "" FORCE)
set(WITH_TESTS OFF CACHE BOOL "" FORCE)
set(WITH_GUI OFF CACHE BOOL "" FORCE)
set(WITH_SIMULATION_MODULES OFF CACHE BOOL "" FORCE)
set(WITH_PROTOBUF ON CACHE BOOL "" FORCE)
set(NO_WERROR ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(allwpilib)

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC wpiutil wpimath ntcore)

###
### Build tests
###

if (BUILD_TESTS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        6910c9d
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()

    set(TEST_EXECUTABLE_NAME ${CMAKE_PROJECT_NAME}Tests)
    add_executable(${TEST_EXECUTABLE_NAME})

    # Test sources
    file(GLOB_RECURSE TEST_SOURCES src/test/native/cpp/*.cpp)
    target_sources(${TEST_EXECUTABLE_NAME} PRIVATE ${TEST_SOURCES})

    # Link to ThunderLib
    target_link_libraries(${TEST_EXECUTABLE_NAME} ${CMAKE_PROJECT_NAME})

    # Link to Google Test
    target_link_libraries(${TEST_EXECUTABLE_NAME} GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(${TEST_EXECUTABLE_NAME})
endif()
