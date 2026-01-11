###
### This CMake project is NOT a replacement for the gradle project to build ThunderLib.
###
### It does however build the ThunderLibCore (i.e. the native code that implements ThunderAuto stuff which is
### used by the Java and C++ robot libraries). This CMake project exists so that ThunderAuto can build the
### code library and use the same implementation for Trajectories and Auto Modes as the Robot-side code. Very 
### cool stuff.
###

cmake_minimum_required(VERSION 3.22)

list(APPEND CMAKE_MESSAGE_CONTEXT "ThunderLibCore")

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "In-source builds are not allowed.")
endif()

###
### Version
###

# ThunderLib version

file(READ "ThunderLib-Version.txt" THUNDERLIB_VERSION_FILE_CONTENTS)

string(REGEX REPLACE "\n" ";" THUNDERLIB_VERSION_FILE_LINES "${THUNDERLIB_VERSION_FILE_CONTENTS}")

list(GET THUNDERLIB_VERSION_FILE_LINES 0 THUNDERLIB_VERSION_MAJOR)
list(GET THUNDERLIB_VERSION_FILE_LINES 1 THUNDERLIB_VERSION_MINOR)
list(GET THUNDERLIB_VERSION_FILE_LINES 2 THUNDERLIB_VERSION_PATCH)

set(THUNDERLIB_VERSION "${THUNDERLIB_VERSION_MAJOR}.${THUNDERLIB_VERSION_MINOR}.${THUNDERLIB_VERSION_PATCH}")

list(APPEND THUNDERLIB_DEF_LIST "THUNDERLIB_VERSION_MAJOR=${THUNDERLIB_VERSION_MAJOR}")
list(APPEND THUNDERLIB_DEF_LIST "THUNDERLIB_VERSION_MINOR=${THUNDERLIB_VERSION_MINOR}")
list(APPEND THUNDERLIB_DEF_LIST "THUNDERLIB_VERSION_PATCH=${THUNDERLIB_VERSION_PATCH}")
list(APPEND THUNDERLIB_DEF_LIST "THUNDERLIB_VERSION_STR=\"${THUNDERLIB_VERSION}\"")

message(STATUS "ThunderLib version: ${THUNDERLIB_VERSION}")

# ThunderAuto project version (independent of ThunderAuto and ThunderLib versions)

file(READ "ThunderAutoProject-Version.txt" THUNDERAUTO_PROJECT_VERSION_FILE_CONTENTS)

string(REGEX REPLACE "\n" ";" THUNDERAUTO_PROJECT_VERSION_FILE_LINES "${THUNDERAUTO_PROJECT_VERSION_FILE_CONTENTS}")

list(GET THUNDERAUTO_PROJECT_VERSION_FILE_LINES 0 THUNDERAUTO_PROJECT_VERSION_MAJOR)
list(GET THUNDERAUTO_PROJECT_VERSION_FILE_LINES 1 THUNDERAUTO_PROJECT_VERSION_MINOR)

set(THUNDERAUTO_PROJECT_VERSION "${THUNDERAUTO_PROJECT_VERSION_MAJOR}.${THUNDERAUTO_PROJECT_VERSION_MINOR}")

list(APPEND THUNDERLIB_DEF_LIST "THUNDERAUTO_PROJECT_VERSION_MAJOR=${THUNDERAUTO_PROJECT_VERSION_MAJOR}")
list(APPEND THUNDERLIB_DEF_LIST "THUNDERAUTO_PROJECT_VERSION_MINOR=${THUNDERAUTO_PROJECT_VERSION_MINOR}")
list(APPEND THUNDERLIB_DEF_LIST "THUNDERAUTO_PROJECT_VERSION_STR=\"${THUNDERAUTO_PROJECT_VERSION}\"")

message(STATUS "ThunderAuto project version: ${THUNDERAUTO_PROJECT_VERSION}")

###
### Project
###

set(CMAKE_PROJECT_NAME ThunderLibCore)
project(${CMAKE_PROJECT_NAME})

###
### Options
###

option(BUILD_TESTS "Build tests" ON)
option(BUILD_SHARED "Build shared libraries" OFF)

option(BUILD_WITH_ASSERTIONS "Build with assertions" OFF) # ON for debug builds
option(DOWNLOAD_WPILIB_BINARIES "Download prebuilt WPILib libraries" OFF) # ON for release builds
set(DOWNLOAD_WPILIB_BINARIES_ARCH "" CACHE STRING "Architecture of WPILib binaries to download")
set(DOWNLOAD_WPILIB_VERSION "2026.1.1" CACHE STRING "Version of WPILib repo/binaries to download")

###
### Platform
###

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  set(THUNDERLIB_WINDOWS ON)
  list(APPEND THUNDERLIB_DEF_LIST "THUNDERLIB_WINDOWS=1")
  set(DOWNLOAD_WPILIB_BINARIES_OS "windows")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  set(THUNDERLIB_MACOS ON)
  list(APPEND THUNDERLIB_DEF_LIST "THUNDERLIB_MACOS=1")
  set(DOWNLOAD_WPILIB_BINARIES_OS "osx")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(THUNDERLIB_LINUX ON)
  list(APPEND THUNDERLIB_DEF_LIST "THUNDERLIB_LINUX=1")
  set(DOWNLOAD_WPILIB_BINARIES_OS "linux")
else()
  message(FATAL_ERROR "Unknown platform: ${CMAKE_SYSTEM_NAME}")
endif()

###
### Languages
###

# Language standards
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Languages
enable_language(C)
enable_language(CXX)

###
### Build settings
###

# Default to debug build
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(BUILD_SHARED ON)

  set(THUNDERLIB_DEBUG ON)
  list(APPEND THUNDERLIB_DEF_LIST "THUNDERLIB_DEBUG")
  set(BUILD_WITH_ASSERTIONS ON)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(DOWNLOAD_WPILIB ON)
endif()

if(BUILD_WITH_ASSERTIONS)
  list(APPEND THUNDERLIB_DEF_LIST "THUNDERLIB_WITH_ASSERTIONS")
endif()

message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# Enable compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

###
### Project directories
###

set(CPP_DIR src/main/core/cpp/ThunderLibCore)
set(INC_DIR src/main/core/include)

###
### Library
###

if(BUILD_SHARED)
  add_library(${CMAKE_PROJECT_NAME} SHARED)
else()
  add_library(${CMAKE_PROJECT_NAME} STATIC)
endif()

###
### Include directories
###

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${INC_DIR})

###
### Sources
###

set(LIB_SOURCES "${CPP_DIR}/Types.cpp"
                "${CPP_DIR}/Logger.cpp"
                "${CPP_DIR}/Auto/ThunderAutoEditorState.cpp"
                "${CPP_DIR}/Auto/ThunderAutoFieldImage.cpp"
                "${CPP_DIR}/Auto/ThunderAutoMode.cpp"
                "${CPP_DIR}/Auto/ThunderAutoOutputTrajectory.cpp"
                "${CPP_DIR}/Auto/ThunderAutoProject.cpp"
                "${CPP_DIR}/Auto/ThunderAutoTrajectorySkeleton.cpp")

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${LIB_SOURCES})

###
### Link WPILib libraries
###

if(DOWNLOAD_WPILIB) 
  if(NOT DOWNLOAD_WPILIB_BINARIES_ARCH)
    # Check architecture.
    if(THUNDERLIB_MACOS)
      set(DOWNLOAD_WPILIB_BINARIES_ARCH "universal")
    else()
      if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64")
        set(DOWNLOAD_WPILIB_BINARIES_ARCH "x86-64")
      elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64|arm64")
        set(DOWNLOAD_WPILIB_BINARIES_ARCH "arm64")
      else()
        message(FATAL_ERROR "No pre-built wpilib binaries available for target architecture '${CMAKE_SYSTEM_PROCESSOR}'")
      endif()
    endif()
  endif()


  set(WPILIB_DOWNLOAD_DIR "${CMAKE_CURRENT_BINARY_DIR}/wpilib_${DOWNLOAD_WPILIB_VERSION}")
  string(REPLACE "." "_" WPILIB_DOWNLOAD_DIR ${WPILIB_DOWNLOAD_DIR})

  function(download_zip LIB_NAME LIB_URL LIB_DEST)
    message(STATUS "Downloading ${LIB_URL}")
    file(DOWNLOAD ${LIB_URL} ${LIB_DEST})
    message(STATUS "Downloaded ${LIB_NAME}")
  endfunction()

  function(extract_zip LIB_NAME ZIP_DIR LIB_DEST)
    message(STATUS "Extracting ${ZIP_DIR}")
    file(ARCHIVE_EXTRACT INPUT "${ZIP_DIR}" DESTINATION "${LIB_DEST}")
    message(STATUS "Extracted ${LIB_NAME}")
  endfunction()

  function(download_extract_lib LIB_NAME)
    set(LIB_URL "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/${LIB_NAME}/${LIB_NAME}-cpp/${DOWNLOAD_WPILIB_VERSION}/${LIB_NAME}-cpp-${DOWNLOAD_WPILIB_VERSION}-${DOWNLOAD_WPILIB_BINARIES_OS}${DOWNLOAD_WPILIB_BINARIES_ARCH}static.zip")
    set(LIB_DEST "${WPILIB_DOWNLOAD_DIR}/${LIB_NAME}")

    download_zip(${LIB_NAME} ${LIB_URL} "${LIB_DEST}.zip")
    extract_zip(${LIB_NAME} "${LIB_DEST}.zip" ${LIB_DEST})
  endfunction()

  function(download_extract_headers LIB_NAME)
    set(INC_URL "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/${LIB_NAME}/${LIB_NAME}-cpp/${DOWNLOAD_WPILIB_VERSION}/${LIB_NAME}-cpp-${DOWNLOAD_WPILIB_VERSION}-headers.zip")
    set(INC_DEST "${WPILIB_DOWNLOAD_DIR}/${LIB_NAME}_include")

    download_zip(${LIB_NAME} ${INC_URL} "${INC_DEST}.zip")
    extract_zip(${LIB_NAME} "${INC_DEST}.zip" ${INC_DEST})
  endfunction()

  set(WPILIB_MODULES "wpiutil;wpimath;wpinet;ntcore")
  foreach(MODULE ${WPILIB_MODULES})
    set(MODULE_LIB_DIR "${WPILIB_DOWNLOAD_DIR}/${MODULE}/${DOWNLOAD_WPILIB_BINARIES_OS}/${DOWNLOAD_WPILIB_BINARIES_ARCH}/static")
    if(THUNDERLIB_WINDOWS)
      set(MODULE_LIB_NAME "${MODULE}.lib")
    else()
      set(MODULE_LIB_NAME "lib${MODULE}.a")
    endif()
    if(NOT EXISTS "${MODULE_LIB_DIR}/${MODULE_LIB_NAME}")
      download_extract_lib(${MODULE})
    endif()
    
    if(NOT EXISTS "${WPILIB_DOWNLOAD_DIR}/${MODULE}_include")
      download_extract_headers(${MODULE})
    endif()

    target_link_directories(${CMAKE_PROJECT_NAME} PUBLIC "${MODULE_LIB_DIR}")
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC "${WPILIB_DOWNLOAD_DIR}/${MODULE}_include")

    message(STATUS "Linking directory: ${MODULE_LIB_DIR}")
    message(STATUS "Include directory: ${WPILIB_DOWNLOAD_DIR}/${MODULE}_include")

    target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC ${MODULE})
  endforeach()

else()
  include(FetchContent)
  FetchContent_Declare(
    allwpilib
    GIT_REPOSITORY https://github.com/wpilibsuite/allwpilib.git
    GIT_TAG v${DOWNLOAD_WPILIB_VERSION}
    GIT_SHALLOW TRUE
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
endif()

###
### Build+Link spdlog
###

set(SPDLOG_INC_DIR src/main/core/include)
set(SPDLOG_CPP_DIR src/main/core/cpp/spdlog)

file(GLOB_RECURSE SPDLOG_SOURCES ${SPDLOG_CPP_DIR}/*.cpp)

add_library(spdlog STATIC ${SPDLOG_SOURCES})
target_include_directories(spdlog PUBLIC ${SPDLOG_INC_DIR})
target_compile_definitions(spdlog PUBLIC SPDLOG_COMPILED_LIB)
target_compile_definitions(spdlog PUBLIC SPDLOG_FMT_EXTERNAL)
target_compile_definitions(spdlog PUBLIC SPDLOG_CLOCK_COARSE)
target_compile_definitions(spdlog PUBLIC SPDLOG_NO_THREAD_ID)
target_compile_definitions(spdlog PUBLIC SPDLOG_NO_ATOMIC_LEVELS)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(spdlog PRIVATE -fPIC)
endif()

target_link_libraries(spdlog PUBLIC wpiutil) # has fmtlib

if (DOWNLOAD_WPILIB)
  target_include_directories(spdlog PUBLIC "${WPILIB_DOWNLOAD_DIR}/wpiutil_include")
  message(STATUS "spdlog include directory: ${WPILIB_DOWNLOAD_DIR}/wpiutil_include")
endif()

if(MSVC)
  target_compile_options(spdlog PUBLIC /utf-8)
endif()

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC spdlog)

###
### Compiler & Linker flags
###

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wall -Wextra)# -Werror -Wpedantic)
elseif(MSVC)
  target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC /Zc:preprocessor /utf-8)
endif()

###
### Compile definitions
###

target_compile_definitions(${PROJECT_NAME} PUBLIC ${THUNDERLIB_DEF_LIST})

###
### Tests
###

if (BUILD_TESTS)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        6910c9d
    GIT_SHALLOW    TRUE
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()

  set(TEST_EXECUTABLE_NAME ${CMAKE_PROJECT_NAME}Tests)
  add_executable(${TEST_EXECUTABLE_NAME})

  set(TESTS_INC_DIR src/test/core/include)
  set(TESTS_CPP_DIR src/test/core/cpp)

  # Test sources
  file(GLOB_RECURSE TEST_SOURCES ${TESTS_CPP_DIR}/*.cpp)
  target_sources(${TEST_EXECUTABLE_NAME} PRIVATE ${TEST_SOURCES})

  # Include directories for tests
  target_include_directories(${TEST_EXECUTABLE_NAME} PRIVATE ${TESTS_INC_DIR})

  # Link to ThunderLibCore
  target_link_libraries(${TEST_EXECUTABLE_NAME} ${CMAKE_PROJECT_NAME})

  # Link to Google Test
  target_link_libraries(${TEST_EXECUTABLE_NAME} GTest::gtest_main GTest::gmock)

  include(GoogleTest)
  # gtest_discover_tests(${TEST_EXECUTABLE_NAME})
endif()

