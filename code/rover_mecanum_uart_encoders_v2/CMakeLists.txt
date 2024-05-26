cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME rover_mecanum_uart_encoders_v2)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

message("Downloading required libraries(cjson, lwpkt, lwrb), it can take some time")

include(FetchContent)

set(ENABLE_CJSON_TEST Off)
set(ENABLE_CJSON_UTILS On)
FetchContent_Declare(
    cjson
    GIT_REPOSITORY https://github.com/DaveGamble/cJSON.git
    GIT_TAG v1.7.18
)
FetchContent_MakeAvailable(cjson)

FetchContent_Declare(
  lwrb
  GIT_REPOSITORY https://github.com/MaJerle/lwrb.git
  GIT_TAG        b32c645 # Replace with the desired version
)
FetchContent_MakeAvailable(lwrb)

# Add ext components
set(LWPKT_OPTS_FILE ${CMAKE_CURRENT_LIST_DIR}/components/app/app_main/inc/lwpkt_opts.h)
FetchContent_Declare(
  lwpkt
  GIT_REPOSITORY https://github.com/MaJerle/lwpkt.git
  GIT_TAG        6a82dab # Replace with the desired version
)
FetchContent_MakeAvailable(lwpkt)

# Add app components
add_subdirectory(components/app/app_main)
add_subdirectory(components/app/uart_rb)

#target_compile_definitions(lwrb INTERFACE LWRB_DISABLE_ATOMIC) # lwrb cpp support

target_link_libraries(stm32cubemx INTERFACE app_main)

target_link_libraries(lwpkt INTERFACE lwrb)
