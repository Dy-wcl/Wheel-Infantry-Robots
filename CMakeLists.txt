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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME jie_max)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable verbose output for detailed build information
set(CMAKE_VERBOSE_MAKEFILE ON)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C C++ languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)
add_subdirectory(component)
add_subdirectory(bsp)
add_subdirectory(arithmetic)
add_subdirectory(tool)
add_subdirectory(modules)
add_subdirectory(calculate)
add_subdirectory(app)
add_subdirectory(task)
# Add CMSIS-DSP sources
add_subdirectory(arithmetic/CMSIS-DSP)
# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
     ${CMAKE_CURRENT_LIST_DIR}/SEGGER/RTT/SEGGER_RTT.c
    ${CMAKE_CURRENT_LIST_DIR}/SEGGER/RTT/SEGGER_RTT_printf.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_LIST_DIR}/SEGGER/RTT
    ${CMAKE_CURRENT_LIST_DIR}/SEGGER/Config
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    $<$<CONFIG:Debug>:MCU_DEBUG_BUILD>
    $<$<CONFIG:Release>:DISABLE_LOG_SYSTEM>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
    CMSISDSP
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined link options here
    LINKER:-u,_printf_float
) 
