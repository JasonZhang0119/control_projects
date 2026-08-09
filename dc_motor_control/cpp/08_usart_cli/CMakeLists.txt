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
set(CMAKE_PROJECT_NAME 08_uart_cli)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Eigen path
set(EIGEN3_INCLUDE_DIR "D:/third_party_libs/cpp/eigen-5.0.0")


# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)
enable_language(CXX)

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
    App/Application/Src/app_bridge.cpp
    App/Application/Src/motor_app.cpp
    App/Application/Src/parameter_manager.cpp
    App/BSP/Src/button_manager.cpp
    App/BSP/Src/encoder_reader.cpp
    App/BSP/Src/motor_driver.cpp
    App/Service/Src/uart_cli.cpp
    App/Service/Src/uart_logger.cpp
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App/Application/Inc
    App/BSP/Inc
    App/Service/Inc
    App/Control
    ${EIGEN3_INCLUDE_DIR}
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    EIGEN_NO_DEBUG
    EIGEN_DONT_VECTORIZE
    EIGEN_MAX_ALIGN_BYTES=0
)


# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
)

set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY LINKER_LANGUAGE CXX)

# ============================================================
# Generate HEX and BIN files after building ELF
# ============================================================

find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy)
find_program(CMAKE_SIZE arm-none-eabi-size)

if(CMAKE_OBJCOPY)
    add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY}
            -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY}
            -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_PROJECT_NAME}.bin
        COMMENT "Generating HEX and BIN files"
    )
else()
    message(WARNING "arm-none-eabi-objcopy not found. HEX/BIN files will not be generated.")
endif()

if(CMAKE_SIZE)
    add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_SIZE}
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        COMMENT "Printing firmware size"
    )
endif()
