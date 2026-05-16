cmake_minimum_required(VERSION 3.20)

project(parallel_mpc
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(main_day_15
    main_day_15.cpp
    src/plant.cpp
    src/cost.cpp
    src/rollout_evaluator.cpp
)

target_include_directories(main_day_15
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if (MSVC)
    target_compile_options(main_day_15
        PRIVATE
            /utf-8
            /W4
            /permissive-
    )
else()
    target_compile_options(main_day_15
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )
endif()
