cmake_minimum_required(VERSION 3.20)

project(Control_Library)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(EIGEN3_INCLUDE_DIR "D:/third_party_libs/cpp/eigen-5.0.0")


add_library(Control_Library INTERFACE)


target_include_directories(Control_Library
    INTERFACE
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)



add_executable(test_parameter_estimator
    test/test_parameter_estimator.cpp
)

target_link_libraries(test_parameter_estimator
    PRIVATE
        Control_Library
)