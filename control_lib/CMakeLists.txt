cmake_minimum_required(VERSION 3.20)

project(Control_Library)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(EIGEN3_INCLUDE_DIR "D:/third_party_libs/cpp/eigen-5.0.0")
set(MATLAB_ROOT "D:/Software/Programming/MATLAB_R2026a" CACHE PATH "MATLAB root directory")
set(MATLAB_MEX_OUTPUT_DIR "${PROJECT_SOURCE_DIR}/matlab/bin" CACHE PATH "Output directory for generated MEX files")
set(GSE_N_Y 1 CACHE STRING "Output dimension for GradientEstimator_Standard MEX/S-function")
set(GSE_N_THETA 2 CACHE STRING "Parameter dimension for GradientEstimator_Standard MEX/S-function")

find_program(MATLAB_MEX_EXECUTABLE NAMES mex mex.exe mex.bat
    HINTS "${MATLAB_ROOT}/bin/win64" "${MATLAB_ROOT}/bin")


add_library(Control_Library INTERFACE)


target_include_directories(Control_Library
    INTERFACE
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)



# add_executable(test_parameter_estimator
#     test/test_parameter_estimator.cpp
# )

# target_link_libraries(test_parameter_estimator
#     PRIVATE
#         Control_Library
# )


if(MATLAB_MEX_EXECUTABLE)
    file(MAKE_DIRECTORY "${MATLAB_MEX_OUTPUT_DIR}")

    # 为源码建立仅供编辑器分析的目标，使 mex.h/simstruc.h 不再显示红线。
    function(add_matlab_intellisense target source)
        add_library(${target} OBJECT EXCLUDE_FROM_ALL ${source})
        target_include_directories(${target} PRIVATE
            ${PROJECT_SOURCE_DIR}/include
            ${EIGEN3_INCLUDE_DIR}
            ${MATLAB_ROOT}/extern/include
            ${ARGN}
        )
        target_compile_definitions(${target} PRIVATE
            MATLAB_MEX_FILE
            GSE_N_Y=${GSE_N_Y} GSE_N_THETA=${GSE_N_THETA}
        )
        if(MSVC)
            target_compile_options(${target} PRIVATE /utf-8)
        endif()
    endfunction()

    # 使用 MATLAB 自带的 mex 驱动编译，确保编译器和链接库与 MATLAB 匹配。
    function(add_matlab_mex target source output_name)
        add_custom_target(${target}
            COMMAND "${MATLAB_MEX_EXECUTABLE}" -R2018a
                COMPFLAGS="$COMPFLAGS /std:c++17 /utf-8"
                -DGSE_N_Y=${GSE_N_Y} -DGSE_N_THETA=${GSE_N_THETA}
                -I"${PROJECT_SOURCE_DIR}/include" -I"${EIGEN3_INCLUDE_DIR}"
                "${PROJECT_SOURCE_DIR}/${source}"
                -output ${output_name} -outdir "${MATLAB_MEX_OUTPUT_DIR}"
            WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
            COMMENT "Building ${output_name}"
            VERBATIM
        )
    endfunction()

    add_matlab_intellisense(gradient_standard_estimator_mex_intellisense
        mex/gradient_standard_estimator_mex.cpp)
    add_matlab_intellisense(sfun_gradient_standard_estimator_intellisense
        mex_mdl/sfun_gradient_standard_estimator.cpp
        ${MATLAB_ROOT}/simulink/include)

    add_matlab_mex(mex_gradient_standard_estimator
        mex/gradient_standard_estimator_mex.cpp gradient_standard_estimator_mex)
    add_matlab_mex(mex_mdl_gradient_standard_estimator
        mex_mdl/sfun_gradient_standard_estimator.cpp sfun_gradient_standard_estimator)

    add_custom_target(matlab_interfaces ALL
        DEPENDS
            mex_gradient_standard_estimator
            mex_mdl_gradient_standard_estimator
    )
else()
    message(STATUS "MATLAB mex executable was not found. Set MATLAB_MEX_EXECUTABLE to enable MEX/S-function targets.")
endif()
