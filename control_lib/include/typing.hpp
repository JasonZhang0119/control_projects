#pragma once

#include <Eigen/Dense>


template<typename DataType, int Rows, int Cols>
using Matrix = Eigen::Matrix<DataType, Rows, Cols>;

template<typename DataType, int Rows>
using Vector = Eigen::Matrix<DataType, Rows, 1>;