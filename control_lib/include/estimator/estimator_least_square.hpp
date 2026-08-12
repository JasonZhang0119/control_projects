#pragma once

#include <stdexcept>

#include <Eigen/Dense>

#include "typing.hpp"


template<typename DataType, int N_y, int N_theta, int N_step>
class LeastSquareEstimator_WeightBatch
{

public:

    // 批量加权最小二乘：每步 N_y 个输出，共 N_step 组数据。
    using RegressorMatrix = Matrix<DataType, N_step * N_y, N_theta>;
    using ParameterVector = Vector<DataType, N_theta>;
    using MeasurementVector = Vector<DataType, N_step * N_y>;
    using WeightVector = Vector<DataType, N_step * N_y>;
    using WeightDiagonalMatrix = Eigen::DiagonalMatrix<DataType, N_step * N_y>;


    const RegressorMatrix& GetRegressor() const{
        return this->regressor_;

    }

    const MeasurementVector& GetMeasurement() const {
        return this->measurement_;
    }

    const WeightVector& GetWeight() const {
        return this->weight_vector_;
    }

    const ParameterVector& GetParameter() const{
        return this->parameter_;
    }

    void SetRegressor(const RegressorMatrix& regressor){
        this->regressor_ = regressor;
    }

    void SetMeasurement(const MeasurementVector& measurement){
        this->measurement_ = measurement;
    }

    void SetWeightVector(const WeightVector& weight_vector){
        // 权重作为对角矩阵使用，要求非负。
        if ((weight_vector.array() < static_cast<DataType>(0)).any())
        {
            throw std::invalid_argument("Weights must be non-negative.");
        }
        this->weight_vector_ = weight_vector;
    }

    void Reset(){
        this->regressor_.setZero();
        this->parameter_.setZero();
        this->measurement_.setZero();
        this->weight_vector_.setOnes();
    }

    void Compute() {
        const WeightDiagonalMatrix weight_matrix(this->weight_vector_);

        // 求解正规方程：(Phi^T W Phi) * theta = Phi^T W y。
        this->parameter_ =
            (this->regressor_.transpose() * weight_matrix * this->regressor_).ldlt().solve(
                this->regressor_.transpose() * weight_matrix * this->measurement_);
    }

    
private:

    RegressorMatrix regressor_{RegressorMatrix::Zero()};
    ParameterVector parameter_{ParameterVector::Zero()};
    MeasurementVector measurement_{MeasurementVector::Zero()};
    WeightVector weight_vector_{WeightVector::Ones()};
    
};



template<typename DataType, int N_y, int N_theta>
class LeastSquareEstimator_RecursiveStandard
{

public:

    // 批量加权最小二乘：每步 N_y 个输出，共 N_step 组数据。
    using RegressorMatrix = Matrix<DataType, N_y, N_theta>;
    using ParameterVector = Vector<DataType, N_theta>;
    using MeasurementVector = Vector<DataType, N_y>;

    using PMatrix = Matrix<DataType, N_theta, N_theta>;


    const RegressorMatrix& GetRegressor() const{
        return this->regressor_;

    }

    const MeasurementVector& GetMeasurement() const {
        return this->measurement_;
    }


    const ParameterVector& GetParameter() const{
        return this->parameter_;
    }

    void SetRegressor(const RegressorMatrix& regressor){
        this->regressor_ = regressor;
    }

    void SetMeasurement(const MeasurementVector& measurement){
        this->measurement_ = measurement;
    }


    void Reset(){
        this->regressor_.setZero();
        
        this->parameter_.setZero();
        this->dot_parameter_.setZero();

        this->P_.setZero();
        this->dot_P_.setZero();

        this->measurement_.setZero();
    }

    void Compute() {

        this->dot_parameter_ = - this->gamma_ * this->P_ * this->regressor_ *
                                (this->regressor_.transpose() * this->parameter_ - this->measurement_);
        this->dot_P_ = this->gamma_ * this->P_ * (this->lambda_ * this->P_ - this->regressor_ * this->regressor_.transpose() * this->P_);

        this->parameter_ += this->dot_parameter_ * this->Ts_;
        this->P_ += this->dot_P_ * this->Ts_;
    }

    
private:

    RegressorMatrix regressor_{RegressorMatrix::Zero()};
    ParameterVector parameter_{ParameterVector::Zero()};
    ParameterVector dot_parameter_{ParameterVector::Zero()};
    MeasurementVector measurement_{MeasurementVector::Zero()};

    PMatrix P_{PMatrix::Identity()};
    PMatrix dot_P_{PMatrix::Zero()};

    float Ts_{1.0F};
    float gamma_{1.0F};
    float lambda_{1.0F};

};
