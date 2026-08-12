#pragma once

#include <stdexcept>

#include <Eigen/Dense>

#include "typing.hpp"


template<typename DataType, int N_theta, int N_measurement>
class LeastSquareEstimator_WeightBatch
{

public:

    using RegressorMatrix = Matrix<DataType, N_measurement, N_theta>;
    using ParameterVector = Vector<DataType, N_theta>;
    using MeasurementVector = Vector<DataType, N_measurement>;
    using WeightVector = Vector<DataType, N_measurement>;
    using WeightDiagonalMatrix = Eigen::DiagonalMatrix<DataType, N_measurement>;


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

        // Ax = b <=> (A).ldlt().solve(b)
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


template<typename DataType, int N_theta>
class GrandientEstimator_Standard{
public:


    using RegressorVector = Vector<DataType, N_theta>;
    using ParameterVector = Vector<DataType, N_theta>;


    const RegressorVector& GetRegressor() const{
        return this->regressor_;

    }

    const ParameterVector& GetParameter() const{
        return this->parameter_;
    }

    const float GetTs() const{
        return this->Ts_;
    }

    void SetRegressor(const RegressorVector& regressor){
        this->regressor_ = regressor;
    }

    void SetMeasurement(const RegressorVector& measurement){
        this->measurement_ = measurement;
    }

    void SetTs(const float Ts){
        this->Ts_ = Ts;
    }

    void Reset(){
        this->regressor_.setZero();
        this->parameter_.setZero();
        this->measurement_.setZero();
    }

    void Compute(){

        this->

    }





private:


    RegressorVector regressor_{RegressorVector::Zero()};
    ParameterVector parameter_{ParameterVector::Zero()};
    ParameterVector dot_parameter_{ParameterVector::Zero()};
    float Ts_{0};

}


