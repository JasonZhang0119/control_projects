#pragma once

#include <stdexcept>

#include <Eigen/Dense>

#include "typing.hpp"


template<typename DataType, int N_theta, int N_step>
class LeastSquareEstimator_WeightBatch
{

public:

    using RegressorMatrix = Matrix<DataType, N_step, N_theta>;
    using ParameterVector = Vector<DataType, N_theta>;
    using MeasurementVector = Vector<DataType, N_step>;
    using WeightVector = Vector<DataType, N_step>;
    using WeightDiagonalMatrix = Eigen::DiagonalMatrix<DataType, N_step>;


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
class LeastSquareEstimator_Recursive
{

public:

    using RegressorVector = Vector<DataType, N_theta>;
    using ParameterVector = Vector<DataType, N_theta>;
    using GainVector = Vector<DataType, N_theta>;
    using CovarianceMatrix = Matrix<DataType, N_theta, N_theta>;


    const RegressorVector& GetRegressor() const {
        return this->regressor_;
    }

    const ParameterVector& GetParameter() const {
        return this->parameter_;
    }

    const GainVector& GetGain() const {
        return this->K_;
    }

    const CovarianceMatrix& GetCovariance() const {
        return this->covariance_;
    }

    DataType GetMeasurement() const {
        return this->measurement_;
    }

    DataType GetInnovation() const {
        return this->innovation_;
    }

    DataType GetForgettingFactor() const {
        return this->forgetting_factor_;
    }

    void SetRegressor(const RegressorVector& regressor) {
        this->regressor_ = regressor;
    }

    void SetMeasurement(DataType measurement) {
        this->measurement_ = measurement;
    }

    void SetParameter(const ParameterVector& parameter) {
        this->parameter_ = parameter;
    }

    void SetCovariance(const CovarianceMatrix& covariance) {
        this->covariance_ = covariance;
    }

    bool SetForgettingFactor(DataType forgetting_factor) {
        if ((forgetting_factor <= static_cast<DataType>(0)) ||
            (forgetting_factor > static_cast<DataType>(1)))
        {
            return false;
        }

        this->forgetting_factor_ = forgetting_factor;
        return true;
    }

    void Reset() {
        this->regressor_.setZero();
        this->parameter_.setZero();
        this->K_.setZero();
        this->covariance_.setIdentity();
        this->measurement_ = static_cast<DataType>(0);
        this->innovation_ = static_cast<DataType>(0);
        this->forgetting_factor_ = static_cast<DataType>(1);
    }

    void Compute() {
        const DataType denominator =
            this->forgetting_factor_ +
            (this->regressor_.transpose() * this->covariance_ * this->regressor_)(0, 0);

        this->K_ = this->covariance_ * this->regressor_ / denominator;
        this->innovation_ =
            this->measurement_ - (this->regressor_.transpose() * this->parameter_)(0, 0);
        this->parameter_ += this->K_ * this->innovation_;
        this->covariance_ =
            (CovarianceMatrix::Identity() - this->K_ * this->regressor_.transpose()) *
            this->covariance_ / this->forgetting_factor_;
    }


private:

    RegressorVector regressor_{RegressorVector::Zero()};
    ParameterVector parameter_{ParameterVector::Zero()};
    GainVector K_{GainVector::Zero()};
    CovarianceMatrix covariance_{CovarianceMatrix::Identity()};

    DataType measurement_{static_cast<DataType>(0)};
    DataType innovation_{static_cast<DataType>(0)};
    DataType forgetting_factor_{static_cast<DataType>(1)};

};


template<typename DataType, int N_theta, int N_step>
class RegressorBuilder_FIRBatch{

};


template<typename DataType, int N_theta, int N_step>
class RegressorBuilder_FIRRecursive{

};


template<typename DataType, int N_theta, int N_step>
class RegressorBuilder_OutputErrorRecursive{

};



template<typename DataType, int N_theta, int N_step>
class RegressorBuilder_EquationErrorRecursive{

};


