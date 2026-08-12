#pragma once

/**
 * @file estimator_gradient.hpp
 * @brief 线性参数模型的梯度估计器。
 *
 * 本文件提供标准梯度估计器和归一化梯度估计器。归一化版本支持可选的
 * 投影函数，用于根据参数约束修正参数变化率。
 */

#include <functional>
#include <stdexcept>

#include <Eigen/Dense>

#include "typing.hpp"

/**
 * @brief 标准梯度参数估计器。
 *
 * 适用于线性参数模型 y = Phi^T * theta，并采用显式欧拉法更新参数：
 * dot(theta) = -(gamma / 2) * Phi * (Phi^T * theta - y)。
 *
 * @tparam DataType 标量类型，例如 float 或 double。
 * @tparam N_y 测量输出维数。
 * @tparam N_theta 待估参数维数。
 */
template<typename DataType, int N_y, int N_theta>
class GradientEstimator_Standard
{
public:
    using RegressorMatrix = Matrix<DataType, N_theta, N_y>;
    using ParameterVector = Vector<DataType, N_theta>;
    using MeasurementVector = Vector<DataType, N_y>;

    const RegressorMatrix& GetRegressor() const { return regressor_; }
    const ParameterVector& GetParameter() const { return parameter_; }
    const MeasurementVector& GetMeasurement() const { return measurement_; }
    const ParameterVector& GetDotParameter() const { return dot_parameter_; }
    float GetTs() const { return Ts_; }
    float GetGamma() const { return gamma_; }

    void SetRegressor(const RegressorMatrix& regressor) { regressor_ = regressor; }
    void SetMeasurement(const MeasurementVector& measurement) { measurement_ = measurement; }

    void SetTs(float Ts)
    {
        if (Ts <= 0.0F) {
            throw std::invalid_argument("Ts must be positive.");
        }
        Ts_ = Ts;
    }

    void SetGamma(float gamma)
    {
        if (gamma <= 0.0F) {
            throw std::invalid_argument("Gamma must be positive.");
        }
        gamma_ = gamma;
    }

    /** @brief 清空估计状态，保留采样时间和增益配置。 */
    void Reset()
    {
        regressor_.setZero();
        parameter_.setZero();
        dot_parameter_.setZero();
        measurement_.setZero();
    }

    /** @brief 根据当前输入计算一次参数更新。 */
    void Compute()
    {
        const MeasurementVector error =
            regressor_.transpose() * parameter_ - measurement_;
        dot_parameter_ = -static_cast<DataType>(gamma_ / 2.0F) * regressor_ * error;
        parameter_ += dot_parameter_ * static_cast<DataType>(Ts_);
    }

protected:
    RegressorMatrix regressor_{RegressorMatrix::Zero()};
    ParameterVector parameter_{ParameterVector::Zero()};
    ParameterVector dot_parameter_{ParameterVector::Zero()};
    MeasurementVector measurement_{MeasurementVector::Zero()};
    float Ts_{1.0F};
    float gamma_{1.0F};
};

/**
 * @brief 带可选投影函数的归一化梯度参数估计器。
 *
 * 归一化项抑制回归量幅值变化对更新速度的影响。投影函数为空时执行普通
 * 归一化梯度更新；设置投影函数后，可在积分前修正参数变化率。
 *
 * @tparam DataType 标量类型，例如 float 或 double。
 * @tparam N_y 测量输出维数。
 * @tparam N_theta 待估参数维数。
 */
template<typename DataType, int N_y, int N_theta>
class GradientEstimator_Normalized
{
public:
    using RegressorMatrix = Matrix<DataType, N_theta, N_y>;
    using ParameterVector = Vector<DataType, N_theta>;
    using MeasurementVector = Vector<DataType, N_y>;
    using ProjectorFunction = std::function<ParameterVector(
        const ParameterVector& parameter,
        const ParameterVector& dot_parameter)>;

    const RegressorMatrix& GetRegressor() const { return regressor_; }
    const ParameterVector& GetParameter() const { return parameter_; }
    const MeasurementVector& GetMeasurement() const { return measurement_; }
    const ParameterVector& GetDotParameter() const { return dot_parameter_; }
    const ProjectorFunction& GetProjector() const { return projector_; }
    float GetTs() const { return Ts_; }
    float GetGamma() const { return gamma_; }
    float GetAlpha() const { return alpha_; }

    void SetRegressor(const RegressorMatrix& regressor) { regressor_ = regressor; }
    void SetMeasurement(const MeasurementVector& measurement) { measurement_ = measurement; }
    void SetProjector(ProjectorFunction projector) { projector_ = std::move(projector); }
    void ClearProjector() { projector_ = nullptr; }

    void SetTs(float Ts)
    {
        if (Ts <= 0.0F) {
            throw std::invalid_argument("Ts must be positive.");
        }
        Ts_ = Ts;
    }

    void SetGamma(float gamma)
    {
        if (gamma <= 0.0F) {
            throw std::invalid_argument("Gamma must be positive.");
        }
        gamma_ = gamma;
    }

    void SetAlpha(float alpha)
    {
        if (alpha < 0.0F) {
            throw std::invalid_argument("Alpha must be non-negative.");
        }
        alpha_ = alpha;
    }

    /** @brief 清空估计状态，保留采样时间、增益、归一化系数和投影函数。 */
    void Reset()
    {
        regressor_.setZero();
        parameter_.setZero();
        dot_parameter_.setZero();
        measurement_.setZero();
    }

    /** @brief 根据当前输入计算一次归一化梯度更新。 */
    void Compute()
    {
        const MeasurementVector error =
            regressor_.transpose() * parameter_ - measurement_;
        dot_parameter_ = -static_cast<DataType>(gamma_) * regressor_ * error;
        dot_parameter_ /= static_cast<DataType>(1.0F) +
            static_cast<DataType>(alpha_) * regressor_.squaredNorm();

        if (projector_) {
            dot_parameter_ = projector_(parameter_, dot_parameter_);
        }

        parameter_ += dot_parameter_ * static_cast<DataType>(Ts_);
    }

protected:
    RegressorMatrix regressor_{RegressorMatrix::Zero()};
    ParameterVector parameter_{ParameterVector::Zero()};
    ParameterVector dot_parameter_{ParameterVector::Zero()};
    MeasurementVector measurement_{MeasurementVector::Zero()};
    float Ts_{1.0F};
    float gamma_{1.0F};
    float alpha_{1.0F};
    ProjectorFunction projector_{};
};
