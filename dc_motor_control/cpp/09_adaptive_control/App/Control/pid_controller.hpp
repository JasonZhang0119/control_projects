#pragma once

#include <Eigen/Dense>

#include "output_feedback_controller.hpp"


/**
 * @brief 带 clamping anti-windup 的 PID 控制器。
 *
 * Parameters
 * ----------
 * DataType : typename
 *     数据类型，例如 float。
 * Ny : int
 *     输出反馈维度。
 * Nu : int
 *     控制输入维度。
 *
 * Notes
 * -----
 * 控制律为：
 *
 *     e[k] = y_ref[k] - y[k]
 *
 *     u_p[k] = Kp * e[k]
 *     u_i[k] = u_i[k-1] + Ki * e[k] * Ts
 *     u_d[k] = Kd * (e[k] - e[k-1]) / Ts
 *
 *     u_unsat[k] = u_p[k] + u_i[k] + u_d[k]
 *     u_sat[k] = saturate(u_unsat[k], u_min, u_max)
 *
 * anti-windup 使用 clamping / conditional integration：
 *
 * - 如果当前输出已经超过上限，并且积分项还在继续把输出往上推，则冻结该通道积分；
 * - 如果当前输出已经低于下限，并且积分项还在继续把输出往下推，则冻结该通道积分；
 * - 否则接受本周期积分更新。
 *
 * 当前实现适合 SISO，也可以用于简单 decoupled MIMO。
 * 对你的 DC motor speed PI 控制，建议：
 *
 *     Ny = 1
 *     Nu = 1
 *     Kd = 0
 *
 * 即先作为 PI 控制器使用。
 */
template<typename DataType, int Ny, int Nu>
class PidController:
    public OutputFeedbackControllerModule<DataType, Ny, Nu>
{
public:
    using Base = OutputFeedbackControllerModule<DataType, Ny, Nu>;
    using YVector = typename Base::YVector;
    using UVector = typename Base::UVector;

    using GainMatrix = Eigen::Matrix<DataType, Nu, Ny>;

    /**
     * @brief 设置采样周期。
     *
     * Parameters
     * ----------
     * Ts : const DataType&
     *     采样周期，单位 s。
     *
     * Returns
     * -------
     * None
     */
    void SetSamplingTime(const DataType& Ts)
    {
        this->Ts_ = Ts;
    }


    /**
     * @brief 设置 PID 增益矩阵。
     *
     * Parameters
     * ----------
     * Kp : const GainMatrix&
     *     比例增益矩阵，size = Nu x Ny。
     * Ki : const GainMatrix&
     *     积分增益矩阵，size = Nu x Ny。
     * Kd : const GainMatrix&
     *     微分增益矩阵，size = Nu x Ny。
     *
     * Returns
     * -------
     * None
     */
    void SetGain(
        const GainMatrix& Kp,
        const GainMatrix& Ki,
        const GainMatrix& Kd
    )
    {
        this->Kp_ = Kp;
        this->Ki_ = Ki;
        this->Kd_ = Kd;
    }


    /**
     * @brief 设置比例增益矩阵。
     *
     * Parameters
     * ----------
     * Kp : const GainMatrix&
     *     比例增益矩阵。
     *
     * Returns
     * -------
     * None
     */
    void SetKp(const GainMatrix& Kp)
    {
        this->Kp_ = Kp;
    }


    /**
     * @brief 设置积分增益矩阵。
     *
     * Parameters
     * ----------
     * Ki : const GainMatrix&
     *     积分增益矩阵。
     *
     * Returns
     * -------
     * None
     */
    void SetKi(const GainMatrix& Ki)
    {
        this->Ki_ = Ki;
    }


    /**
     * @brief 设置微分增益矩阵。
     *
     * Parameters
     * ----------
     * Kd : const GainMatrix&
     *     微分增益矩阵。
     *
     * Returns
     * -------
     * None
     */
    void SetKd(const GainMatrix& Kd)
    {
        this->Kd_ = Kd;
    }


    /**
     * @brief 获取比例增益矩阵。
     *
     * Returns
     * -------
     * const GainMatrix&
     *     当前比例增益矩阵。
     */
    const GainMatrix& GetKp() const
    {
        return this->Kp_;
    }


    /**
     * @brief 获取积分增益矩阵。
     *
     * Returns
     * -------
     * const GainMatrix&
     *     当前积分增益矩阵。
     */
    const GainMatrix& GetKi() const
    {
        return this->Ki_;
    }


    /**
     * @brief 获取微分增益矩阵。
     *
     * Returns
     * -------
     * const GainMatrix&
     *     当前微分增益矩阵。
     */
    const GainMatrix& GetKd() const
    {
        return this->Kd_;
    }


    /**
     * @brief 获取当前积分输出项。
     *
     * Returns
     * -------
     * const UVector&
     *     当前积分项对控制输出的贡献。
     */
    const UVector& GetIntegralOutput() const
    {
        return this->integral_output_;
    }


    /**
     * @brief 获取上一周期误差。
     *
     * Returns
     * -------
     * const YVector&
     *     上一周期误差。
     */
    const YVector& GetPreviousError() const
    {
        return this->previous_error_;
    }


    /**
     * @brief 重置控制器状态。
     *
     * Returns
     * -------
     * None
     */
    void Reset() override
    {
        this->integral_output_.setZero();
        this->previous_error_.setZero();
        this->is_first_step_ = true;
    }


    /**
     * @brief 初始化控制器。
     *
     * Returns
     * -------
     * None
     */
    void Initialize() override
    {
        this->Reset();
    }


    /**
     * @brief 终止控制器。
     *
     * Returns
     * -------
     * None
     */
    void Terminate() override
    {
        this->Reset();
    }


    /**
     * @brief 执行一步 PID 控制。
     *
     * Parameters
     * ----------
     * y_ref : Eigen::Ref<const YVector>&
     *     参考输出。
     * y : Eigen::Ref<const YVector>&
     *     当前测量输出。
     * u_min : Eigen::Ref<const UVector>&
     *     控制输出下限。
     * u_max : Eigen::Ref<const UVector>&
     *     控制输出上限。
     *
     * Returns
     * -------
     * UVector
     *     限幅后的控制输出。
     */
    UVector Step(
        const Eigen::Ref<const YVector>& y_ref,
        const Eigen::Ref<const YVector>& y,
        const Eigen::Ref<const UVector>& u_min,
        const Eigen::Ref<const UVector>& u_max
    ) override
    {
        const YVector error = y_ref - y;

        const UVector proportional_output =
            this->Kp_ * error;

        UVector derivative_output = UVector::Zero();

        if (this->is_first_step_)
        {
            derivative_output.setZero();
            this->is_first_step_ = false;
        }
        else
        {
            derivative_output =
                this->Kd_ * ((error - this->previous_error_) / this->Ts_);
        }

        const UVector integral_increment =
            (this->Ki_ * error) * this->Ts_;

        const UVector integral_candidate =
            this->integral_output_ + integral_increment;

        const UVector non_integral_output =
            proportional_output + derivative_output;

        const UVector unsat_candidate =
            non_integral_output + integral_candidate;

        /*
         * Clamping anti-windup:
         *
         * 如果控制量已经超过上限，并且积分增量仍然为正，则冻结该通道积分。
         * 如果控制量已经低于下限，并且积分增量仍然为负，则冻结该通道积分。
         * 否则接受积分更新。
         */
        for (int i = 0; i < Nu; ++i)
        {
            const bool saturated_high =
                (unsat_candidate(i, 0) > u_max(i, 0));

            const bool saturated_low =
                (unsat_candidate(i, 0) < u_min(i, 0));

            const bool integration_pushes_high =
                (integral_increment(i, 0) > static_cast<DataType>(0));

            const bool integration_pushes_low =
                (integral_increment(i, 0) < static_cast<DataType>(0));

            const bool should_freeze_integrator =
                (saturated_high && integration_pushes_high) ||
                (saturated_low && integration_pushes_low);

            if (!should_freeze_integrator)
            {
                this->integral_output_(i, 0) = integral_candidate(i, 0);
            }
        }

        const UVector u_unsat =
            non_integral_output + this->integral_output_;

        const UVector u_sat =
            u_unsat.cwiseMax(u_min).cwiseMin(u_max);

        this->previous_error_ = error;

        return u_sat;
    }

private:
    /** @brief 比例增益矩阵。 */
    GainMatrix Kp_{GainMatrix::Zero()};

    /** @brief 积分增益矩阵。 */
    GainMatrix Ki_{GainMatrix::Zero()};

    /** @brief 微分增益矩阵。 */
    GainMatrix Kd_{GainMatrix::Zero()};

    /** @brief 采样周期，单位 s。 */
    DataType Ts_{static_cast<DataType>(0.1)};

    /** @brief 当前积分项输出。 */
    UVector integral_output_{UVector::Zero()};

    /** @brief 上一采样周期误差，用于微分项计算。 */
    YVector previous_error_{YVector::Zero()};

    /** @brief 标记是否为第一次 Step()，用于抑制首次微分尖峰。 */
    bool is_first_step_{true};
};
