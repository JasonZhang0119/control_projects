#pragma once

#include <Eigen/Dense>

/**
 * @brief 输出反馈控制模块基类。
 *
 * Parameters
 * ----------
 * DataType : typename
 *     标量类型，例如 float。
 * Ny : int
 *     输出维度。
 * Nu : int
 *     输入维度。
 *
 * Notes
 * -----
 * - 该类用于统一输出反馈模块接口。
 * - 在 STM32 上，当前主要用于 signal generator / controller 的接口抽象。
 * - 高频控制内环中如果不需要多态，可以后续去掉 virtual 以减少开销。
 */
template<typename DataType, int Ny, int Nu>
class OutputFeedbackControllerModule
{
public:
    using YVector = Eigen::Matrix<DataType, Ny, 1>;
    using UVector = Eigen::Matrix<DataType, Nu, 1>;

    /**
     * @brief 虚析构函数，保证通过基类指针释放派生类时行为正确。
     */
    virtual ~OutputFeedbackControllerModule() = default;

    /**
     * @brief 执行一步输出反馈控制。
     *
     * Parameters
     * ----------
     * y_ref : const Eigen::Ref<const YVector>&
     *     参考输出。
     * y : const Eigen::Ref<const YVector>&
     *     当前输出。
     * u_min : const Eigen::Ref<const UVector>&
     *     控制量下限。
     * u_max : const Eigen::Ref<const UVector>&
     *     控制量上限。
     *
     * Returns
     * -------
     * UVector
     *     控制输出。
     */
    virtual UVector Step(
        const Eigen::Ref<const YVector>& y_ref,
        const Eigen::Ref<const YVector>& y,
        const Eigen::Ref<const UVector>& u_min,
        const Eigen::Ref<const UVector>& u_max
    ) = 0;

    /**
     * @brief 重置模块内部状态。
     */
    virtual void Reset() {}

    /**
     * @brief 初始化模块。
     */
    virtual void Initialize() {}

    /**
     * @brief 终止模块并释放或清空运行状态。
     */
    virtual void Terminate() {}
};
