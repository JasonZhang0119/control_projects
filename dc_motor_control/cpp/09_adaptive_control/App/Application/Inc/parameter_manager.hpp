#pragma once

#include "pid_controller.hpp"


/**
 * @brief 应用层参数管理器。
 *
 * Notes
 * -----
 * 该类负责把文本参数名映射到应用内部对象，例如 PID 控制器。
 * UartCli 只需要调用 Get() / Set()，不需要知道参数实际存放在哪个模块。
 */
class ParameterManager
{
public:
    using SisoPid = PidController<float, 1, 1>;
    using GainMatrix = SisoPid::GainMatrix;

    /**
     * @brief 绑定当前应用使用的 PID 控制器。
     *
     * Parameters
     * ----------
     * speed_pid : SisoPid*
     *     速度环 PID 控制器。
     * position_pid : SisoPid*
     *     位置环 PID 控制器。
     */
    void BindPidControllers(
        SisoPid* speed_pid,
        SisoPid* position_pid
    );

    /**
     * @brief 根据参数名读取参数值。
     *
     * Parameters
     * ----------
     * name : const char*
     *     参数名，例如 kp、speed_kp、position_kp。
     * value : float*
     *     输出参数值。
     *
     * Returns
     * -------
     * bool
     *     参数存在且读取成功返回 true。
     */
    bool Get(const char* name, float* value) const;

    /**
     * @brief 根据参数名设置参数值。
     *
     * Parameters
     * ----------
     * name : const char*
     *     参数名，例如 kp、speed_kp、position_kp。
     * value : float
     *     新参数值。
     *
     * Returns
     * -------
     * bool
     *     参数存在且设置成功返回 true。
     */
    bool Set(const char* name, float value);

private:
    /** @brief 参数所属的 PID 控制器。 */
    enum class PidTarget
    {
        Speed,
        Position
    };

    /** @brief 参数对应的 PID 增益项。 */
    enum class GainTarget
    {
        Kp,
        Ki,
        Kd
    };

    /** @brief 速度环 PID 控制器，不由 ParameterManager 拥有。 */
    SisoPid* speed_pid_{nullptr};

    /** @brief 位置环 PID 控制器，不由 ParameterManager 拥有。 */
    SisoPid* position_pid_{nullptr};

    /**
     * @brief 将文本参数名解析为 PID 对象和增益项。
     */
    bool ResolveName(
        const char* name,
        PidTarget* pid_target,
        GainTarget* gain_target
    ) const;

    /**
     * @brief 根据目标选择可写 PID 控制器。
     */
    SisoPid* SelectPid(PidTarget target);

    /**
     * @brief 根据目标选择只读 PID 控制器。
     */
    const SisoPid* SelectPid(PidTarget target) const;
};
