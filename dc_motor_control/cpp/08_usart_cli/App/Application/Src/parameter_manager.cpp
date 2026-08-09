#include "parameter_manager.hpp"

#include <cstring>


/**
 * @brief 绑定应用层实际使用的 PID 控制器。
 *
 * Notes
 * -----
 * ParameterManager 不拥有这些对象，只保存指针。对象生命周期由
 * DCMotorApp 管理，因此这里不做动态分配和释放。
 */
void ParameterManager::BindPidControllers(
    SisoPid* speed_pid,
    SisoPid* position_pid
)
{
    this->speed_pid_ = speed_pid;
    this->position_pid_ = position_pid;
}


/**
 * @brief 读取指定参数名对应的浮点参数。
 *
 * Notes
 * -----
 * 当前只管理 SISO PID 的三个增益，所以最终读取矩阵中的 (0, 0) 元素。
 */
bool ParameterManager::Get(const char* name, float* value) const
{
    if ((name == nullptr) || (value == nullptr))
    {
        return false;
    }

    PidTarget pid_target{};
    GainTarget gain_target{};
    if (!this->ResolveName(name, &pid_target, &gain_target))
    {
        return false;
    }

    const SisoPid* pid = this->SelectPid(pid_target);
    if (pid == nullptr)
    {
        return false;
    }

    switch (gain_target)
    {
    case GainTarget::Kp:
        *value = pid->GetKp()(0, 0);
        break;

    case GainTarget::Ki:
        *value = pid->GetKi()(0, 0);
        break;

    case GainTarget::Kd:
        *value = pid->GetKd()(0, 0);
        break;

    default:
        return false;
    }

    return true;
}


/**
 * @brief 设置指定参数名对应的浮点参数。
 *
 * Notes
 * -----
 * 当前 PID 控制器使用 1x1 增益矩阵，因此先把 value 写入 GainMatrix，
 * 再调用对应的 SetKp()/SetKi()/SetKd()。
 */
bool ParameterManager::Set(const char* name, const float value)
{
    if (name == nullptr)
    {
        return false;
    }

    PidTarget pid_target{};
    GainTarget gain_target{};
    if (!this->ResolveName(name, &pid_target, &gain_target))
    {
        return false;
    }

    SisoPid* pid = this->SelectPid(pid_target);
    if (pid == nullptr)
    {
        return false;
    }

    GainMatrix gain;
    gain << value;

    switch (gain_target)
    {
    case GainTarget::Kp:
        pid->SetKp(gain);
        break;

    case GainTarget::Ki:
        pid->SetKi(gain);
        break;

    case GainTarget::Kd:
        pid->SetKd(gain);
        break;

    default:
        return false;
    }

    return true;
}


/**
 * @brief 将 CLI 文本参数名解析成内部参数目标。
 *
 * Notes
 * -----
 * 为了输入方便，kp/ki/kd 默认映射到速度环；如果需要访问位置环，
 * 使用 position_kp、position_ki、position_kd。
 */
bool ParameterManager::ResolveName(
    const char* name,
    PidTarget* pid_target,
    GainTarget* gain_target
) const
{
    if ((name == nullptr) || (pid_target == nullptr) || (gain_target == nullptr))
    {
        return false;
    }

    if ((std::strcmp(name, "kp") == 0) ||
        (std::strcmp(name, "speed_kp") == 0))
    {
        *pid_target = PidTarget::Speed;
        *gain_target = GainTarget::Kp;
        return true;
    }

    if ((std::strcmp(name, "ki") == 0) ||
        (std::strcmp(name, "speed_ki") == 0))
    {
        *pid_target = PidTarget::Speed;
        *gain_target = GainTarget::Ki;
        return true;
    }

    if ((std::strcmp(name, "kd") == 0) ||
        (std::strcmp(name, "speed_kd") == 0))
    {
        *pid_target = PidTarget::Speed;
        *gain_target = GainTarget::Kd;
        return true;
    }

    if (std::strcmp(name, "position_kp") == 0)
    {
        *pid_target = PidTarget::Position;
        *gain_target = GainTarget::Kp;
        return true;
    }

    if (std::strcmp(name, "position_ki") == 0)
    {
        *pid_target = PidTarget::Position;
        *gain_target = GainTarget::Ki;
        return true;
    }

    if (std::strcmp(name, "position_kd") == 0)
    {
        *pid_target = PidTarget::Position;
        *gain_target = GainTarget::Kd;
        return true;
    }

    return false;
}


/**
 * @brief 选择可修改的 PID 控制器。
 */
ParameterManager::SisoPid* ParameterManager::SelectPid(const PidTarget target)
{
    return (target == PidTarget::Speed)
        ? this->speed_pid_
        : this->position_pid_;
}


/**
 * @brief 选择只读 PID 控制器。
 */
const ParameterManager::SisoPid* ParameterManager::SelectPid(
    const PidTarget target
) const
{
    return (target == PidTarget::Speed)
        ? this->speed_pid_
        : this->position_pid_;
}
