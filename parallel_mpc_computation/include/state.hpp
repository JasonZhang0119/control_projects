#pragma once

/**
 * @brief Double integrator 的系统状态。
 *
 * Notes
 * -----
 * - position 表示位置状态 x1。
 * - velocity 表示速度状态 x2。
 */
struct State
{
    double position{};
    double velocity{};
};