#pragma once

#include <vector>

/**
 * @brief 一条候选控制序列。
 *
 * Notes
 * -----
 * - 在 Random Shooting / Sampling MPC 中，一条 trajectory 可以理解为
 *   一组候选控制输入序列。
 */
struct Trajectory
{
    int id{};
    std::vector<double> u_sequence;
};