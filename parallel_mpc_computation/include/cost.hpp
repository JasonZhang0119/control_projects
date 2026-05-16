#pragma once

#include "state.hpp"

/**
 * @brief 二次型 stage cost。
 */
class QuadraticCost
{
public:
    QuadraticCost(double q_pos, double q_vel, double r_u);

    /**
     * @brief 计算单个时刻的 stage cost。
     *
     * ----
     * 实现：
     * J_k = q_pos * position^2
     *     + q_vel * velocity^2
     *     + r_u   * u^2
     */
    double StageCost(const State& x, double u) const;

private:
    double q_pos_{};
    double q_vel_{};
    double r_u_{};
};