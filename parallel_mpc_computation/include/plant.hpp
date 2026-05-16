#pragma once

#include "state.hpp"

/**
 * @brief Double integrator 离散系统模型。
 */
class DoubleIntegratorPlant
{
public:
    explicit DoubleIntegratorPlant(double dt);

    /**
     * @brief 执行一步离散状态更新。
     *
     **/
    State Step(const State& x, double u) const;

private:
    double dt_{};
};