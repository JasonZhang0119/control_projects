#pragma once

#include "trajectory.hpp"
#include "rollout_result.hpp"
#include "plant.hpp"
#include "cost.hpp"

/**
 * @brief 负责评估一条候选控制序列的总代价。
 */
class RolloutEvaluator
{
public:
    RolloutEvaluator(
        DoubleIntegratorPlant plant,
        QuadraticCost cost);

    /**
     * @brief 对一条 trajectory 做完整 rollout。
     */
    RolloutResult Evaluate(
        const Trajectory& trajectory,
        const State& initial_state) const;

private:
    DoubleIntegratorPlant plant_;
    QuadraticCost cost_;
};