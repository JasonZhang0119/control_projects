#include "rollout_evaluator.hpp"

RolloutEvaluator::RolloutEvaluator(
    DoubleIntegratorPlant plant,
    QuadraticCost cost)
    : plant_{plant},
      cost_{cost}
{
}

RolloutResult RolloutEvaluator::Evaluate(
    const Trajectory& trajectory,
    const State& initial_state) const
{
    // 1. 初始化当前状态 x = initial_state。
    // 2. 初始化 total_cost = 0.0。
    // 3. 遍历 trajectory.u_sequence。
    // 4. 对每一个 u：
    //    - 更新状态 x。
    //    - 累加 stage cost。
    // 5. 构造并返回 RolloutResult。

    double cost = 0;

    State temp_state{initial_state};
    
    for (auto iter_u = trajectory.u_sequence.begin(); 
              iter_u < trajectory.u_sequence.end(); 
              iter_u++){

        double temp_u = (*iter_u);
        cost += this->cost_.StageCost(temp_state, temp_u);
        temp_state = this->plant_.Step(temp_state, temp_u);

    }

    RolloutResult result{trajectory.id, cost, temp_state};

    return result;
}