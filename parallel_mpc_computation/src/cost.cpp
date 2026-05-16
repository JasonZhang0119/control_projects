#include "cost.hpp"

QuadraticCost::QuadraticCost(double q_pos, double q_vel, double r_u)
    : q_pos_{q_pos},
      q_vel_{q_vel},
      r_u_{r_u}
{
}

double QuadraticCost::StageCost(const State& x, double u) const
{
    // 1. 计算 position penalty。
    // 2. 计算 velocity penalty。
    // 3. 计算 control penalty。
    // 4. 返回三者之和。

    double stage_cost = x.position * this -> q_pos_ * x.position
                            + x.velocity * this -> q_vel_ * x.velocity
                            + u * this -> r_u_ * u;

    return stage_cost;
}