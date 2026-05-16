
#include "plant.hpp"

DoubleIntegratorPlant::DoubleIntegratorPlant(double dt)
    : dt_{dt}
{
}

State DoubleIntegratorPlant::Step(const State& x, double u) const
{
    // 1. 构造 State x_next。
    // 2. 根据 double integrator 离散模型更新 position。
    // 3. 根据 double integrator 离散模型更新 velocity。
    // 4. 返回 x_next。

    State x_next{0, 0};
    
    x_next.velocity = x.velocity + u * this->dt_;
    x_next.position = x.position + x.velocity * this->dt_;

    return x_next;
}