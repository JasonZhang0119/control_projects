#pragma once

#include "state.hpp"

/**
 * @brief 一次 rollout evaluation 的结果。
 */
struct RolloutResult
{
    int trajectory_id{};
    double total_cost{};
    State final_state{};
};