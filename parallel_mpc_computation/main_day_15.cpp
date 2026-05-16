#include <future>
#include <iostream>
#include <random>
#include <vector>
#include <chrono>

#include "state.hpp"
#include "trajectory.hpp"
#include "plant.hpp"
#include "cost.hpp"
#include "rollout_result.hpp"
#include "rollout_evaluator.hpp"

/**
 * @brief 随机生成多条候选控制序列。
 *
 * 1. 创建随机数生成器。
 * 2. 为每条 trajectory 分配 id。
 * 3. 为每条 trajectory 生成 horizon 个控制输入。
 * 4. 返回 trajectories。
 **/
std::vector<Trajectory> GenerateRandomTrajectories(
    int num_trajectories,
    int horizon,
    double u_min,
    double u_max)
{
    // 获取硬件随机数作为种子
    std::random_device rd;
    // 用种子初始化 MT19937 引擎
     std::mt19937 gen(rd());
    // 初始化随机数分布
    std::uniform_real_distribution<double> distrib(u_min, u_max);

    std::vector<Trajectory> trajectories(num_trajectories);

    for (int i = 0; i < num_trajectories; i ++){

        Trajectory temp_trajectory{};
        std::vector<double> u_trajectory(horizon);
        temp_trajectory.id = i;

        for (int j = 0; j < horizon; j++){
            u_trajectory[j] = distrib(gen);
        }
        temp_trajectory.u_sequence = u_trajectory;
        trajectories[i] = temp_trajectory;
    }
    return trajectories;
}

/**
 * @brief 串行评估所有候选 trajectory，并找到最优结果。
 *
 * 1. 遍历所有 trajectories。
 * 2. 调用 evaluator.Evaluate(...)。
 * 3. 比较 total_cost。
 * 4. 返回最优 RolloutResult。
 */
RolloutResult FindBestSerial(
    const std::vector<Trajectory>& trajectories,
    const RolloutEvaluator& evaluator,
    const State& x0)
{

    RolloutResult result{};
    RolloutResult temp_result{};
    for (auto iter_traj = trajectories.begin(); 
              iter_traj < trajectories.end();
              iter_traj ++){
        temp_result = evaluator.Evaluate((*iter_traj), x0);
        if ((*iter_traj).id == 0){
            result =  temp_result;
        }else{
            if (result.total_cost > temp_result.total_cost){
                result = temp_result;
            }
        }
    }
    return result;
}

/**
 * @brief 使用 std::async 并行评估所有候选 trajectory，并找到最优结果。
 *
 * 1. 创建 std::vector<std::future<RolloutResult>>。
 * 2. 为每条 trajectory 提交 std::async task。
 * 3. 每个 task 内部调用 evaluator.Evaluate(...)。
 * 4. 主线程最后逐个 future.get()。
 * 5. 对结果做 reduction，找出最小 cost。
 */
RolloutResult FindBestAsync(
    const std::vector<Trajectory>& trajectories,
    const RolloutEvaluator& evaluator,
    const State& x0)
{
    std::vector<std::future<RolloutResult>> result_vector(trajectories.size());

    for (auto iter_traj = trajectories.begin(); 
              iter_traj < trajectories.end();
              iter_traj ++){
        result_vector[(*iter_traj).id] = std::async(std::launch::async, 
                                             &RolloutEvaluator::Evaluate, 
                                             &evaluator, 
                                             std::cref(*iter_traj),
                                             std::cref(x0));        
    }

    RolloutResult result = result_vector[0].get();
    double cost = result.total_cost;
    for (int i = 1; i < trajectories.size(); i++){
        RolloutResult temp_result = result_vector[i].get();
        double temp_cost = temp_result.total_cost;
        if (temp_cost < cost){
            cost = temp_cost;
            result = temp_result;
        }
    }
    
    return result;
}

int main()
{
    // 1. 设置参数
    const double dt = 0.01;
    const int horizon = 5000000000;
    const int num_trajectories = 20;
    const double u_min = -2.0;
    const double u_max = 2.0;
    const State x0{1.0, 0.0};
    const double q_pos = 10.0;
    const double q_vel = 1.0;
    const double r_u = 0.1;

    // 2. 构造 plant、cost、evaluator
    DoubleIntegratorPlant plant(dt);
    QuadraticCost cost(q_pos, q_vel, r_u);
    RolloutEvaluator evaluator(plant, cost);

    // 3. 生成随机 trajectories
    const std::vector<Trajectory> trajectories =
        GenerateRandomTrajectories(num_trajectories, horizon, u_min, u_max);

    // 4. 运行串行版本，并记录耗时
    const auto serial_start = std::chrono::steady_clock::now();
    const RolloutResult serial_best = FindBestSerial(trajectories, evaluator, x0);
    const auto serial_end = std::chrono::steady_clock::now();
    const auto serial_elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(serial_end - serial_start).count();

    // 5. 运行 async 版本，并记录耗时
    const auto async_start = std::chrono::steady_clock::now();
    const RolloutResult async_best = FindBestAsync(trajectories, evaluator, x0);
    const auto async_end = std::chrono::steady_clock::now();
    const auto async_elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(async_end - async_start).count();

    // 6. 打印结果
    std::cout << "Serial best trajectory id: " << serial_best.trajectory_id << '\n';
    std::cout << "Serial best cost: " << serial_best.total_cost << '\n';
    std::cout << "Async best trajectory id: " << async_best.trajectory_id << '\n';
    std::cout << "Async best cost: " << async_best.total_cost << '\n';
    std::cout << "Serial elapsed time (ms): " << serial_elapsed_ms << '\n';
    std::cout << "Async elapsed time (ms): " << async_elapsed_ms << '\n';

    return 0;
}
