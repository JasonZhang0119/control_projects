#pragma once

#include "stm32f1xx_hal.h"

#include <cstdint>


/**
 * @brief 编码器采样结果。
 *
 * Notes
 * -----
 * 该结构体同时包含：
 * - 原始 16-bit timer counter；
 * - 单周期增量 delta_count；
 * - 累积连续位置 position_count；
 * - 位置换算结果；
 * - 速度换算结果。
 */
struct EncoderMeasurement
{
    /**
     * @brief TIMx 当前 16-bit counter 原始值。
     */
    uint16_t count;

    /**
     * @brief 当前采样周期内 encoder count 增量。
     *
     * Notes
     * -----
     * 该值通过 16-bit 回绕差分得到：
     *
     *     delta = static_cast<int16_t>(current_count - last_count)
     */
    int16_t delta_count;

    /**
     * @brief 累积连续位置，单位 count。
     *
     * Notes
     * -----
     * position_count 不会像 TIMx->CNT 一样在 0~65535 回绕。
     * 它通过持续累积 delta_count 得到：
     *
     *     position_count += delta_count
     */
    int32_t position_count;

    /**
     * @brief 连续位置，单位 revolution。
     */
    float position_revolution;

    /**
     * @brief 连续位置，单位 rad。
     */
    float position_rad;

    /**
     * @brief 速度，单位 counts/s。
     */
    float counts_per_second;

    /**
     * @brief 速度，单位 rpm。
     */
    float rpm;

    /**
     * @brief 速度，单位 rad/s。
     */
    float rad_per_second;
};


/**
 * @brief TIM Encoder Mode 编码器读取与位置/速度换算封装。
 *
 * Notes
 * -----
 * 该类不负责 CubeMX 硬件初始化。
 *
 * CubeMX 负责：
 * - TIMx Encoder Mode 配置；
 * - GPIO 配置；
 * - RCC clock enable；
 * - NVIC 配置，如果需要。
 *
 * EncoderReader 负责：
 * - 启动 TIM Encoder Mode；
 * - 清零 counter；
 * - 读取 count；
 * - 计算 delta_count；
 * - 累积连续位置；
 * - 换算位置和速度。
 */
class EncoderReader
{
public:
    /**
     * @brief 构造编码器读取器。
     *
     * Parameters
     * ----------
     * htim : TIM_HandleTypeDef*
     *     已经由 MX_TIMx_Init() 初始化好的 timer handle。
     * counts_per_revolution : float
     *     输出轴每转对应的 encoder count 数。
     *
     * Notes
     * -----
     * counts_per_revolution 应该是输出轴每转的总 count。
     * 如果 encoder 使用 x4 解码，并且电机带减速箱，则通常：
     *
     *     counts_per_revolution = encoder_ppr * 4 * gear_ratio
     */
    EncoderReader(
        TIM_HandleTypeDef* htim,
        float counts_per_revolution
    );

    /**
     * @brief 启动 TIM Encoder Mode。
     *
     * Returns
     * -------
     * None
     */
    void Start();

    /**
     * @brief 停止 TIM Encoder Mode。
     *
     * Returns
     * -------
     * None
     */
    void Stop();

    /**
     * @brief 清零硬件 counter 和软件连续位置。
     *
     * Returns
     * -------
     * None
     */
    void Reset();

    /**
     * @brief 清零硬件 counter，并将软件连续位置设置为指定值。
     *
     * Parameters
     * ----------
     * position_count : int32_t
     *     重置后的软件连续位置，单位 count。
     *
     * Returns
     * -------
     * None
     */
    void Reset(int32_t position_count);

    /**
     * @brief 读取当前 TIMx counter。
     *
     * Returns
     * -------
     * uint16_t
     *     当前 16-bit counter 原始值。
     */
    uint16_t GetCount() const;

    /**
     * @brief 获取当前软件连续位置。
     *
     * Returns
     * -------
     * int32_t
     *     当前连续位置，单位 count。
     */
    int32_t GetPositionCount() const;

    /**
     * @brief 手动设置当前软件连续位置。
     *
     * Parameters
     * ----------
     * position_count : int32_t
     *     新的软件连续位置，单位 count。
     *
     * Returns
     * -------
     * None
     *
     * Notes
     * -----
     * 该函数不会修改 TIMx->CNT，只修改软件累积位置。
     * 可以用于回零、设定当前位置为某个参考点。
     */
    void SetPositionCount(int32_t position_count);

    /**
     * @brief 获取当前位置，单位 revolution。
     *
     * Returns
     * -------
     * float
     *     当前连续位置，单位 revolution。
     */
    float GetPositionRevolution() const;

    /**
     * @brief 获取当前位置，单位 rad。
     *
     * Returns
     * -------
     * float
     *     当前连续位置，单位 rad。
     */
    float GetPositionRad() const;

    /**
     * @brief 读取当前 encoder 增量。
     *
     * Returns
     * -------
     * int16_t
     *     相对上一次调用 GetDelta() 或 Sample() 的 count 增量。
     *
     * Notes
     * -----
     * 该函数会更新 last_count_ 和 position_count_。
     * 如果主控制循环已经调用 Sample()，同一周期内不要再调用 GetDelta()，
     * 否则会导致 delta 被重复消费。
     */
    int16_t GetDelta();

    /**
     * @brief 采样编码器并计算位置与速度。
     *
     * Parameters
     * ----------
     * sample_time_s : float
     *     采样周期，单位 s。
     *
     * Returns
     * -------
     * EncoderMeasurement
     *     当前编码器采样结果。
     *
     * Notes
     * -----
     * 该函数是控制循环中推荐使用的唯一读取接口。
     * 它会：
     * - 读取当前 counter；
     * - 计算 delta_count；
     * - 更新 last_count_；
     * - 更新 position_count_；
     * - 换算位置；
     * - 换算速度。
     */
    EncoderMeasurement Sample(float sample_time_s);

private:
    /** @brief 编码器定时器句柄。 */
    TIM_HandleTypeDef* htim_;

    /** @brief 上一次读取到的 16-bit 硬件计数值。 */
    uint16_t last_count_;

    /** @brief 软件累积的连续位置，单位 count。 */
    int32_t position_count_;

    /** @brief 输出轴每转对应的编码器 count 数。 */
    float counts_per_revolution_;
};
