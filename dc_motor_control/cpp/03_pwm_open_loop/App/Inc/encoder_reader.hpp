#pragma once

#include "stm32f1xx_hal.h"

#include <cstdint>


/**
 * @brief 编码器采样结果。
 */
struct EncoderMeasurement
{
    uint16_t count;
    int16_t delta_count;

    float counts_per_second;
    float rpm;
    float rad_per_second;
};


/**
 * @brief TIM Encoder Mode 编码器读取与速度换算封装。
 *
 * Notes
 * -----
 * 该类不负责 CubeMX 硬件初始化。
 * 它只负责：
 * - 启动 encoder；
 * - 清零 counter；
 * - 读取 count；
 * - 计算 delta_count；
 * - 将 delta_count 换算为 counts/s、rpm、rad/s。
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
     *     已由 MX_TIMx_Init() 初始化好的 Timer handle。
     * counts_per_revolution : float
     *     输出轴每转对应的 encoder count 数。
     */
    EncoderReader(
        TIM_HandleTypeDef* htim,
        float counts_per_revolution
    );

    void Start();
    void Reset();

    uint16_t GetCount() const;
    int16_t GetDelta();

    /**
     * @brief 采样编码器并计算速度。
     *
     * Parameters
     * ----------
     * sample_time_s : float
     *     采样周期，单位 s。
     *
     * Returns
     * -------
     * EncoderMeasurement
     *     本周期编码器采样和速度换算结果。
     */
    EncoderMeasurement Sample(float sample_time_s);

private:
    TIM_HandleTypeDef* htim_;
    uint16_t last_count_;
    float counts_per_revolution_;
};