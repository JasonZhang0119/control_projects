#pragma once

#include "stm32f1xx_hal.h"
#include <cstdint>


/**
 * @brief 增量式编码器读取封装。
 *
 * Notes
 * -----
 * 当前用于 TIM Encoder Mode。
 * 例如 M1:
 * - PA6 -> TIM3_CH1
 * - PA7 -> TIM3_CH2
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
     *     Encoder Mode 定时器句柄。
     */
    explicit EncoderReader(TIM_HandleTypeDef* htim);

    /**
     * @brief 启动 Encoder Mode。
     */
    void Start();

    /**
     * @brief 清零 encoder counter。
     */
    void Reset();

    /**
     * @brief 读取当前 encoder count。
     *
     * Returns
     * -------
     * uint16_t
     *     当前 16-bit encoder count。
     */
    uint16_t GetCount() const;

    /**
     * @brief 根据当前 count 计算增量。
     *
     * Returns
     * -------
     * int16_t
     *     相对上一次调用的 count 增量。
     */
    int16_t GetDelta();

private:
    TIM_HandleTypeDef* htim_;
    uint16_t last_count_;
};