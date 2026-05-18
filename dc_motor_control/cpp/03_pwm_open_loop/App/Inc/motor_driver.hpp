#pragma once

#include "stm32f1xx_hal.h"
#include <cstdint>


enum class MotorDirection : int8_t
{
    Reverse = -1,
    Stop = 0,
    Forward = 1
};


/**
 * @brief DC 电机 PWM + 方向驱动封装。
 *
 * Notes
 * -----
 * 该类封装一个 TB6612 通道：
 * - 一个 PWM 定时器通道；
 * - 两个方向 GPIO；
 * - 使用 duty_permille 表示占空比。
 */
class MotorDriver
{
public:
    /**
     * @brief 构造电机驱动对象。
     *
     * Parameters
     * ----------
     * htim : TIM_HandleTypeDef*
     *     PWM 定时器句柄。
     * channel : uint32_t
     *     PWM 通道，例如 TIM_CHANNEL_1。
     * in1_port : GPIO_TypeDef*
     *     方向脚 IN1 的 GPIO port。
     * in1_pin : uint16_t
     *     方向脚 IN1 的 GPIO pin。
     * in2_port : GPIO_TypeDef*
     *     方向脚 IN2 的 GPIO port。
     * in2_pin : uint16_t
     *     方向脚 IN2 的 GPIO pin。
     */
    MotorDriver(
        TIM_HandleTypeDef* htim,
        uint32_t channel,
        GPIO_TypeDef* in1_port,
        uint16_t in1_pin,
        GPIO_TypeDef* in2_port,
        uint16_t in2_pin
    );

    /**
     * @brief 启动 PWM 输出。
     */
    void Start();

    /**
     * @brief 停止电机。
     */
    void Stop();

    /**
     * @brief 设置电机方向。
     *
     * Parameters
     * ----------
     * direction : MotorDirection
     *     电机方向。
     */
    void SetDirection(MotorDirection direction);

    /**
     * @brief 设置 PWM 占空比。
     *
     * Parameters
     * ----------
     * duty_permille : uint32_t
     *     占空比千分数，范围 [0, 1000]。
     */
    void SetDutyPermille(uint32_t duty_permille);

    /**
     * @brief 设置带方向的电机命令。
     *
     * Parameters
     * ----------
     * direction : MotorDirection
     *     电机方向。
     * duty_permille : uint32_t
     *     占空比千分数。
     */
    void SetCommand(MotorDirection direction, uint32_t duty_permille);

private:
    static constexpr uint32_t kMaxDutyPermille = 1000U;

    TIM_HandleTypeDef* htim_;
    uint32_t channel_;

    GPIO_TypeDef* in1_port_;
    uint16_t in1_pin_;

    GPIO_TypeDef* in2_port_;
    uint16_t in2_pin_;

    uint32_t DutyPermilleToCompare(uint32_t duty_permille) const;
};