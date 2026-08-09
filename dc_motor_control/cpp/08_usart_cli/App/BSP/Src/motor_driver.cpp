#include "motor_driver.hpp"

#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"

#include <cstdint>


/**
 * @brief 构造 DC 电机驱动对象。
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
MotorDriver::MotorDriver(
    TIM_HandleTypeDef* htim,
    uint32_t channel,
    GPIO_TypeDef* in1_port,
    uint16_t in1_pin,
    GPIO_TypeDef* in2_port,
    uint16_t in2_pin
)
    : htim_(htim),
      channel_(channel),
      in1_port_(in1_port),
      in1_pin_(in1_pin),
      in2_port_(in2_port),
      in2_pin_(in2_pin)
{
}


/**
 * @brief 启动 PWM 输出。
 *
 * Notes
 * -----
 * 该函数只启动 PWM 外设输出，不设置方向和占空比。
 */
void MotorDriver::Start()
{
    HAL_TIM_PWM_Start(this->htim_, this->channel_);
}


/**
 * @brief 停止电机。
 *
 * Notes
 * -----
 * 停止策略为：
 * - PWM duty 置 0；
 * - IN1/IN2 均置低，进入 coast stop。
 */
void MotorDriver::Stop()
{
    this->SetDutyPermille(0U);
    this->SetDirection(MotorDirection::Stop);
}


/**
 * @brief 设置电机方向。
 *
 * Parameters
 * ----------
 * direction : MotorDirection
 *     电机方向命令。
 */
void MotorDriver::SetDirection(const MotorDirection direction)
{
    switch (direction)
    {
    case MotorDirection::Forward:
        HAL_GPIO_WritePin(this->in1_port_, this->in1_pin_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(this->in2_port_, this->in2_pin_, GPIO_PIN_RESET);
        break;

    case MotorDirection::Reverse:
        HAL_GPIO_WritePin(this->in1_port_, this->in1_pin_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(this->in2_port_, this->in2_pin_, GPIO_PIN_SET);
        break;

    case MotorDirection::Stop:
    default:
        HAL_GPIO_WritePin(this->in1_port_, this->in1_pin_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(this->in2_port_, this->in2_pin_, GPIO_PIN_RESET);
        break;
    }
}


/**
 * @brief 设置 PWM 占空比。
 *
 * Parameters
 * ----------
 * duty_permille : uint32_t
 *     占空比千分数，范围建议为 [0, 1000]。
 */
void MotorDriver::SetDutyPermille(const uint32_t duty_permille)
{
    const uint32_t compare =
        this->DutyPermilleToCompare(duty_permille);

    __HAL_TIM_SET_COMPARE(
        this->htim_,
        this->channel_,
        compare
    );
}


/**
 * @brief 设置带方向的电机命令。
 *
 * Parameters
 * ----------
 * direction : MotorDirection
 *     电机方向命令。
 * duty_permille : uint32_t
 *     占空比千分数。
 *
 * Notes
 * -----
 * 如果 direction 为 Stop 或 duty 为 0，则强制停止。
 */
void MotorDriver::SetCommand(
    const MotorDirection direction,
    const uint32_t duty_permille
)
{
    if ((direction == MotorDirection::Stop) || (duty_permille == 0U))
    {
        this->Stop();
        return;
    }

    this->SetDirection(direction);
    this->SetDutyPermille(duty_permille);
}


/**
 * @brief 将 duty_permille 转换为 PWM compare 值。
 *
 * Parameters
 * ----------
 * duty_permille : uint32_t
 *     占空比千分数。
 *
 * Returns
 * -------
 * uint32_t
 *     写入 CCR 的 compare 值。
 */
uint32_t MotorDriver::DutyPermilleToCompare(
    const uint32_t duty_permille
) const
{
    const uint32_t arr =
        static_cast<uint32_t>(__HAL_TIM_GET_AUTORELOAD(this->htim_));

    const uint32_t clipped_duty =
        (duty_permille > kMaxDutyPermille)
            ? kMaxDutyPermille
            : duty_permille;

    uint32_t compare =
        ((arr + 1U) * clipped_duty) / kMaxDutyPermille;

    if (compare > arr)
    {
        compare = arr;
    }

    return compare;
}