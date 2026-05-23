#include "button_manager.hpp"

#include "stm32f1xx_hal.h"


/**
 * @brief 构造按键管理器。
 */
ButtonManager::ButtonManager(
    GPIO_TypeDef* speed_down_port,
    const uint16_t speed_down_pin,
    GPIO_TypeDef* speed_up_port,
    const uint16_t speed_up_pin
)
    : speed_down_port_(speed_down_port),
      speed_down_pin_(speed_down_pin),
      speed_up_port_(speed_up_port),
      speed_up_pin_(speed_up_pin),
      pending_event_(ButtonEvent::None),
      last_speed_down_tick_ms_(0U),
      last_speed_up_tick_ms_(0U)
{
}


/**
 * @brief 判断当前按键事件是否通过消抖检查。
 */
bool ButtonManager::IsDebounced(
    const uint32_t now_tick_ms,
    const uint32_t last_tick_ms
) const
{

    if ((now_tick_ms - last_tick_ms) > kDebounceTimeMs){
        return true;
    }else{
        return false;
    }
}


/**
 * @brief GPIO EXTI 中断回调入口。
 */
void ButtonManager::OnExtiInterrupt(const uint16_t gpio_pin)
{
    if (gpio_pin == this->speed_up_pin_) {
        this->pending_event_ = ButtonEvent::SpeedUp;
        return;
    }

    if (gpio_pin == this->speed_down_pin_) {
        this->pending_event_ = ButtonEvent::SpeedDown;
        return;
    }
}

/**
 * @brief 消费 pending event。
 */
ButtonEvent ButtonManager::ConsumeEvent()
{
    __disable_irq();

    const ButtonEvent event = this->pending_event_;
    this->pending_event_ = ButtonEvent::None;

    __enable_irq();

    if (event == ButtonEvent::SpeedUp) {
        if (HAL_GPIO_ReadPin(this->speed_up_port_, this->speed_up_pin_) == GPIO_PIN_RESET) {
            return ButtonEvent::SpeedUp;
        }
    }

    if (event == ButtonEvent::SpeedDown) {
        if (HAL_GPIO_ReadPin(this->speed_down_port_, this->speed_down_pin_) == GPIO_PIN_RESET) {
            return ButtonEvent::SpeedDown;
        }
    }

    return ButtonEvent::None;
}


/**
 * @brief 重置按键管理器状态。
 */
void ButtonManager::Reset()
{
    this->pending_event_ = ButtonEvent::None;
    this->last_speed_down_tick_ms_ = 0;
    this->last_speed_up_tick_ms_ = 0;
}