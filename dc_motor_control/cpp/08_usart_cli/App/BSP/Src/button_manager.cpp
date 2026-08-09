#include "button_manager.hpp"

#include "stm32f1xx_hal.h"


/**
 * @brief 构造按键管理器。
 */
ButtonManager::ButtonManager(
    GPIO_TypeDef* down_port,
    const uint16_t down_pin,
    GPIO_TypeDef* up_port,
    const uint16_t up_pin,
    const GPIO_PinState down_active_level,
    const GPIO_PinState up_active_level
)
    : down_port_(down_port),
      down_pin_(down_pin),
      up_port_(up_port),
      up_pin_(up_pin),
      down_active_level_(down_active_level),
      up_active_level_(up_active_level),
      pending_event_(ButtonEvent::None),
      down_debouncing_(false),
      up_debouncing_(false),
      down_debounce_start_tick_ms_(0U),
      up_debounce_start_tick_ms_(0U)
{
}


/**
 * @brief 判断当前按键事件是否通过消抖检查。
 */
bool ButtonManager::IsDebounceExpired(
    const uint32_t now_tick_ms,
    const uint32_t start_tick_ms
) const
{
    return ((now_tick_ms - start_tick_ms) >= kDebounceTimeMs);
}


bool ButtonManager::IsDownPressed() const
{
    return HAL_GPIO_ReadPin(this->down_port_, this->down_pin_) ==
           this->down_active_level_;
}


bool ButtonManager::IsUpPressed() const
{
    return HAL_GPIO_ReadPin(this->up_port_, this->up_pin_) ==
           this->up_active_level_;
}


/**
 * @brief GPIO EXTI 中断回调入口。
 */
void ButtonManager::OnExtiInterrupt(const uint16_t gpio_pin)
{
    const uint32_t now_tick_ms = HAL_GetTick();

    if (gpio_pin == this->up_pin_){
        this->up_debouncing_ = true;
        this->up_debounce_start_tick_ms_ = now_tick_ms;
        return;
    }

    if (gpio_pin == this->down_pin_){
        this->down_debouncing_ = true;
        this->down_debounce_start_tick_ms_ = now_tick_ms;
        return;
    }
}


/**
 * @brief 主循环中的按键消抖确认。
 */
void ButtonManager::Update()
{
    const uint32_t now_tick_ms = HAL_GetTick();

    if (this->up_debouncing_ &&
        this->IsDebounceExpired(now_tick_ms, this->up_debounce_start_tick_ms_))
    {
        this->up_debouncing_ = false;

        if (this->IsUpPressed())
        {
            this->pending_event_ = ButtonEvent::Up;
        }
    }

    if (this->down_debouncing_ &&
        this->IsDebounceExpired(now_tick_ms, this->down_debounce_start_tick_ms_))
    {
        this->down_debouncing_ = false;

        if (this->IsDownPressed())
        {
            this->pending_event_ = ButtonEvent::Down;
        }
    }
}


/**
 * @brief 消费 pending event。
 */
ButtonEvent ButtonManager::ConsumeEvent()
{
    ButtonEvent event = this->pending_event_;
    this->pending_event_ = ButtonEvent::None;

    return event;

}


/**
 * @brief 重置按键管理器状态。
 */
void ButtonManager::Reset()
{
    this->pending_event_ = ButtonEvent::None;
    this->down_debouncing_ = false;
    this->up_debouncing_ = false;
    this->down_debounce_start_tick_ms_ = 0;
    this->up_debounce_start_tick_ms_ = 0;
}
