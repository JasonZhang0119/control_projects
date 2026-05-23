#pragma once

#include "stm32f1xx_hal.h"

#include <cstdint>


/**
 * @brief 按键事件类型。
 *
 * Notes
 * -----
 * 当前阶段只使用 SpeedUp 和 SpeedDown。
 * PositionUp / PositionDown 先预留，后续做位置环时再接入。
 */
enum class ButtonEvent : uint8_t
{
    None = 0,
    SpeedUp,
    SpeedDown,
    PositionUp,
    PositionDown
};


/**
 * @brief 按键中断事件管理器。
 *
 * Notes
 * -----
 * 该类不负责 CubeMX 的 GPIO/EXTI 初始化。
 *
 * CubeMX 负责：
 * - GPIO 配置为 EXTI；
 * - Pull-up / Pull-down；
 * - Falling edge / Rising edge；
 * - NVIC enable。
 *
 * ButtonManager 负责：
 * - 在 EXTI callback 中识别按键；
 * - 软件消抖；
 * - 保存 pending event；
 * - 在主循环中消费 event。
 */
class ButtonManager
{
public:
    /**
     * @brief 构造按键管理器。
     *
     * Parameters
     * ----------
     * speed_down_port : GPIO_TypeDef*
     *     减速按键 GPIO port。
     * speed_down_pin : uint16_t
     *     减速按键 GPIO pin。
     * speed_up_port : GPIO_TypeDef*
     *     加速按键 GPIO port。
     * speed_up_pin : uint16_t
     *     加速按键 GPIO pin。
     */
    ButtonManager(
        GPIO_TypeDef* speed_down_port,
        uint16_t speed_down_pin,
        GPIO_TypeDef* speed_up_port,
        uint16_t speed_up_pin
    );

    /**
     * @brief GPIO EXTI 中断回调入口。
     *
     * Parameters
     * ----------
     * gpio_pin : uint16_t
     *     HAL_GPIO_EXTI_Callback() 传入的 GPIO_Pin。
     *
     * Returns
     * -------
     * None
     *
     * Notes
     * -----
     * 该函数会在中断上下文中被调用。
     * 因此内部不要做复杂计算、不要 printf、不要 HAL_UART_Transmit。
     */
    void OnExtiInterrupt(uint16_t gpio_pin);

    /**
     * @brief 消费一个按键事件。
     *
     * Returns
     * -------
     * ButtonEvent
     *     当前 pending event。
     *     如果没有事件，返回 ButtonEvent::None。
     *
     * Notes
     * -----
     * 主循环调用该函数后，pending event 会被清空。
     */
    ButtonEvent ConsumeEvent();

    /**
     * @brief 清空所有 pending event 和消抖状态。
     *
     * Returns
     * -------
     * None
     */
    void Reset();

private:
    GPIO_TypeDef* speed_down_port_;
    uint16_t speed_down_pin_;

    GPIO_TypeDef* speed_up_port_;
    uint16_t speed_up_pin_;

    volatile ButtonEvent pending_event_;

    uint32_t last_speed_down_tick_ms_;
    uint32_t last_speed_up_tick_ms_;

    static constexpr uint32_t kDebounceTimeMs = 50U;

    /**
     * @brief 判断按键是否已经过消抖时间。
     *
     * Parameters
     * ----------
     * now_tick_ms : uint32_t
     *     当前 HAL tick，单位 ms。
     * last_tick_ms : uint32_t
     *     上一次有效按键事件时间，单位 ms。
     *
     * Returns
     * -------
     * bool
     *     如果已经超过消抖时间，返回 true。
     */
    bool IsDebounced(
        uint32_t now_tick_ms,
        uint32_t last_tick_ms
    ) const;
};