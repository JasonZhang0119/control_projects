#pragma once

#include "stm32f1xx_hal.h"

#include <cstdint>


/**
 * @brief 按键事件类型。
 *
 * Notes
 * -----
 * ButtonManager 只区分物理方向 Up / Down，不关心应用层含义。
 * 具体是调速度、位置还是菜单，由 Application 层决定。
 */
enum class ButtonEvent : uint8_t
{
    None = 0,
    Up,
    Down
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
     * down_port : GPIO_TypeDef*
     *     Down 按键 GPIO port。
     * down_pin : uint16_t
     *     Down 按键 GPIO pin。
     * up_port : GPIO_TypeDef*
     *     Up 按键 GPIO port。
     * up_pin : uint16_t
     *     Up 按键 GPIO pin。
     */
    ButtonManager(
        GPIO_TypeDef* down_port,
        uint16_t down_pin,
        GPIO_TypeDef* up_port,
        uint16_t up_pin,
        GPIO_PinState down_active_level = GPIO_PIN_RESET,
        GPIO_PinState up_active_level = GPIO_PIN_SET
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
     * @brief 在主循环中更新按键消抖状态。
     *
     * Notes
     * -----
     * EXTI 中断只记录候选按键，Update() 在消抖时间到达后读取 GPIO 电平。
     * 如果电平仍然处于按下状态，才生成 ButtonEvent。
     */
    void Update();

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
    /** @brief 减速按键所在 GPIO port。 */
    GPIO_TypeDef* down_port_;

    /** @brief 减速按键 pin。 */
    uint16_t down_pin_;

    /** @brief 加速按键所在 GPIO port。 */
    GPIO_TypeDef* up_port_;

    /** @brief 加速按键 pin。 */
    uint16_t up_pin_;

    GPIO_PinState down_active_level_;
    GPIO_PinState up_active_level_;

    /** @brief 中断侧写入、主循环侧消费的待处理按键事件。 */
    volatile ButtonEvent pending_event_;

    /** @brief 上一次有效减速按键事件时间。 */
    volatile bool down_debouncing_;

    /** @brief 上一次有效加速按键事件时间。 */
    volatile bool up_debouncing_;

    uint32_t down_debounce_start_tick_ms_;

    uint32_t up_debounce_start_tick_ms_;

    /** @brief 软件消抖时间，单位 ms。 */
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
    bool IsDebounceExpired(
        uint32_t now_tick_ms,
        uint32_t start_tick_ms
    ) const;

    bool IsDownPressed() const;
    bool IsUpPressed() const;
};
