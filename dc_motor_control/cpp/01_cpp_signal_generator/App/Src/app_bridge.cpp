#include "app_bridge.h"

#include "main.h"
#include "usart.h"
#include "gpio.h"

#include <cstdio>
#include <cstring>

namespace
{

/**
 * @brief 通过 USART1 发送字符串。
 *
 * Parameters
 * ----------
 * msg : const char*
 *     待发送的 C 字符串。
 *
 * Returns
 * -------
 * None
 *
 * Notes
 * -----
 * - 当前使用阻塞式 HAL_UART_Transmit。
 * - 超时时间为 10 ms，避免串口异常导致主循环永久阻塞。
 */
void SendString(const char* msg)
{
    HAL_UART_Transmit(
        &huart1,
        reinterpret_cast<const uint8_t*>(msg),
        static_cast<uint16_t>(std::strlen(msg)),
        10U
    );
}

}  // namespace

void App_Init(void)
{
    SendString("app started\r\n");
}

void App_Step(void)
{
    static uint32_t last_tick = 0U;
    static uint32_t sample_index = 0U;

    const uint32_t now_tick = HAL_GetTick();

    if ((now_tick - last_tick) >= 100U)
    {
        last_tick = now_tick;

        const uint32_t time_ms = now_tick;
        const int32_t pwm_cmd_permille = 0;
        const int32_t encoder_count = static_cast<int32_t>(sample_index * 10U);
        const int32_t theta_mrad = static_cast<int32_t>(sample_index * 10U);
        const int32_t omega_mrad_s = 1000;

        char tx_buf[128];

        std::snprintf(
            tx_buf,
            sizeof(tx_buf),
            "%lu,%ld,%ld,%ld,%ld\r\n",
            static_cast<unsigned long>(time_ms),
            static_cast<long>(pwm_cmd_permille),
            static_cast<long>(encoder_count),
            static_cast<long>(theta_mrad),
            static_cast<long>(omega_mrad_s)
        );

        SendString(tx_buf);
        HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);

        sample_index++;
    }
}