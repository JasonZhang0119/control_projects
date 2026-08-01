#include "main.h"
#include "stm32f1xx_hal_tim.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

#include <Eigen/Dense>

#include "app_bridge.h"

#include "signal_generator.hpp"


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

void App_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    SendString("encoder test started\r\n");
}


void App_Step(void)
{
    static uint32_t last_tick = 0U;

    const uint32_t now_tick = HAL_GetTick();

    if ((now_tick - last_tick) >= 100U)
    {
        last_tick = now_tick;

        char tx_buf[128];

        const uint16_t encoder_raw_count = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&htim3));

        std::snprintf(
            tx_buf,
            sizeof(tx_buf), 
            "%lu,%u\r\n",
            static_cast<unsigned long>(now_tick),
            static_cast<unsigned int>(encoder_raw_count)
        );

        SendString(tx_buf);
        HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
    }
}


// void App_Step(void)
// {
//     static constexpr uint32_t kSamplePeriodMs = 100U;
//     static uint32_t last_tick = 0U;

//     const uint32_t now_tick = HAL_GetTick();

//     if ((now_tick - last_tick) >= kSamplePeriodMs)
//     {
//         last_tick = now_tick;

//         const uint16_t encoder_raw_count =
//             static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&htim2));

//         const GPIO_PinState a_state =
//             HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

//         const GPIO_PinState b_state =
//             HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

//         char tx_buf[128];

//         std::snprintf(
//             tx_buf,
//             sizeof(tx_buf),
//             "%lu,%u,%u,%u\r\n",
//             static_cast<unsigned long>(now_tick),
//             static_cast<unsigned int>(encoder_raw_count),
//             static_cast<unsigned int>(a_state == GPIO_PIN_SET),
//             static_cast<unsigned int>(b_state == GPIO_PIN_SET)
//         );

//         SendString(tx_buf);
//         HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
//     }
// }


// void App_Step(void)
// {
//     static constexpr uint32_t kSamplePeriodMs = 100U;
//     static uint32_t last_tick = 0U;

//     const uint32_t now_tick = HAL_GetTick();

//     if ((now_tick - last_tick) >= kSamplePeriodMs)
//     {
//         last_tick = now_tick;

//         const GPIO_PinState a_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
//         const GPIO_PinState b_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

//         char tx_buf[128];

//         std::snprintf(
//             tx_buf,
//             sizeof(tx_buf),
//             "%lu,%u,%u\r\n",
//             static_cast<unsigned long>(now_tick),
//             static_cast<unsigned int>(a_state == GPIO_PIN_SET),
//             static_cast<unsigned int>(b_state == GPIO_PIN_SET)
//         );

//         SendString(tx_buf);
//         HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
//     }
// }