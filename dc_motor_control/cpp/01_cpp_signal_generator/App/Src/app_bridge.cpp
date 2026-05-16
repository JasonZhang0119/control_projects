#include "main.h"
#include "usart.h"
#include "gpio.h"

#include <cstdio>
#include <cstring>

#include <Eigen/Dense>

#include "app_bridge.h"

#include "signal_generator.hpp"

namespace
{

using DataType = float;

constexpr int Ny = 1;
constexpr int Nu = 1;
constexpr int NStair = 5;

using YVector = Eigen::Matrix<DataType, Ny, 1>;
using UVector = Eigen::Matrix<DataType, Nu, 1>;
using StairTime = Eigen::Matrix<DataType, NStair, Nu>;
using StairValue = Eigen::Matrix<DataType, NStair, Nu>;

StairSignalGenerator<DataType, Ny, Nu, NStair> stair_gen;

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

}  // namespace

void App_Init(void)
{
    StairTime timestamps;
    StairValue values;

    timestamps << 0.0f,
                  1.0f,
                  2.0f,
                  3.0f,
                  4.0f;

    values << 0.0f,
              200.0f,
              0.0f,
              -200.0f,
              0.0f;

    stair_gen.SetSamplingTime(0.1f);
    stair_gen.SetStairInfo(timestamps, values);
    stair_gen.Initialize();

    SendString("stair generator started\r\n");
}


void App_Step(void)
{
    static uint32_t last_tick = 0U;

    const uint32_t now_tick = HAL_GetTick();

    if ((now_tick - last_tick) >= 100U)
    {
        last_tick = now_tick;

        const YVector y_ref = YVector::Zero();
        const YVector y = YVector::Zero();

        UVector u_min;
        UVector u_max;

        u_min << -1000.0f;
        u_max << 1000.0f;

        const UVector u = stair_gen.Step(y_ref, y, u_min, u_max);

        const int32_t u_cmd_permille =
            static_cast<int32_t>(u(0, 0));

        char tx_buf[128];

        std::snprintf(
            tx_buf,
            sizeof(tx_buf),
            "%lu,%ld\r\n",
            static_cast<unsigned long>(now_tick),
            static_cast<long>(u_cmd_permille)
        );

        SendString(tx_buf);
        HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
    }
}