#pragma once

#include <cstdint>


/**
 * @brief DC 电机应用的控制模式。
 *
 * Notes
 * -----
 * 该类型是 Application 与 Service 之间共享的应用层概念。
 * UartCli 可以解析/显示该模式，DCMotorApp 根据该模式选择控制逻辑。
 */
enum class MotorMode : uint8_t
{
    Position = 0,
    Speed
};
