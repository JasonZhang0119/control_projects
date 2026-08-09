#include "app_bridge.h"
#include "motor_app.hpp"

#include "stm32f1xx_hal.h"

namespace
{
/**
 * @brief 全局应用对象。
 *
 * Notes
 * -----
 * main.c 和 HAL 回调都是 C 接口，因此通过本文件把 C 入口桥接到 C++ 对象。
 */
DCMotorApp motor_app;
}

/**
 * @brief C 入口：初始化应用层。
 */
extern "C" void App_Init(void)
{
    motor_app.Initialize();
}

/**
 * @brief C 入口：执行应用层周期任务。
 */
extern "C" void App_Step(void)
{
    motor_app.Step();
}

/**
 * @brief C 入口：转发 GPIO EXTI 事件。
 */
extern "C" void App_OnGpioExti(uint16_t GPIO_Pin)
{
    motor_app.OnGpioExti(GPIO_Pin);
}

/**
 * @brief HAL 回调：转发 UART 单字节接收完成事件。
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    motor_app.OnUartRxComplete(huart);
}
