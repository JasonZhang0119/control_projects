#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化应用层。
 *
 * Notes
 * -----
 * - 由 main.c 在 HAL 和外设初始化完成后调用。
 * - 后续适合放置 C++ 对象初始化、控制器初始化、日志启动信息等。
 */
void App_Init(void);

/**
 * @brief 应用层周期执行函数。
 *
 * Notes
 * -----
 * - 由 main.c 在 while(1) 中不断调用。
 * - 该函数内部可以自行判断采样周期。
 */
void App_Step(void);

/**
 * @brief GPIO EXTI 中断事件桥接入口。
 *
 * Parameters
 * ----------
 * GPIO_Pin : uint16_t
 *     触发 EXTI 的 GPIO pin。
 */
void App_OnGpioExti(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif
