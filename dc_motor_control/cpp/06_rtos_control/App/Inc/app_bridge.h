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
void  Module_Init(void);

void InitializeSpeedController();
void InitializePositionController();


#ifdef __cplusplus
}
#endif