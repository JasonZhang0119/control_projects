#pragma once

#include "stm32f1xx_hal.h"


/**
 * @brief UART 字符串日志发送器。
 */
class UartLogger
{
public:
    /**
     * @brief 构造 UART 日志器。
     *
     * Parameters
     * ----------
     * huart : UART_HandleTypeDef*
     *     UART 句柄。
     */
    explicit UartLogger(UART_HandleTypeDef* huart);

    /**
     * @brief 发送 C 字符串。
     *
     * Parameters
     * ----------
     * msg : const char*
     *     待发送字符串。
     */
    void SendString(const char* msg);

private:
    /** @brief 实际用于发送字符串的 UART 句柄。 */
    UART_HandleTypeDef* huart_;
};
