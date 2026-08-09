
#include "uart_logger.hpp"
#include <cstring>


/**
    * @brief 构造 UART 日志器。
    *
    * Parameters
    * ----------
    * huart : UART_HandleTypeDef*
    *     UART 句柄。
    */
UartLogger::UartLogger(UART_HandleTypeDef* huart){
    this->huart_ = huart;
}

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
void UartLogger::SendString(const char* msg)
{
    const uint16_t length =
        static_cast<uint16_t>(std::strlen(msg));

    const uint32_t baudrate = this->huart_->Init.BaudRate;
    uint32_t timeout_ms = 50U;

    if (baudrate > 0U)
    {
        timeout_ms +=
            (static_cast<uint32_t>(length) * 10U * 1000U) / baudrate;
    }

    HAL_UART_Transmit(
        this->huart_,
        reinterpret_cast<const uint8_t*>(msg),
        length,
        timeout_ms
    );
}
