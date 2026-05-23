
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
    HAL_UART_Transmit(
        this->huart_,
        reinterpret_cast<const uint8_t*>(msg),
        static_cast<uint16_t>(std::strlen(msg)),
        10U
    );
}
