#pragma once

#include "encoder_reader.hpp"
#include "motor_mode.hpp"
#include "parameter_manager.hpp"
#include "stm32f1xx_hal.h"
#include "uart_logger.hpp"

#include <cstddef>
#include <cstdint>

class ParameterManager;


struct CliStatusSnapshot
{
    /** @brief 当前控制模式，用于 status 命令显示。 */
    MotorMode mode;

    /** @brief 系统运行时间，单位 ms。 */
    uint32_t runtime_ms;

    /** @brief 最近一次控制输出，带符号，占空比千分数。 */
    int32_t signed_duty_permille;

    /** @brief 最近一次编码器采样结果，用于 status 命令显示。 */
    EncoderMeasurement encoder;

    /** @brief 当前速度参考值，单位 rad/s。 */
    float speed_ref_rad_s;

    /** @brief 当前位置参考值，单位 rad。 */
    float position_ref_rad;
};


/**
 * @brief 基于 USART 中断接收的文本命令行接口。
 *
 * Notes
 * -----
 * 该类只负责 CLI 基础框架：
 * - 启动 USART 单字节中断接收；
 * - 将接收到的字符拼成一行命令；
 * - 在主循环中解析并分发命令；
 * - 输出命令回复和提示符。
 */
class UartCli
{
public:
    /** @brief CLI 请求切换模式时调用的应用层回调。 */
    using ModeSetter = void (*)(void* context, MotorMode mode);

    /**
     * @brief 构造 CLI 对象。
     *
     * Parameters
     * ----------
     * huart : UART_HandleTypeDef*
     *     CLI 使用的 UART 句柄。
     * logger : UartLogger*
     *     用于发送回复字符串的 UART 日志器。
     * parameter_manager : ParameterManager*
     *     参数管理器，不由 UartCli 拥有。
     */
    UartCli(
        UART_HandleTypeDef* huart,
        UartLogger* logger,
        ParameterManager* parameter_manager
    );

    /** @brief 输出启动信息并启动 UART 中断接收。 */
    void Start();

    /**
     * @brief UART 接收完成中断桥接入口。
     *
     * Notes
     * -----
     * 该函数在 HAL UART 中断回调中调用，只做轻量级字符缓存，复杂解析放在 Process() 中执行。
     */
    void OnRxComplete(UART_HandleTypeDef* huart);

    /** @brief 在主循环中处理已经接收完成的一行命令。 */
    void Process();

    /** @brief 更新 status 命令使用的状态快照。 */
    void SetStatusSnapshot(const CliStatusSnapshot& snapshot);

    /** @brief 绑定模式设置回调，CLI 通过它通知应用层切换 MotorMode。 */
    void BindModeSetter(ModeSetter setter, void* context);

    /** @brief 查询周期日志是否开启。 */
    bool IsLoggingEnabled() const;

private:
    /** @brief 单行命令最大长度，包含结尾 '\0'。 */
    static constexpr std::size_t kLineBufferSize = 64U;

    /** @brief 单条命令最多解析出的 token 数量。 */
    static constexpr std::size_t kMaxArgCount = 4U;

    UART_HandleTypeDef* huart_;
    UartLogger* logger_;
    ParameterManager* parameter_manager_;
    ModeSetter mode_setter_;
    void* mode_context_;

    /** @brief HAL_UART_Receive_IT() 使用的单字节接收缓存。 */
    uint8_t rx_byte_;

    /** @brief 中断侧写入的一行命令缓存。 */
    char rx_buffer_[kLineBufferSize];

    /** @brief 主循环侧复制出来的命令缓存，避免解析时被中断改写。 */
    char command_buffer_[kLineBufferSize];

    /** @brief 当前行缓存写入位置。 */
    std::size_t rx_index_;

    /** @brief 标记已经收到完整命令行。 */
    volatile bool command_ready_;

    /** @brief 标记命令行超过缓冲区长度。 */
    volatile bool overflow_;

    /** @brief log start / log stop 控制的周期日志开关。 */
    bool logging_enabled_;

    /** @brief 最近一次状态快照。 */
    CliStatusSnapshot status_;

    /** @brief 重新启动下一字节 UART 中断接收。 */
    void RestartReceive();

    /** @brief 将一个接收字符写入行缓冲。 */
    void AppendRxByte(char ch);

    /** @brief 解析并执行一行命令。 */
    void ExecuteCommand(char* line);

    /** @brief 发送 CLI 提示符。 */
    void SendPrompt();

    /** @brief 输出 help 命令内容。 */
    void SendHelp();

    /** @brief 输出设备状态。 */
    void SendStatus();

    /** @brief 处理 set 命令。 */
    void HandleSet(char** argv, std::size_t argc);

    /** @brief 处理 get 命令。 */
    void HandleGet(char** argv, std::size_t argc);

    /** @brief 处理 mode 命令。 */
    void HandleMode(char** argv, std::size_t argc);

    /** @brief 处理 log 命令。 */
    void HandleLog(char** argv, std::size_t argc);

    /**
     * @brief 按空格和 tab 将命令行切分为 token。
     *
     * Returns
     * -------
     * std::size_t
     *     实际解析出的 token 数量。
     */
    static std::size_t Tokenize(
        char* line,
        char** argv,
        std::size_t max_argc
    );

    /** @brief 将 MotorMode 转换为可读字符串。 */
    static const char* ModeToString(MotorMode mode);
};