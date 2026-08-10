#include "uart_cli.hpp"

#include "main.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>


UartCli::UartCli(
    UART_HandleTypeDef* huart,
    UartLogger* logger,
    ParameterManager* parameter_manager
)
    : huart_(huart),
      logger_(logger),
      parameter_manager_(parameter_manager),
      mode_setter_(nullptr),
      mode_context_(nullptr),
      rx_byte_(0U),
      rx_buffer_{},
      command_buffer_{},
      rx_index_(0U),
      command_ready_(false),
      overflow_(false),
      logging_enabled_(false),
      status_{}
{
}


/**
 * @brief 启动 CLI，发送欢迎信息并开始 USART 中断接收。
 */
void UartCli::Start()
{
    this->logger_->SendString(
        "\r\nSTM32 CLI Ready\r\n"
        "Version 1.0\r\n"
        "Type 'help' for commands.\r\n"
    );
    this->SendPrompt();
    this->RestartReceive();
}


/**
 * @brief 处理 HAL UART 接收完成事件。
 */
void UartCli::OnRxComplete(UART_HandleTypeDef* huart)
{
    if (huart == this->huart_)
    {
        this->AppendRxByte(static_cast<char>(this->rx_byte_));
        this->RestartReceive();
    }
}


/**
 * @brief 在主循环中处理已经完成的命令行。
 */
void UartCli::Process()
{
    bool has_command = false;
    bool had_overflow = false;

    __disable_irq();
    if (this->command_ready_)
    {
        std::strncpy(
            this->command_buffer_,
            this->rx_buffer_,
            sizeof(this->command_buffer_) - 1U
        );
        this->command_buffer_[sizeof(this->command_buffer_) - 1U] = '\0';
        this->command_ready_ = false;
        has_command = true;
    }

    if (this->overflow_)
    {
        this->overflow_ = false;
        had_overflow = true;
    }
    __enable_irq();

    if (had_overflow)
    {
        this->logger_->SendString("\r\nERR: command too long\r\n");
        this->SendPrompt();
    }

    if (has_command)
    {
        this->ExecuteCommand(this->command_buffer_);
        this->SendPrompt();
    }
}


/**
 * @brief 保存应用层状态快照，供 status 命令输出。
 */
void UartCli::SetStatusSnapshot(const CliStatusSnapshot& snapshot)
{
    this->status_ = snapshot;
}


/**
 * @brief 绑定应用层模式设置回调。
 */
void UartCli::BindModeSetter(
    const ModeSetter setter,
    void* const context
)
{
    this->mode_setter_ = setter;
    this->mode_context_ = context;
}


/**
 * @brief 返回周期日志开关状态。
 */
bool UartCli::IsLoggingEnabled() const
{
    return this->logging_enabled_;
}


/**
 * @brief 重新提交 1 字节 UART 中断接收请求。
 */
void UartCli::RestartReceive()
{
    static_cast<void>(HAL_UART_Receive_IT(
        this->huart_,
        &this->rx_byte_,
        1U
    ));
}


/**
 * @brief 将中断收到的一个字符加入行缓冲。
 *
 * Notes
 * -----
 * 收到 '\r' 或 '\n' 时认为命令结束。若上一条命令尚未被主循环处理，
 * 暂时忽略后续字符；后续可以升级为命令队列。
 */
void UartCli::AppendRxByte(const char ch)
{
    if (this->command_ready_)
    {
        return;
    }

    if (ch == '\r' || ch == '\n')
    {
        if (this->rx_index_ == 0U)
        {
            return;
        }

        if (!this->command_ready_)
        {
            this->rx_buffer_[this->rx_index_] = '\0';
            this->command_ready_ = true;
        }

        this->rx_index_ = 0U;
        return;
    }

    if (ch == '\b' || ch == 0x7F)
    {
        if (this->rx_index_ > 0U)
        {
            --this->rx_index_;
        }
        return;
    }

    if (this->rx_index_ >= (kLineBufferSize - 1U))
    {
        this->rx_index_ = 0U;
        this->overflow_ = true;
        return;
    }

    this->rx_buffer_[this->rx_index_] = ch;
    ++this->rx_index_;
}


/**
 * @brief 根据第一个 token 分发命令。
 */
void UartCli::ExecuteCommand(char* line)
{
    char* argv[kMaxArgCount] = {};
    const std::size_t argc = this->Tokenize(line, argv, kMaxArgCount);

    if (argc == 0U)
    {
        return;
    }

    if (std::strcmp(argv[0], "help") == 0)
    {
        this->SendHelp();
    }
    else if (std::strcmp(argv[0], "status") == 0)
    {
        this->SendStatus();
    }
    else if (std::strcmp(argv[0], "set") == 0)
    {
        this->HandleSet(argv, argc);
    }
    else if (std::strcmp(argv[0], "get") == 0)
    {
        this->HandleGet(argv, argc);
    }
    else if (std::strcmp(argv[0], "mode") == 0)
    {
        this->HandleMode(argv, argc);
    }
    else if (std::strcmp(argv[0], "log") == 0)
    {
        this->HandleLog(argv, argc);
    }
    else
    {
        this->logger_->SendString("\r\nERR: unknown command\r\n");
    }
}


/**
 * @brief 发送 CLI 提示符。
 */
void UartCli::SendPrompt()
{
    this->logger_->SendString("\r\n> ");
}


/**
 * @brief 输出当前支持的 CLI 命令列表。
 */
void UartCli::SendHelp()
{
    this->logger_->SendString(
        "\r\nAvailable commands:\r\n"
        "help\r\n"
        "status\r\n"
        "set <name> <value>\r\n"
        "get <name>\r\n"
        "mode position|speed\r\n"
        "log start|stop|status\r\n"
    );
}


/**
 * @brief 输出设备状态快照。
 */
void UartCli::SendStatus()
{
    char tx_buf[512];

    std::snprintf(
        tx_buf,
        sizeof(tx_buf),
        "\r\nSystem Status:\r\n"
        "mode=%s\r\n"
        "runtime=%lus\r\n"
        "error=TODO\r\n"
        "log=%s\r\n"
        "duty_permille=%ld\r\n"
        "speed_ref_x1000=%ld\r\n"
        "speed_meas_x1000=%ld\r\n"
        "position_ref_x1000=%ld\r\n"
        "position_meas_x1000=%ld\r\n",
        this->ModeToString(this->status_.mode),
        static_cast<unsigned long>(this->status_.runtime_ms / 1000U),
        this->logging_enabled_ ? "on" : "off",
        static_cast<long>(this->status_.signed_duty_permille),
        static_cast<long>(this->status_.speed_ref_rad_s * 1000.0F),
        static_cast<long>(this->status_.encoder.rad_per_second * 1000.0F),
        static_cast<long>(this->status_.position_ref_rad * 1000.0F),
        static_cast<long>(this->status_.encoder.position_rad * 1000.0F)
    );

    this->logger_->SendString(tx_buf);
}


/**
 * @brief 解析 set 命令参数。
 */
void UartCli::HandleSet(char** argv, const std::size_t argc)
{
    if (argc < 3U)
    {
        this->logger_->SendString("\r\nERR: usage set <name> <value>\r\n");
        return;
    }

    char* end = nullptr;
    const float value = std::strtof(argv[2], &end);
    if (end == argv[2] || *end != '\0')
    {
        this->logger_->SendString("\r\nERR: invalid value\r\n");
        return;
    }

    if (this->parameter_manager_ == nullptr)
    {
        this->logger_->SendString("\r\nERR: parameter manager not bound\r\n");
        return;
    }

    if (this->parameter_manager_->Set(argv[1], value))
    {
        char tx_buf[80];
        std::snprintf(
            tx_buf,
            sizeof(tx_buf),
            "\r\nset %s_x1000=%ld\r\n",
            argv[1],
            static_cast<long>(value * 1000.0F)
        );
        this->logger_->SendString(tx_buf);
        return;
    }

    this->logger_->SendString("\r\nERR: unknown parameter\r\n");
}


/**
 * @brief 解析 get 命令参数。
 */
void UartCli::HandleGet(char** argv, const std::size_t argc)
{
    if (argc < 2U)
    {
        this->logger_->SendString("\r\nERR: usage get <name>\r\n");
        return;
    }

    if (this->parameter_manager_ == nullptr)
    {
        this->logger_->SendString("\r\nERR: parameter manager not bound\r\n");
        return;
    }

    float value = 0.0F;
    if (!this->parameter_manager_->Get(argv[1], &value))
    {
        this->logger_->SendString("\r\nERR: unknown parameter\r\n");
        return;
    }

    char tx_buf[80];
    std::snprintf(
        tx_buf,
        sizeof(tx_buf),
        "\r\n%s_x1000=%ld\r\n",
        argv[1],
        static_cast<long>(value * 1000.0F)
    );

    this->logger_->SendString(tx_buf);
}


/**
 * @brief 处理控制模式切换命令。
 */
void UartCli::HandleMode(char** argv, const std::size_t argc)
{
    if (argc < 2U)
    {
        this->logger_->SendString("\r\nERR: usage mode position|speed\r\n");
        return;
    }

    MotorMode mode = MotorMode::Position;
    if (std::strcmp(argv[1], "position") == 0)
    {
        mode = MotorMode::Position;
    }
    else if (std::strcmp(argv[1], "speed") == 0)
    {
        mode = MotorMode::Speed;
    }
    else
    {
        this->logger_->SendString("\r\nERR: usage mode position|speed\r\n");
        return;
    }

    if (this->mode_setter_ == nullptr)
    {
        this->logger_->SendString("\r\nERR: mode setter not bound\r\n");
        return;
    }

    this->mode_setter_(this->mode_context_, mode);

    char tx_buf[48];
    std::snprintf(
        tx_buf,
        sizeof(tx_buf),
        "\r\nmode=%s\r\n",
        this->ModeToString(mode)
    );
    this->logger_->SendString(tx_buf);
}


/**
 * @brief 控制周期 telemetry 日志输出。
 */
void UartCli::HandleLog(char** argv, const std::size_t argc)
{
    if (argc < 2U)
    {
        this->logger_->SendString("\r\nERR: usage log start|stop|status\r\n");
        return;
    }

    if (std::strcmp(argv[1], "start") == 0)
    {
        this->logging_enabled_ = true;
        this->logger_->SendString(
            "\r\nlogging...\r\n"
            "format: time_ms,duty_permille,count,delta_count,"
            "speed_ref_x1000,speed_meas_x1000,"
            "position_ref_x1000,position_meas_x1000\r\n"
        );
    }
    else if (std::strcmp(argv[1], "stop") == 0)
    {
        this->logging_enabled_ = false;
        this->logger_->SendString("\r\nlogging stopped\r\n");
    }
    else if (std::strcmp(argv[1], "status") == 0)
    {
        this->logger_->SendString(
            this->logging_enabled_
                ? "\r\nlog=on\r\n"
                : "\r\nlog=off\r\n"
        );
    }
    else
    {
        this->logger_->SendString("\r\nERR: usage log start|stop|status\r\n");
    }
}


/**
 * @brief 将命令行按空白字符拆分为 argv。
 */
std::size_t UartCli::Tokenize(
    char* line,
    char** argv,
    const std::size_t max_argc
)
{
    std::size_t argc = 0U;
    char* token = std::strtok(line, " \t");

    while ((token != nullptr) && (argc < max_argc))
    {
        argv[argc] = token;
        ++argc;
        token = std::strtok(nullptr, " \t");
    }

    return argc;
}


/**
 * @brief 将控制模式转换为字符串。
 */
const char* UartCli::ModeToString(const MotorMode mode)
{
    switch (mode)
    {
    case MotorMode::Position:
        return "position";
    case MotorMode::Speed:
        return "speed";
    default:
        return "unknown";
    }
}