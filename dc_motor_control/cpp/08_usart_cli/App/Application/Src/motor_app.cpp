#include "motor_app.hpp"
#include "motor_driver.hpp"

#include <cstdio>


namespace
{
void SetMotorModeFromCli(void* context, const MotorMode mode)
{
    if (context == nullptr)
    {
        return;
    }

    static_cast<DCMotorApp*>(context)->SetMode(mode);
}
}


/**
 * @brief 初始化电机应用层对象和 CLI。
 *
 * Notes
 * -----
 * 初始化顺序为：启动编码器、启动并停止电机 PWM、清空按键事件、
 * 初始化控制器和参数管理器，最后启动串口 CLI。
 */
void DCMotorApp::Initialize()
{
    this->encoder_reader_.Start();
    this->encoder_reader_.Reset();

    this->motor_driver_.Start();
    this->motor_driver_.Stop();

    this->button_manager_.Reset();

    this->InitializeSpeedController();
    this->InitializePositionController();
    this->parameter_manager_.BindPidControllers(
        &this->speed_pid_,
        &this->position_pid_
    );

    this->last_tick_ms_ = HAL_GetTick();

    this->uart_cli_.BindModeSetter(SetMotorModeFromCli, this);
    this->uart_cli_.Start();
}


/**
 * @brief 初始化速度环 PID 参数。
 *
 * TODO: 后续将硬编码增益迁移到参数管理模块，由 CLI 动态设置。
 */
void DCMotorApp::InitializeSpeedController()
{
    GainMatrix Kp{0};
    GainMatrix Ki{0};
    GainMatrix Kd{0};

    // Kp << 0.1F;
    // Ki << 3.0F;
    // Kd << 0.0F;


    this->speed_pid_.SetSamplingTime(kSampleTimeS);
    this->speed_pid_.SetGain(Kp, Ki, Kd);
    this->speed_pid_.Reset();
}

/**
 * @brief 初始化位置环 PID 参数。
 *
 * TODO: 后续将硬编码增益迁移到参数管理模块，由 CLI 动态设置。
 */
void DCMotorApp::InitializePositionController(){
    GainMatrix Kp{0};
    GainMatrix Ki{0};
    GainMatrix Kd{0};

    // Kp << 1.2F;
    // Ki << 0.0F;
    // Kd << 0.0F;

    this->position_pid_.SetSamplingTime(kSampleTimeS);
    this->position_pid_.SetGain(Kp, Ki, Kd);
    this->position_pid_.Reset();
}


/**
 * @brief 接收 GPIO EXTI 事件并交给按键管理器。
 */
void DCMotorApp::OnGpioExti(const uint16_t gpio_pin)
{
    this->button_manager_.OnExtiInterrupt(gpio_pin);
}


/**
 * @brief 接收 UART 中断回调并交给 CLI。
 */
void DCMotorApp::OnUartRxComplete(UART_HandleTypeDef* huart)
{
    this->uart_cli_.OnRxComplete(huart);
}

/**
 * @brief 设置电机控制模式。
 */
void DCMotorApp::SetMode(const MotorMode mode)
{
    this->mode_ = mode;
    this->position_ref_rad_ = 0.0F;
    this->speed_ref_rad_s_ = 0.0F;
}


/**
 * @brief 获取当前电机控制模式。
 */
MotorMode DCMotorApp::GetMode() const
{
    return this->mode_;
}


/**
 * @brief 消费按键事件，并根据当前模式更新位置或速度参考值。
 */
void DCMotorApp::HandleButtonEvents()
{
    this->button_manager_.Update();

    const ButtonEvent event =
        this->button_manager_.ConsumeEvent();

    if (this->mode_ == MotorMode::Position){
        switch (event)
        {
        case ButtonEvent::Up:

            this->position_ref_rad_ += kPositionRefStepRadS;
            if(this->position_ref_rad_ > kPositionRefMaxRadS){
                this->position_ref_rad_ = kPositionRefMaxRadS;
            }
            break;

        case ButtonEvent::Down:
            this->position_ref_rad_ -= kPositionRefStepRadS;
            if (this->position_ref_rad_ < kPositionRefMinRadS){
                this->position_ref_rad_ = kPositionRefMinRadS;
            }
            break;

        case ButtonEvent::None:
        default:
            break;
        }
    }else{
        switch (event)
        {
        case ButtonEvent::Up:

            this->speed_ref_rad_s_ += kSpeedRefStepRadS;
            if(this->speed_ref_rad_s_ > kSpeedRefMaxRadS){
                this->speed_ref_rad_s_ = kSpeedRefMaxRadS;
            }
            break;

        case ButtonEvent::Down:
            this->speed_ref_rad_s_ -= kSpeedRefStepRadS;
            if (this->speed_ref_rad_s_ < kSpeedRefMinRadS){
                this->speed_ref_rad_s_ = kSpeedRefMinRadS;
            }
            break;

        case ButtonEvent::None:
        default:
            break;
        }
    }

}

/**
 * @brief 将带符号 duty 转换为电机方向和绝对占空比。
 */
uint32_t DCMotorApp::ApplySignedDuty(const float signed_duty_permille)
{
    if (signed_duty_permille > 0.01){
        this->motor_driver_.SetCommand(MotorDirection::Forward, std::abs(signed_duty_permille));
    }else if (signed_duty_permille < -0.01) {
        this->motor_driver_.SetCommand(MotorDirection::Reverse, std::abs(signed_duty_permille));
    }else{
        this->motor_driver_.Stop();
    }

    return std::abs(signed_duty_permille);
}


/**
 * @brief 执行位置环与速度环的级联控制。
 *
 */
int32_t DCMotorApp::RunPositionControl(
    const EncoderMeasurement& encoder_meas
)
{
    YVector speed_ref;
    YVector speed_meas;
    YVector position_ref;
    YVector position_meas;

    position_ref << this->position_ref_rad_;
    position_meas << encoder_meas.position_rad;

    speed_ref << this->position_pid_.Step(
            position_ref,
            position_meas,
            UVector{-60},
            UVector{60}
    );

    if (std::abs(position_ref(0) - position_meas(0)) < 0.1){
        speed_ref << 0;
    }
    this->speed_ref_rad_s_ = speed_ref(0);

    speed_meas << encoder_meas.rad_per_second;

    UVector u_ff;
    UVector u_fb;
    UVector signed_duty;
    UVector u_min;
    UVector u_max;

    if (speed_ref(0)<0){
        u_ff << -0.2;
        u_min << -0.8;
        u_max << 1.2;
    }else{
        u_ff << 0.2;
        u_min << -1.2;
        u_max << 0.8;
    }

    u_fb = this->speed_pid_.Step(
            speed_ref,
            speed_meas,
            u_min,
            u_max
        );
    
    signed_duty = (u_fb + u_ff) * 1000;

    const uint32_t duty_permille =
        this->ApplySignedDuty(signed_duty(0));
    (void) duty_permille;


    return signed_duty(0);
}




/**
 * @brief 执行速度环控制。
 *
 */
int32_t DCMotorApp::RunSpeedControl(
    const EncoderMeasurement& encoder_meas
)
{
    YVector speed_ref;
    YVector speed_meas;

    speed_ref << this->speed_ref_rad_s_;
    speed_meas << encoder_meas.rad_per_second;

    UVector u_ff;
    UVector u_fb;
    UVector signed_duty;
    UVector u_min;
    UVector u_max;

    if (speed_ref(0)<0){
        u_ff << -0.2;
        u_min << -0.8;
        u_max << 1.2;
    }else{
        u_ff << 0.2;
        u_min << -1.2;
        u_max << 0.8;
    }

    u_fb = this->speed_pid_.Step(
            speed_ref,
            speed_meas,
            u_min,
            u_max
        );
    
    signed_duty = (u_fb + u_ff) * 1000;

    const uint32_t duty_permille =
        this->ApplySignedDuty(signed_duty(0));
    (void) duty_permille;

    return signed_duty(0);
}



/**
 * @brief 输出一行 CSV 格式 telemetry。
 *
 * Notes
 * -----
 * 该函数只负责格式化并发送日志，是否输出由 CLI 的 log 开关决定。
 */
void DCMotorApp::LogTelemetry(
    const uint32_t now_tick_ms,
    const int32_t signed_duty_permille,
    const EncoderMeasurement& encoder_meas
)
{
    char tx_buf[160];

    std::snprintf(
        tx_buf,
        sizeof(tx_buf),
        "%lu,%ld,%u,%d,%ld,%ld, %ld,%ld\r\n",
        static_cast<unsigned long>(now_tick_ms),
        static_cast<long>(signed_duty_permille),
        static_cast<unsigned int>(encoder_meas.count),
        static_cast<int>(encoder_meas.delta_count),
        static_cast<long>(this->speed_ref_rad_s_ * 1000.0F),
        static_cast<long>(encoder_meas.rad_per_second * 1000.0F),
        static_cast<long>(this->position_ref_rad_ * 1000.0F),
        static_cast<long>(encoder_meas.position_rad * 1000.0F)
    );

    this->uart_logger_.SendString(tx_buf);
}

/**
 * @brief 应用层主循环步骤。
 *
 * Notes
 * -----
 * CLI 命令每次主循环都处理；控制算法按 kSamplePeriodMs 节拍执行。
 */
void DCMotorApp::Step()
{
    this->uart_cli_.Process();

    this->HandleButtonEvents();

    const uint32_t now_tick_ms = HAL_GetTick();

    if ((now_tick_ms - this->last_tick_ms_) < kSamplePeriodMs)
    {
        return;
    }

    this->last_tick_ms_ = now_tick_ms;

    const EncoderMeasurement encoder_meas =
        this->encoder_reader_.Sample(kSampleTimeS);

    int32_t signed_duty_permille = 0;
    if (this->mode_ == MotorMode::Position){
        signed_duty_permille =
            this->RunPositionControl(encoder_meas);
    }else{
        signed_duty_permille =
            this->RunSpeedControl(encoder_meas);
    }

    this->signed_duty_permille_ = signed_duty_permille;
    this->last_encoder_meas_ = encoder_meas;

    CliStatusSnapshot status{};
    status.mode = this->mode_;
    status.runtime_ms = now_tick_ms;
    status.signed_duty_permille = this->signed_duty_permille_;
    status.encoder = this->last_encoder_meas_;
    status.speed_ref_rad_s = this->speed_ref_rad_s_;
    status.position_ref_rad = this->position_ref_rad_;
    this->uart_cli_.SetStatusSnapshot(status);

    if (this->uart_cli_.IsLoggingEnabled())
    {
        this->LogTelemetry(
            now_tick_ms,
            signed_duty_permille,
            encoder_meas
        );
    }

}
