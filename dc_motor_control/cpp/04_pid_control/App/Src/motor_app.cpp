#include "motor_app.hpp"
#include "motor_driver.hpp"

#include <cstdio>


void MotorApp::Initialize()
{
    this->encoder_reader_.Start();
    this->encoder_reader_.Reset();

    this->motor_driver_.Start();
    this->motor_driver_.Stop();

    this->button_manager_.Reset();

    this->InitializeSpeedController();

    this->last_tick_ms_ = HAL_GetTick();

    this->uart_logger_.SendString("motor app started\r\n");
    this->uart_logger_.SendString(
        "format: time_ms,duty_permille,count,delta_count,speed_ref_x1000,speed_meas_x1000\r\n"
    );
}


void MotorApp::InitializeSpeedController()
{
    GainMatrix Kp;
    GainMatrix Ki;
    GainMatrix Kd;

    // TODO:
    // 1. 填入你扫频/一阶辨识后设计的速度环 PI 参数
    // 2. Kd 先设为 0
    Kp << 0.01F;
    Ki << 0.3F;
    Kd << 0.0F;

    this->speed_pid_.SetSamplingTime(kSampleTimeS);
    this->speed_pid_.SetGain(Kp, Ki, Kd);
    this->speed_pid_.Reset();
}


void MotorApp::OnGpioExti(const uint16_t gpio_pin)
{
    this->button_manager_.OnExtiInterrupt(gpio_pin);
}


void MotorApp::HandleButtonEvents()
{
    const ButtonEvent event =
        this->button_manager_.ConsumeEvent();

    switch (event)
    {
    case ButtonEvent::SpeedUp:

        this->speed_ref_rad_s_ += kSpeedRefStepRadS;
        if(this->speed_ref_rad_s_ > kSpeedRefMaxRadS){
            this->speed_ref_rad_s_ = kSpeedRefStepRadS;
        }
        break;

    case ButtonEvent::SpeedDown:
        this->speed_ref_rad_s_ -= kSpeedRefStepRadS;
        if (this->speed_ref_rad_s_ < kSpeedRefMinRadS){
            this->speed_ref_rad_s_ = kSpeedRefMaxRadS;
        }
        break;

    case ButtonEvent::None:
    default:
        break;
    }
}


uint32_t MotorApp::ApplySignedDuty(const float signed_duty_permille)
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


int32_t MotorApp::RunSpeedControl(
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

    return signed_duty(0);
}


void MotorApp::LogTelemetry(
    const uint32_t now_tick_ms,
    const int32_t signed_duty_permille,
    const EncoderMeasurement& encoder_meas,
    const float speed_meas_rad_s
)
{
    char tx_buf[160];

    std::snprintf(
        tx_buf,
        sizeof(tx_buf),
        "%lu,%ld,%u,%d,%ld,%ld\r\n",
        static_cast<unsigned long>(now_tick_ms),
        static_cast<long>(signed_duty_permille),
        static_cast<unsigned int>(encoder_meas.count),
        static_cast<int>(encoder_meas.delta_count),
        static_cast<long>(this->speed_ref_rad_s_ * 1000.0F),
        static_cast<long>(speed_meas_rad_s * 1000.0F)
    );

    this->uart_logger_.SendString(tx_buf);
}


void MotorApp::Step()
{
    this->HandleButtonEvents();

    const uint32_t now_tick_ms = HAL_GetTick();

    if ((now_tick_ms - this->last_tick_ms_) < kSamplePeriodMs)
    {
        return;
    }

    this->last_tick_ms_ = now_tick_ms;

    const EncoderMeasurement encoder_meas =
        this->encoder_reader_.Sample(kSampleTimeS);

    const uint32_t duty_permille =
        this->RunSpeedControl(encoder_meas);

    float speed_meas_rad_s = encoder_meas.rad_per_second;

    this->LogTelemetry(
        now_tick_ms,
        duty_permille,
        encoder_meas,
        speed_meas_rad_s
    );

    HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
}