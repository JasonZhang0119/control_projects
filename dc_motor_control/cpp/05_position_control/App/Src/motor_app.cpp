#include "motor_app.hpp"
#include "motor_driver.hpp"

#include <cstdio>


void MotorPositionApp::Initialize()
{
    this->encoder_reader_.Start();
    this->encoder_reader_.Reset();

    this->motor_driver_.Start();
    this->motor_driver_.Stop();

    this->button_manager_.Reset();

    this->InitializeSpeedController();
    this->InitializePositionController();

    this->last_tick_ms_ = HAL_GetTick();

    this->uart_logger_.SendString("motor app started\r\n");
    this->uart_logger_.SendString(
        "format: time_ms,duty_permille,count,delta_count,speed_ref_x1000,speed_meas_x1000\r\n"
    );
}


void MotorPositionApp::InitializeSpeedController()
{
    GainMatrix Kp;
    GainMatrix Ki;
    GainMatrix Kd;


    // Kp << 0.01F;
    // Ki << 0.3F;
    // Kd << 0.0F;

    Kp << 0.1F;
    Ki << 3.0F;
    Kd << 0.0F;

    this->speed_pid_.SetSamplingTime(kSampleTimeS);
    this->speed_pid_.SetGain(Kp, Ki, Kd);
    this->speed_pid_.Reset();
}

void MotorPositionApp::InitializePositionController(){
    GainMatrix Kp;
    GainMatrix Ki;
    GainMatrix Kd;

    Kp << 1.2F;
    Ki << 0.0F;
    Kd << 0.0F;

    this->position_pid_.SetSamplingTime(kSampleTimeS);
    this->position_pid_.SetGain(Kp, Ki, Kd);
    this->position_pid_.Reset();
}


void MotorPositionApp::OnGpioExti(const uint16_t gpio_pin)
{
    this->button_manager_.OnExtiInterrupt(gpio_pin);
}


void MotorPositionApp::HandleButtonEvents()
{
    const ButtonEvent event =
        this->button_manager_.ConsumeEvent();

    switch (event)
    {
    case ButtonEvent::SpeedUp:

        this->position_ref_rad_ += kPositionRefStepRadS;
        if(this->position_ref_rad_ > kPositionRefMaxRadS){
            this->position_ref_rad_ = kPositionRefMaxRadS;
        }
        break;

    case ButtonEvent::SpeedDown:
        this->position_ref_rad_ -= kPositionRefStepRadS;
        if (this->position_ref_rad_ < kPositionRefMinRadS){
            this->position_ref_rad_ = kPositionRefMinRadS;
        }
        break;

    case ButtonEvent::None:
    default:
        break;
    }
}


uint32_t MotorPositionApp::ApplySignedDuty(const float signed_duty_permille)
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


int32_t MotorPositionApp::RunPositionControl(
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

    return signed_duty(0);
}


void MotorPositionApp::LogTelemetry(
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


void MotorPositionApp::Step()
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
        this->RunPositionControl(encoder_meas);

    this->LogTelemetry(
        now_tick_ms,
        duty_permille,
        encoder_meas
    );

    HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
}