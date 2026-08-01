#include "task_wrapper.h"

#include "usart.h"
#include "tim.h"

#include <Eigen/Dense>


static TaskHandle_t g_button_task_handle = nullptr;

static EncoderReader g_encoder{
    &htim3,
    Config::kEncoderCountsPerRevolution
};

static MotorDriver g_motor{
    &htim2,
    TIM_CHANNEL_1,
    MOTOR1_AIN1_GPIO_Port,
    MOTOR1_AIN1_Pin,
    MOTOR1_AIN2_GPIO_Port,
    MOTOR1_AIN2_Pin
};

static PidController<float, Config::Ny, Config::Nu> g_speed_pid{};
static PidController<float, Config::Ny, Config::Nu> g_pos_pid{};
static UartLogger g_logger{&huart1};

static ButtonManager g_button{
    KEY_POSITION_DOWN_GPIO_Port,
    KEY_POSITION_DOWN_Pin,
    KEY_POSITION_UP_GPIO_Port,
    KEY_POSITION_UP_Pin
};

static SystemContext g_sys_ctx;



static void InitializeSpeedController(void)
{
    using GainMatrix =
        Eigen::Matrix<float, 1, 1>;

    GainMatrix Kp;
    GainMatrix Ki;
    GainMatrix Kd;

    Kp << 0.01F;
    Ki << 0.3F;
    Kd << 0.0F;

    g_speed_pid.SetSamplingTime(
        Config::kSampleTimeS
    );

    g_speed_pid.SetGain(
        Kp,
        Ki,
        Kd
    );

    g_speed_pid.Reset();
}


static void InitializePositionController(void)
{
    using GainMatrix =
        Eigen::Matrix<float, 1, 1>;

    GainMatrix Kp;
    GainMatrix Ki;
    GainMatrix Kd;

    Kp << 2.0F;
    Ki << 0.0F;
    Kd << 0.0F;

    g_pos_pid.SetSamplingTime(
        Config::kSampleTimeS
    );

    g_pos_pid.SetGain(
        Kp,
        Ki,
        Kd
    );

    g_pos_pid.Reset();
}

extern "C" void Module_Init(void)
{
    g_encoder.Start();
    g_motor.Start();

    g_sys_ctx.encoder = &g_encoder;
    g_sys_ctx.motor_driver = &g_motor;
    g_sys_ctx.speed_controller = &g_speed_pid;
    g_sys_ctx.position_controller = &g_pos_pid;
    g_sys_ctx.uart_logger = &g_logger;
    g_sys_ctx.position_button = &g_button;

    g_sys_ctx.encoder->Reset();
    g_sys_ctx.position_button->Reset();
    g_sys_ctx.motor_driver->Start();
    g_sys_ctx.motor_driver->Stop();
    InitializePositionController();
    InitializeSpeedController();

    g_sys_ctx.button_event_queue =
        xQueueCreate(10, sizeof(ButtonEvent));

    g_sys_ctx.position_ref = 0.0F;
    g_sys_ctx.position_meas = 0.0F;
    g_sys_ctx.speed_ref = 0.0F;
    g_sys_ctx.speed_meas = 0.0F;
    g_sys_ctx.duty_cyle = 0.0F;
}

extern "C" void* Module_GetSystemContext(void)
{
    return static_cast<void*>(&g_sys_ctx);
}


extern "C" void Module_RegisterButtonTaskHandle(void* handle){
    g_button_task_handle = static_cast<TaskHandle_t>(handle);
}

extern "C" void Module_OnButtonExtiFromISR(uint16_t gpio_pin)
{
    if (g_button_task_handle == nullptr) {
        return;
    }

    if ((gpio_pin != KEY_POSITION_UP_Pin) &&
        (gpio_pin != KEY_POSITION_DOWN_Pin)) {
        return;
    }

    g_sys_ctx.position_button->OnExtiInterrupt(gpio_pin);
    

    BaseType_t higher_priority_task_woken = pdFALSE;

    vTaskNotifyGiveFromISR(
        g_button_task_handle,
        &higher_priority_task_woken
    );

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

