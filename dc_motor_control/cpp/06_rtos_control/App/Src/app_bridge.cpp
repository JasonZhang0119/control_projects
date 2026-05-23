
#include "usart.h"
#include "tim.h"
#include "app_bridge.h"
#include "task_wrapper.h" // 包含 SystemContext 定义

static EncoderReader  g_encoder{&htim3,
                                Config::kEncoderCountsPerRevolution
                            };
static MotorDriver    g_motor{&htim2,
                            TIM_CHANNEL_1,
                            MOTOR1_AIN1_GPIO_Port,
                            MOTOR1_AIN1_Pin,
                            MOTOR1_AIN2_GPIO_Port,
                            MOTOR1_AIN2_Pin
                        };
static PidController<float, Config::Ny, Config::Nu> g_speed_pid{};
static PidController<float, Config::Ny, Config::Nu> g_pos_pid{};
static UartLogger     g_logger{&huart1};
static ButtonManager  g_button{KEY_POSITION_DOWN_GPIO_Port,
        KEY_POSITION_DOWN_Pin,
        KEY_POSITION_UP_GPIO_Port,
        KEY_POSITION_UP_Pin
    };

// 2. 这是核心的全局资源容器
static SystemContext g_sys_ctx;

// 3. 供 main.c 调用的初始化接口
extern "C" void Module_Init(void) {
    // 硬件驱动初始化
    g_encoder.Start();
    g_motor.Start();

    //控制器参数初始化
    InitializePositionController();
    InitializeSpeedController();

    // 组装 Context
    g_sys_ctx.encoder = &g_encoder;
    g_sys_ctx.motor_driver = &g_motor;
    g_sys_ctx.speed_controller = &g_speed_pid;
    g_sys_ctx.position_controller = &g_pos_pid;
    g_sys_ctx.uart_logger = &g_logger;
    g_sys_ctx.position_button = &g_button;
    
    // 初始化 Queue (假设大小为 10)
    g_sys_ctx.button_event_queue = xQueueCreate(10, sizeof(ButtonEvent));
}


extern "C" void InitializeSpeedController()
{

    using GainMatrix = Eigen::Matrix<float, 1, 1>;

    GainMatrix Kp;
    GainMatrix Ki;
    GainMatrix Kd;


    Kp << 0.1F;
    Ki << 3.0F;
    Kd << 0.0F;

    g_speed_pid.SetSamplingTime(Config::kSampleTimeS);
    g_speed_pid.SetGain(Kp, Ki, Kd);
    g_speed_pid.Reset();
}

extern "C" void InitializePositionController(){

    using GainMatrix = Eigen::Matrix<float, 1, 1>;
    
    GainMatrix Kp;
    GainMatrix Ki;
    GainMatrix Kd;

    Kp << 1.2F;
    Ki << 0.0F;
    Kd << 0.0F;

    g_pos_pid.SetSamplingTime(Config::kSampleTimeS);
    g_pos_pid.SetGain(Kp, Ki, Kd);
    g_pos_pid.Reset();
}
