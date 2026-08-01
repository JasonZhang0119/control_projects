#include "main.h"
#include "stm32f1xx_hal_tim.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

#include <Eigen/Dense>

#include "app_bridge.h"
#include "signal_generator.hpp"
#include "encoder_reader.hpp"
#include "uart_logger.hpp"
#include "motor_driver.hpp"


namespace
{

using YVector = Eigen::Matrix<float, 1, 1>;
using UVector = Eigen::Matrix<float, 1, 1>;

/**
 * @brief 控制与采样周期。
 *
 * Notes
 * -----
 * 当前 polling 阶段让 signal generator、PWM 更新、encoder 采样和 UART 日志使用同一个周期。
 * 后续进入中断或 RTOS 后，可以再拆成多速率结构。
 */
static constexpr uint32_t kSamplePeriodMs = 50U;
static constexpr float kSampleTimeS = 0.02F;


/**
 * @brief 编码器每转计数。
 *
 * Notes
 * -----
 * 当前先使用 390.0F。
 * 如果后续发现电机输出轴一圈实际 count 不同，需要重新标定该值。
 */
static constexpr float kEncoderCountsPerRevolution = 390.0F;


UartLogger uart_logger_1{&huart1};

EncoderReader encoder_reader_1{
    &htim3,
    kEncoderCountsPerRevolution
};

MotorDriver motor_driver_1{
    &htim2,
    TIM_CHANNEL_1,
    MOTOR1_AIN1_GPIO_Port,
    MOTOR1_AIN1_Pin,
    MOTOR1_AIN2_GPIO_Port,
    MOTOR1_AIN2_Pin
};

StairSignalGenerator<float, 1, 1, 9> signal_generator_stairs;


/**
 * @brief Stair duty 序列。
 *
 * Notes
 * -----
 * 当前 duty 单位为 permille：
 * - 500 表示 50%
 * - 700 表示 70%
 *
 * 你已经测试到 duty = 500 左右电机可以正常旋转，
 * 因此当前序列覆盖启动死区附近和可旋转区域。
 */
Eigen::Matrix<float, 9, 1> stairs{
    0.0F,
    100.0F,
    200.0F,
    300.0F,
    400.0F,
    500.0F,
    600.0F,
    700.0F,
    600.0F
};


/**
 * @brief Stair 切换时间或切换索引。
 *
 * Notes
 * -----
 * 这里的单位取决于你的 StairSignalGenerator 实现：
 *
 * - 如果内部按真实时间比较，则这里表示秒；
 * - 如果内部按采样点比较，则这里表示 sample index。
 *
 * 当前保留你已经跑通的写法逻辑。
 */
Eigen::Matrix<float, 9, 1> timestamp{
    0.0F,
    10.0F,
    20.0F,
    30.0F,
    40.0F,
    50.0F,
    60.0F,
    70.0F,
    80.0F
};

UVector u_min{0.0F};
UVector u_max{1000.0F};

ChirpSignalGenerator<float, 1, 1> signal_generator_chirp;


const YVector y_ref = YVector::Zero();
const YVector y = YVector::Zero();

UVector u_min_chirp{300.0F};
UVector u_max_chirp{900.0F};

UVector start_time{5.0F};
UVector end_time{100.0F};
UVector start_frequency{0.05F};
UVector end_frequency{2.0F};
UVector bias{600.0F};
UVector amplitude{200.0F};

}  // namespace


void App_Init(void)
{
    encoder_reader_1.Start();
    encoder_reader_1.Reset();

    motor_driver_1.Start();
    motor_driver_1.Stop();

    signal_generator_stairs.SetSamplingTime(kSampleTimeS);
    signal_generator_stairs.SetStairInfo(timestamp, stairs);

    signal_generator_chirp.SetSamplingTime(kSampleTimeS);

    signal_generator_chirp.SetChirpInfo(
        start_time,
        end_time,
        start_frequency,
        end_frequency,
        bias,
        amplitude
    );

    uart_logger_1.SendString("03_pwm_open_loop started\r\n");
    uart_logger_1.SendString(
        "format: time_ms,duty_permille,count,delta_count,rpm_x1000,rad_s_x1000\r\n"
    );
}


void App_Step(void)
{
    static uint32_t last_tick = 0U;

    const uint32_t now_tick = HAL_GetTick();

    if ((now_tick - last_tick) >= kSamplePeriodMs)
    {
        last_tick = now_tick;

        // const UVector u =
        //     signal_generator_stairs.Step(y_ref, y, u_min, u_max);

        const UVector u = signal_generator_chirp.Step(y_ref, y, u_min_chirp, u_max_chirp);

        const uint32_t duty_permille =
            static_cast<uint32_t>(u(0));

        motor_driver_1.SetCommand(
            MotorDirection::Reverse,
            duty_permille
        );

        const EncoderMeasurement encoder_meas =
            encoder_reader_1.Sample(kSampleTimeS);

        const int32_t rpm_x1000 =
            static_cast<int32_t>(encoder_meas.rpm * 1000.0F);

        const int32_t rad_s_x1000 =
            static_cast<int32_t>(encoder_meas.rad_per_second * 1000.0F);

        char tx_buf[160];

        std::snprintf(
            tx_buf,
            sizeof(tx_buf),
            "%lu,%lu,%u,%d,%ld,%ld\r\n",
            static_cast<unsigned long>(now_tick),
            static_cast<unsigned long>(duty_permille),
            static_cast<unsigned int>(encoder_meas.count),
            static_cast<int>(encoder_meas.delta_count),
            static_cast<long>(rpm_x1000),
            static_cast<long>(rad_s_x1000)
        );

        uart_logger_1.SendString(tx_buf);

        HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
    }
}