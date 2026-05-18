#include "Eigen/Core"
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


UartLogger uart_logger_1{&huart1};
EncoderReader encoder_reader_1{&htim3};
MotorDriver motor_driver_1{&htim2, TIM_CHANNEL_1, MOTOR1_AIN1_GPIO_Port, MOTOR1_AIN1_Pin, MOTOR1_AIN2_GPIO_Port, MOTOR1_AIN2_Pin};
StairSignalGenerator<float, 1, 1, 9> signal_generator_1;
Eigen::Matrix<float, 9, 1> stairs{100, 200, 300, 400, 500, 600, 700, 800, 900};
Eigen::Matrix<float, 9, 1> timestamp{10, 20, 30, 40, 50, 60, 70, 80, 90};


using YVector = Eigen::Matrix<float, 1, 1>;
using UVector = Eigen::Matrix<float, 1, 1>;

const YVector y_ref = YVector::Zero();
const YVector y = YVector::Zero();

UVector u_min{0};
UVector u_max{1000};

void App_Init(void)
{
    encoder_reader_1.Start();
    encoder_reader_1.Reset();

    motor_driver_1.Start();
    motor_driver_1.Stop();

    signal_generator_1.SetSamplingTime(0.1);
    signal_generator_1.SetStairInfo(timestamp, stairs);

    uart_logger_1.SendString("03_pwm_open_loop started\r\n");
    uart_logger_1.SendString("format: time_ms,encoder_count,delta_count\r\n");
}


void App_Step(void)
{
    static uint32_t last_tick = 0U;

    const uint32_t now_tick = HAL_GetTick();

    if ((now_tick - last_tick) >= 100U)
    {
        last_tick = now_tick;

        const UVector u =  signal_generator_1.Step(y_ref, y, u_min, u_max);
        motor_driver_1.SetCommand(MotorDirection::Reverse, static_cast<uint32_t>(u(0)));
        char tx_buf[128];

        const uint16_t encoder_raw_count = encoder_reader_1.GetCount();
        const int16_t encoder_raw_delta = encoder_reader_1.GetDelta();
        std::snprintf(
            tx_buf,
            sizeof(tx_buf), 
            "%lu,%u, %d, %u\r\n",
            static_cast<unsigned long>(now_tick),
            static_cast<unsigned int>(encoder_raw_count),
            static_cast<int>(encoder_raw_delta),
            static_cast<unsigned int>(u(0))
        );
        uart_logger_1.SendString(tx_buf);
        HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
    }
}
