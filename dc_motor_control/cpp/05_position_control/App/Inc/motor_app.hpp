#pragma once

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "motor_driver.hpp"
#include "encoder_reader.hpp"
#include "uart_logger.hpp"
#include "button_manager.hpp"
#include "pid_controller.hpp"

#include <Eigen/Dense>
#include <cstdint>


/**
 * @brief 电机控制应用主类。
 *
 * Notes
 * -----
 * 该类负责组合底层模块并执行应用层逻辑：
 * - encoder 采样；
 * - speed reference 管理；
 * - speed PI 控制；
 * - 后续 position-speed cascade；
 * - button event 处理；
 * - UART 日志输出。
 */
class MotorPositionApp
{
public:
    /**
     * @brief 初始化应用。
     *
     * Returns
     * -------
     * None
     */
    void Initialize();

    /**
     * @brief 应用周期执行函数。
     *
     * Returns
     * -------
     * None
     */
    void Step();

    /**
     * @brief GPIO EXTI 中断桥接入口。
     *
     * Parameters
     * ----------
     * gpio_pin : uint16_t
     *     触发中断的 GPIO pin。
     *
     * Returns
     * -------
     * None
     */
    void OnGpioExti(uint16_t gpio_pin);

private:
    using YVector = Eigen::Matrix<float, 1, 1>;
    using UVector = Eigen::Matrix<float, 1, 1>;
    using GainMatrix = Eigen::Matrix<float, 1, 1>;

    static constexpr uint32_t kSamplePeriodMs = 100U;
    static constexpr float kSampleTimeS = 0.1F;
    static constexpr float kEncoderCountsPerRevolution = 390.0F;

    static constexpr float kPositionRefStepRadS = 5.0F;
    static constexpr float kPositionRefMinRadS = -30.0F;
    static constexpr float kPositionRefMaxRadS = 30.0F;

    MotorDriver motor_driver_{
        &htim2,
        TIM_CHANNEL_1,
        MOTOR1_AIN1_GPIO_Port,
        MOTOR1_AIN1_Pin,
        MOTOR1_AIN2_GPIO_Port,
        MOTOR1_AIN2_Pin
    };

    EncoderReader encoder_reader_{
        &htim3,
        kEncoderCountsPerRevolution
    };

    UartLogger uart_logger_{&huart1};

    ButtonManager button_manager_{
        KEY_POSITION_DOWN_GPIO_Port,
        KEY_POSITION_DOWN_Pin,
        KEY_POSITION_UP_GPIO_Port,
        KEY_POSITION_UP_Pin
    };

    PidController<float, 1, 1> speed_pid_;
    PidController<float, 1, 1> position_pid_;

    uint32_t last_tick_ms_{0U};

    float speed_ref_rad_s_{0.0F};
    float position_ref_rad_{0.0F};

    /**
     * @brief 初始化速度 PI 控制器。
     *
     * Returns
     * -------
     * None
     */
    void InitializeSpeedController();

    /**
     * @brief 初始化速度 PI 控制器。
     *
     * Returns
     * -------
     * None
     */
    void InitializePositionController();

    /**
     * @brief 处理按键事件并更新 reference。
     *
     * Returns
     * -------
     * None
     */
    void HandleButtonEvents();

    /**
     * @brief 执行速度闭环控制。
     *
     * Parameters
     * ----------
     * encoder_meas : const EncoderMeasurement&
     *     当前 encoder 采样结果。
     *
     * Returns
     * -------
     * uint32_t
     *     最终执行的 duty permille。
     */
    int32_t RunPositionControl(const EncoderMeasurement& encoder_meas);

    /**
     * @brief 将 signed duty 输出到电机驱动。
     *
     * Parameters
     * ----------
     * signed_duty_permille : float
     *     带符号 duty，正值 Forward，负值 Reverse。
     *
     * Returns
     * -------
     * uint32_t
     *     实际 duty 绝对值。
     */
    uint32_t ApplySignedDuty(float signed_duty_permille);

    /**
     * @brief 输出 UART 日志。
     *
     * Parameters
     * ----------
     * now_tick_ms : uint32_t
     *     当前 HAL tick。
     * duty_permille : uint32_t
     *     实际 duty。
     * encoder_meas : const EncoderMeasurement&
     *     当前 encoder 采样结果。
     * speed_meas_rad_s : float
     *     当前速度反馈。
     *
     * Returns
     * -------
     * None
     */
    void LogTelemetry(
        const uint32_t now_tick_ms,
        const int32_t signed_duty_permille,
        const EncoderMeasurement& encoder_meas
    );
};
