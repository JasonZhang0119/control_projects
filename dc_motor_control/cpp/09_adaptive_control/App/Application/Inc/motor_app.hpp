#pragma once

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "motor_driver.hpp"
#include "encoder_reader.hpp"
#include "uart_logger.hpp"
#include "uart_cli.hpp"
#include "button_manager.hpp"
#include "pid_controller.hpp"
#include "parameter_manager.hpp"
#include "motor_mode.hpp"

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
class DCMotorApp
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

    /**
     * @brief UART 接收完成中断桥接入口。
     *
     * Parameters
     * ----------
     * huart : UART_HandleTypeDef*
     *     触发接收完成回调的 UART 句柄。
     */
    void OnUartRxComplete(UART_HandleTypeDef* huart);

    void SetMode(MotorMode mode);
    MotorMode GetMode() const;

private:
    /** @brief 控制器输出/测量向量类型，当前为 SISO。 */
    using YVector = Eigen::Matrix<float, 1, 1>;

    /** @brief 控制输入向量类型，当前为 SISO。 */
    using UVector = Eigen::Matrix<float, 1, 1>;

    /** @brief PID 增益矩阵类型。 */
    using GainMatrix = Eigen::Matrix<float, 1, 1>;

    /** @brief 控制循环采样周期，单位 ms。 */
    static constexpr uint32_t kSamplePeriodMs = 100U;

    /** @brief 控制循环采样周期，单位 s。 */
    static constexpr float kSampleTimeS = 0.1F;

    /** @brief 输出轴每转对应的编码器 count 数。 */
    static constexpr float kEncoderCountsPerRevolution = 390.0F;

    /** @brief 每次按键调整的位置参考步长，单位 rad。 */
    static constexpr float kPositionRefStepRadS = 5.0F;

    /** @brief 位置参考最小值，单位 rad。 */
    static constexpr float kPositionRefMinRadS = -30.0F;

    /** @brief 位置参考最大值，单位 rad。 */
    static constexpr float kPositionRefMaxRadS = 30.0F;


    /** @brief 每次按键调整的速度参考步长，单位 rad。 */
    static constexpr float kSpeedRefStepRadS = 0.5F;

    /** @brief 速度参考最小值，单位 rad/s。 */
    static constexpr float kSpeedRefMinRadS = -50.0F;

    /** @brief 速度参考最大值，单位 rad/s。 */
    static constexpr float kSpeedRefMaxRadS = 50.0F;


    /** @brief 电机 PWM 与方向驱动模块。 */
    MotorDriver motor_driver_{
        &htim2,
        TIM_CHANNEL_1,
        MOTOR1_AIN1_GPIO_Port,
        MOTOR1_AIN1_Pin,
        MOTOR1_AIN2_GPIO_Port,
        MOTOR1_AIN2_Pin
    };

    /** @brief 编码器读取与速度/位置换算模块。 */
    EncoderReader encoder_reader_{
        &htim3,
        kEncoderCountsPerRevolution
    };

    /** @brief UART 字符串输出模块。 */
    UartLogger uart_logger_{&huart1};

    /** @brief 应用参数管理模块。 */
    ParameterManager parameter_manager_;

    /** @brief USART 文本命令行接口。 */
    UartCli uart_cli_{&huart1, &uart_logger_, &parameter_manager_};

    /** @brief 按键事件管理模块。 */
    ButtonManager button_manager_{
        KEY_POSITION_DOWN_GPIO_Port,
        KEY_POSITION_DOWN_Pin,
        KEY_POSITION_UP_GPIO_Port,
        KEY_POSITION_UP_Pin
    };

    /** @brief 速度环 PID 控制器。 */
    PidController<float, 1, 1> speed_pid_;

    /** @brief 位置环 PID 控制器。 */
    PidController<float, 1, 1> position_pid_;

    /** @brief 上一次控制循环执行时间，单位 ms。 */
    uint32_t last_tick_ms_{0U};

    /** @brief 当前速度参考值，单位 rad/s。 */
    float speed_ref_rad_s_{0.0F};

    /** @brief 当前位置参考值，单位 rad。 */
    float position_ref_rad_{0.0F};

    /** @brief 最近一次带符号 duty 输出。 */
    int32_t signed_duty_permille_{0};

    /** @brief 最近一次编码器采样结果。 */
    EncoderMeasurement last_encoder_meas_{};

    /** @brief 控制模式 */
    volatile MotorMode mode_{MotorMode::Position};

    /**
     * @brief 初始化位置 PI 控制器。
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
     * @brief 执行位置闭环控制。
     *
     * Parameters
     * ----------
     * encoder_meas : const EncoderMeasurement&
     *     当前 encoder 采样结果。
     *
     * Returns
     * -------
     * int32_t
     *     最终执行的带符号 duty permille。
     */
    int32_t RunPositionControl(const EncoderMeasurement& encoder_meas);


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
     * int32_t
     *     最终执行的带符号 duty permille。
     */
    int32_t RunSpeedControl(const EncoderMeasurement& encoder_meas);


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
