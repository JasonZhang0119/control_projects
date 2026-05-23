#pragma once

#include "FreeRTOS.h" // 必须先包含这个
#include "queue.h"    // 然后包含这个

#include "button_manager.hpp"
#include "motor_driver.hpp"
#include "uart_logger.hpp"
#include "encoder_reader.hpp"
#include "pid_controller.hpp"

namespace Config {
    static constexpr int Ny = 1;
    static constexpr int Nu = 1;
    static constexpr uint32_t kSamplePeriodMs = 100U;
    static constexpr float kSampleTimeS = 0.1F;
    static constexpr float kEncoderCountsPerRevolution = 390.0F;

    static constexpr float kPositionRefStepRadS = 5.0F;
    static constexpr float kPositionRefMinRadS = -30.0F;
    static constexpr float kPositionRefMaxRadS = 30.0F;
}


struct SystemContext {

    // 硬件驱动指针
    ButtonManager* position_button;
    MotorDriver* motor_driver;
    EncoderReader* encoder;
    UartLogger* uart_logger;

    // PID 控制器 (使用 Config 命名空间里的常量)
    PidController<float, Config::Ny, Config::Nu>* speed_controller;
    PidController<float, Config::Ny, Config::Nu>* position_controller;
    
    // 通信需求
    xQueueHandle button_event_queue; 

    // 共享状态 (Telemetry 数据区)
    // 使用 volatile 关键字防止任务间访问时编译器进行错误的寄存器优化
    volatile float position_ref{0.0F};
    volatile float position_meas{0.0F};
    volatile float speed_ref{0.0F};
    volatile float speed_meas{0.0F};
};


// 在 task_wrapper.hpp 中加入声明
extern "C" void ControlTask(void* pv);
extern "C" void LoggerTask(void* pv);
extern "C" void ButtonTask(void* pv);