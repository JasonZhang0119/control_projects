#pragma once

#ifdef __cplusplus

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "button_manager.hpp"
#include "motor_driver.hpp"
#include "uart_logger.hpp"
#include "encoder_reader.hpp"
#include "pid_controller.hpp"

namespace Config {
    static constexpr int Ny = 1;
    static constexpr int Nu = 1;

    static constexpr uint32_t kSamplePeriodMs = 10U;
    static constexpr float kSampleTimeS = 0.01F;

    static constexpr float kEncoderCountsPerRevolution = 390.0F;

    static constexpr float kPositionRefStepRadS = 5.0F;
    static constexpr float kPositionRefMinRadS = -30.0F;
    static constexpr float kPositionRefMaxRadS = 30.0F;
}

struct SystemContext {
    ButtonManager* position_button;
    MotorDriver* motor_driver;
    EncoderReader* encoder;
    UartLogger* uart_logger;

    PidController<float, Config::Ny, Config::Nu>* speed_controller;
    PidController<float, Config::Ny, Config::Nu>* position_controller;

    QueueHandle_t button_event_queue;

    volatile float position_ref;
    volatile float position_meas;
    volatile float speed_ref;
    volatile float speed_meas;
    volatile float duty_cyle;
};


#endif

#ifdef __cplusplus
extern "C" {
#endif

void Module_Init(void);
void* Module_GetSystemContext(void);

void ControlTask(void* pv);
void ButtonTask(void* pv);
void LoggerTask(void* pv);

void Module_RegisterButtonTaskHandle(void* handle);
void Module_OnButtonExtiFromISR(uint16_t gpio_pin);

#ifdef __cplusplus
}
#endif