#include "app_bridge.h"
#include "motor_app.hpp"

namespace
{
MotorVelocityApp motor_app;
}

extern "C" void App_Init(void)
{
    motor_app.Initialize();
}

extern "C" void App_Step(void)
{
    motor_app.Step();
}

extern "C" void App_OnGpioExti(uint16_t GPIO_Pin)
{
    motor_app.OnGpioExti(GPIO_Pin);
}