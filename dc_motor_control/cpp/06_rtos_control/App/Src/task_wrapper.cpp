
#include "Eigen/Dense"

#include "task_wrapper.h"

#include "button_manager.hpp"
#include "motor_driver.hpp"
#include "encoder_reader.hpp"
#include "pid_controller.hpp"


extern "C" void ControlTask(void* pv) {
    auto* ctx = static_cast<SystemContext*>(pv);
    // 假设 ControlEngine 内部有 PID 状态
    ctx->position_controller->Initialize();
    ctx->speed_controller->Initialize();
    ctx->encoder->Reset();
    ctx->motor_driver->Start();

    using YVector = Eigen::Matrix<float, 1, 1>;
    using UVector = Eigen::Matrix<float, 1, 1>;

    YVector speed_ref;
    YVector speed_meas;
    YVector position_ref;
    YVector position_meas;

    for(;;) {

        const EncoderMeasurement encoder_meas =
            ctx->encoder->Sample(Config::kSampleTimeS);

        float temp_ref = ctx->position_ref; 
        position_ref << temp_ref;
        position_meas << encoder_meas.position_rad;

        speed_ref << ctx->position_controller->Step(
                position_ref,
                position_meas,
                UVector{-60},
                UVector{60}
        );

        if (std::abs(position_ref(0) - position_meas(0)) < 0.1){
            speed_ref << 0;
        }

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

        u_fb = ctx->speed_controller->Step(
                speed_ref,
                speed_meas,
                u_min,
                u_max
            );
        
        signed_duty = (u_fb + u_ff) * 1000;

        ctx->motor_driver->ApplySignedDuty(signed_duty(0));

        ctx->position_meas = position_meas(0);
        ctx->speed_ref = speed_ref(0);
        ctx->speed_meas = speed_meas(0);
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz 周期
    }
}


extern "C" void LoggerTask(void* pv) {
    
    auto* ctx = static_cast<SystemContext*>(pv);
    ctx->uart_logger->SendString("logger task started\r\n");

    uint32_t last_tick = HAL_GetTick();

    char tx_buf[160];

    for(;;) {

        uint32_t now = HAL_GetTick();
        uint32_t delta = now - last_tick;

        std::snprintf(
            tx_buf,
            sizeof(tx_buf),
            "%lu,%ld,%ld, %ld,%ld\r\n",
            static_cast<unsigned long>(delta),
            static_cast<long>(ctx->speed_ref * 1000.0F),
            static_cast<long>(ctx->speed_meas * 1000.0F),
            static_cast<long>(ctx->position_ref * 1000.0F),
            static_cast<long>(ctx->position_ref * 1000.0F)
        );

        ctx->uart_logger->SendString(tx_buf);
        last_tick = now;
        vTaskDelay(pdMS_TO_TICKS(50)); // 100Hz 周期
    }
}


extern "C" void ButtonTask(void* pv) {
    // 1. 获取上下文
    auto* ctx = static_cast<SystemContext*>(pv);
    
    // 2. 初始日志
    if (ctx->uart_logger != nullptr) {
        ctx->uart_logger->SendString("button task started\r\n");
    }

    ButtonEvent event;

    for(;;) {

        if (xQueueReceive(ctx->button_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            
            // 4. 处理获取到的第一个事件

            switch (event)
            {
            case ButtonEvent::SpeedUp:

                ctx->position_ref += Config::kPositionRefStepRadS;
                if(ctx->position_ref > Config::kPositionRefMaxRadS){
                    ctx->position_ref = Config::kPositionRefMaxRadS;
                }
                break;

            case ButtonEvent::SpeedDown:
                ctx->position_ref -= Config::kPositionRefStepRadS;
                if (ctx->position_ref < Config::kPositionRefMinRadS){
                   ctx->position_ref = Config::kPositionRefMinRadS;
                }
                break;

            case ButtonEvent::None:
            default:
                break;
            }

            // 5. 快速排空：如果有积压的按键事件，一次性处理完
            // 使用 uxQueueMessagesWaiting 检查队列是否还有残留
            while (uxQueueMessagesWaiting(ctx->button_event_queue) > 0) {
                if (xQueueReceive(ctx->button_event_queue, &event, 0) == pdTRUE) {

                    switch (event)
                    {
                    case ButtonEvent::SpeedUp:

                        ctx->position_ref += Config::kPositionRefStepRadS;
                        if(ctx->position_ref > Config::kPositionRefMaxRadS){
                            ctx->position_ref = Config::kPositionRefMaxRadS;
                        }
                        break;

                    case ButtonEvent::SpeedDown:
                        ctx->position_ref -= Config::kPositionRefStepRadS;
                        if (ctx->position_ref < Config::kPositionRefMinRadS){
                        ctx->position_ref = Config::kPositionRefMinRadS;
                        }
                        break;

                    case ButtonEvent::None:
                    default:
                        break;
                    }
                }
            }
        }
    }
}