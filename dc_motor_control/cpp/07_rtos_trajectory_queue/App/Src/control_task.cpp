
#include "task_wrapper.h"

extern "C" void ControlTask(void* pv)
{
    auto* ctx = static_cast<SystemContext*>(pv);

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

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(Config::kSamplePeriodMs);

    for (;;) {
        const EncoderMeasurement encoder_meas =
            ctx->encoder->Sample(Config::kSampleTimeS);

        float point{0};
        if (xQueueReceive(ctx->trajectory_queue, &point, 0) == pdTRUE) {
            ctx->position_ref = point;
        }else{
            point = ctx->position_ref;
        }

        // const float temp_ref = ctx->position_ref;

        position_ref << point;
        position_meas << encoder_meas.position_rad;

        UVector position_u_min;
        UVector position_u_max;

        position_u_min << -60.0F;
        position_u_max << 60.0F;

        speed_ref << ctx->position_controller->Step(
            position_ref,
            position_meas,
            position_u_min,
            position_u_max
        );

        if (std::abs(position_ref(0) - position_meas(0)) < 0.01F) {
            speed_ref << 0.0F;
        }

        speed_meas << encoder_meas.rad_per_second;

        UVector u_ff;
        UVector u_fb;
        UVector signed_duty;
        UVector u_min;
        UVector u_max;

        if (speed_ref(0) < 0.0F) {
            u_ff << -0.2F;
            u_min << -0.8F;
            u_max << 1.2F;
        } else {
            u_ff << 0.2F;
            u_min << -1.2F;
            u_max << 0.8F;
        }

        u_fb = ctx->speed_controller->Step(
            speed_ref,
            speed_meas,
            u_min,
            u_max
        );

        signed_duty = (u_fb + u_ff) * 1000.0F;

        ctx->motor_driver->ApplySignedDuty(signed_duty(0));

        ctx->position_meas = position_meas(0);
        ctx->speed_ref = speed_ref(0);
        ctx->speed_meas = speed_meas(0);
        ctx->duty_cyle = signed_duty(0);

        vTaskDelayUntil(&last_wake_time, period);
    }
}

