
#include "task_wrapper.h"

extern "C" void LoggerTask(void* pv)
{
    auto* ctx = static_cast<SystemContext*>(pv);

    ctx->uart_logger->SendString("logger task started\r\n");

    uint32_t last_tick = HAL_GetTick();
    char tx_buf[160];

    for (;;) {
        const uint32_t now = HAL_GetTick();
        const uint32_t delta = now - last_tick;

        std::snprintf(
            tx_buf,
            sizeof(tx_buf),
            "%lu,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
            static_cast<unsigned long>(delta),
            static_cast<long>(ctx->speed_ref * 1000.0F),
            static_cast<long>(ctx->speed_meas * 1000.0F),
            static_cast<long>(ctx->position_ref * 1000.0F),
            static_cast<long>(ctx->position_meas * 1000.0F),
            static_cast<long>(ctx->position_ref_filtered * 1000.0F),
            static_cast<long>(ctx->duty_cyle * 1000.0F)
        );

        ctx->uart_logger->SendString(tx_buf);

        last_tick = now;

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

