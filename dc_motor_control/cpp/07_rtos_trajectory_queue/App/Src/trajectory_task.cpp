
#include "task_wrapper.h"
#include "main.h"


extern "C" void TrajectoryTask(void* pv)
{
    auto* ctx = static_cast<SystemContext*>(pv);

    if (ctx->uart_logger != nullptr) {
        ctx->uart_logger->SendString("trajectory task started\r\n");
    }

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ctx->trajectory_generator->Set(ctx->position_meas, 
                                        ctx->position_ref);
        xQueueReset(ctx->trajectory_queue);
        for (int i = 0; i < Config::kTrajectoryPoint; i++){
            float point = ctx->trajectory_generator->Next();
            xQueueSend(
                ctx->trajectory_queue,
                &point,
                portMAX_DELAY
            );
        }

    }
}